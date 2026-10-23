#include "sapphirelib/motion/holonomic_tracking.hpp"

#include <cmath>

#include "sapphirelib/motion/holonomic_feedforward.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// the reference with its motion taken out: where the plan ends, at rest
TrajectorySample atRest(TrajectorySample sample) {
    sample.speedInPerS = 0.0;
    sample.accelInPerS2 = 0.0;
    sample.vxInPerS = 0.0;
    sample.vyInPerS = 0.0;
    sample.omegaDegPerS = 0.0;
    sample.forwardInPerS = 0.0;
    sample.strafeInPerS = 0.0;
    sample.forwardAccelInPerS2 = 0.0;
    sample.strafeAccelInPerS2 = 0.0;
    sample.turnAccelDegPerS2 = 0.0;
    sample.stop = true;
    return sample;
}

// how close the start of a trajectory that carries on from another must be to that one's end:
// planned with TrajectoryOptions::from they match exactly
constexpr double kJoinPositionIn = 0.25;
constexpr double kJoinHeadingDeg = 1.0;
constexpr double kJoinSpeedInPerS = 0.5;

// whether a trajectory ends moving, so another has to carry on from it
bool endsMoving(const Trajectory& trajectory) {
    const TrajectorySample& end = trajectory.samples().back();
    return !end.stop && end.speedInPerS > 0.0;
}

bool continuesFrom(const Trajectory& next, const Trajectory& previous) {
    const TrajectorySample& a = previous.samples().back();
    const TrajectorySample& b = next.samples().front();
    return std::hypot(b.xIn - a.xIn, b.yIn - a.yIn) <= kJoinPositionIn &&
           std::fabs(wrapDegrees180(b.headingDeg - a.headingDeg)) <= kJoinHeadingDeg &&
           std::fabs(b.speedInPerS - a.speedInPerS) <= kJoinSpeedInPerS;
}

} // namespace

TrackingErrors trackingErrors(const TrajectorySample& reference, const odom::Pose& pose,
                              const odom::Velocity& velocity) {
    const double dxIn = reference.xIn - pose.xIn;
    const double dyIn = reference.yIn - pose.yIn;
    const LocalOffset position = toLocalFrame(dxIn, dyIn, pose.headingDeg);
    const LocalOffset rate = toLocalFrame(reference.vxInPerS - velocity.xInPerS,
                                          reference.vyInPerS - velocity.yInPerS, pose.headingDeg);

    // the path's direction on the field, from the planned robot-frame direction: defined even
    // where the plan is at rest
    const double heading = reference.headingDeg * kDegToRad;
    const double ux =
        reference.forwardPerIn * std::sin(heading) + reference.strafePerIn * std::cos(heading);
    const double uy =
        reference.forwardPerIn * std::cos(heading) - reference.strafePerIn * std::sin(heading);

    return TrackingErrors{
        .forwardIn = position.forwardIn,
        .strafeIn = position.lateralIn,
        .headingDeg = wrapDegrees180(reference.headingDeg - pose.headingDeg),
        .forwardRateInPerS = rate.forwardIn,
        .strafeRateInPerS = rate.lateralIn,
        .headingRateDegPerS = reference.omegaDegPerS - velocity.headingDegPerS,
        .alongTrackIn = dxIn * ux + dyIn * uy,
        // the robot's offset (minus the error) onto the direction's right, (uy, -ux)
        .crossTrackIn = -(dxIn * uy - dyIn * ux),
    };
}

ClockStep advanceTrajectoryClock(double tS, double dtS, double alongTrackIn, double maxLagIn) {
    if (maxLagIn > 0.0 && alongTrackIn > maxLagIn) return ClockStep{.tS = tS, .paused = true};
    return ClockStep{.tS = tS + (dtS > 0.0 ? dtS : 0.0), .paused = false};
}

TrajectoryFollower::TrajectoryFollower(PID& forward, PID& strafe, PID& turn)
    : forward_(forward), strafe_(strafe), turn_(turn) {}

void TrajectoryFollower::begin(const Trajectory& trajectory,
                               const chassis::HolonomicAxisModels& models,
                               TrajectoryFollowConfig config, std::uint32_t nowMs) {
    trajectory_ = &trajectory;
    models_ = models;
    config_ = config;
    const auto durationMs = static_cast<std::uint32_t>(std::ceil(trajectory.durationS() * 1000.0));
    tracker_.emplace(config.settleTimeMs, durationMs + config.overtimeMs, nowMs);
    reason_ = ExitReason::running;
    referenceTimeS_ = 0.0;
    lastAlongTrackIn_ = 0.0;
    lastMs_ = nowMs;
    started_ = false;
    finalErrorIn_ = 0.0;
    progress_ = TrajectoryProgress{.running = true};
    piece_ = 0;
    pieceStartS_ = 0.0;
    pieceFirstSection_ = 0;
    pieceStartIn_ = 0.0;
    chainEnded_ = false;
    forward_.reset();
    strafe_.reset();
    turn_.reset();
}

TrackingTick TrajectoryFollower::update(const odom::Pose& pose, const odom::Velocity& velocity,
                                        std::uint32_t nowMs) {
    const double dtS = started_ ? sapphirelib::elapsedMs(lastMs_, nowMs) / 1000.0 : 0.0;
    lastMs_ = nowMs;

    // the clock moves on by the time since the last tick, unless the robot was too far behind
    // the reference it last chased
    const ClockStep clock =
        started_ ? advanceTrajectoryClock(referenceTimeS_, dtS, lastAlongTrackIn_, config_.maxLagIn)
                 : ClockStep{.tS = 0.0, .paused = false};
    started_ = true;
    referenceTimeS_ = clock.tS;

    // past the end of a trajectory that ends moving: carry on into the next one, the clock
    // running on into it. A loop, in case a whole short one fits in one tick
    ChainEvent chain = ChainEvent::none;
    while (!chainEnded_ && referenceTimeS_ >= trajectory_->durationS() &&
           endsMoving(*trajectory_)) {
        const Trajectory* next = config_.next ? config_.next(*trajectory_) : nullptr;
        if (next == nullptr || next->empty() || !continuesFrom(*next, *trajectory_)) {
            chain = next == nullptr || next->empty() ? ChainEvent::noNext : ChainEvent::mismatched;
            chainEnded_ = true;
            break;
        }
        referenceTimeS_ -= trajectory_->durationS();
        pieceStartS_ += trajectory_->durationS();
        pieceFirstSection_ += trajectory_->sectionCount();
        pieceStartIn_ += trajectory_->lengthIn();
        tracker_->extendTimeout(static_cast<std::uint32_t>(std::ceil(next->durationS() * 1000.0)));
        trajectory_ = next;
        ++piece_;
        chain = ChainEvent::continued;
    }

    const Trajectory& trajectory = *trajectory_;
    const bool holding = referenceTimeS_ >= trajectory.durationS();

    TrackingTick tick;
    tick.referenceTimeS = pieceStartS_ + referenceTimeS_;
    tick.reference =
        holding ? atRest(trajectory.sample(referenceTimeS_)) : trajectory.sample(referenceTimeS_);
    tick.errors = trackingErrors(tick.reference, pose, velocity);
    if (!holding) {
        tick.feedforward = trajectoryFeedforward(
            models_, tick.reference, wrapDegrees180(pose.headingDeg - tick.reference.headingDeg));
    }
    // a PD per axis on the error, with the measured velocity error as the D term: the same
    // trick as the turn loops, the error as the target and 0 as the measurement
    const double pidDtS = dtS > 0.0 ? dtS : 0.0;
    tick.volts = chassis::AxisVolts{
        .forward = tick.feedforward.forward + forward_.updateWithRate(tick.errors.forwardIn, 0.0,
                                                                      tick.errors.forwardRateInPerS,
                                                                      pidDtS),
        .strafe =
            tick.feedforward.strafe +
            strafe_.updateWithRate(tick.errors.strafeIn, 0.0, tick.errors.strafeRateInPerS, pidDtS),
        .turn =
            tick.feedforward.turn + turn_.updateWithRate(tick.errors.headingDeg, 0.0,
                                                         tick.errors.headingRateDegPerS, pidDtS),
    };
    tick.paused = clock.paused;
    tick.holding = holding;
    tick.section = pieceFirstSection_ + trajectory.sectionAt(tick.reference.sIn);
    tick.piece = piece_;
    tick.chain = chain;
    lastAlongTrackIn_ = tick.errors.alongTrackIn;

    const TrajectorySample& last = trajectory.samples().back();
    finalErrorIn_ = std::hypot(last.xIn - pose.xIn, last.yIn - pose.yIn);
    const double headingErrorDeg = std::fabs(wrapDegrees180(last.headingDeg - pose.headingDeg));
    const bool within = holding && finalErrorIn_ <= config_.positionErrorThresholdIn &&
                        headingErrorDeg <= config_.headingErrorThresholdDeg;
    reason_ = tracker_->update(within, nowMs);

    progress_ = TrajectoryProgress{.running = reason_ == ExitReason::running,
                                   .section = tick.section,
                                   .distanceIn = pieceStartIn_ + tick.reference.sIn,
                                   .timeS = tick.referenceTimeS,
                                   .paused = clock.paused,
                                   .piece = piece_};
    return tick;
}

ExitReason TrajectoryFollower::reason() const { return reason_; }

MotionResult TrajectoryFollower::result() const {
    return MotionResult{.reason = reason_,
                        .finalError = finalErrorIn_,
                        .elapsedMs = tracker_ ? tracker_->elapsedMs() : 0};
}

TrajectoryProgress TrajectoryFollower::progress() const { return progress_; }

const Trajectory& TrajectoryFollower::trajectory() const { return *trajectory_; }

double TrajectoryFollower::pieceStartS() const { return pieceStartS_; }

std::size_t TrajectoryFollower::pieceFirstSection() const { return pieceFirstSection_; }

} // namespace sapphirelib::motion
