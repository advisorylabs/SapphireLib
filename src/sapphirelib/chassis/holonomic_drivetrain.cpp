#include "sapphirelib/chassis/holonomic_drivetrain.hpp"

#include <algorithm>
#include <cmath>

#include "pros/rtos.hpp"
#include "sapphirelib/chassis/thermal_math.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/sensors/imu_scale_math.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/log.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kLoopDelayMs = 10;

constexpr const char* kNoOdometry = "no odometry - call setOdometry() first, or pass one";

/// Full motor voltage. holonomic() takes normalized [-1, 1] sticks while
/// holonomicVolts() and the PIDs work in volts; this is the scale between
/// them in DriverInputMode::voltage.
constexpr double kFullScaleVolts = 12.0;

/// Stick magnitude treated as centered in DriverInputMode::velocity — see
/// stickVolts().
constexpr double kVelocityModeDeadband = 0.03;

/// A gap longer than this between heading-hold calls means the driver loop
/// wasn't running — an autonomous routine, a PID tuner test, a calibration
/// spin — and whatever heading was being held belongs to before that, not
/// now. Generous next to a 10-20ms driver loop, short next to any of those
/// interruptions.
constexpr double kHeadingHoldResumeGapS = 0.35;

/// How often motor temperatures are actually re-read. A V5 motor takes tens
/// of seconds to move a degree under load, so polling this near the 100Hz
/// rate setWheelVoltages() runs at would be six device reads per tick for a
/// number that hasn't changed.
constexpr std::uint32_t kThermalPollIntervalMs = 500;

struct WheelMix {
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
};

/// Standard mecanum/X-drive mixing equations. `throttle`/`strafe`/`turn`
/// share units (either normalized [-1, 1] driver input, or PID output volts
/// — the caller decides); the result is in those same units, unclamped.
WheelMix mixHolonomic(double throttle, double strafe, double turn) {
    return {throttle + strafe + turn, throttle - strafe - turn, throttle - strafe + turn,
            throttle + strafe - turn};
}

/// Logs a motion's `end` event (see docs/TELEMETRY_FORMAT.md) and hands its
/// result back, so every way out of a motion reads `return finishMotion(...)`.
motion::MotionResult finishMotion(const char* kind, const motion::MotionResult& result) {
    telemetry::event("motion", "end,%s,reason=%s,error=%.3f,ms=%u", kind,
                     motion::exitReasonName(result.reason), result.finalError,
                     static_cast<unsigned>(result.elapsedMs));
    return result;
}

/// For a motion that can't start at all (no odometry to read, an empty
/// path): says why on the terminal, and still logs a start/end pair — the
/// start with no parameters — so a telemetry log shows the motion was asked
/// for and why nothing happened.
motion::MotionResult abortMotion(const char* kind, const char* why) {
    SAPPHIRELIB_LOG_ERROR("motion", "%s: %s", kind, why);
    telemetry::event("motion", "start,%s", kind);
    return finishMotion(kind, motion::MotionResult{.reason = motion::ExitReason::aborted});
}

} // namespace

HolonomicDrivetrain::HolonomicDrivetrain(std::int8_t frontLeftPort, std::int8_t frontRightPort,
                                          std::int8_t backLeftPort, std::int8_t backRightPort,
                                          Gearset gearset, std::uint8_t imuPort,
                                          DrivetrainConfig config, PID::Config drivePIDConfig,
                                          PID::Config turnPIDConfig, double imuHeadingScale,
                                          std::optional<AsteriskConfig> asterisk)
    : frontLeft_({frontLeftPort}, gearset),
      frontRight_({frontRightPort}, gearset),
      backLeft_({backLeftPort}, gearset),
      backRight_({backRightPort}, gearset),
      imu_(imuPort, imuHeadingScale),
      config_(config),
      drivePID_(drivePIDConfig),
      turnPID_(turnPIDConfig),
      headingHoldPID_(turnPIDConfig),
      asterisk_(asterisk) {
    if (asterisk_) {
        const std::initializer_list<std::int8_t> middleLeftPorts{asterisk_->middleLeftPort};
        const std::initializer_list<std::int8_t> middleRightPorts{asterisk_->middleRightPort};
        middleLeft_.emplace(middleLeftPorts, gearset);
        middleRight_.emplace(middleRightPorts, gearset);

        // Center wheels coast whenever they're not actively driving,
        // turning, or correcting drift — see setWheelVoltages().
        middleLeft_->setBrakeMode(BrakeMode::coast);
        middleRight_->setBrakeMode(BrakeMode::coast);
    }

    // sensors::Imu's constructor already blocks until IMU calibration
    // finishes, so the heading (used here and by
    // driveDistance()/turnToHeading()/headingDeg()) is valid as soon as
    // this constructor returns.
    fieldHeadingZeroDeg_ = rotationHeadingDeg();
}

sensors::Imu& HolonomicDrivetrain::imu() { return imu_; }

PID& HolonomicDrivetrain::drivePID() { return drivePID_; }

PID& HolonomicDrivetrain::turnPID() { return turnPID_; }

PID& HolonomicDrivetrain::headingHoldPID() { return headingHoldPID_; }

void HolonomicDrivetrain::setDriverInputMode(DriverInputMode mode) { driverInputMode_.store(mode); }

DriverInputMode HolonomicDrivetrain::driverInputMode() const { return driverInputMode_.load(); }

void HolonomicDrivetrain::setAxisModels(HolonomicAxisModels models) { *axisModels_.lock() = models; }

HolonomicAxisModels HolonomicDrivetrain::axisModels() const { return *axisModels_.lock(); }

void HolonomicDrivetrain::setOdometry(const odom::Odometry* odometry) { odometry_ = odometry; }

AxisVolts HolonomicDrivetrain::appliedAxisVolts() const {
    return AxisVolts{.forward = appliedForwardVolts_.load(),
                     .strafe = appliedStrafeVolts_.load(),
                     .turn = appliedTurnVolts_.load()};
}

void HolonomicDrivetrain::recordAppliedVolts(double forwardVolts, double strafeVolts,
                                             double turnVolts) {
    appliedForwardVolts_.store(forwardVolts);
    appliedStrafeVolts_.store(strafeVolts);
    appliedTurnVolts_.store(turnVolts);
}

void HolonomicDrivetrain::setDriftSource(const odom::TrackingWheel* verticalWheel,
                                         const odom::Odometry* odometry) {
    driftSource_ = verticalWheel;
    driftOffsetSource_ = odometry;
    if (driftSource_ != nullptr) {
        lastDriftVerticalIn_ = driftSource_->getDistanceIn();
        lastDriftStrafeIn_ = encoderStrafeIn();
        lastDriftHeadingDeg_ = imu_.getCumulativeHeadingDeg();
        lastDriftTickMs_ = pros::millis();
    }
}

double HolonomicDrivetrain::encoderStrafeIn() const {
    return degreesToInches((frontLeft_.getPositionDegrees() - frontRight_.getPositionDegrees() -
                            backLeft_.getPositionDegrees() + backRight_.getPositionDegrees()) /
                           4.0);
}

void HolonomicDrivetrain::refreshThermalFractions() {
    // Only reached with the Asterisk wheels configured (setWheelVoltages()
    // returns before this otherwise). Bailing here when the feature is off
    // keeps six device reads per poll off a chassis that isn't using it —
    // the fractions stay at their 1.0 defaults, which produce no correction.
    if (!asterisk_ || asterisk_->thermalCompensation <= 0.0) return;

    const std::uint32_t now = pros::millis();
    if (lastThermalPollMs_ != 0 && now - lastThermalPollMs_ < kThermalPollIntervalMs) return;
    lastThermalPollMs_ = now;

    thermalFractions_ = CornerValues{
        thermalPowerFraction(frontLeft_.getTemperatureC()),
        thermalPowerFraction(frontRight_.getTemperatureC()),
        thermalPowerFraction(backLeft_.getTemperatureC()),
        thermalPowerFraction(backRight_.getTemperatureC()),
    };

    // Averaged, unlike the corners: this one only ever scales the whole
    // correction down, so which of the two center wheels is hotter doesn't
    // change what the correction should be, only how much of it to ask for.
    centerThermalFraction_ = (thermalPowerFraction(middleLeft_->getTemperatureC()) +
                              thermalPowerFraction(middleRight_->getTemperatureC())) /
                             2.0;
}

void HolonomicDrivetrain::setWheelVoltages(double frontLeft, double frontRight, double backLeft,
                                            double backRight) {
    frontLeft_.moveVoltage(frontLeft);
    frontRight_.moveVoltage(frontRight);
    backLeft_.moveVoltage(backLeft);
    backRight_.moveVoltage(backRight);

    // Recover the pure throttle/strafe/turn components from the four
    // already-mixed corner voltages (see mixHolonomic()): summing all four
    // cancels strafe and turn, leaving 4x throttle; each cross-combination
    // below cancels two of the three, leaving 4x the remaining one. This
    // lets the center wheels react correctly no matter which call site
    // produced the mix — holonomic() driver input, driveDistance()'s
    // heading-corrected drive, or turnToHeading()'s turn-only mix.
    //
    // For the same reason it's what appliedAxisVolts() reports, which is
    // why it happens ahead of the Asterisk check below: every chassis gets
    // a readout, center wheels or not.
    const double throttleVolts = (frontLeft + frontRight + backLeft + backRight) / 4.0;
    const double strafeVolts = (frontLeft - frontRight - backLeft + backRight) / 4.0;
    const double turnVolts = (frontLeft - frontRight + backLeft - backRight) / 4.0;
    recordAppliedVolts(throttleVolts, strafeVolts, turnVolts);

    if (!asterisk_) return;

    // Feedforward from what the corners are failing to deliver. Computed
    // from this tick's corner voltages rather than cached, because the same
    // derating means different things depending on what's being driven —
    // see refreshThermalFractions(). Purely additive: with every corner cool
    // this is zero and the terms below are untouched.
    refreshThermalFractions();
    const CenterCorrection thermal = centerThermalCorrection(
        CornerValues{frontLeft, frontRight, backLeft, backRight}, thermalFractions_,
        centerThermalFraction_, asterisk_->thermalCompensation,
        asterisk_->maxThermalCorrectionVolts);

    double centerVolts = throttleVolts + thermal.commonVolts;

    if (driftSource_ != nullptr) {
        const std::uint32_t now = pros::millis();
        const double verticalIn = driftSource_->getDistanceIn();
        const double strafeIn = encoderStrafeIn();
        const double headingDeg = imu_.getCumulativeHeadingDeg();
        const double dtS = (now - lastDriftTickMs_) / 1000.0;

        // Only correct while strafing dominates — during forward/back
        // driving the center wheels are just adding power. Skip a zero dt
        // and unusually long gaps (e.g. a paused motion) rather than treat
        // them as a drift spike. The baseline below refreshes on every call
        // either way, so a stale gap never carries into the next strafe.
        const bool strafingDominant = std::fabs(strafeVolts) > std::fabs(throttleVolts);
        if (asterisk_->driftCorrectionKP != 0.0 && strafingDominant && dtS > 1e-3 && dtS < 1.0) {
            const double verticalOffsetIn = driftOffsetSource_ != nullptr
                                                ? driftOffsetSource_->getConfig().verticalOffsetIn
                                                : 0.0;
            const double driftIn =
                strafeDriftIn(verticalIn - lastDriftVerticalIn_, verticalOffsetIn,
                              headingDeg - lastDriftHeadingDeg_, strafeIn - lastDriftStrafeIn_,
                              throttleVolts, strafeVolts);
            centerVolts -= asterisk_->driftCorrectionKP * (driftIn / dtS);
        }

        lastDriftVerticalIn_ = verticalIn;
        lastDriftStrafeIn_ = strafeIn;
        lastDriftHeadingDeg_ = headingDeg;
        lastDriftTickMs_ = now;
    }

    // Differential turn term, on top of the common forward/back term. The
    // left side takes +turn in mixHolonomic(), and the middle ports use the
    // same sign convention as the corner ports, so middle-left matching
    // front-left's sign is what makes these push the turn rather than fight
    // it. Without this the center wheels saw a zero net command during a
    // pure turn and just coasted through it; they now carry their share.
    //
    // This also means driveDistance()'s heading correction — which reaches
    // here as a small turn component riding on top of the drive output —
    // gets the center wheels helping hold the line, not just watching.
    //
    // The thermal term rides along here for the same reason it does on the
    // throttle: a lopsided set of corner temperatures twists the chassis as
    // well as pushing it off line, and cancelling that twist is differential
    // work.
    const double centerTurnVolts =
        turnVolts * asterisk_->turnContribution + thermal.differentialVolts;

    // Clamped per side after summing, since it's the sum that has to fit in
    // a motor's range. holonomic() normalizes its mix so neither side can
    // reach the limit from driver input; this bounds the autonomous paths,
    // whose raw PID volts have always been free to run past it (the corner
    // wheels clamp the same way, inside moveVoltage()).
    const double leftVolts = std::clamp(centerVolts + centerTurnVolts, -12.0, 12.0);
    const double rightVolts = std::clamp(centerVolts - centerTurnVolts, -12.0, 12.0);
    middleLeft_->moveVoltage(leftVolts);
    middleRight_->moveVoltage(rightVolts);
}

void HolonomicDrivetrain::holonomicVolts(double forwardVolts, double strafeVolts, double turnVolts) {
    const WheelMix mix = mixHolonomic(forwardVolts, strafeVolts, turnVolts);

    // Scale the whole mix down (never up) so the largest wheel command never
    // exceeds full voltage — preserves the requested direction instead of
    // clipping one wheel and distorting it.
    const double largest =
        std::max({std::fabs(mix.frontLeft), std::fabs(mix.frontRight), std::fabs(mix.backLeft),
                  std::fabs(mix.backRight), kFullScaleVolts}) /
        kFullScaleVolts;

    setWheelVoltages(mix.frontLeft / largest, mix.frontRight / largest, mix.backLeft / largest,
                     mix.backRight / largest);
}

double HolonomicDrivetrain::stickVolts(double input, const MotorFeedforward& model) const {
    if (driverInputMode_.load() != DriverInputMode::velocity || !model.valid()) {
        return input * kFullScaleVolts;
    }

    // A resting V5 stick still reads a count or two. In voltage mode that's
    // a harmless millivolt; here it would add the whole kS and creep the
    // chassis, so centered has to mean exactly zero.
    if (std::fabs(input) < kVelocityModeDeadband) return 0.0;

    // Full stick asks for the speed full voltage reaches, so this mode never
    // costs top speed — it only reshapes the stick below that.
    const double clamped = std::clamp(input, -1.0, 1.0);
    return model.volts(clamped * model.maxVelocity(kFullScaleVolts));
}

void HolonomicDrivetrain::holonomic(double throttle, double strafe, double turn) {
    const HolonomicAxisModels models = axisModels();
    holonomicVolts(stickVolts(throttle, models.forward), stickVolts(strafe, models.strafe),
                   stickVolts(turn, models.turn));
}

double HolonomicDrivetrain::headingHoldTurnVolts(double turnInput) {
    const std::uint32_t now = pros::millis();
    // Rotation frame, like heldHeadingDeg_ — see heldHeadingDeg().
    const double currentHeadingDeg = rotationHeadingDeg();
    const double dtS = (now - lastHeadingHoldMs_) / 1000.0;
    const bool resuming =
        lastHeadingHoldMs_ == 0 || !(dtS > 0.0) || dtS > kHeadingHoldResumeGapS;
    lastHeadingHoldMs_ = now;

    if (resuming) {
        // Adopt wherever the chassis is pointing rather than steering it
        // back to a heading from before whatever just interrupted driver
        // control. The PID is reset for the same reason: its accumulated
        // integral and last error describe a situation that no longer
        // exists.
        heldHeadingDeg_ = currentHeadingDeg;
        headingHoldPID_.reset();
        return 0.0;
    }

    heldHeadingDeg_ =
        advanceHeldHeadingDeg(heldHeadingDeg_, currentHeadingDeg, turnInput, dtS, headingHold_);

    // Same target/measurement=0 trick as turnToHeading() — see that
    // function's comment. Explicit dt because a driver loop's period is the
    // caller's business and needn't match PID::Config::nominalDtS.
    return headingHoldPID_.update(wrapDegrees180(heldHeadingDeg_ - currentHeadingDeg), 0.0, dtS);
}

void HolonomicDrivetrain::holonomicHeadingHold(double throttle, double strafe, double turnInput) {
    const HolonomicAxisModels models = axisModels();
    holonomicVolts(stickVolts(throttle, models.forward), stickVolts(strafe, models.strafe),
                   headingHoldTurnVolts(turnInput));
}

void HolonomicDrivetrain::holonomicFieldCentricHeadingHold(double throttle, double strafe,
                                                            double turnInput) {
    fieldToRobot(throttle, strafe);
    holonomicHeadingHold(throttle, strafe, turnInput);
}

void HolonomicDrivetrain::setHeadingHold(HeadingHoldConfig config) { headingHold_ = config; }

double HolonomicDrivetrain::heldHeadingDeg() const {
    return sensors::fieldHeadingDeg(heldHeadingDeg_, imu_.headingOffsetDeg());
}

double HolonomicDrivetrain::rotationHeadingDeg() {
    return sensors::wrapDegrees360(imu_.getCumulativeHeadingDeg());
}

void HolonomicDrivetrain::fieldToRobot(double& throttle, double& strafe) {
    // Rotate the field-relative stick vector into the robot's current frame
    // by the heading it has picked up since the last field-heading zero —
    // matches pros::Imu::get_heading()'s clockwise-positive convention.
    const double headingDeltaRad =
        wrapDegrees180(rotationHeadingDeg() - fieldHeadingZeroDeg_) * (kPi / 180.0);
    const double cosHeading = std::cos(headingDeltaRad);
    const double sinHeading = std::sin(headingDeltaRad);
    const double robotThrottle = throttle * cosHeading + strafe * sinHeading;
    const double robotStrafe = -throttle * sinHeading + strafe * cosHeading;
    throttle = robotThrottle;
    strafe = robotStrafe;
}

void HolonomicDrivetrain::holonomicFieldCentric(double throttle, double strafe, double turn) {
    fieldToRobot(throttle, strafe);
    holonomic(throttle, strafe, turn);
}

void HolonomicDrivetrain::resetFieldHeading() { fieldHeadingZeroDeg_ = rotationHeadingDeg(); }

motion::MotionResult HolonomicDrivetrain::moveToPoint(double xIn, double yIn,
                                                      const odom::Odometry& odometry,
                                                      ExitConditions exit) {
    const double holdHeadingDeg = odometry.getPose().headingDeg;
    telemetry::event("motion",
                     "start,moveToPoint,x=%.3f,y=%.3f,hold_deg=%.3f,threshold=%.3f,settle_ms=%u,"
                     "timeout_ms=%u",
                     xIn, yIn, holdHeadingDeg, exit.errorThreshold,
                     static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));
    return finishMotion("moveToPoint",
                        moveToPointHolding(xIn, yIn, holdHeadingDeg, odometry, exit));
}

motion::MotionResult HolonomicDrivetrain::moveToPoint(double xIn, double yIn,
                                                      ExitConditions exit) {
    if (odometry_ == nullptr) return abortMotion("moveToPoint", kNoOdometry);
    return moveToPoint(xIn, yIn, *odometry_, exit);
}

motion::MotionResult HolonomicDrivetrain::moveToPointHolding(double xIn, double yIn,
                                                             double holdHeadingDeg,
                                                             const odom::Odometry& odometry,
                                                             ExitConditions exit) {
    drivePID_.reset();
    turnPID_.reset();

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorIn = 0.0;

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double dxIn = xIn - pose.xIn;
        const double dyIn = yIn - pose.yIn;
        const double distanceIn = std::hypot(dxIn, dyIn);

        const double outputVolts = drivePID_.update(distanceIn, 0.0);
        const motion::LocalOffset local = motion::toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const double scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;

        // Hold heading the same way moveToPose() does. Translation would
        // still arrive if the chassis yawed (toLocalFrame() uses the live
        // heading); this just keeps it from yawing on the way.
        const double turnOutput =
            turnPID_.update(wrapDegrees180(holdHeadingDeg - pose.headingDeg), 0.0);

        holonomicVolts(local.forwardIn * scale, local.lateralIn * scale, turnOutput);

        finalErrorIn = distanceIn;
        reason = tracker.update(distanceIn <= exit.errorThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return motion::MotionResult{
        .reason = reason, .finalError = finalErrorIn, .elapsedMs = tracker.elapsedMs()};
}

motion::MotionResult HolonomicDrivetrain::moveToPose(double xIn, double yIn, double headingDeg,
                                                     const odom::Odometry& odometry,
                                                     motion::PoseExitConditions exit) {
    telemetry::event("motion",
                     "start,moveToPose,x=%.3f,y=%.3f,heading_deg=%.3f,pos_threshold=%.3f,"
                     "heading_threshold=%.3f,settle_ms=%u,timeout_ms=%u",
                     xIn, yIn, headingDeg, exit.positionErrorThresholdIn,
                     exit.headingErrorThresholdDeg, static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));

    drivePID_.reset();
    turnPID_.reset();

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorIn = 0.0;

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double dxIn = xIn - pose.xIn;
        const double dyIn = yIn - pose.yIn;
        const double distanceIn = std::hypot(dxIn, dyIn);

        const double outputVolts = drivePID_.update(distanceIn, 0.0);
        const motion::LocalOffset local = motion::toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const double scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;

        const double headingError = wrapDegrees180(headingDeg - pose.headingDeg);
        const double turnOutput = turnPID_.update(headingError, 0.0);

        holonomicVolts(local.forwardIn * scale, local.lateralIn * scale, turnOutput);

        const bool withinThreshold = distanceIn <= exit.positionErrorThresholdIn &&
                                     std::fabs(headingError) <= exit.headingErrorThresholdDeg;
        finalErrorIn = distanceIn;
        reason = tracker.update(withinThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return finishMotion("moveToPose", motion::MotionResult{.reason = reason,
                                                           .finalError = finalErrorIn,
                                                           .elapsedMs = tracker.elapsedMs()});
}

motion::MotionResult HolonomicDrivetrain::moveToPose(double xIn, double yIn, double headingDeg,
                                                     motion::PoseExitConditions exit) {
    if (odometry_ == nullptr) return abortMotion("moveToPose", kNoOdometry);
    return moveToPose(xIn, yIn, headingDeg, *odometry_, exit);
}

motion::MotionResult HolonomicDrivetrain::followPath(const motion::Path& path,
                                                     const odom::Odometry& odometry,
                                                     motion::PursuitConfig config) {
    // Path's constructor doesn't reject an empty list, and .back() below
    // would be undefined behavior on one.
    if (path.waypoints().empty()) return abortMotion("followPath", "empty path");

    telemetry::event("motion",
                     "start,followPath,waypoints=%u,lookahead_in=%.3f,cruise_v=%.3f,timeout_ms=%u",
                     static_cast<unsigned>(path.waypoints().size()), config.lookaheadIn,
                     config.cruiseVoltage, static_cast<unsigned>(config.timeoutMs));

    std::size_t segmentIndex = 0;
    const motion::Waypoint& finalPoint = path.waypoints().back();
    const double holdHeadingDeg = odometry.getPose().headingDeg;
    turnPID_.reset();
    const std::uint32_t startMs = pros::millis();

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double distToFinalIn = std::hypot(finalPoint.xIn - pose.xIn, finalPoint.yIn - pose.yIn);
        if (distToFinalIn <= config.finalApproachIn) break;

        // Checked after the distance, so reaching the final approach on the
        // same tick the timer runs out still gets the settled stop there.
        const std::uint32_t pursuitMs = pros::millis() - startMs;
        if (config.timeoutMs > 0 && pursuitMs >= config.timeoutMs) {
            stop();
            return finishMotion("followPath",
                                motion::MotionResult{.reason = motion::ExitReason::timedOut,
                                                     .finalError = distToFinalIn,
                                                     .elapsedMs = pursuitMs});
        }

        const motion::LookaheadResult lookahead =
            motion::findLookaheadPoint(pose.xIn, pose.yIn, path, config.lookaheadIn, segmentIndex);
        segmentIndex = lookahead.segmentIndex;

        const double dxIn = lookahead.point.xIn - pose.xIn;
        const double dyIn = lookahead.point.yIn - pose.yIn;
        const double distanceIn = std::hypot(dxIn, dyIn);
        const motion::LocalOffset local = motion::toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const double scale = distanceIn > 1e-6 ? config.cruiseVoltage / distanceIn : 0.0;
        const double turnOutput =
            turnPID_.update(wrapDegrees180(holdHeadingDeg - pose.headingDeg), 0.0);

        holonomicVolts(local.forwardIn * scale, local.lateralIn * scale, turnOutput);

        pros::delay(kLoopDelayMs);
    }

    // Keep holding the path's starting heading through the final approach,
    // rather than re-capturing whatever the chassis yawed to on the way.
    motion::MotionResult result = moveToPointHolding(finalPoint.xIn, finalPoint.yIn,
                                                     holdHeadingDeg, odometry, config.finalExit);
    // The final approach decides how the path ended, but the time is the
    // whole path's.
    result.elapsedMs = pros::millis() - startMs;
    return finishMotion("followPath", result);
}

motion::MotionResult HolonomicDrivetrain::followPath(const motion::Path& path,
                                                     motion::PursuitConfig config) {
    if (odometry_ == nullptr) return abortMotion("followPath", kNoOdometry);
    return followPath(path, *odometry_, config);
}

double HolonomicDrivetrain::degreesToInches(double degrees) const {
    const double wheelCircumferenceIn = config_.wheelDiameterIn * kPi;
    return (degrees / 360.0 / config_.externalGearRatio) * wheelCircumferenceIn;
}

double HolonomicDrivetrain::cornerAverageDegrees() const {
    return (frontLeft_.getPositionDegrees() + frontRight_.getPositionDegrees() +
            backLeft_.getPositionDegrees() + backRight_.getPositionDegrees()) /
           4.0;
}

motion::MotionResult HolonomicDrivetrain::driveDistance(double inches, ExitConditions exit) {
    telemetry::event("motion",
                     "start,driveDistance,target_in=%.3f,threshold=%.3f,settle_ms=%u,timeout_ms=%u",
                     inches, exit.errorThreshold, static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));

    // Rotation frame: only the drift from here matters, and a setPose() from
    // another task mustn't read as a sudden heading error.
    const double startHeading = rotationHeadingDeg();

    // Measured from wherever the encoders already read instead of taring
    // them. A tare is device-level, so it would also zero any
    // MotorGroupTrackingWheel on these motors and jump the odometry pose
    // back by everything driven so far.
    const double startDegrees = cornerAverageDegrees();
    drivePID_.reset();

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorIn = 0.0;

    while (true) {
        const double traveledInches = degreesToInches(cornerAverageDegrees() - startDegrees);
        const double error = inches - traveledInches;

        const double output = drivePID_.update(inches, traveledInches);

        double correction = 0.0;
        if (config_.headingCorrectionKP != 0.0) {
            const double headingError = wrapDegrees180(startHeading - rotationHeadingDeg());
            correction = config_.headingCorrectionKP * headingError;
        }

        const WheelMix mix = mixHolonomic(output, /*strafe=*/0.0, correction);
        setWheelVoltages(mix.frontLeft, mix.frontRight, mix.backLeft, mix.backRight);

        finalErrorIn = std::fabs(error);
        reason = tracker.update(std::fabs(error) <= exit.errorThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return finishMotion("driveDistance", motion::MotionResult{.reason = reason,
                                                              .finalError = finalErrorIn,
                                                              .elapsedMs = tracker.elapsedMs()});
}

motion::MotionResult HolonomicDrivetrain::turnToHeading(double headingDeg, ExitConditions exit) {
    telemetry::event("motion",
                     "start,turnToHeading,target_deg=%.3f,threshold=%.3f,settle_ms=%u,"
                     "timeout_ms=%u",
                     headingDeg, exit.errorThreshold, static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));

    turnPID_.reset();

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorDeg = 0.0;

    while (true) {
        const double error = wrapDegrees180(headingDeg - imu_.getHeadingDeg());

        // Same target/measurement=0 trick as TankDrivetrain::turnToHeading —
        // see that function's comment for why.
        const double output = turnPID_.update(error, 0.0);

        const WheelMix mix = mixHolonomic(/*throttle=*/0.0, /*strafe=*/0.0, output);
        setWheelVoltages(mix.frontLeft, mix.frontRight, mix.backLeft, mix.backRight);

        finalErrorDeg = std::fabs(error);
        reason = tracker.update(std::fabs(error) <= exit.errorThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return finishMotion("turnToHeading", motion::MotionResult{.reason = reason,
                                                              .finalError = finalErrorDeg,
                                                              .elapsedMs = tracker.elapsedMs()});
}

double HolonomicDrivetrain::headingDeg() { return imu_.getHeadingDeg(); }

void HolonomicDrivetrain::stop(BrakeMode mode) {
    frontLeft_.setBrakeMode(mode);
    frontRight_.setBrakeMode(mode);
    backLeft_.setBrakeMode(mode);
    backRight_.setBrakeMode(mode);
    frontLeft_.brake();
    frontRight_.brake();
    backLeft_.brake();
    backRight_.brake();

    // Braking commands no voltage, so appliedAxisVolts() shouldn't keep
    // reporting the last tick of whatever motion just ended.
    recordAppliedVolts(0.0, 0.0, 0.0);

    // Center wheels keep their own permanent coast brake mode (set at
    // construction) regardless of `mode` — see AsteriskConfig.
    if (asterisk_) {
        middleLeft_->brake();
        middleRight_->brake();
    }
}

} // namespace sapphirelib::chassis
