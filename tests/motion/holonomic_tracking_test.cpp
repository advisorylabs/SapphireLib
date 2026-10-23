// Host-side unit test for the trajectory follower's math
// (sapphirelib::motion::trackingErrors(), advanceTrajectoryClock(),
// rotateAxisMotion(), TrajectoryFollower), no PROS/embedded dependencies, so
// it builds and runs with a normal desktop compiler.
//
// The closed-loop cases drive a simulated chassis (each axis kS/kV/kA with
// static friction and a command delay, the wheel mix scaled like
// HolonomicDrivetrain::holonomicVolts()) with gains designed by
// tuning::designPositionGains(), the way Auto-Tune would.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/holonomic_tracking_test.cpp \
//       src/sapphirelib/motion/holonomic_tracking.cpp \
//       src/sapphirelib/motion/holonomic_feedforward.cpp \
//       src/sapphirelib/motion/trajectory_generator.cpp src/sapphirelib/motion/trajectory.cpp \
//       src/sapphirelib/motion/trajectory_path.cpp src/sapphirelib/motion/pure_pursuit_math.cpp \
//       src/sapphirelib/motion/path.cpp src/sapphirelib/motion/exit_tracker.cpp \
//       src/sapphirelib/control/pid.cpp src/sapphirelib/control/feedforward.cpp \
//       src/sapphirelib/tuning/gain_design.cpp src/sapphirelib/util/angle.cpp \
//       -o holonomic_tracking_test && ./holonomic_tracking_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <vector>

#include "sapphirelib/motion/holonomic_feedforward.hpp"
#include "sapphirelib/motion/holonomic_tracking.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/motion/trajectory_generator.hpp"
#include "sapphirelib/tuning/gain_design.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::PID;
using sapphirelib::chassis::AxisVolts;
using sapphirelib::chassis::HolonomicAxisModels;
using sapphirelib::motion::advanceTrajectoryClock;
using sapphirelib::motion::AxisMotion;
using sapphirelib::motion::ChainEvent;
using sapphirelib::motion::ClockStep;
using sapphirelib::motion::ExitReason;
using sapphirelib::motion::generateTrajectory;
using sapphirelib::motion::LocalOffset;
using sapphirelib::motion::MotionResult;
using sapphirelib::motion::rotateAxisMotion;
using sapphirelib::motion::toLocalFrame;
using sapphirelib::motion::TrackingErrors;
using sapphirelib::motion::trackingErrors;
using sapphirelib::motion::TrackingTick;
using sapphirelib::motion::Trajectory;
using sapphirelib::motion::TrajectoryFollowConfig;
using sapphirelib::motion::TrajectoryFollower;
using sapphirelib::motion::TrajectorySample;
using sapphirelib::motion::TrajectorySection;
using sapphirelib::odom::Pose;
using sapphirelib::odom::Velocity;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// the simulator's default chassis (tools/sim/js/world.js)
const HolonomicAxisModels kModels{
    .forward = {.kS = 1.0, .kV = 0.2, .kA = 0.045},
    .strafe = {.kS = 1.3, .kV = 0.24, .kA = 0.055},
    .turn = {.kS = 0.6, .kV = 0.045, .kA = 0.0065},
};

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.9f, expected %.9f (+-%g)\n", label, actual, expected,
                    tolerance);
        assert(false);
    }
}

Trajectory plan(const std::vector<TrajectorySection>& sections) {
    const auto result = generateTrajectory(sections, kModels);
    if (!result.ok) {
        std::printf("FAIL plan: %s\n", result.error.c_str());
        assert(false);
    }
    return result.trajectory;
}

// the golden chain from trajectory_generator_test.cpp: curves, a heading change, a slower section
// and a stop
const std::vector<TrajectorySection> kChain{
    {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},
                   {.xIn = -40, .yIn = -30},
                   {.xIn = -20, .yIn = -10, .headingDeg = 45}}},
    {.waypoints = {{.xIn = 10, .yIn = 0}},
     .constraints = {.voltageBudget = 7.0},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -10, .yIn = 20, .headingDeg = 0}}},
};

void testErrorsAreInTheRobotFrame() {
    // the reference 3in east and 4in north of the robot, moving east at 10in/s
    TrajectorySample reference;
    reference.xIn = 3.0;
    reference.yIn = 4.0;
    reference.headingDeg = 30.0;
    reference.vxInPerS = 10.0;
    reference.omegaDegPerS = 20.0;
    // east, seen from a robot facing 30 (toLocalFrame(1, 0, 30))
    reference.forwardPerIn = std::sin(30.0 * kDegToRad);
    reference.strafePerIn = std::cos(30.0 * kDegToRad);
    for (double heading : {0.0, 37.0, 90.0, 180.0, 270.0, 359.0}) {
        const Pose pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = heading};
        const Velocity velocity{.xInPerS = 2.0, .yInPerS = -1.0, .headingDegPerS = 5.0};
        const TrackingErrors e = trackingErrors(reference, pose, velocity);
        const LocalOffset position = toLocalFrame(3.0, 4.0, heading);
        const LocalOffset rate = toLocalFrame(8.0, 1.0, heading);
        expectNear(e.forwardIn, position.forwardIn, 1e-12, "forward error");
        expectNear(e.strafeIn, position.lateralIn, 1e-12, "strafe error");
        expectNear(e.forwardRateInPerS, rate.forwardIn, 1e-12, "forward rate error");
        expectNear(e.strafeRateInPerS, rate.lateralIn, 1e-12, "strafe rate error");
        expectNear(e.headingRateDegPerS, 15.0, 1e-12, "turn rate error");
        // the path runs east: the robot is 3in behind it and 4in to its right (south)
        expectNear(e.alongTrackIn, 3.0, 1e-12, "along track");
        expectNear(e.crossTrackIn, 4.0, 1e-12, "cross track");
    }
    // heading error wraps the short way
    reference.headingDeg = 370.0;
    expectNear(trackingErrors(reference, {.headingDeg = 350.0}, {}).headingDeg, 20.0, 1e-9,
               "heading error wraps");
}

void testRotateAxisMotionMatchesToLocalFrame() {
    // one field vector seen from the planned heading, rotated to the actual one, is what the
    // actual heading sees directly
    for (double planned : {0.0, 45.0, 170.0, 300.0}) {
        for (double offset : {-30.0, -5.0, 0.0, 12.0, 90.0}) {
            const LocalOffset atPlanned = toLocalFrame(7.0, -3.0, planned);
            const LocalOffset atActual = toLocalFrame(7.0, -3.0, planned + offset);
            const AxisMotion rotated =
                rotateAxisMotion({atPlanned.forwardIn, atPlanned.lateralIn, 11.0}, offset);
            expectNear(rotated.forward, atActual.forwardIn, 1e-12, "rotated forward");
            expectNear(rotated.strafe, atActual.lateralIn, 1e-12, "rotated strafe");
            expectNear(rotated.turn, 11.0, 0.0, "turn unchanged");
        }
    }
}

void testAxisAccelerationsMatchTheirVelocities() {
    // straight ahead while spinning half a turn: each robot-frame acceleration is the time
    // derivative of its robot-frame velocity, the spin's share included
    const Trajectory t = plan({{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0},
                                              {.xIn = 0, .yIn = 80, .headingDeg = 180}}}});
    const std::vector<TrajectorySample>& s = t.samples();
    double worst = 0.0;
    double scale = 0.0;
    for (std::size_t k = 2; k + 2 < s.size(); ++k) {
        const double dt = s[k + 1].tS - s[k - 1].tS;
        const double numeric[3] = {(s[k + 1].forwardInPerS - s[k - 1].forwardInPerS) / dt,
                                   (s[k + 1].strafeInPerS - s[k - 1].strafeInPerS) / dt,
                                   (s[k + 1].omegaDegPerS - s[k - 1].omegaDegPerS) / dt};
        // the planned value across the same two steps
        const double planned[3] = {0.5 * (s[k - 1].forwardAccelInPerS2 + s[k].forwardAccelInPerS2),
                                   0.5 * (s[k - 1].strafeAccelInPerS2 + s[k].strafeAccelInPerS2),
                                   0.5 * (s[k - 1].turnAccelDegPerS2 + s[k].turnAccelDegPerS2)};
        for (int i = 0; i < 2; ++i) {
            worst = std::fmax(worst, std::fabs(numeric[i] - planned[i]));
            scale = std::fmax(scale, std::fabs(planned[i]));
        }
    }
    // within a few percent of the largest acceleration: the samples are only 0.5in apart, and the
    // stored accelerations are each step's
    if (worst > 0.05 * scale) {
        std::printf("FAIL axis accelerations off by %.3f in/s^2 (largest %.3f)\n", worst, scale);
        assert(false);
    }
    // and the strafe axis really is used: spinning while driving straight needs it
    double strafe = 0.0;
    for (const TrajectorySample& sample : s)
        strafe = std::fmax(strafe, std::fabs(sample.strafeInPerS));
    assert(strafe > 5.0);
}

void testClock() {
    ClockStep step = advanceTrajectoryClock(1.0, 0.01, 2.0, 4.0);
    expectNear(step.tS, 1.01, 1e-12, "runs while close");
    assert(!step.paused);
    step = advanceTrajectoryClock(1.0, 0.01, 4.5, 4.0);
    expectNear(step.tS, 1.0, 0.0, "holds while far behind");
    assert(step.paused);
    step = advanceTrajectoryClock(1.0, 0.01, -9.0, 4.0);
    assert(!step.paused); // ahead of the plan is never a reason to wait
    step = advanceTrajectoryClock(1.0, 0.01, 100.0, 0.0);
    assert(!step.paused && step.tS > 1.0); // 0 never holds
    step = advanceTrajectoryClock(1.0, -0.5, 0.0, 4.0);
    expectNear(step.tS, 1.0, 0.0, "a bad dt doesn't run it backward");
}

// --- Closed loop ------------------------------------------------------------------------------

double signOf(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }

// a holonomic chassis: three robot-frame axes, each kA dv/dt = V - friction - kV v, with static
// friction holding it until the volts beat kS, commands arriving delayTicks late
struct Chassis {
    Pose pose;
    double v[3] = {0.0, 0.0, 0.0};
    std::deque<AxisVolts> pending;
    int delayTicks = 2;
    bool blocked = false;

    Velocity fieldVelocity() const {
        const double h = pose.headingDeg * kDegToRad;
        return Velocity{.xInPerS = v[0] * std::sin(h) + v[1] * std::cos(h),
                        .yInPerS = v[0] * std::cos(h) - v[1] * std::sin(h),
                        .headingDegPerS = v[2]};
    }

    void step(AxisVolts command, double dtS) {
        // the wheel mix, scaled so no corner passes 12V (holonomicVolts())
        const double corners[4] = {command.forward + command.strafe + command.turn,
                                   command.forward - command.strafe - command.turn,
                                   command.forward - command.strafe + command.turn,
                                   command.forward + command.strafe - command.turn};
        double largest = 12.0;
        for (double c : corners) largest = std::fmax(largest, std::fabs(c));
        const double scale = 12.0 / largest;
        pending.push_back({command.forward * scale, command.strafe * scale, command.turn * scale});
        AxisVolts applied{};
        if (static_cast<int>(pending.size()) > delayTicks) {
            applied = pending.front();
            pending.pop_front();
        }
        const double volts[3] = {applied.forward, applied.strafe, applied.turn};
        const MotorFeedforward* models[3] = {&kModels.forward, &kModels.strafe, &kModels.turn};
        const int substeps = 20;
        const double h = dtS / substeps;
        for (int k = 0; k < substeps; ++k) {
            for (int i = 0; i < 3; ++i) {
                const MotorFeedforward& m = *models[i];
                const double friction =
                    std::fabs(v[i]) > 1e-3
                        ? m.kS * signOf(v[i])
                        : signOf(volts[i]) * std::fmin(std::fabs(volts[i]), m.kS);
                v[i] += (volts[i] - friction - m.kV * v[i]) / m.kA * h;
                if (blocked) v[i] = 0.0;
            }
            const Velocity field = fieldVelocity();
            pose.xIn += field.xInPerS * h;
            pose.yIn += field.yInPerS * h;
            pose.headingDeg += field.headingDegPerS * h;
        }
    }
};

struct Run {
    MotionResult result;
    double worstErrorIn = 0.0;
    double worstHeadingDeg = 0.0;
    int pausedTicks = 0;
    std::size_t sectionsSeen = 0;
    // a chain: what happened at its junctions, and the robot's speed through them
    int continued = 0;
    ChainEvent refused = ChainEvent::none;
    std::size_t lastPiece = 0;
    bool clockRanBackward = false;
    // the plan's clock and the robot's speed, every tick
    std::vector<std::pair<double, double>> speeds;

    double slowestBetween(double fromS, double toS) const {
        double slowest = 1e9;
        for (const auto& [t, v] : speeds) {
            if (t >= fromS && t <= toS) slowest = std::fmin(slowest, v);
        }
        return slowest;
    }
};

Run follow(const Trajectory& trajectory, double blockFromS = -1.0, double blockToS = -1.0,
           TrajectoryFollowConfig config = {}) {
    // gains the way Auto-Tune designs them, with the chassis's delay in the design
    const sapphirelib::tuning::ResponseSpec spec{.settleTimeS = 0.4};
    PID forward(
        {.gains = sapphirelib::tuning::designPositionGains(kModels.forward, spec, 0.02).gains,
         .outputLimit = 12.0});
    PID strafe({.gains = sapphirelib::tuning::designPositionGains(kModels.strafe, spec, 0.02).gains,
                .outputLimit = 12.0});
    PID turn({.gains = sapphirelib::tuning::designPositionGains(kModels.turn, spec, 0.02).gains,
              .outputLimit = 12.0});
    TrajectoryFollower follower(forward, strafe, turn);

    Chassis chassis;
    const TrajectorySample& start = trajectory.samples().front();
    chassis.pose = {.xIn = start.xIn, .yIn = start.yIn, .headingDeg = start.headingDeg};

    Run run;
    std::uint32_t now = 1000;
    follower.begin(trajectory, kModels, config, now);
    double lastReferenceS = 0.0;
    while (true) {
        const double elapsedS = (now - 1000) / 1000.0;
        chassis.blocked = elapsedS >= blockFromS && elapsedS < blockToS;
        const TrackingTick tick = follower.update(chassis.pose, chassis.fieldVelocity(), now);
        if (!tick.holding && !chassis.blocked) {
            run.worstErrorIn = std::fmax(run.worstErrorIn,
                                         std::hypot(tick.errors.forwardIn, tick.errors.strafeIn));
            run.worstHeadingDeg = std::fmax(run.worstHeadingDeg, std::fabs(tick.errors.headingDeg));
        }
        if (tick.paused) ++run.pausedTicks;
        run.sectionsSeen = std::max(run.sectionsSeen, tick.section + 1);
        assert(follower.progress().section == tick.section);
        assert(follower.progress().piece == tick.piece);
        if (tick.chain == ChainEvent::continued) ++run.continued;
        if (tick.chain == ChainEvent::noNext || tick.chain == ChainEvent::mismatched) {
            run.refused = tick.chain;
        }
        run.lastPiece = tick.piece;
        if (tick.referenceTimeS < lastReferenceS) run.clockRanBackward = true;
        lastReferenceS = tick.referenceTimeS;
        const Velocity v = chassis.fieldVelocity();
        run.speeds.push_back({tick.referenceTimeS, std::hypot(v.xInPerS, v.yInPerS)});
        if (follower.reason() != ExitReason::running) break;
        chassis.step(tick.volts, 0.01);
        now += 10;
    }
    run.result = follower.result();
    assert(!follower.progress().running);
    return run;
}

void testFollowsTheGoldenChain() {
    const Trajectory trajectory = plan(kChain);
    const Run run = follow(trajectory);
    std::printf("follow: %s in %ums (plan %.0fms), worst %.3fin %.2fdeg, final %.3fin\n",
                sapphirelib::motion::exitReasonName(run.result.reason), run.result.elapsedMs,
                trajectory.durationS() * 1000.0, run.worstErrorIn, run.worstHeadingDeg,
                run.result.finalError);
    assert(run.result.reason == ExitReason::settled);
    assert(run.worstErrorIn < 1.0);
    assert(run.worstHeadingDeg < 3.0);
    assert(run.result.finalError <= 1.0);
    assert(run.result.elapsedMs < trajectory.durationS() * 1000.0 + 500.0);
    assert(run.pausedTicks == 0);
    assert(run.sectionsSeen == 3);
}

void testABlockedRobotPausesThePlan() {
    const Trajectory trajectory = plan(kChain);
    // held still for 0.8s, a second into the run
    const Run run = follow(trajectory, 1.0, 1.8);
    std::printf("blocked: %s in %ums, paused %d ticks, final %.3fin\n",
                sapphirelib::motion::exitReasonName(run.result.reason), run.result.elapsedMs,
                run.pausedTicks, run.result.finalError);
    assert(run.pausedTicks > 30);
    assert(run.result.reason == ExitReason::settled);
    assert(run.result.elapsedMs > trajectory.durationS() * 1000.0 + 500.0);
    // the plan waited, so once free the robot never chased a reference far ahead of it
    assert(run.worstErrorIn < 6.0);

    // held still for longer than the overtime: it gives up instead of waiting forever
    const Run stuck = follow(trajectory, 1.0, 100.0);
    assert(stuck.result.reason == ExitReason::timedOut);
    expectNear(stuck.result.elapsedMs, std::ceil(trajectory.durationS() * 1000.0) + 1500.0, 10.0,
               "timeout at the plan plus the overtime");
}

// a branch at speed: 60in ahead ending at 30in/s, then one of two branches bearing off
const std::vector<TrajectorySection> kTrunk{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 60}}}};

Trajectory plan(const std::vector<TrajectorySection>& sections,
                const sapphirelib::motion::TrajectoryOptions& options) {
    const auto result = generateTrajectory(sections, kModels, options);
    if (!result.ok) {
        std::printf("FAIL plan: %s\n", result.error.c_str());
        assert(false);
    }
    return result.trajectory;
}

void testCarriesOnIntoTheChosenBranch() {
    const Trajectory trunk = plan(kTrunk, {.endSpeedInPerS = 30.0});
    const Trajectory left = plan({{.waypoints = {{.xIn = -24, .yIn = 96, .headingDeg = -30}}},
                                  {.waypoints = {{.xIn = -40, .yIn = 110}}}},
                                 {.from = &trunk});
    const Trajectory right =
        plan({{.waypoints = {{.xIn = 24, .yIn = 96, .headingDeg = 30}}}}, {.from = &trunk});

    for (const bool goLeft : {true, false}) {
        int asked = 0;
        const Trajectory* chosen = goLeft ? &left : &right;
        TrajectoryFollowConfig config;
        config.next = [&](const Trajectory& ending) -> const Trajectory* {
            ++asked;
            assert(&ending == &trunk); // only the trunk ends moving
            return chosen;
        };
        const Run run = follow(trunk, -1.0, -1.0, config);
        const double plannedS = trunk.durationS() + chosen->durationS();
        std::printf("branch %s: %s in %ums (plan %.0fms), worst %.3fin %.2fdeg, final %.3fin, "
                    "slowest at the junction %.1fin/s\n",
                    goLeft ? "left" : "right",
                    sapphirelib::motion::exitReasonName(run.result.reason), run.result.elapsedMs,
                    plannedS * 1000.0, run.worstErrorIn, run.worstHeadingDeg,
                    run.result.finalError,
                    run.slowestBetween(trunk.durationS() - 0.2, trunk.durationS() + 0.2));
        assert(asked == 1 && run.continued == 1);
        assert(run.refused == ChainEvent::none);
        assert(run.lastPiece == 1);
        assert(run.result.reason == ExitReason::settled);
        assert(run.worstErrorIn < 1.0);
        assert(run.worstHeadingDeg < 3.0);
        assert(run.result.finalError <= 1.0);
        assert(!run.clockRanBackward);
        // the motion ends at the branch's end, not the trunk's
        assert(run.result.elapsedMs > plannedS * 1000.0);
        assert(run.result.elapsedMs < plannedS * 1000.0 + 500.0);
        // the robot drives through the junction instead of stopping at it
        assert(run.slowestBetween(trunk.durationS() - 0.2, trunk.durationS() + 0.2) > 20.0);
        // the sections count on: the trunk's one, then the branch's
        assert(run.sectionsSeen == 1 + chosen->sectionCount());
    }
}

void testAJunctionWithNothingNextSettles() {
    const Trajectory trunk = plan(kTrunk, {.endSpeedInPerS = 30.0});
    // no next at all: the robot brakes from 30in/s onto the last point on feedback alone
    Run run = follow(trunk);
    assert(run.refused == ChainEvent::noNext && run.continued == 0);
    assert(run.result.reason == ExitReason::settled && run.result.finalError <= 1.0);

    // a next that doesn't start at the junction is refused the same way
    const Trajectory elsewhere = plan(
        {{.waypoints = {{.xIn = 10, .yIn = 0, .headingDeg = 0}, {.xIn = 10, .yIn = 30}}}}, {});
    TrajectoryFollowConfig config;
    config.next = [&](const Trajectory&) { return &elsewhere; };
    run = follow(trunk, -1.0, -1.0, config);
    assert(run.refused == ChainEvent::mismatched && run.continued == 0 && run.lastPiece == 0);
    assert(run.result.reason == ExitReason::settled && run.result.finalError <= 1.0);

    // a trajectory that stops at its end never asks
    int asked = 0;
    config.next = [&](const Trajectory&) -> const Trajectory* {
        ++asked;
        return nullptr;
    };
    run = follow(plan(kTrunk, {}), -1.0, -1.0, config);
    assert(asked == 0 && run.refused == ChainEvent::none);
}

} // namespace

int main() {
    testErrorsAreInTheRobotFrame();
    testRotateAxisMotionMatchesToLocalFrame();
    testAxisAccelerationsMatchTheirVelocities();
    testClock();
    testFollowsTheGoldenChain();
    testABlockedRobotPausesThePlan();
    testCarriesOnIntoTheChosenBranch();
    testAJunctionWithNothingNextSettles();
    std::puts("holonomic_tracking_test: all assertions passed");
    return 0;
}
