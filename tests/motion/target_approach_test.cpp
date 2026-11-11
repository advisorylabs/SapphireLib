// Host-side unit test for sapphirelib::motion's target approach (target_approach.cpp): the camera
// math, the approach's speed, and TargetApproach driving a plant of the robot's axis models. No
// PROS/embedded dependencies, so it builds and runs with a normal desktop compiler. The simulator
// runs the same cases on its port (tools/sim/test/vision.test.js).
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/target_approach_test.cpp \
//       src/sapphirelib/motion/target_approach.cpp src/sapphirelib/motion/exit_tracker.cpp \
//       src/sapphirelib/control/pid.cpp src/sapphirelib/control/feedforward.cpp \
//       src/sapphirelib/util/angle.cpp -o target_approach_test && ./target_approach_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/target_approach.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::PID;
using sapphirelib::motion::ApproachConfig;
using sapphirelib::motion::ApproachTick;
using sapphirelib::motion::CameraMount;
using sapphirelib::motion::ExitReason;
using sapphirelib::motion::FieldPoint;
using sapphirelib::motion::TargetApproach;
using sapphirelib::odom::Pose;
using sapphirelib::odom::Velocity;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %f, expected %f (+-%f)\n", label, actual, expected, tolerance);
        assert(false);
    }
}

void testCameraMath() {
    const CameraMount camera;
    // 320 across, 74 degrees: the middle is straight ahead, the edges 37 either side
    expectNear(sapphirelib::motion::cameraBearingDeg(camera, 160), 0, 1e-9, "middle");
    expectNear(sapphirelib::motion::cameraBearingDeg(camera, 320), 37, 1e-9, "right edge");
    expectNear(sapphirelib::motion::cameraBearingDeg(camera, 0), -37, 1e-9, "left edge");
    expectNear(sapphirelib::motion::cameraFocalPx(camera), 160.0 / std::tan(37.0 * kPi / 180.0),
               1e-9, "focal length");

    const Pose robot{.xIn = 0, .yIn = 0, .headingDeg = 90}; // facing +x
    expectNear(*sapphirelib::motion::cameraBearingToDeg(camera, robot, {.xIn = 10, .yIn = 0}), 0,
               1e-9, "straight ahead");
    expectNear(*sapphirelib::motion::cameraBearingToDeg(camera, robot, {.xIn = 10, .yIn = -10}), 45,
               1e-9, "right of it (-y, facing +x)");
    assert(!sapphirelib::motion::cameraBearingToDeg(camera, robot, {.xIn = -5, .yIn = 0}));
    expectNear(*sapphirelib::motion::cameraDepthIn(camera, robot, {.xIn = 10, .yIn = -10}), 10,
               1e-9, "depth along the axis");

    // seen 10 degrees right, a target 20in out keeps its 20in and swings onto that line
    const FieldPoint moved =
        sapphirelib::motion::onCameraRay(camera, robot, 10, {.xIn = 20, .yIn = 0});
    expectNear(std::hypot(moved.xIn, moved.yIn), 20, 1e-9, "range kept");
    expectNear(*sapphirelib::motion::cameraBearingToDeg(camera, robot, moved), 10, 1e-9,
               "on the line");

    // a camera 6in ahead and 2in right, turned 5 degrees right: the lens is the ray's origin
    const CameraMount offset{.forwardIn = 6, .rightIn = 2, .yawDeg = 5};
    const Pose facingY{.xIn = 0, .yIn = 0, .headingDeg = 0};
    const FieldPoint onRay =
        sapphirelib::motion::onCameraRay(offset, facingY, -5, {.xIn = 2, .yIn = 30});
    expectNear(onRay.xIn, 2, 1e-9, "axis 5 right, seen 5 left: straight ahead of the lens, x");
    expectNear(onRay.yIn, 30, 1e-9, "and y");
    expectNear(*sapphirelib::motion::cameraBearingToDeg(offset, facingY, {.xIn = 2, .yIn = 30}), -5,
               1e-9, "and back");
}

void testApproachSpeed() {
    expectNear(sapphirelib::motion::approachSpeedInPerS(20, 18, 80), 18, 1e-12, "far off");
    expectNear(sapphirelib::motion::approachSpeedInPerS(1, 18, 80), std::sqrt(160.0), 1e-12,
               "1in off");
    expectNear(sapphirelib::motion::approachSpeedInPerS(0, 18, 80), 0, 1e-12, "there");
    expectNear(sapphirelib::motion::approachSpeedInPerS(-1, 18, 80), 0, 1e-12, "past");
    expectNear(sapphirelib::motion::approachSpeedInPerS(0.1, 18, 0), 18, 1e-12,
               "no decel: cruise to the end");
}

// The robot's axis models and tracking gains (devices.cpp), and a plant of them: forward and turn
// on their own models, each wheel capped at 12V
const MotorFeedforward kForward{.kS = 0.30, .kV = 0.1698, .kA = 0.0383};
const MotorFeedforward kTurn{.kS = 1.10, .kV = 0.0318, .kA = 0.0019};

double signOf(double x) { return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0); }

double axisStep(const MotorFeedforward& model, double volts, double v, double dtS) {
    if (v == 0.0 && std::fabs(volts) <= model.kS) return 0.0;
    const double friction = model.kS * signOf(v != 0.0 ? v : volts);
    const double next = v + ((volts - friction - model.kV * v) / model.kA) * dtS;
    // friction stops it rather than reversing it
    return v != 0.0 && signOf(next) != signOf(v) && std::fabs(volts) <= model.kS ? 0.0 : next;
}

PID::Config gains(double kP, double kD) {
    return PID::Config{.gains = {.kP = kP, .kI = 0.0, .kD = kD}, .outputLimit = 12.0};
}

struct Sample {
    std::uint32_t ms;
    double speedInPerS;
    Pose pose;
    ApproachTick tick;
};

void testDrivesOntoATargetToTheSideAtCruise() {
    // the Triple Pin's: 18in/s through a Pin that seats 5.65in ahead, to 3in past that
    const ApproachConfig config{.cruiseInPerS = 18,
                                .accelInPerS2 = 60,
                                .decelInPerS2 = 80,
                                .endShortIn = 2.65,
                                .steerMinIn = 7.65,
                                .timeoutMs = 3000};
    const FieldPoint target{.xIn = 4, .yIn = 36};
    PID forward(gains(2.086, 0.396));
    PID turn(gains(0.405, 0.024));
    TargetApproach approach(forward, turn);

    double x = 0, y = 0, headingDeg = 0, v = 24, w = 0;
    approach.begin(config, v, 0);
    std::vector<Sample> samples;
    for (std::uint32_t ms = 0; ms <= 5000; ms += 10) {
        const double h = headingDeg * kPi / 180.0;
        const Pose pose{.xIn = x, .yIn = y, .headingDeg = headingDeg};
        const ApproachTick tick = approach.update(
            pose, Velocity{.xInPerS = v * std::sin(h), .yInPerS = v * std::cos(h), .headingDegPerS = w},
            target, kForward, kTurn, ms);
        samples.push_back(Sample{ms, v, pose, tick});
        if (approach.reason() != ExitReason::running) break;
        const double largest =
            std::max(std::fabs(tick.forwardVolts) + std::fabs(tick.turnVolts), 12.0) / 12.0;
        for (int i = 0; i < 10; ++i) {
            v = axisStep(kForward, tick.forwardVolts / largest, v, 0.001);
            w = axisStep(kTurn, tick.turnVolts / largest, w, 0.001);
            const double a = headingDeg * kPi / 180.0;
            headingDeg += w * 0.001;
            x += v * std::sin(a) * 0.001;
            y += v * std::cos(a) * 0.001;
        }
    }
    assert(approach.reason() == ExitReason::settled);

    // the reference eases down from 24 to cruise, never faster than it started
    bool reachedCruise = false;
    for (const Sample& s : samples) {
        assert(s.tick.speedRefInPerS <= 24 + 1e-9);
        if (std::fabs(s.tick.speedRefInPerS - 18) < 1e-9) reachedCruise = true;
    }
    assert(reachedCruise);

    // where the target would seat (5.65in ahead), the robot is at cruise, and facing it: the
    // target within a quarter inch of its centerline
    const auto seat = std::find_if(samples.begin(), samples.end(),
                                   [](const Sample& s) { return s.tick.distanceIn <= 5.65; });
    assert(seat != samples.end());
    expectNear(seat->speedInPerS, 18, 1.0, "speed as the target seats");
    const double a = seat->pose.headingDeg * kPi / 180.0;
    const double across =
        (target.xIn - seat->pose.xIn) * std::cos(a) - (target.yIn - seat->pose.yIn) * std::sin(a);
    expectNear(across, 0, 0.25, "target off the centerline as it seats");

    // it ends at its end, slowing for it: braking from there is the drivetrain's stop()
    expectNear(std::hypot(target.xIn - x, target.yIn - y), 2.65, 0.75, "where it ends");
    assert(v < 12);

    // the turn's static friction: no steady heading error while it re-aims
    for (const Sample& s : samples) {
        if (s.tick.steering && s.ms > 400) expectNear(s.tick.headingErrorDeg, 0, 1.5, "re-aiming");
    }
}

void testAMovingTargetSteersWithoutAKick() {
    const ApproachConfig config{.cruiseInPerS = 18,
                                .accelInPerS2 = 60,
                                .decelInPerS2 = 80,
                                .endShortIn = 2.65,
                                .steerMinIn = 7.65};
    PID forward(gains(2.086, 0.396));
    PID turn(gains(0.405, 0.024));
    TargetApproach approach(forward, turn);
    approach.begin(config, 18, 0);
    const Pose pose{.xIn = 0, .yIn = 0, .headingDeg = 0};
    const Velocity velocity{.xInPerS = 0, .yInPerS = 18, .headingDegPerS = 0};
    approach.update(pose, velocity, {.xIn = 0, .yIn = 30}, kForward, kTurn, 0);
    const ApproachTick steady =
        approach.update(pose, velocity, {.xIn = 0, .yIn = 30}, kForward, kTurn, 10);
    expectNear(steady.turnVolts, 0, 1e-9, "on it: no turn");
    // 2in right, 30in out: kS and P on the error, D on the robot's own turn rate (still 0), not
    // the target's jump
    const ApproachTick moved =
        approach.update(pose, velocity, {.xIn = 2, .yIn = 30}, kForward, kTurn, 20);
    const double errorDeg = std::atan2(2.0, 30.0) * 180.0 / kPi;
    expectNear(moved.headingErrorDeg, errorDeg, 1e-9, "heading error");
    expectNear(moved.turnVolts, 1.10 + 0.405 * errorDeg, 1e-9, "kS and P, no D kick");
}

void testTimesOut() {
    PID forward(gains(2.086, 0.396));
    PID turn(gains(0.405, 0.024));
    TargetApproach approach(forward, turn);
    approach.begin(ApproachConfig{.timeoutMs = 100}, 0, 1000);
    // held still, 20in off: never there
    for (std::uint32_t ms = 1000; ms <= 1200 && approach.reason() == ExitReason::running; ms += 10) {
        approach.update(Pose{}, Velocity{}, {.xIn = 0, .yIn = 20}, kForward, kTurn, ms);
    }
    assert(approach.reason() == ExitReason::timedOut);
    expectNear(approach.result().finalError, 20, 1e-9, "distance to the end");
}

} // namespace

int main() {
    testCameraMath();
    testApproachSpeed();
    testDrivesOntoATargetToTheSideAtCruise();
    testAMovingTargetSteersWithoutAKick();
    testTimesOut();
    std::printf("target_approach_test: all passed\n");
    return 0;
}
