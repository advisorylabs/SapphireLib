// Host-side unit test for sapphirelib::motion::steerToward() and boomerangStep(), the steering
// TankDrivetrain::moveToPose() drives with, no PROS/embedded dependencies, so it builds and runs
// with a normal desktop compiler.
//
// The closed-loop case drives a simulated tank (forward and turn axes, each kS/kV/kA with static
// friction and a command delay, each side clamped to 12V) to random poses with moveToPose()'s loop
// and Auto-Tune-designed gains, with and without settling, and checks the exit settles.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/boomerang_math_test.cpp src/sapphirelib/motion/boomerang_math.cpp src/sapphirelib/motion/exit_tracker.cpp src/sapphirelib/control/pid.cpp src/sapphirelib/control/feedforward.cpp src/sapphirelib/tuning/gain_design.cpp src/sapphirelib/util/angle.cpp -o boomerang_math_test && ./boomerang_math_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <random>

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/boomerang_math.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/tuning/gain_design.hpp"
#include "sapphirelib/util/angle.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::PID;
using sapphirelib::wrapDegrees180;
using sapphirelib::motion::BoomerangStep;
using sapphirelib::motion::boomerangStep;
using sapphirelib::motion::ExitReason;
using sapphirelib::motion::ExitTracker;
using sapphirelib::motion::SteeringError;
using sapphirelib::motion::steerToward;
using sapphirelib::odom::Pose;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// a tank chassis's two axes, as Auto-Tune would measure them: inches and degrees
const MotorFeedforward kForward{.kS = 1.0, .kV = 0.2, .kA = 0.045};
const MotorFeedforward kTurn{.kS = 0.6, .kV = 0.045, .kA = 0.0065};

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.6f, expected %.6f (+-%g)\n", label, actual, expected,
                    tolerance);
        assert(false);
    }
}

void testSteerToward() {
    SteeringError s = steerToward(30.0, 0.0);
    expectNear(s.headingErrorDeg, 30.0, 1e-12, "ahead: turn");
    assert(s.direction == 1.0);
    s = steerToward(170.0, 0.0);
    expectNear(s.headingErrorDeg, -10.0, 1e-12, "behind: back up, turning the short way");
    assert(s.direction == -1.0);
    s = steerToward(-100.0, 350.0);
    expectNear(s.headingErrorDeg, -90.0, 1e-12, "across 0, exactly 90 is still forward");
    assert(s.direction == 1.0);
    s = steerToward(-101.0, 350.0);
    expectNear(s.headingErrorDeg, 89.0, 1e-12, "past 90: back up");
    assert(s.direction == -1.0);
}

// the controller before settling, written out as TankDrivetrain::moveToPose() had it
BoomerangStep oldStep(const Pose& pose, double x, double y, double headingDeg, double lead) {
    BoomerangStep step;
    step.distanceIn = std::hypot(x - pose.xIn, y - pose.yIn);
    const double carrotOffsetIn = step.distanceIn * lead;
    const double carrotX = x - carrotOffsetIn * std::sin(headingDeg * kDegToRad);
    const double carrotY = y - carrotOffsetIn * std::cos(headingDeg * kDegToRad);
    const double bearing = std::atan2(carrotX - pose.xIn, carrotY - pose.yIn) / kPi * 180.0;
    const SteeringError steer = steerToward(bearing, pose.headingDeg);
    step.direction = steer.direction;
    step.driveErrorIn = step.distanceIn;
    step.turnErrorDeg = steer.headingErrorDeg;
    return step;
}

void testItPicksADirectionAndKeepsIt() {
    // a target straight ahead, to finish facing the same way: forward, carrot on the line behind
    // the target, so no turn
    BoomerangStep step = boomerangStep(Pose{}, 0.0, 30.0, 0.0, 0.6, 3.0, 0.5, {});
    assert(step.started && !step.settling && step.direction == 1.0);
    expectNear(step.turnErrorDeg, 0.0, 1e-12, "ahead: no turn");
    expectNear(step.driveScale, 1.0, 1e-12, "ahead: all the drive");
    expectNear(step.driveErrorIn, 30.0, 1e-12, "the distance");

    // the same target behind the robot, facing the way the robot does: it backs in, the carrot
    // in front of the target, and ends facing 0
    step = boomerangStep(Pose{}, 0.0, -30.0, 0.0, 0.6, 3.0, 0.5, {});
    assert(step.direction == -1.0);
    expectNear(step.turnErrorDeg, 0.0, 1e-12, "backing straight in: no turn");

    // off to the side, finishing facing +x: the carrot is 18in left of the target, so it turns
    // toward that
    step = boomerangStep(Pose{}, 30.0, 0.0, 90.0, 0.6, 3.0, 0.5, {});
    assert(step.direction == 1.0);
    expectNear(step.turnErrorDeg, 90.0, 1e-9, "the carrot is at (12, 0)");
    expectNear(step.driveScale, 0.0, 1e-12, "90 degrees off: turn first");

    // once picked it stays, even where the other would now point it better
    BoomerangStep previous = step;
    previous.direction = -1.0;
    step = boomerangStep(Pose{}, 0.0, 30.0, 0.0, 0.6, 3.0, 0.5, previous);
    assert(step.direction == -1.0);
    expectNear(std::fabs(step.turnErrorDeg), 180.0, 1e-9, "backing toward it means turning round");
    expectNear(step.driveScale, 0.0, 1e-12, "and not driving until it has");
}

void testSettlingGetsOnThePointThenTurns() {
    // 2in short of the target, straight ahead, facing 30 degrees, to finish facing 45: inside the
    // 3in settle radius it heads for the point itself, not the carrot
    BoomerangStep previous;
    previous.started = true;
    previous.direction = 1.0;
    const Pose pose{.xIn = 0.0, .yIn = -2.0, .headingDeg = 30.0};
    BoomerangStep step = boomerangStep(pose, 0.0, 0.0, 45.0, 0.6, 3.0, 0.5, previous);
    assert(step.settling && !step.turning);
    expectNear(step.turnErrorDeg, -30.0, 1e-9, "points at the target");
    expectNear(step.driveErrorIn, 2.0 * std::cos(30.0 * kDegToRad), 1e-12,
               "closes what's left along the way it faces");
    expectNear(step.driveScale, std::cos(30.0 * kDegToRad), 1e-9, "slowed by the turn");

    // on the point (within the turn radius): turns in place to the final heading
    step = boomerangStep(Pose{.xIn = 0.1, .yIn = -0.3, .headingDeg = 30.0}, 0.0, 0.0, 45.0, 0.6, 3.0,
                         0.5, step);
    assert(step.settling && step.turning);
    expectNear(step.turnErrorDeg, 15.0, 1e-12, "turns to the final heading");
    expectNear(step.driveScale, 1.0, 1e-12, "holding where it is along the way it faces");
    // latched out to twice the turn radius
    assert(boomerangStep(Pose{.xIn = 0.0, .yIn = -0.9}, 0.0, 0.0, 45.0, 0.6, 3.0, 0.5, step).turning);
    assert(!boomerangStep(Pose{.xIn = 0.0, .yIn = -1.1}, 0.0, 0.0, 45.0, 0.6, 3.0, 0.5, step).turning);

    // arrived backing in: the drive error keeps the sign it had, and the back points at it
    previous.direction = -1.0;
    const BoomerangStep backing =
        boomerangStep(Pose{.xIn = 0.0, .yIn = 2.0, .headingDeg = 0.0}, 0.0, 0.0, 0.0, 0.6, 3.0,
                      0.5, previous);
    assert(backing.settling && !backing.turning && backing.direction == -1.0);
    expectNear(backing.driveErrorIn, 2.0, 1e-12, "backing: the distance, positive");
    expectNear(backing.turnErrorDeg, 0.0, 1e-9, "its back already points at the target");

    // settling stays out to twice the settle radius, then it steers at the carrot again
    assert(boomerangStep(Pose{.xIn = 0.0, .yIn = -5.0}, 0.0, 0.0, 0.0, 0.6, 3.0, 0.5, step)
               .settling);
    assert(!boomerangStep(Pose{.xIn = 0.0, .yIn = -7.0}, 0.0, 0.0, 0.0, 0.6, 3.0, 0.5, step)
                .settling);
    assert(!boomerangStep(Pose{.xIn = 0.0, .yIn = -5.0}, 0.0, 0.0, 0.0, 0.6, 3.0, 0.5, previous)
                .settling);
    // 0 never settles
    assert(!boomerangStep(pose, 0.0, 0.0, 0.0, 0.6, 0.0, 0.5, previous).settling);
}

double signOf(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }

// a tank: forward and turn axes, commands arriving two ticks late, each side clamped to 12V
struct Tank {
    Pose pose;
    double v[2] = {0.0, 0.0};
    std::deque<std::pair<double, double>> pending;
    // odometry's reading of the pose: white noise on top, harsher on the bearing to a point an
    // inch away than odometry's slow drift
    double noiseIn = 0.0;
    double noiseDeg = 0.0;
    std::mt19937 rng{7};

    Pose measured() {
        std::normal_distribution<double> unit(0.0, 1.0);
        return Pose{.xIn = pose.xIn + noiseIn * unit(rng),
                    .yIn = pose.yIn + noiseIn * unit(rng),
                    .headingDeg = pose.headingDeg + noiseDeg * unit(rng)};
    }

    void step(double leftVolts, double rightVolts, double dtS) {
        leftVolts = std::clamp(leftVolts, -12.0, 12.0);
        rightVolts = std::clamp(rightVolts, -12.0, 12.0);
        pending.push_back({(leftVolts + rightVolts) / 2.0, (leftVolts - rightVolts) / 2.0});
        double volts[2] = {0.0, 0.0};
        if (pending.size() > 2) {
            volts[0] = pending.front().first;
            volts[1] = pending.front().second;
            pending.pop_front();
        }
        const MotorFeedforward* models[2] = {&kForward, &kTurn};
        const int substeps = 20;
        const double h = dtS / substeps;
        for (int k = 0; k < substeps; ++k) {
            for (int i = 0; i < 2; ++i) {
                const MotorFeedforward& m = *models[i];
                const double friction =
                    std::fabs(v[i]) > 1e-3
                        ? m.kS * signOf(v[i])
                        : signOf(volts[i]) * std::fmin(std::fabs(volts[i]), m.kS);
                v[i] += (volts[i] - friction - m.kV * v[i]) / m.kA * h;
            }
            const double heading = pose.headingDeg * kDegToRad;
            pose.xIn += v[0] * std::sin(heading) * h;
            pose.yIn += v[0] * std::cos(heading) * h;
            pose.headingDeg += v[1] * h;
        }
    }
};

struct Result {
    ExitReason reason = ExitReason::running;
    double positionErrorIn = 0.0;
    double headingErrorDeg = 0.0;
    std::uint32_t elapsedMs = 0;
};

// TankDrivetrain::moveToPose()'s loop on the simulated tank: settleRadiusIn < 0 runs the
// controller before boomerangStep() (oldStep(), its direction picked afresh every tick)
Result moveToPose(Pose start, double x, double y, double headingDeg, double settleRadiusIn,
                  double turnRadiusIn = 0.5, double noiseIn = 0.0) {
    const sapphirelib::tuning::ResponseSpec spec{.settleTimeS = 0.4};
    PID drive({.gains = sapphirelib::tuning::designPositionGains(kForward, spec, 0.02).gains,
               .outputLimit = 12.0});
    PID turn({.gains = sapphirelib::tuning::designPositionGains(kTurn, spec, 0.02).gains,
              .outputLimit = 12.0});
    Tank tank;
    tank.pose = start;
    tank.noiseIn = noiseIn;
    tank.noiseDeg = 2.0 * noiseIn;
    std::uint32_t now = 1000;
    ExitTracker tracker(200, 4000, now);
    BoomerangStep step;
    Result result;
    while (true) {
        const Pose pose = tank.measured();
        const BoomerangStep next =
            settleRadiusIn < 0.0
                ? oldStep(pose, x, y, headingDeg, 0.6)
                : boomerangStep(pose, x, y, headingDeg, 0.6, settleRadiusIn, turnRadiusIn, step);
        // as TankDrivetrain::moveToPose() does: the turn error jumps to something new
        if (next.settling != step.settling || next.turning != step.turning) turn.reset();
        step = next;
        const double forward =
            step.direction * step.driveScale * drive.update(step.driveErrorIn, 0.0);
        const double turnOut = turn.update(step.turnErrorDeg, 0.0);
        result.positionErrorIn = step.distanceIn;
        result.headingErrorDeg = std::fabs(wrapDegrees180(headingDeg - pose.headingDeg));
        result.reason = tracker.update(
            result.positionErrorIn <= 1.0 && result.headingErrorDeg <= 2.0, now);
        if (result.reason != ExitReason::running) break;
        tank.step(forward + turnOut, forward - turnOut, 0.01);
        now += 10;
    }
    result.elapsedMs = tracker.elapsedMs();
    return result;
}

struct Tally {
    int settled = 0;
    double worstPositionIn = 0.0;
    double worstHeadingDeg = 0.0;
    double meanMs = 0.0;
};

// moveToPose() to random poses from the origin: 18 to 48in away in any direction, to finish
// facing within 90 degrees of the way there (what a boomerang can curve into), from any heading
Tally tally(double settleRadiusIn, int cases, bool print = false, double turnRadiusIn = 0.5,
            double noiseIn = 0.0) {
    std::mt19937 rng(96671);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Tally t;
    for (int i = 0; i < cases; ++i) {
        const double bearing = 360.0 * unit(rng);
        const double distance = 18.0 + 30.0 * unit(rng);
        const double x = distance * std::sin(bearing * kDegToRad);
        const double y = distance * std::cos(bearing * kDegToRad);
        const double finalHeading = bearing + 180.0 * (unit(rng) - 0.5);
        const Pose start{.headingDeg = 360.0 * unit(rng)};
        const Result r =
            moveToPose(start, x, y, finalHeading, settleRadiusIn, turnRadiusIn, noiseIn);
        if (r.reason == ExitReason::settled) {
            ++t.settled;
            t.meanMs += r.elapsedMs;
        } else if (print) {
            std::printf("  unsettled: to (%.1f, %.1f) facing %.0f from %.0f: %.2fin %.2fdeg off\n",
                        x, y, finalHeading, start.headingDeg, r.positionErrorIn,
                        r.headingErrorDeg);
        }
        t.worstPositionIn = std::fmax(t.worstPositionIn, r.positionErrorIn);
        t.worstHeadingDeg = std::fmax(t.worstHeadingDeg, r.headingErrorDeg);
    }
    if (t.settled > 0) t.meanMs /= t.settled;
    return t;
}

void testItSettlesOnTheFinalHeading() {
    // TankDrivetrain::moveToPose()'s defaults: settle within 3in, turn in place within half the
    // 1in position threshold. The controller before it picked its direction every tick and never
    // turned to the final heading: on these poses it mostly backed in facing the wrong way, or
    // wandered on the bearing to a point a fraction of an inch away
    const int kCases = 300;
    const Tally before = tally(-1.0, kCases);
    const Tally after = tally(3.0, kCases, true);
    std::printf("moveToPose: settled %d/%d (before: %d/%d), %.0fms on average, worst left %.2fin "
                "%.2fdeg\n",
                after.settled, kCases, before.settled, kCases, after.meanMs,
                after.worstPositionIn, after.worstHeadingDeg);
    assert(after.settled >= kCases * 99 / 100);
    assert(before.settled < kCases / 3);
    assert(after.worstPositionIn < 1.0 && after.worstHeadingDeg < 2.0);
    // odometry's noise makes the old bearing worse still; the new one doesn't care
    for (const double noise : {0.03, 0.1}) {
        const Tally noisy = tally(3.0, kCases, true, 0.5, noise);
        const Tally old = tally(-1.0, kCases, false, 0.5, noise);
        std::printf("  with %.2fin and %.2fdeg of noise: %d/%d (before: %d/%d)\n", noise,
                    2.0 * noise, noisy.settled, kCases, old.settled, kCases);
        assert(noisy.settled >= kCases * 98 / 100);
    }
    // never settling, it reaches the point but not the heading
    assert(tally(0.0, kCases).settled < kCases / 2);
}

} // namespace

int main() {
    testSteerToward();
    testItPicksADirectionAndKeepsIt();
    testSettlingGetsOnThePointThenTurns();
    testItSettlesOnTheFinalHeading();
    std::puts("boomerang_math_test: all assertions passed");
    return 0;
}
