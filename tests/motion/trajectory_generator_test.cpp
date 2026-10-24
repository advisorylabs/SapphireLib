// Host-side unit test for sapphirelib::motion::generateTrajectory() and
// Trajectory, no PROS/embedded dependencies, so it builds and runs with a
// normal desktop compiler.
//
// The golden case at the bottom is shared with tools/sim/test/trajectory.test.js,
// which holds the simulator's port to the same numbers.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/trajectory_generator_test.cpp \
//       src/sapphirelib/motion/trajectory_generator.cpp src/sapphirelib/motion/trajectory.cpp \
//       src/sapphirelib/motion/trajectory_path.cpp \
//       src/sapphirelib/motion/holonomic_feedforward.cpp \
//       src/sapphirelib/control/feedforward.cpp src/sapphirelib/util/angle.cpp \
//       -o trajectory_generator_test && ./trajectory_generator_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "sapphirelib/motion/holonomic_feedforward.hpp"
#include "sapphirelib/motion/trajectory_generator.hpp"

using sapphirelib::chassis::AxisVolts;
using sapphirelib::chassis::HolonomicAxisModels;
using sapphirelib::motion::generateTrajectory;
using sapphirelib::motion::Trajectory;
using sapphirelib::motion::TrajectoryConstraints;
using sapphirelib::motion::trajectoryFeedforward;
using sapphirelib::motion::TrajectoryResult;
using sapphirelib::motion::TrajectorySample;
using sapphirelib::motion::TrajectorySection;
using sapphirelib::motion::TrajectoryWaypoint;
using sapphirelib::motion::voltageAccelRange;

namespace {

// the simulator's default chassis (tools/sim/js/world.js)
const HolonomicAxisModels kModels{
    .forward = {.kS = 1.0, .kV = 0.2, .kA = 0.045},
    .strafe = {.kS = 1.3, .kV = 0.24, .kA = 0.055},
    .turn = {.kS = 0.6, .kV = 0.045, .kA = 0.0065},
};

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.12g, expected %.12g (+-%g)\n", label, actual, expected,
                    tolerance);
        assert(false);
    }
}

double sumOfSizes(const double p[3], const double q[3], double a) {
    return std::fabs(p[0] + q[0] * a) + std::fabs(p[1] + q[1] * a) + std::fabs(p[2] + q[2] * a);
}

// the worst wheel's volts at a sample: what the budget limits
double wheelVolts(const TrajectorySample& s) {
    const AxisVolts v = trajectoryFeedforward(kModels, s);
    return std::fabs(v.forward) + std::fabs(v.strafe) + std::fabs(v.turn);
}

Trajectory plan(const std::vector<TrajectorySection>& sections) {
    const TrajectoryResult result = generateTrajectory(sections, kModels);
    if (!result.ok) {
        std::printf("FAIL plan: %s\n", result.error.c_str());
        assert(false);
    }
    return result.trajectory;
}

void testVoltageAccelRange() {
    // one axis: |1 + 0.05a| <= 10 for a in [-220, 180]
    double lo = 0.0;
    double hi = 0.0;
    const double p1[3] = {1.0, 0.0, 0.0};
    const double q1[3] = {0.05, 0.0, 0.0};
    assert(voltageAccelRange(p1, q1, 10.0, lo, hi));
    expectNear(lo, -220.0, 1e-9, "one axis: lo");
    expectNear(hi, 180.0, 1e-9, "one axis: hi");

    // no acceleration term: the whole range, or nothing
    const double pFlat[3] = {3.0, -2.0, 1.0};
    const double qFlat[3] = {0.0, 0.0, 0.0};
    assert(voltageAccelRange(pFlat, qFlat, 6.0, lo, hi) && std::isinf(lo) && std::isinf(hi));
    assert(!voltageAccelRange(pFlat, qFlat, 5.9, lo, hi));

    // random cases: the range's ends sit exactly on the budget, inside is within it, outside not
    std::mt19937 rng(9667);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    int feasible = 0;
    for (int run = 0; run < 5000; ++run) {
        double p[3];
        double q[3];
        for (int i = 0; i < 3; ++i) {
            p[i] = 6.0 * unit(rng);
            q[i] = run % 7 == i ? 0.0 : 0.1 * unit(rng);
        }
        const double budget = 4.0 + 8.0 * (unit(rng) + 1.0) / 2.0;
        const bool ok = voltageAccelRange(p, q, budget, lo, hi);
        // brute force: is any a on a fine grid within budget?
        double bestF = 1e18;
        for (double a = -1e4; a <= 1e4; a += 0.5) bestF = std::fmin(bestF, sumOfSizes(p, q, a));
        if (!ok) {
            assert(bestF > budget - 1e-6);
            continue;
        }
        ++feasible;
        assert(lo <= hi);
        expectNear(sumOfSizes(p, q, lo), budget, 1e-9 * budget, "lo on the budget");
        expectNear(sumOfSizes(p, q, hi), budget, 1e-9 * budget, "hi on the budget");
        assert(sumOfSizes(p, q, 0.5 * (lo + hi)) <= budget + 1e-9);
        assert(sumOfSizes(p, q, lo - 1e-3) > budget);
        assert(sumOfSizes(p, q, hi + 1e-3) > budget);
    }
    assert(feasible > 1000);
}

void testStraightForwardMatchesTheClosedForm() {
    // 120in straight ahead, facing the way it drives: only the forward axis
    const Trajectory t =
        plan({{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 120}}}});
    const double budget = 10.0;
    const auto& fwd = kModels.forward;
    const double top = (budget - fwd.kS) / fwd.kV; // 45 in/s
    double fastest = 0.0;
    for (const TrajectorySample& s : t.samples()) {
        fastest = std::fmax(fastest, s.speedInPerS);
        expectNear(s.strafeInPerS, 0.0, 1e-9, "no strafe");
        expectNear(s.omegaDegPerS, 0.0, 0.0, "no turn");
        assert(wheelVolts(s) <= budget + 1e-6);
    }
    assert(fastest <= top + 1e-9 && fastest > 0.99 * top);
    assert(t.samples().front().speedInPerS == 0.0 && t.samples().back().speedInPerS == 0.0);

    // speeding up: v dv/ds = (budget - kS - kV v) / kA has the exact solution
    // s(v) = -(kA/kV) v - kA (budget - kS) / kV^2 * ln(1 - kV v / (budget - kS))
    for (const TrajectorySample& s : t.samples()) {
        if (s.sIn > 40.0 || s.sIn < 1.0) continue;
        const double v = s.speedInPerS;
        const double exact =
            -(fwd.kA / fwd.kV) * v - fwd.kA * (budget - fwd.kS) / (fwd.kV * fwd.kV) *
                                         std::log(1.0 - fwd.kV * v / (budget - fwd.kS));
        expectNear(exact, s.sIn, 0.02 * s.sIn + 0.3, "speeding up matches the exact solution");
    }
    // braking can use the friction too: harder than speeding up, so it takes less distance
    std::size_t topAt = 0;
    std::size_t brakeAt = 0;
    for (std::size_t k = 0; k < t.samples().size(); ++k) {
        if (topAt == 0 && t.samples()[k].speedInPerS > 0.95 * top) topAt = k;
        if (t.samples()[k].speedInPerS > 0.95 * top) brakeAt = k;
    }
    const double speedingUpIn = t.samples()[topAt].sIn;
    const double slowingIn = t.lengthIn() - t.samples()[brakeAt].sIn;
    assert(slowingIn < speedingUpIn);

    // the duration against the true minimum time: full budget forward until braking at full
    // budget (friction helping) just stops at the end, integrated finely in time
    const double brakeVolts = budget + fwd.kS;
    const auto brakingIn = [&](double v) {
        return (fwd.kA / fwd.kV) * v -
               fwd.kA * brakeVolts / (fwd.kV * fwd.kV) * std::log(1.0 + fwd.kV * v / brakeVolts);
    };
    double s = 0.0;
    double v = 0.0;
    double time = 0.0;
    const double dt = 1e-5;
    bool braking = false;
    while (!braking || v > 0.0) {
        braking = braking || s + brakingIn(v) >= 120.0;
        const double a =
            braking ? -(brakeVolts + fwd.kV * v) / fwd.kA : (budget - fwd.kS - fwd.kV * v) / fwd.kA;
        v += a * dt;
        s += v * dt;
        time += dt;
    }
    expectNear(s, 120.0, 0.01, "the exact run's length");
    expectNear(t.durationS(), time, 0.01 * time, "duration against the minimum time");
}

void testStrafingIsSlowerByTheStrafeModel() {
    // the same 120in, but facing 90 degrees (+x), so the robot strafes left the whole way
    const Trajectory forward =
        plan({{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 120}}}});
    const Trajectory sideways =
        plan({{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 90}, {.xIn = 0, .yIn = 120}}}});
    const double top = (10.0 - kModels.strafe.kS) / kModels.strafe.kV; // 36.25 in/s
    double fastest = 0.0;
    for (const TrajectorySample& s : sideways.samples()) {
        fastest = std::fmax(fastest, -s.strafeInPerS);
        assert(s.strafeInPerS <= 1e-9); // left, the whole way
        expectNear(s.forwardInPerS, 0.0, 1e-9, "no forward");
        assert(wheelVolts(s) <= 10.0 + 1e-6);
    }
    assert(fastest <= top + 1e-9 && fastest > 0.99 * top);
    assert(sideways.durationS() > 1.15 * forward.durationS());
}

void testEverySampleStaysWithinItsConstraints() {
    // an S-bend that turns 180 degrees on the way, with every cap set
    const TrajectoryConstraints limits{.voltageBudget = 9.0,
                                       .forward = {.maxVelocity = 40.0, .maxAcceleration = 120.0},
                                       .strafe = {.maxVelocity = 25.0, .maxAcceleration = 80.0},
                                       .turn = {.maxVelocity = 200.0, .maxAcceleration = 600.0}};
    const Trajectory t = plan({{.waypoints = {{.xIn = -48, .yIn = -48, .headingDeg = 0},
                                              {.xIn = -20, .yIn = -10},
                                              {.xIn = 10, .yIn = 0, .headingDeg = 90},
                                              {.xIn = 30, .yIn = 30},
                                              {.xIn = 40, .yIn = 60, .headingDeg = 180}},
                                .constraints = limits}});
    double worst = 0.0;
    for (const TrajectorySample& s : t.samples()) {
        worst = std::fmax(worst, wheelVolts(s));
        assert(std::fabs(s.forwardInPerS) <= 40.0 + 1e-6);
        assert(std::fabs(s.strafeInPerS) <= 25.0 + 1e-6);
        assert(std::fabs(s.omegaDegPerS) <= 200.0 + 1e-6);
        // accelerations are per step, so allow the step-to-step change in the turning term
        assert(std::fabs(s.forwardAccelInPerS2) <= 120.0 * 1.05 + 1.0);
        assert(std::fabs(s.strafeAccelInPerS2) <= 80.0 * 1.05 + 1.0);
        assert(std::fabs(s.turnAccelDegPerS2) <= 600.0 * 1.05 + 5.0);
    }
    // the budget, to within what a 0.5in step's change in the turning terms adds
    if (worst > 9.0 + 0.15) {
        std::printf("FAIL worst wheel %.3fV over the 9V budget\n", worst);
        assert(false);
    }
    // field velocity is the path direction at the planned speed
    for (const TrajectorySample& s : t.samples()) {
        expectNear(std::hypot(s.vxInPerS, s.vyInPerS), s.speedInPerS, 1e-9, "field speed");
        expectNear(std::hypot(s.forwardInPerS, s.strafeInPerS), s.speedInPerS, 1e-9,
                   "robot-frame speed");
    }
    // times increase, and each step's time is its distance over its average speed
    for (std::size_t k = 1; k < t.samples().size(); ++k) {
        const TrajectorySample& a = t.samples()[k - 1];
        const TrajectorySample& b = t.samples()[k];
        assert(b.tS > a.tS);
        expectNear(b.tS - a.tS, 2.0 * (b.sIn - a.sIn) / (a.speedInPerS + b.speedInPerS), 1e-12,
                   "step time");
    }
}

void testCornersAreSlower() {
    // a tight 90 degree bend in the middle of 96in of straights
    const Trajectory t = plan({{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0},
                                              {.xIn = 0, .yIn = 44},
                                              {.xIn = 4, .yIn = 48},
                                              {.xIn = 48, .yIn = 48}}}});
    double straight = 0.0;
    double bend = 1e9;
    for (const TrajectorySample& s : t.samples()) {
        if (s.sIn > 20.0 && s.sIn < 30.0) straight = std::fmax(straight, s.speedInPerS);
        if (std::hypot(s.xIn - 2.0, s.yIn - 46.0) < 3.0) bend = std::fmin(bend, s.speedInPerS);
    }
    assert(bend < 0.8 * straight);
}

void testSectionsChangeLimitsAndStop() {
    // fast for 60in, then capped at 20in/s for 40in, then a stop, then 30in more
    const std::vector<TrajectorySection> chain{
        {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 60}}},
        {.waypoints = {{.xIn = 0, .yIn = 100}},
         .constraints = {.forward = {.maxVelocity = 20.0}},
         .stopAtEnd = true},
        {.waypoints = {{.xIn = 30, .yIn = 100}}},
    };
    const Trajectory t = plan(chain);
    assert(t.sectionCount() == 3);
    expectNear(t.sectionStart(1).sIn, 60.0, 1e-6, "section 1 starts at 60in");
    expectNear(t.sectionStart(2).sIn, 100.0, 1e-6, "section 2 starts at 100in");
    assert(t.sectionStart(0).tS == 0.0);
    assert(t.sectionStart(1).tS > 0.0 && t.sectionStart(2).tS > t.sectionStart(1).tS);
    assert(t.sectionStart(5).tS == t.durationS()); // past the last: the end

    bool sawStop = false;
    double fastestFirst = 0.0;
    for (const TrajectorySample& s : t.samples()) {
        assert(s.section == t.sectionAt(s.sIn));
        if (s.sIn < 50.0) fastestFirst = std::fmax(fastestFirst, s.speedInPerS);
        // the slower section's cap holds from its first sample on: the plan brakes for it ahead
        if (s.sIn > 60.0 + 1e-9 && s.sIn < 100.0) assert(s.speedInPerS <= 20.0 + 1e-9);
        if (std::fabs(s.sIn - 100.0) < 1e-6) {
            assert(s.stop && s.speedInPerS == 0.0);
            sawStop = true;
        }
    }
    assert(sawStop && fastestFirst > 30.0);
    // braking for section 1 starts before it does
    const TrajectorySample before = t.sample(t.sectionStart(1).tS - 0.05);
    assert(before.speedInPerS > 20.0 && before.accelInPerS2 < 0.0);
    // a plain junction is driven through at speed
    expectNear(t.sample(t.sectionStart(1).tS).speedInPerS, 20.0, 1.0, "through the junction");
    assert(t.sectionAt(59.0) == 0 && t.sectionAt(60.0) == 1 && t.sectionAt(1000.0) == 2);
}

void testSampleInterpolates() {
    const Trajectory t = plan({{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0},
                                              {.xIn = 0, .yIn = 30, .headingDeg = 45}}}});
    const TrajectorySample& a = t.samples()[10];
    const TrajectorySample& b = t.samples()[11];
    const TrajectorySample mid = t.sample(0.25 * a.tS + 0.75 * b.tS);
    expectNear(mid.sIn, 0.25 * a.sIn + 0.75 * b.sIn, 1e-9, "lerp s");
    expectNear(mid.headingDeg, 0.25 * a.headingDeg + 0.75 * b.headingDeg, 1e-9, "lerp heading");
    expectNear(mid.speedInPerS, 0.25 * a.speedInPerS + 0.75 * b.speedInPerS, 1e-9, "lerp speed");
    assert(!mid.stop);
    // clamped at both ends
    assert(t.sample(-1.0).tS == 0.0 && t.sample(-1.0).stop);
    assert(t.sample(t.durationS() + 5.0).sIn == t.lengthIn());
    // empty
    const Trajectory none;
    assert(none.empty() && none.durationS() == 0.0 && none.sectionCount() == 0);
    assert(none.sample(1.0).sIn == 0.0);
}

void testRejections() {
    HolonomicAxisModels unmeasured = kModels;
    unmeasured.strafe = {};
    TrajectoryResult result = generateTrajectory(
        {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 10}}, {}, unmeasured);
    assert(!result.ok && result.trajectory.empty());
    assert(result.error ==
           "the Strafe model isn't measured (kV and kA must be above 0): run Auto-Tune");

    result = generateTrajectory({{.xIn = 0, .yIn = 0}, {.xIn = 0, .yIn = 10}}, {}, kModels);
    assert(!result.ok &&
           result.error == "the first waypoint needs a heading (where the robot starts)");

    // less budget than friction: the robot can't get going
    result = generateTrajectory({{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 10}},
                                {.voltageBudget = 0.5}, kModels);
    assert(!result.ok);
    assert(result.error ==
           "the voltage budget can't start the robot moving at 0.0in along the path");

    // a second section with less budget than friction, entered at speed: it can't be driven
    result = generateTrajectory(
        std::vector<TrajectorySection>{
            {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 30}}},
            {.waypoints = {{.xIn = 0, .yIn = 60}}, .constraints = {.voltageBudget = 0.5}}},
        kModels);
    assert(!result.ok);
    assert(result.error.rfind("the voltage budget ", 0) == 0);
}

Trajectory plan(const std::vector<TrajectorySection>& sections,
                const sapphirelib::motion::TrajectoryOptions& options) {
    const TrajectoryResult result = generateTrajectory(sections, kModels, options);
    if (!result.ok) {
        std::printf("FAIL plan: %s\n", result.error.c_str());
        assert(false);
    }
    return result.trajectory;
}

void testEndingMovingAndCarryingOn() {
    // a trunk 60in straight ahead that ends at 30in/s instead of stopping
    const std::vector<TrajectorySection> kTrunk{
        {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 60}}}};
    const Trajectory trunk = plan(kTrunk, {.endSpeedInPerS = 30.0});
    const TrajectorySample& end = trunk.samples().back();
    assert(!end.stop);
    expectNear(end.speedInPerS, 30.0, 1e-9, "the trunk ends at the speed asked for");
    expectNear(end.yIn, 60.0, 1e-9, "the trunk ends at its last point");
    // the same trunk stopping takes longer: it brakes to 0 instead of 30
    assert(plan(kTrunk).durationS() > trunk.durationS());

    // asking for more than the path allows just ends at the top speed
    const Trajectory flatOut = plan(kTrunk, {.endSpeedInPerS = 1000.0});
    const double top = (10.0 - kModels.forward.kS) / kModels.forward.kV;
    assert(flatOut.samples().back().speedInPerS <= top + 1e-9);
    assert(flatOut.samples().back().speedInPerS > 0.9 * top);

    // a branch bearing left from the trunk's end: it starts where the trunk ends, at its speed,
    // facing its heading and moving its way, and lists only the points after the junction
    const Trajectory left = plan({{.waypoints = {{.xIn = -24, .yIn = 96, .headingDeg = -30}}}},
                                 {.from = &trunk});
    const TrajectorySample& start = left.samples().front();
    assert(!start.stop);
    expectNear(start.tS, 0.0, 0.0, "the branch's own clock starts at 0");
    expectNear(start.xIn, end.xIn, 1e-9, "junction x");
    expectNear(start.yIn, end.yIn, 1e-9, "junction y");
    expectNear(start.headingDeg, end.headingDeg, 1e-9, "junction heading");
    expectNear(start.speedInPerS, end.speedInPerS, 1e-9, "junction speed");
    expectNear(start.vxInPerS, end.vxInPerS, 1e-6, "junction field velocity x");
    expectNear(start.vyInPerS, end.vyInPerS, 1e-6, "junction field velocity y");
    expectNear(start.omegaDegPerS, 0.0, 1e-9, "no turn yet at the junction");
    expectNear(start.forwardPerIn, 1.0, 1e-6, "still driving straight ahead");
    assert(left.samples().back().stop && left.samples().back().speedInPerS == 0.0);
    expectNear(left.samples().back().headingDeg, -30.0, 1e-9, "the branch turns as planned");
    for (const TrajectorySample& s : left.samples()) assert(wheelVolts(s) <= 10.0 + 0.15);
    // the first steps carry the speed on smoothly: no jump either side of the junction
    assert(std::fabs(left.samples()[1].speedInPerS - start.speedInPerS) < 2.0);

    // a branch from a trajectory that stops starts at rest, free to leave any way: here backward
    const Trajectory stopped = plan(kTrunk);
    const Trajectory back =
        plan({{.waypoints = {{.xIn = 0, .yIn = 20}}}}, {.from = &stopped});
    assert(back.samples().front().stop && back.samples().front().speedInPerS == 0.0);
    expectNear(back.samples().front().yIn, 60.0, 1e-9, "starts where the stop was");
    expectNear(back.lengthIn(), 40.0, 1e-6, "straight back 40in");
    assert(back.samples()[back.samples().size() / 2].forwardInPerS < -1.0); // reversing

    // a branch that can't take the junction at that speed: a hairpin right after it
    const TrajectoryResult hairpin = generateTrajectory(
        {{.waypoints = {{.xIn = 4, .yIn = 61}, {.xIn = 8, .yIn = 56}}}}, kModels, {.from = &trunk});
    assert(!hairpin.ok);
    assert(hairpin.error.rfind("can't ", 0) == 0);
    assert(hairpin.error.find("endSpeedInPerS") != std::string::npos);
    // ...but it can at a walking pace
    const Trajectory slow = plan(kTrunk, {.endSpeedInPerS = 3.0});
    assert(generateTrajectory({{.waypoints = {{.xIn = 4, .yIn = 64}, {.xIn = 8, .yIn = 66}}}},
                              kModels, {.from = &slow})
               .ok);

    // nothing to continue from
    const Trajectory none;
    const TrajectoryResult empty =
        generateTrajectory({{.waypoints = {{.xIn = 0, .yIn = 10}}}}, kModels, {.from = &none});
    assert(!empty.ok && empty.error == "the trajectory to continue from is empty");
}

// --- The golden case (keep in step with tools/sim/test/trajectory.test.js) --------------------

void testGolden() {
    const std::vector<TrajectorySection> chain{
        {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},
                       {.xIn = -40, .yIn = -30},
                       {.xIn = -20, .yIn = -10, .headingDeg = 45}}},
        {.waypoints = {{.xIn = 10, .yIn = 0}},
         .constraints = {.voltageBudget = 7.0},
         .stopAtEnd = true},
        {.waypoints = {{.xIn = -10, .yIn = 20, .headingDeg = 0}}},
    };
    const Trajectory t = plan(chain);
    const std::vector<TrajectorySample>& s = t.samples();
    // printed with %.12g; libm differences between C++ and JavaScript stay far inside 1e-8
    const auto golden = [](double actual, double expected, const char* label) {
        expectNear(actual, expected, 1e-8 * std::fmax(1.0, std::fabs(expected)), label);
    };
    assert(s.size() == 243);
    golden(t.durationS(), 5.36486852518, "golden duration");
    golden(t.lengthIn(), 119.887390117, "golden length");
    golden(t.sectionStart(1).tS, 2.00665200769, "golden section 1 start");
    golden(t.sectionStart(2).tS, 3.89911266776, "golden section 2 start");
    struct Row {
        std::size_t k;
        double t, s, x, y, h, v, a, f, st, w;
    };
    const Row rows[] = {
        {40, 0.787062010217, 19.9137214934, -44.217670956, -40.4658270212, 9.40188960883,
         31.2564846359, -1.28482452167, 31.0308703569, 3.74872200722, 34.7462129676},
        {100, 1.71232144608, 49.7843037335, -28.2350556326, -15.7742581088, 43.3608611768,
         34.8779544272, 2.39572562407, 34.6603808457, 3.88969210276, 15.4074679151},
        {160, 3.00004474926, 79.6548859736, -1.61302529974, -2.80653120002, 35.789193823,
         15.1340479579, -3.5663346733, 11.7523101905, 9.53533495884, -16.6277431448},
        {223, 4.92289194887, 110.459299701, -3.33333333333, 13.3333333333, 1.35833455607,
         24.6209472117, 6.75910022089, 16.9920477058, -17.8174452817, -9.70397594546},
    };
    for (const Row& row : rows) {
        const TrajectorySample& g = s[row.k];
        golden(g.tS, row.t, "golden t");
        golden(g.sIn, row.s, "golden s");
        golden(g.xIn, row.x, "golden x");
        golden(g.yIn, row.y, "golden y");
        golden(g.headingDeg, row.h, "golden heading");
        golden(g.speedInPerS, row.v, "golden speed");
        golden(g.accelInPerS2, row.a, "golden accel");
        golden(g.forwardInPerS, row.f, "golden forward");
        golden(g.strafeInPerS, row.st, "golden strafe");
        golden(g.omegaDegPerS, row.w, "golden turn rate");
    }
}

void testGoldenBranch() {
    // the golden chain's first section ending at 25in/s, and a branch carrying on from it
    const Trajectory trunk = plan({{.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},
                                                  {.xIn = -40, .yIn = -30},
                                                  {.xIn = -20, .yIn = -10, .headingDeg = 45}}}},
                                  {.endSpeedInPerS = 25.0});
    const Trajectory branch =
        plan({{.waypoints = {{.xIn = 10, .yIn = 0}, {.xIn = 20, .yIn = 24, .headingDeg = 90}},
               .constraints = {.voltageBudget = 8.0}}},
             {.from = &trunk});
    const auto golden = [](double actual, double expected, const char* label) {
        expectNear(actual, expected, 1e-8 * std::fmax(1.0, std::fabs(expected)), label);
    };
    assert(trunk.samples().size() == 121 && branch.samples().size() == 119);
    golden(trunk.durationS(), 2.01067677543, "golden branch: trunk duration");
    golden(trunk.samples().back().speedInPerS, 25.0, "golden branch: junction speed");
    golden(branch.durationS(), 2.98658036529, "golden branch: duration");
    golden(branch.lengthIn(), 58.5257483556, "golden branch: length");
    struct Row {
        std::size_t k;
        double t, s, x, y, h, v, a, f, st, w;
    };
    const Row rows[] = {
        {0, 0, 0, -20, -10, 45, 25, -14.9181223458, 24.834295924, 2.87362940515, 0},
        {1, 0.019958081963, 0.495980918267, -19.6066287741, -9.69794477816, 45.0002704141,
         24.7022628915, -14.6687615374, 24.4402120439, 3.588568964, 0.0402315927545},
        {15, 0.323834021373, 7.43971377401, -13.4156474867, -6.62058706667, 45.7570655131,
         21.3950535901, -7.24651901054, 19.2287856045, 9.3809446378, 6.07615871889},
        {40, 0.946144341287, 19.8392367307, -1.30721735675, -3.99843596606, 54.8242569316,
         19.3283698973, 2.00030408028, 17.5955293187, 7.99895186146, 22.3853134531},
        {70, 1.69358260801, 34.7186642787, 12.0741504601, 1.85680027705, 75.1850970868,
         17.2133970174, -4.85410762631, 14.5107632718, -9.25952407774, 23.1208557435},
    };
    for (const Row& row : rows) {
        const TrajectorySample& g = branch.samples()[row.k];
        golden(g.tS, row.t, "golden branch t");
        golden(g.sIn, row.s, "golden branch s");
        golden(g.xIn, row.x, "golden branch x");
        golden(g.yIn, row.y, "golden branch y");
        golden(g.headingDeg, row.h, "golden branch heading");
        golden(g.speedInPerS, row.v, "golden branch speed");
        golden(g.accelInPerS2, row.a, "golden branch accel");
        golden(g.forwardInPerS, row.f, "golden branch forward");
        golden(g.strafeInPerS, row.st, "golden branch strafe");
        golden(g.omegaDegPerS, row.w, "golden branch turn rate");
    }
}

} // namespace

int main() {
    testVoltageAccelRange();
    testStraightForwardMatchesTheClosedForm();
    testStrafingIsSlowerByTheStrafeModel();
    testEverySampleStaysWithinItsConstraints();
    testCornersAreSlower();
    testSectionsChangeLimitsAndStop();
    testSampleInterpolates();
    testRejections();
    testEndingMovingAndCarryingOn();
    testGolden();
    testGoldenBranch();
    std::puts("trajectory_generator_test: all assertions passed");
    return 0;
}
