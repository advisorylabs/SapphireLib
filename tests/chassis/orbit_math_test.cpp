// Host-side unit test for sapphirelib::chassis's orbit math (pivotBehind,
// measureOrbit, orbitTranslation), no PROS/embedded dependencies, so it
// builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/chassis/orbit_math_test.cpp
//   src/sapphirelib/chassis/orbit_math.cpp src/sapphirelib/util/angle.cpp -o orbit_math_test &&
//   ./orbit_math_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "sapphirelib/chassis/orbit_math.hpp"

using sapphirelib::chassis::measureOrbit;
using sapphirelib::chassis::OrbitFrame;
using sapphirelib::chassis::OrbitPivot;
using sapphirelib::chassis::orbitTranslation;
using sapphirelib::chassis::OrbitTranslation;
using sapphirelib::chassis::pivotBehind;
using sapphirelib::odom::Pose;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinRadiusIn = 6.0;

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (std::fabs(actual - expected) >= tolerance) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
        assert(false);
    }
}

// Where a robot-frame translation goes on the field, at a heading (0 faces +y,
// 90 faces +x).
void fieldMotion(OrbitTranslation move, double headingDeg, double& dxIn, double& dyIn) {
    const double headingRad = headingDeg * kPi / 180.0;
    dxIn = move.forward * std::sin(headingRad) + move.strafe * std::cos(headingRad);
    dyIn = move.forward * std::cos(headingRad) - move.strafe * std::sin(headingRad);
}

void testPivotStartsStraightBehind() {
    const OrbitPivot behind =
        pivotBehind(Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0}, 12.0, false);
    expectNear(behind.xIn, 0.0, 1e-9, "facing +y: x");
    expectNear(behind.yIn, -12.0, 1e-9, "facing +y: y");

    const OrbitPivot east =
        pivotBehind(Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 90.0}, 12.0, false);
    expectNear(east.xIn, -12.0, 1e-9, "facing +x: x");
    expectNear(east.yIn, 0.0, 1e-9, "facing +x: y");

    const OrbitPivot ahead =
        pivotBehind(Pose{.xIn = 5.0, .yIn = 5.0, .headingDeg = 0.0}, 12.0, true);
    expectNear(ahead.xIn, 5.0, 1e-9, "pivot ahead: x");
    expectNear(ahead.yIn, 17.0, 1e-9, "pivot ahead: y");
}

void testFacingAwayIsNoError() {
    const OrbitPivot pivot{.xIn = 0.0, .yIn = -12.0};
    const OrbitFrame start =
        measureOrbit(pivot, Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0}, false, kMinRadiusIn);
    expectNear(start.radiusIn, 12.0, 1e-9, "start: radius");
    expectNear(start.headingErrorDeg, 0.0, 1e-9, "start: facing away");
    expectNear(start.pivot.yIn, -12.0, 1e-12, "start: pivot untouched");

    // A quarter of the way around clockwise: east of the pivot, facing +x.
    const OrbitFrame quarter = measureOrbit(
        pivot, Pose{.xIn = 12.0, .yIn = -12.0, .headingDeg = 90.0}, false, kMinRadiusIn);
    expectNear(quarter.radiusIn, 12.0, 1e-9, "quarter: radius");
    expectNear(quarter.headingErrorDeg, 0.0, 1e-9, "quarter: facing away");
}

void testLaggingHeadingIsTheTurnLeftToMake() {
    const OrbitPivot pivot{.xIn = 0.0, .yIn = -12.0};
    // A quarter of the way around, but still facing 80: 10 more degrees to go.
    const OrbitFrame behind = measureOrbit(
        pivot, Pose{.xIn = 12.0, .yIn = -12.0, .headingDeg = 80.0}, false, kMinRadiusIn);
    expectNear(behind.headingErrorDeg, 10.0, 1e-9, "lagging clockwise");

    // Across the 0/360 seam.
    const OrbitFrame seam =
        measureOrbit(pivot, Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 350.0}, false, kMinRadiusIn);
    expectNear(seam.headingErrorDeg, 10.0, 1e-9, "lagging across the seam");
}

void testPivotAheadFacesItWithTheFront() {
    const OrbitPivot pivot{.xIn = 0.0, .yIn = 12.0};
    const OrbitFrame facing =
        measureOrbit(pivot, Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0}, true, kMinRadiusIn);
    expectNear(facing.radiusIn, 12.0, 1e-9, "pivot ahead: radius");
    expectNear(facing.headingErrorDeg, 0.0, 1e-9, "pivot ahead: front at it");

    const OrbitFrame turned =
        measureOrbit(pivot, Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 90.0}, true, kMinRadiusIn);
    expectNear(turned.headingErrorDeg, -90.0, 1e-9, "pivot ahead: turn back to face it");
}

void testPivotIsPushedBackAtTheMinimum() {
    const OrbitFrame close =
        measureOrbit(OrbitPivot{.xIn = 0.0, .yIn = -3.0},
                     Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0}, false, kMinRadiusIn);
    expectNear(close.radiusIn, kMinRadiusIn, 1e-12, "close: radius at the minimum");
    expectNear(close.pivot.xIn, 0.0, 1e-9, "close: pivot x");
    expectNear(close.pivot.yIn, -kMinRadiusIn, 1e-9, "close: pivot pushed back");
    expectNear(close.headingErrorDeg, 0.0, 1e-9, "close: same direction");

    // Right on top of it, there's no line to push along: it goes straight behind.
    const OrbitFrame onTop =
        measureOrbit(OrbitPivot{.xIn = 0.0, .yIn = 0.0},
                     Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 90.0}, false, kMinRadiusIn);
    expectNear(onTop.pivot.xIn, -kMinRadiusIn, 1e-9, "on top: pivot behind (x)");
    expectNear(onTop.pivot.yIn, 0.0, 1e-9, "on top: pivot behind (y)");
    expectNear(onTop.headingErrorDeg, 0.0, 1e-9, "on top: facing away");
}

void testNoErrorPassesTheRequestThrough() {
    const OrbitTranslation behind = orbitTranslation(0.5, 0.25, 0.0, false);
    expectNear(behind.forward, 0.5, 1e-12, "no error: radial is forward");
    expectNear(behind.strafe, 0.25, 1e-12, "no error: around is strafe");

    const OrbitTranslation ahead = orbitTranslation(0.5, 0.25, 0.0, true);
    expectNear(ahead.forward, -0.5, 1e-12, "pivot ahead: away is backward");
    expectNear(ahead.strafe, -0.25, 1e-12, "pivot ahead: around is to the left");
}

void testMotionFollowsTheRealCircleWhileTheHeadingLags() {
    // Right above the pivot, away is +y and clockwise around it is +x,
    // whichever way the chassis happens to be facing.
    const OrbitPivot pivot{.xIn = 0.0, .yIn = -12.0};
    for (const double headingDeg : {-25.0, -10.0, 0.0, 10.0, 40.0}) {
        const Pose pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = headingDeg};
        const OrbitFrame frame = measureOrbit(pivot, pose, false, kMinRadiusIn);
        double dxIn = 0.0;
        double dyIn = 0.0;
        fieldMotion(orbitTranslation(0.0, 1.0, frame.headingErrorDeg, false), headingDeg, dxIn,
                    dyIn);
        expectNear(dxIn, 1.0, 1e-9, "around: along the tangent (x)");
        expectNear(dyIn, 0.0, 1e-9, "around: along the tangent (y)");
        fieldMotion(orbitTranslation(1.0, 0.0, frame.headingErrorDeg, false), headingDeg, dxIn,
                    dyIn);
        expectNear(dxIn, 0.0, 1e-9, "radial: straight away (x)");
        expectNear(dyIn, 1.0, 1e-9, "radial: straight away (y)");
    }

    // Pivot ahead: the robot sits south of it, so away is -y, and clockwise
    // around it (to the driver's right, with their back to it) is -x.
    const OrbitPivot aheadPivot{.xIn = 0.0, .yIn = 12.0};
    for (const double headingDeg : {-15.0, 0.0, 20.0}) {
        const Pose pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = headingDeg};
        const OrbitFrame frame = measureOrbit(aheadPivot, pose, true, kMinRadiusIn);
        double dxIn = 0.0;
        double dyIn = 0.0;
        fieldMotion(orbitTranslation(0.0, 1.0, frame.headingErrorDeg, true), headingDeg, dxIn,
                    dyIn);
        expectNear(dxIn, -1.0, 1e-9, "pivot ahead, around (x)");
        expectNear(dyIn, 0.0, 1e-9, "pivot ahead, around (y)");
        fieldMotion(orbitTranslation(1.0, 0.0, frame.headingErrorDeg, true), headingDeg, dxIn,
                    dyIn);
        expectNear(dxIn, 0.0, 1e-9, "pivot ahead, radial (x)");
        expectNear(dyIn, -1.0, 1e-9, "pivot ahead, radial (y)");
    }
}

void testSimulatedLapWithALaggingHeading() {
    // Go all the way around in 0.01in steps with the heading always 15
    // degrees behind facing away. The circle mustn't care: still 12in out
    // (give or take what straight steps along a curve add, ~0.03in a lap),
    // the pivot never moved, and back where it started.
    const OrbitPivot pivot{.xIn = 0.0, .yIn = -12.0};
    Pose pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = -15.0};
    const double stepIn = 0.01;
    const int steps = static_cast<int>(std::lround(2.0 * kPi * 12.0 / stepIn));
    for (int i = 0; i < steps; ++i) {
        const OrbitFrame frame = measureOrbit(pivot, pose, false, kMinRadiusIn);
        expectNear(frame.pivot.xIn, pivot.xIn, 1e-12, "lap: pivot x never moves");
        expectNear(frame.pivot.yIn, pivot.yIn, 1e-12, "lap: pivot y never moves");
        double dxIn = 0.0;
        double dyIn = 0.0;
        fieldMotion(orbitTranslation(0.0, stepIn, frame.headingErrorDeg, false), pose.headingDeg,
                    dxIn, dyIn);
        pose.xIn += dxIn;
        pose.yIn += dyIn;
        // the turn catches up to 15 degrees short of facing away again
        const OrbitFrame after = measureOrbit(pivot, pose, false, kMinRadiusIn);
        pose.headingDeg += after.headingErrorDeg - 15.0;
    }
    const OrbitFrame end = measureOrbit(pivot, pose, false, kMinRadiusIn);
    expectNear(end.radiusIn, 12.0, 0.05, "lap: still on the circle");
    expectNear(end.headingErrorDeg, 15.0, 1e-6, "lap: still lagging the same");
    // ~0.1in short of the full lap: the same steps at that slightly bigger
    // radius sweep a little less angle.
    expectNear(pose.xIn, 0.0, 0.2, "lap: back where it started (x)");
    expectNear(pose.yIn, 0.0, 0.2, "lap: back where it started (y)");
}

} // namespace

int main() {
    testPivotStartsStraightBehind();
    testFacingAwayIsNoError();
    testLaggingHeadingIsTheTurnLeftToMake();
    testPivotAheadFacesItWithTheFront();
    testPivotIsPushedBackAtTheMinimum();
    testNoErrorPassesTheRequestThrough();
    testMotionFollowsTheRealCircleWhileTheHeadingLags();
    testSimulatedLapWithALaggingHeading();
    std::printf("orbit_math_test: all assertions passed\n");
    return 0;
}
