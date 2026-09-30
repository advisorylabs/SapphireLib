// Host-side unit test for sapphirelib::odom::computeOdometryDelta and
// calibrateTrackingWheelOffsetIn, no PROS/embedded dependencies, so it
// builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/odom/odometry_math_test.cpp \
//       src/sapphirelib/odom/odometry_math.cpp src/sapphirelib/util/angle.cpp \
//       -o odometry_math_test && ./odometry_math_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/odom/odometry_math.hpp"

using sapphirelib::odom::calibrateTrackingWheelOffsetIn;
using sapphirelib::odom::computeOdometryDelta;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-6) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
        assert(false);
    }
}

void testForwardTravelAtHeadingZero() {
    // Facing "north" (heading 0), 10in of forward wheel travel should be
    // pure +y with no sideways drift.
    const auto delta = computeOdometryDelta(/*lastHeadingDeg=*/0.0, /*headingDeg=*/0.0,
                                             /*verticalDeltaIn=*/10.0, /*horizontalDeltaIn=*/0.0,
                                             /*verticalOffsetIn=*/0.0, /*horizontalOffsetIn=*/0.0);
    expectNear(delta.dxIn, 0.0, "forward@0: dx");
    expectNear(delta.dyIn, 10.0, "forward@0: dy");
}

void testForwardTravelAtHeadingEast() {
    // Facing "east" (heading 90, clockwise-positive), 10in of forward wheel
    // travel should be pure +x.
    const auto delta = computeOdometryDelta(/*lastHeadingDeg=*/90.0, /*headingDeg=*/90.0,
                                             /*verticalDeltaIn=*/10.0, /*horizontalDeltaIn=*/0.0,
                                             /*verticalOffsetIn=*/0.0, /*horizontalOffsetIn=*/0.0);
    expectNear(delta.dxIn, 10.0, "forward@90: dx");
    expectNear(delta.dyIn, 0.0, "forward@90: dy");
}

void testPureRotationDoesNotDriftWithCorrectOffset() {
    // Spinning 90 degrees in place about the tracking center: a wheel offset
    // 3in to the right measures pure arc length (offset * radians turned)
    // with zero true translation. Once that arc contribution is subtracted
    // out via the configured offset, position shouldn't move at all.
    const double dThetaRad = 90.0 * kPi / 180.0;
    const double horizontalOffsetIn = 3.0;
    const double horizontalDeltaIn = horizontalOffsetIn * dThetaRad;

    const auto delta = computeOdometryDelta(/*lastHeadingDeg=*/0.0, /*headingDeg=*/90.0,
                                             /*verticalDeltaIn=*/0.0, horizontalDeltaIn,
                                             /*verticalOffsetIn=*/0.0, horizontalOffsetIn);
    expectNear(delta.dxIn, 0.0, "pure rotation: dx");
    expectNear(delta.dyIn, 0.0, "pure rotation: dy");
}

void testHeadingWrapAcrossSeam() {
    // Heading crossing the 0/360 seam (355 -> 5) is a +10 degree turn, not
    // -350; this should behave identically to the small-turn case above,
    // not send the average heading off to ~180.
    const auto delta = computeOdometryDelta(/*lastHeadingDeg=*/355.0, /*headingDeg=*/5.0,
                                             /*verticalDeltaIn=*/10.0, /*horizontalDeltaIn=*/0.0,
                                             /*verticalOffsetIn=*/0.0, /*horizontalOffsetIn=*/0.0);
    // Average heading ~0 degrees, so travel should be ~pure +y.
    expectNear(delta.dxIn, 10.0 * std::sin(0.0 * kPi / 180.0), "wrap: dx");
    expectNear(delta.dyIn, 10.0 * std::cos(0.0 * kPi / 180.0), "wrap: dy");
}

void testReframingRotatesTheDeltaButIsNotATurn() {
    // Odometry::update() re-expresses the previous heading in the current
    // heading frame (rotation + offset) so that a setPose() between updates
    // shifts both endpoints by the same amount. That has to be exactly a
    // rotation of the field-frame delta, with no change to how much the
    // robot is considered to have turned, or the tracking-wheel arc
    // correction would fire for a turn that never happened.
    const double offsets[] = {0.0, 90.0, -90.0, 180.0, 233.7, -721.0};
    for (const double offset : offsets) {
        for (double last = -400.0; last <= 400.0; last += 37.0) {
            const double turned = 13.0;
            const auto base = computeOdometryDelta(last, last + turned, 7.0, -2.0, 3.59, 4.18);
            const auto shifted =
                computeOdometryDelta(last + offset, last + turned + offset, 7.0, -2.0, 3.59, 4.18);
            // Rotating (dx, dy) clockwise by `offset` degrees, in the same
            // convention as the field frame (x = forward·sin, y = forward·cos).
            const double rad = offset * kPi / 180.0;
            const double expectedDx = base.dxIn * std::cos(rad) + base.dyIn * std::sin(rad);
            const double expectedDy = -base.dxIn * std::sin(rad) + base.dyIn * std::cos(rad);
            expectNear(shifted.dxIn, expectedDx, "re-framed delta: dx");
            expectNear(shifted.dyIn, expectedDy, "re-framed delta: dy");
            // Same distance traveled, the arc correction saw the same turn.
            expectNear(std::hypot(shifted.dxIn, shifted.dyIn), std::hypot(base.dxIn, base.dyIn),
                       "re-framed delta: length");
        }
    }

    // And an unwrapped previous heading (rotation + offset, never wrapped) is
    // the same as its wrapped equivalent.
    const auto unwrapped = computeOdometryDelta(-350.0 + 720.0, 15.0, 10.0, 0.0, 0.0, 0.0);
    const auto wrapped = computeOdometryDelta(10.0, 15.0, 10.0, 0.0, 0.0, 0.0);
    expectNear(unwrapped.dxIn, wrapped.dxIn, "unwrapped last: dx");
    expectNear(unwrapped.dyIn, wrapped.dyIn, "unwrapped last: dy");
}

void testCalibrateTrackingWheelOffset() {
    // 1 full turn (2*pi radians) moving a wheel 5in-offset tracking wheel
    // through an arc length of offset * 2*pi.
    const double offsetIn = 5.0;
    const double wheelDistanceIn = offsetIn * 2.0 * kPi;
    expectNear(calibrateTrackingWheelOffsetIn(wheelDistanceIn, 2.0 * kPi), offsetIn, "calibrate: 1 turn");

    // 10 turns should give the same offset.
    expectNear(calibrateTrackingWheelOffsetIn(offsetIn * 10.0 * 2.0 * kPi, 10.0 * 2.0 * kPi), offsetIn,
               "calibrate: 10 turns");
}

// How far a wheel fixed to the chassis rolls while the chassis turns in
// place clockwise by `turnDeg`, worked out from the geometry rather than
// from a formula: step the rigid body through the turn and add up each
// step's displacement of the wheel along the way it rolls. `rightIn` /
// `forwardIn` place the wheel; `rollsForward` picks a vertical wheel (rolls
// along the robot's forward axis) or a horizontal one (along its right axis).
double wheelTravelOnTurnIn(double rightIn, double forwardIn, bool rollsForward, double turnDeg) {
    // the odom::Pose frame: heading clockwise from +y, forward = (sin, cos),
    // right = (cos, -sin)
    const auto position = [&](double h, double& x, double& y) {
        x = rightIn * std::cos(h) + forwardIn * std::sin(h);
        y = -rightIn * std::sin(h) + forwardIn * std::cos(h);
    };
    const int steps = 20000;
    const double total = turnDeg * kPi / 180.0;
    double travel = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double h0 = total * i / steps;
        const double h1 = total * (i + 1) / steps;
        double x0, y0, x1, y1;
        position(h0, x0, y0);
        position(h1, x1, y1);
        const double mid = (h0 + h1) / 2.0;
        const double dirX = rollsForward ? std::sin(mid) : std::cos(mid);
        const double dirY = rollsForward ? std::cos(mid) : -std::sin(mid);
        travel += (x1 - x0) * dirX + (y1 - y0) * dirY;
    }
    return travel;
}

void testOffsetSignsMatchWhereTheWheelsAre() {
    // Pins OdometryConfig's sign convention to the physical placement: turn
    // in place and see which offset makes odometry read no movement. A
    // vertical wheel RIGHT of center rolls backward on a clockwise turn (a
    // tank's right side), so it needs a NEGATIVE offset; left is positive.
    // The horizontal wheel is the intuitive way round: ahead is positive.
    const double r = 3.59;
    const double rightWheel = wheelTravelOnTurnIn(r, 0.0, true, 90.0);
    const double leftWheel = wheelTravelOnTurnIn(-r, 0.0, true, 90.0);
    expectNear(rightWheel, -r * kPi / 2.0, "right-side vertical wheel rolls backward");
    expectNear(leftWheel, r * kPi / 2.0, "left-side vertical wheel rolls forward");

    auto moved = [](double vertical, double horizontal, double vOffset, double hOffset) {
        const auto d = computeOdometryDelta(0.0, 90.0, vertical, horizontal, vOffset, hOffset);
        return std::hypot(d.dxIn, d.dyIn);
    };
    expectNear(moved(rightWheel, 0.0, -r, 0.0), 0.0, "right-side wheel, offset -r");
    expectNear(moved(leftWheel, 0.0, r, 0.0), 0.0, "left-side wheel, offset +r");
    // the wrong sign doubles the arc instead of removing it: 11in of motion
    // on a 90 degree turn that never went anywhere
    expectNear(moved(rightWheel, 0.0, r, 0.0), r * kPi, "wrong sign on a right-side wheel");

    const double f = 4.18;
    const double aheadWheel = wheelTravelOnTurnIn(0.0, f, false, 90.0);
    const double behindWheel = wheelTravelOnTurnIn(0.0, -f, false, 90.0);
    expectNear(moved(0.0, aheadWheel, 0.0, f), 0.0, "horizontal wheel ahead, offset +f");
    expectNear(moved(0.0, behindWheel, 0.0, -f), 0.0, "horizontal wheel behind, offset -f");

    // and calibration measures exactly those signs
    expectNear(calibrateTrackingWheelOffsetIn(rightWheel, kPi / 2.0), -r, "calibrates right as -r");
    expectNear(calibrateTrackingWheelOffsetIn(aheadWheel, kPi / 2.0), f, "calibrates ahead as +f");
}

} // namespace

int main() {
    testForwardTravelAtHeadingZero();
    testForwardTravelAtHeadingEast();
    testPureRotationDoesNotDriftWithCorrectOffset();
    testHeadingWrapAcrossSeam();
    testReframingRotatesTheDeltaButIsNotATurn();
    testCalibrateTrackingWheelOffset();
    testOffsetSignsMatchWhereTheWheelsAre();
    std::puts("odometry_math_test: all assertions passed");
    return 0;
}
