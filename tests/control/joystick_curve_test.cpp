// Host-side unit test for sapphirelib::curveJoystick and applyDeadband — no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/control/joystick_curve_test.cpp src/sapphirelib/control/joystick_curve.cpp -o joystick_curve_test && ./joystick_curve_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "sapphirelib/control/joystick_curve.hpp"

using sapphirelib::applyDeadband;
using sapphirelib::curveJoystick;

namespace {

void testEndpointsAndCenterAreFixed() {
    for (double curve : {0.0, 0.25, 0.5, 1.0}) {
        assert(std::fabs(curveJoystick(-1.0, curve) - -1.0) < 1e-9);
        assert(std::fabs(curveJoystick(0.0, curve) - 0.0) < 1e-9);
        assert(std::fabs(curveJoystick(1.0, curve) - 1.0) < 1e-9);
    }
}

void testLinearIsIdentity() {
    assert(std::fabs(curveJoystick(0.37, 0.0) - 0.37) < 1e-9);
    assert(std::fabs(curveJoystick(-0.6, 0.0) - -0.6) < 1e-9);
}

void testCurveSoftensSmallInputs() {
    // At curve = 1 (pure cubic), a small input should map to something
    // smaller in magnitude than the linear response.
    const double small = 0.25;
    assert(std::fabs(curveJoystick(small, 1.0)) < small);
}

void testCurveIsClamped() {
    // curve outside [0, 1] should behave as if clamped to the boundary.
    assert(std::fabs(curveJoystick(0.5, 2.0) - curveJoystick(0.5, 1.0)) < 1e-9);
    assert(std::fabs(curveJoystick(0.5, -5.0) - curveJoystick(0.5, 0.0)) < 1e-9);
}

void testMonotonic() {
    double prev = curveJoystick(-1.0, 0.7);
    for (int i = -100; i <= 100; ++i) {
        const double x = i / 100.0;
        const double y = curveJoystick(x, 0.7);
        assert(y + 1e-9 >= prev);
        prev = y;
    }
}

void testDeadbandZeroesTheBand() {
    for (double x : {0.0, 0.01, -0.01, 0.049, -0.049, 0.05, -0.05}) {
        assert(applyDeadband(x, 0.05) == 0.0); // the edge itself is inside
    }
    assert(applyDeadband(0.0501, 0.05) > 0.0);
    assert(applyDeadband(-0.0501, 0.05) < 0.0);
}

void testDeadbandIsContinuousAtTheEdge() {
    // No jump from 0 to ~deadband just outside the band: the output rises
    // from 0 there.
    assert(applyDeadband(0.05 + 1e-9, 0.05) < 1e-8);
    assert(applyDeadband(-0.05 - 1e-9, 0.05) > -1e-8);
}

void testDeadbandKeepsFullScale() {
    for (double deadband : {0.01, 0.05, 0.2, 0.9}) {
        assert(std::fabs(applyDeadband(1.0, deadband) - 1.0) < 1e-12);
        assert(std::fabs(applyDeadband(-1.0, deadband) - -1.0) < 1e-12);
        // Past full scale still comes out as full scale.
        assert(std::fabs(applyDeadband(1.5, deadband) - 1.0) < 1e-12);
        assert(std::fabs(applyDeadband(-1.5, deadband) - -1.0) < 1e-12);
    }
}

void testDeadbandRescalesLinearly() {
    // Halfway between the edge and full scale comes out as half.
    assert(std::fabs(applyDeadband(0.55, 0.1) - 0.5) < 1e-12);
    assert(std::fabs(applyDeadband(-0.55, 0.1) - -0.5) < 1e-12);
    assert(std::fabs(applyDeadband(0.28, 0.1) - 0.2) < 1e-12);
}

void testDeadbandIsOddAndMonotonic() {
    double prev = applyDeadband(-1.0, 0.1);
    for (int i = -100; i <= 100; ++i) {
        const double x = i / 100.0;
        const double y = applyDeadband(x, 0.1);
        assert(y == -applyDeadband(-x, 0.1));
        assert(y >= prev);
        prev = y;
    }
}

void testDeadbandOutOfRangeSettings() {
    // <= 0 is no deadband at all: the input comes back untouched, even
    // beyond +-1.
    for (double x : {-1.5, -0.3, 0.0, 0.001, 0.7, 1.2}) {
        assert(applyDeadband(x, 0.0) == x);
        assert(applyDeadband(x, -0.2) == x);
    }
    // >= 1 swallows the whole stick.
    for (double x : {-1.0, -0.5, 0.0, 0.99, 1.0}) {
        assert(applyDeadband(x, 1.0) == 0.0);
        assert(applyDeadband(x, 3.0) == 0.0);
    }
}

void testDeadbandThenCurve() {
    // The intended order: a stick resting a count off center gives exactly 0
    // after the deadband, and the curve keeps it there.
    const double resting = 2.0 / 127.0;
    assert(curveJoystick(applyDeadband(resting, 0.05), 0.3) == 0.0);
    assert(std::fabs(curveJoystick(applyDeadband(1.0, 0.05), 0.3) - 1.0) < 1e-12);
}

} // namespace

int main() {
    testEndpointsAndCenterAreFixed();
    testLinearIsIdentity();
    testCurveSoftensSmallInputs();
    testCurveIsClamped();
    testMonotonic();
    testDeadbandZeroesTheBand();
    testDeadbandIsContinuousAtTheEdge();
    testDeadbandKeepsFullScale();
    testDeadbandRescalesLinearly();
    testDeadbandIsOddAndMonotonic();
    testDeadbandOutOfRangeSettings();
    testDeadbandThenCurve();
    std::puts("joystick_curve_test: all assertions passed");
    return 0;
}
