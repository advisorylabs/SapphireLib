// Host-side unit test for sapphirelib::curveJoystick — no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/control/joystick_curve_test.cpp src/sapphirelib/control/joystick_curve.cpp -o joystick_curve_test && ./joystick_curve_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "sapphirelib/control/joystick_curve.hpp"

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

} // namespace

int main() {
    testEndpointsAndCenterAreFixed();
    testLinearIsIdentity();
    testCurveSoftensSmallInputs();
    testCurveIsClamped();
    testMonotonic();
    std::puts("joystick_curve_test: all assertions passed");
    return 0;
}
