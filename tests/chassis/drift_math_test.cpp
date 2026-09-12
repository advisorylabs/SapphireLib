// Host-side unit test for sapphirelib::chassis::strafeDriftIn — no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/chassis/drift_math_test.cpp src/sapphirelib/chassis/drift_math.cpp -o drift_math_test && ./drift_math_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/chassis/drift_math.hpp"

using sapphirelib::chassis::strafeDriftIn;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-9) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
        assert(false);
    }
}

void testPureStrafeWithNoForwardMotionIsNotDrift() {
    expectNear(strafeDriftIn(/*verticalWheelDeltaIn=*/0.0, /*verticalOffsetIn=*/3.5,
                             /*rotationDeltaDeg=*/0.0, /*strafeTravelIn=*/0.4,
                             /*throttleVolts=*/0.0, /*strafeVolts=*/10.0),
               0.0, "pure strafe");
}

void testForwardMotionDuringPureStrafeIsDrift() {
    expectNear(strafeDriftIn(0.1, 0.0, 0.0, 0.4, 0.0, 10.0), 0.1, "creeping forward");
    expectNear(strafeDriftIn(-0.1, 0.0, 0.0, -0.4, 0.0, -10.0), -0.1, "creeping back, strafing left");
}

void testRotationArcIsNotDrift() {
    // Wheel 3.5in right of center: a 90 degree clockwise turn rolls it
    // 3.5 * pi/2 inches "forward" with no translation at all.
    expectNear(strafeDriftIn(3.5 * kPi / 2.0, 3.5, 90.0, 0.4, 0.0, 10.0), 0.0, "rotation arc");
    expectNear(strafeDriftIn(3.5 * kPi / 2.0 + 0.2, 3.5, 90.0, 0.4, 0.0, 10.0), 0.2,
               "drift on top of rotation arc");
}

void testCommandedDiagonalIsNotDrift() {
    // Command is half as much forward as sideways; chassis went 0.4in right,
    // so 0.2in forward is exactly what was asked for.
    expectNear(strafeDriftIn(0.2, 0.0, 0.0, 0.4, 5.0, 10.0), 0.0, "diagonal right-forward");
    expectNear(strafeDriftIn(0.2, 0.0, 0.0, -0.4, 5.0, -10.0), 0.0, "diagonal left-forward");
    expectNear(strafeDriftIn(-0.2, 0.0, 0.0, 0.4, -5.0, 10.0), 0.0, "diagonal right-back");
}

void testDiagonalShortOfCommandIsDrift() {
    expectNear(strafeDriftIn(0.15, 0.0, 0.0, 0.4, 5.0, 10.0), -0.05, "fell behind the diagonal");
}

void testNoStrafeCommandReportsZero() {
    expectNear(strafeDriftIn(1.0, 0.0, 0.0, 0.0, 10.0, 0.0), 0.0, "no strafe command");
}

} // namespace

int main() {
    testPureStrafeWithNoForwardMotionIsNotDrift();
    testForwardMotionDuringPureStrafeIsDrift();
    testRotationArcIsNotDrift();
    testCommandedDiagonalIsNotDrift();
    testDiagonalShortOfCommandIsDrift();
    testNoStrafeCommandReportsZero();
    std::printf("drift_math_test: all assertions passed\n");
    return 0;
}
