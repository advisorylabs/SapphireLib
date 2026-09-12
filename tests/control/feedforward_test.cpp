// Host-side unit test for sapphirelib::MotorFeedforward — no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/control/feedforward_test.cpp \
//       src/sapphirelib/control/feedforward.cpp -o feedforward_test && ./feedforward_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/control/feedforward.hpp"

using sapphirelib::MotorFeedforward;

namespace {

constexpr MotorFeedforward kModel{.kS = 1.0, .kV = 0.2, .kA = 0.05};

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-9) {
        std::printf("FAIL %s: got %f, expected %f\n", label, actual, expected);
        assert(false);
    }
}

void expectTrue(bool condition, const char* label) {
    if (!condition) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

void testVoltsSumsAllThreeTerms() {
    expectNear(kModel.volts(10.0, 20.0), 1.0 + 2.0 + 1.0, "moving forward and accelerating");
    expectNear(kModel.volts(-10.0, 0.0), -1.0 - 2.0, "moving backward at steady speed");
}

void testFrictionFollowsVelocityNotAcceleration() {
    // Braking while still moving forward: friction still helps the brake,
    // so it keeps velocity's sign even though acceleration is negative.
    expectNear(kModel.volts(10.0, -20.0), 1.0 + 2.0 - 1.0, "decelerating forward");
}

void testFrictionFollowsAccelerationFromRest() {
    expectNear(kModel.volts(0.0, 20.0), 1.0 + 1.0, "starting forward from rest");
    expectNear(kModel.volts(0.0, -20.0), -1.0 - 1.0, "starting backward from rest");
    expectNear(kModel.volts(0.0, 0.0), 0.0, "at rest and staying there");
}

void testMaxVelocity() {
    expectNear(kModel.maxVelocity(12.0), 55.0, "(12 - 1) / 0.2");
    expectNear(kModel.maxVelocity(-12.0), 55.0, "symmetric in sign");
    expectNear(kModel.maxVelocity(0.5), 0.0, "below static friction");
    expectNear(MotorFeedforward{}.maxVelocity(12.0), 0.0, "unmeasured model");
}

void testMaxVelocityRoundTripsThroughVolts() {
    // Full stick in velocity mode relies on this: asking for the speed 12V
    // reaches should ask for exactly 12V.
    expectNear(kModel.volts(kModel.maxVelocity(12.0)), 12.0, "round trip at full voltage");
}

void testValidity() {
    expectTrue(kModel.valid(), "measured model is valid");
    expectTrue(!MotorFeedforward{}.valid(), "default model is not");
    expectTrue(!MotorFeedforward{.kS = 1.0, .kV = 0.2, .kA = 0.0}.valid(), "no inertia term");
}

} // namespace

int main() {
    testVoltsSumsAllThreeTerms();
    testFrictionFollowsVelocityNotAcceleration();
    testFrictionFollowsAccelerationFromRest();
    testMaxVelocity();
    testMaxVelocityRoundTripsThroughVolts();
    testValidity();
    std::puts("feedforward_test: all assertions passed");
    return 0;
}
