// Host-side unit test for sapphirelib::PID — no PROS/embedded dependencies,
// so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/control/pid_test.cpp src/sapphirelib/control/pid.cpp -o pid_test && ./pid_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/control/pid.hpp"

using sapphirelib::PID;

namespace {

void testProportionalOnly() {
    PID pid(PID::Config{.gains = {.kP = 2.0}});
    const double output = pid.update(/*target=*/10.0, /*measurement=*/4.0);
    assert(std::fabs(output - 12.0) < 1e-9); // kP * (10 - 4)
}

void testConvergesToTarget() {
    PID pid(PID::Config{.gains = {.kP = 0.5, .kI = 0.02, .kD = 0.05}});
    double measurement = 0.0;
    for (int i = 0; i < 500; ++i) {
        const double output = pid.update(100.0, measurement);
        measurement += output * 0.05; // simple first-order plant
    }
    assert(std::fabs(measurement - 100.0) < 1.0);
}

void testIntegralWindupGuard() {
    PID pid(PID::Config{.gains = {.kI = 1.0}, .integralLimit = 5.0});
    for (int i = 0; i < 100; ++i) {
        pid.update(10.0, 0.0); // error stays 10; integral would explode unclamped
    }
    const double output = pid.update(10.0, 0.0);
    assert(std::fabs(output - 5.0) < 1e-9); // clamped to integralLimit * kI
}

void testSlewRateLimitsOutputChange() {
    PID pid(PID::Config{.gains = {.kP = 100.0}, .slewRate = 1.0});
    pid.update(1.0, 0.0);                        // first call: output = 100, not slew-limited
    const double second = pid.update(1.0, -1.0); // error jumps to 2 -> wants output = 200
    assert(std::fabs(second - 101.0) < 1e-9);    // limited to prevOutput + slewRate
}

void testResetClearsState() {
    PID pid(PID::Config{.gains = {.kI = 1.0}});
    pid.update(10.0, 0.0);
    pid.update(10.0, 0.0);
    pid.reset();
    const double output = pid.update(10.0, 0.0);
    assert(std::fabs(output - 10.0) < 1e-9); // integral reset; only this call's error counted
}

} // namespace

int main() {
    testProportionalOnly();
    testConvergesToTarget();
    testIntegralWindupGuard();
    testSlewRateLimitsOutputChange();
    testResetClearsState();
    std::puts("pid_test: all assertions passed");
    return 0;
}
