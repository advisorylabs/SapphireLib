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
    // kI/kD are per-second (see PIDGains): at the default 10ms timestep
    // these are the same effective gains as a per-tick 0.02 / 0.05.
    PID pid(PID::Config{.gains = {.kP = 0.5, .kI = 2.0, .kD = 0.0005}});
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

void testIntegralScalesWithTimestep() {
    // kI is per-second, so one second's worth of a constant error produces
    // the same integral term no matter how it's sliced up.
    PID coarse(PID::Config{.gains = {.kI = 2.0}, .nominalDtS = 0.1});
    double coarseOut = 0.0;
    for (int i = 0; i < 10; ++i) coarseOut = coarse.update(3.0, 0.0); // 10 x 0.1s = 1s

    PID fine(PID::Config{.gains = {.kI = 2.0}, .nominalDtS = 0.01});
    double fineOut = 0.0;
    for (int i = 0; i < 100; ++i) fineOut = fine.update(3.0, 0.0); // 100 x 0.01s = 1s

    // integral = 3 error-seconds after 1s, times kI = 2 -> 6.
    assert(std::fabs(coarseOut - 6.0) < 1e-9);
    assert(std::fabs(fineOut - 6.0) < 1e-9);
}

void testDerivativeScalesWithTimestep() {
    // kD is per-second too: the same error ramp rate gives the same
    // derivative term regardless of sample period.
    PID coarse(PID::Config{.gains = {.kD = 1.0}, .nominalDtS = 0.1});
    coarse.update(0.0, 0.0);
    const double coarseOut = coarse.update(5.0, 0.0); // error 0 -> 5 over 0.1s = 50/s

    PID fine(PID::Config{.gains = {.kD = 1.0}, .nominalDtS = 0.01});
    fine.update(0.0, 0.0);
    const double fineOut = fine.update(0.5, 0.0); // error 0 -> 0.5 over 0.01s = 50/s

    assert(std::fabs(coarseOut - 50.0) < 1e-9);
    assert(std::fabs(fineOut - 50.0) < 1e-9);
}

void testExplicitTimestepOverload() {
    PID pid(PID::Config{.gains = {.kI = 1.0}, .nominalDtS = 0.01});
    const double output = pid.update(10.0, 0.0, /*dtS=*/0.5);
    assert(std::fabs(output - 5.0) < 1e-9); // 10 error * 0.5s * kI

    // A nonsense timestep falls back to the nominal one instead of blowing
    // up the integral or dividing the derivative by zero.
    pid.reset();
    const double fallback = pid.update(10.0, 0.0, /*dtS=*/0.0);
    assert(std::fabs(fallback - 0.1) < 1e-9); // 10 error * 0.01s * kI
}

void testAntiWindupStopsIntegratingWhileSaturated() {
    // Output limit reached almost immediately, then held far from target
    // for a long time. Without conditional integration the integral would
    // charge to 1000 error-seconds over this stretch and take just as long
    // to unwind once the error reverses; with it, the integral stops at
    // roughly the value that saturates the output (5) and recovers in a
    // handful of ticks.
    PID pid(PID::Config{.gains = {.kI = 1.0}, .outputLimit = 5.0});
    for (int i = 0; i < 1000; ++i) pid.update(100.0, 0.0); // 10s stuck at +100 error

    int ticksToRecover = 0;
    while (pid.update(-100.0, 0.0) > 0.0) {
        ++ticksToRecover;
        assert(ticksToRecover < 100); // unwound, not wound up for the full 1000
    }
    assert(ticksToRecover <= 10);
}

void testSlewRateLimitsOutputChange() {
    PID pid(PID::Config{.gains = {.kP = 100.0}, .slewRate = 1.0});
    pid.update(1.0, 0.0);                        // first call: output = 100, not slew-limited
    const double second = pid.update(1.0, -1.0); // error jumps to 2 -> wants output = 200
    assert(std::fabs(second - 101.0) < 1e-9);    // limited to prevOutput + slewRate
}

void testResetClearsState() {
    PID pid(PID::Config{.gains = {.kI = 1.0}, .nominalDtS = 0.01});
    pid.update(10.0, 0.0);
    pid.update(10.0, 0.0);
    pid.reset();
    const double output = pid.update(10.0, 0.0);
    // Integral reset; only this call's error x timestep counted.
    assert(std::fabs(output - 0.1) < 1e-9);
}

} // namespace

int main() {
    testProportionalOnly();
    testConvergesToTarget();
    testIntegralWindupGuard();
    testIntegralScalesWithTimestep();
    testDerivativeScalesWithTimestep();
    testExplicitTimestepOverload();
    testAntiWindupStopsIntegratingWhileSaturated();
    testSlewRateLimitsOutputChange();
    testResetClearsState();
    std::puts("pid_test: all assertions passed");
    return 0;
}
