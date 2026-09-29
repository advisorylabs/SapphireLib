// Host-side unit test for sapphirelib::PID — no PROS/embedded dependencies,
// so it builds and runs with a normal desktop compiler.
//
// Besides the control math itself, this covers what the telemetry hook added
// (PidObserver, lastStep(), config()) and holds update() to the exact bits
// the pre-observer version produced, so logging a PID can never change how
// it drives.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/control/pid_test.cpp src/sapphirelib/control/pid.cpp -o pid_test && ./pid_test

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "sapphirelib/control/pid.hpp"

using sapphirelib::PID;
using sapphirelib::PIDGains;
using sapphirelib::PidObserver;
using sapphirelib::PidStep;

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

// --- Observer, lastStep(), and config() ------------------------------------

/// Same bits, not just the same value: -0.0 vs 0.0 or a different NaN
/// payload counts as a difference.
bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool sameStep(const PidStep& a, const PidStep& b) {
    return sameBits(a.target, b.target) && sameBits(a.measurement, b.measurement) &&
           sameBits(a.error, b.error) && sameBits(a.pTerm, b.pTerm) &&
           sameBits(a.iTerm, b.iTerm) && sameBits(a.dTerm, b.dTerm) &&
           sameBits(a.rawOutput, b.rawOutput) && sameBits(a.output, b.output) &&
           sameBits(a.dtS, b.dtS) && a.flags == b.flags;
}

/// Records every call it gets.
class RecordingObserver : public PidObserver {
public:
    void onPidUpdate(const PID& pid, const PidStep& step) override {
        // By the time an observer hears about a step, lastStep() is it.
        assert(sameStep(pid.lastStep(), step));
        updatedPids.push_back(&pid);
        steps.push_back(step);
    }

    void onPidReset(const PID& pid) override { resetPids.push_back(&pid); }

    std::vector<const PID*> updatedPids;
    std::vector<PidStep> steps;
    std::vector<const PID*> resetPids;
};

void testObserverSeesEveryStepWithItsTerms() {
    PID pid(PID::Config{.gains = {.kP = 2.0, .kI = 1.0, .kD = 0.5}, .nominalDtS = 0.01});
    RecordingObserver observer;
    pid.setObserver(&observer);
    assert(pid.observer() == &observer);

    const double first = pid.update(10.0, 4.0);
    const double second = pid.update(10.0, 5.0);
    assert(observer.steps.size() == 2);
    assert(observer.updatedPids[0] == &pid && observer.updatedPids[1] == &pid);

    // First step: error 6, integral 6 x 0.01s, no derivative yet.
    const PidStep& a = observer.steps[0];
    assert(a.target == 10.0 && a.measurement == 4.0 && a.error == 6.0);
    assert(std::fabs(a.pTerm - 12.0) < 1e-9);
    assert(std::fabs(a.iTerm - 0.06) < 1e-9);
    assert(a.dTerm == 0.0);
    assert(std::fabs(a.rawOutput - 12.06) < 1e-9);
    assert(sameBits(a.output, first)); // exactly what update() returned
    assert(a.dtS == 0.01);
    assert(a.flags == PidStep::kFirstStep);

    // Second: error 5, integral 0.11, error fell 1 over 0.01s = -100/s.
    const PidStep& b = observer.steps[1];
    assert(b.error == 5.0);
    assert(std::fabs(b.pTerm - 10.0) < 1e-9);
    assert(std::fabs(b.iTerm - 0.11) < 1e-9);
    assert(std::fabs(b.dTerm - -50.0) < 1e-9);
    assert(std::fabs(b.rawOutput - (10.0 + 0.11 - 50.0)) < 1e-9);
    assert(sameBits(b.output, second));
    assert(b.flags == 0);

    // Many more steps: one observer call each, every one matching what
    // update() returned.
    for (int i = 0; i < 100; ++i) {
        const double output = pid.update(10.0, i * 0.1);
        assert(observer.steps.size() == static_cast<std::size_t>(3 + i));
        assert(sameBits(observer.steps.back().output, output));
    }
}

void testDerivativeOnMeasurementTerm() {
    // The target jumps but the measurement only moves 0.2: the derivative
    // comes from the measurement, so no kick from the jump.
    PID pid(PID::Config{.gains = {.kD = 1.0}, .derivativeOnMeasurement = true, .nominalDtS = 0.02});
    RecordingObserver observer;
    pid.setObserver(&observer);
    pid.update(0.0, 1.0);
    pid.update(100.0, 1.2);
    assert(std::fabs(observer.steps[1].dTerm - -10.0) < 1e-9); // -(0.2 / 0.02)
}

void testDtFallbackFlag() {
    PID pid(PID::Config{.gains = {.kP = 1.0}, .nominalDtS = 0.02});
    RecordingObserver observer;
    pid.setObserver(&observer);
    for (double dtS : {0.0, -0.01, 0.51, std::nan("")}) {
        pid.update(1.0, 0.0, dtS);
        assert((observer.steps.back().flags & PidStep::kDtFallback) != 0);
        assert(observer.steps.back().dtS == 0.02); // what was actually used
    }
    pid.update(1.0, 0.0, 0.5); // the largest plausible timestep is used as given
    assert((observer.steps.back().flags & PidStep::kDtFallback) == 0);
    assert(observer.steps.back().dtS == 0.5);
    pid.update(1.0, 0.0); // the two-argument overload uses the nominal dt
    assert(observer.steps.back().flags == 0);
    assert(observer.steps.back().dtS == 0.02);
}

void testSaturatedAndIntegralHeldFlags() {
    PID pid(PID::Config{.gains = {.kP = 1.0, .kI = 1.0}, .outputLimit = 5.0});
    RecordingObserver observer;
    pid.setObserver(&observer);

    // 10 + 0.1 of integral is past the limit and the integration pushes
    // further out, so it's rolled back; then the output is clamped.
    const double output = pid.update(10.0, 0.0);
    const PidStep& step = observer.steps.back();
    assert(output == 5.0);
    assert(step.flags == (PidStep::kFirstStep | PidStep::kIntegralHeld | PidStep::kSaturated));
    assert(step.iTerm == 0.0);      // the rolled-back integral
    assert(step.rawOutput == 10.0); // after anti-windup, before the clamp
    assert(step.output == 5.0);

    // Well inside the limit: neither flag.
    pid.update(3.0, 0.0);
    assert(observer.steps.back().flags == 0);
}

void testSlewLimitedFlag() {
    PID pid(PID::Config{.gains = {.kP = 100.0}, .slewRate = 1.0});
    RecordingObserver observer;
    pid.setObserver(&observer);
    pid.update(1.0, 0.0); // first step is never slew-limited
    assert(observer.steps.back().flags == PidStep::kFirstStep);
    pid.update(1.0, -1.0); // wants 200, gets 101
    assert(observer.steps.back().flags == PidStep::kSlewLimited);
    assert(observer.steps.back().rawOutput == 200.0);
    assert(observer.steps.back().output == 101.0);
    pid.update(1.015, 0.0); // wants 101.5: a change of 0.5, within the limit
    assert(observer.steps.back().flags == 0);
}

void testOnPidResetOnlyWhenThereWasState() {
    PID pid(PID::Config{.gains = {.kP = 1.0, .kI = 1.0}});
    RecordingObserver observer;
    pid.setObserver(&observer);

    pid.reset(); // nothing to clear yet
    assert(observer.resetPids.empty());

    pid.update(1.0, 0.0);
    pid.reset();
    assert(observer.resetPids.size() == 1);
    assert(observer.resetPids[0] == &pid);

    // An idle loop that resets every tick (a lift resting on its hard stop)
    // reports it once, not every tick.
    for (int i = 0; i < 10; ++i) pid.reset();
    assert(observer.resetPids.size() == 1);

    // The step after a reset starts a new response.
    pid.update(1.0, 0.0);
    assert(observer.steps.back().flags == PidStep::kFirstStep);
    pid.reset();
    assert(observer.resetPids.size() == 2);
}

void testLastStep() {
    PID pid(PID::Config{.gains = {.kP = 2.0}});
    assert(sameStep(pid.lastStep(), PidStep{})); // all zeros before the first update

    // Kept up to date with no observer attached.
    const double output = pid.update(3.0, 1.0);
    assert(sameBits(pid.lastStep().output, output));
    assert(pid.lastStep().error == 2.0);
    assert(pid.lastStep().flags == PidStep::kFirstStep);

    // reset() leaves it alone, so a readout can still show a finished
    // motion's last step.
    const PidStep before = pid.lastStep();
    pid.reset();
    assert(sameStep(pid.lastStep(), before));
}

void testConfigReflectsConstructionAndGains() {
    const PID::Config config{.gains = {.kP = 1.5, .kI = 0.25, .kD = 0.125},
                             .integralLimit = 3.0,
                             .outputLimit = 12.0,
                             .slewRate = 0.5,
                             .derivativeOnMeasurement = true,
                             .nominalDtS = 0.02};
    PID pid(config);
    assert(pid.config().gains.kP == 1.5 && pid.config().gains.kI == 0.25 &&
           pid.config().gains.kD == 0.125);
    assert(pid.config().integralLimit == 3.0);
    assert(pid.config().outputLimit == 12.0);
    assert(pid.config().slewRate == 0.5);
    assert(pid.config().derivativeOnMeasurement);
    assert(pid.config().nominalDtS == 0.02);

    pid.setGains({.kP = 4.0, .kI = 0.0, .kD = 1.0});
    assert(pid.config().gains.kP == 4.0 && pid.config().gains.kD == 1.0);
    assert(&pid.gains() == &pid.config().gains); // one set of gains, not two
    assert(pid.config().outputLimit == 12.0);    // nothing else changed
}

void testDetachingAndCopying() {
    PID pid(PID::Config{.gains = {.kP = 1.0}});
    RecordingObserver observer;
    pid.setObserver(&observer);
    pid.update(1.0, 0.0);

    // A copy carries the observer pointer, and the observer is told which PID
    // each step came from.
    PID copy = pid;
    copy.update(2.0, 0.0);
    assert(observer.updatedPids.size() == 2);
    assert(observer.updatedPids[1] == &copy);

    pid.setObserver(nullptr);
    assert(pid.observer() == nullptr);
    pid.update(1.0, 0.0);
    pid.reset();
    assert(observer.steps.size() == 2);
    assert(observer.resetPids.empty());
}

// --- Bit-exact against the old math ----------------------------------------

/// PID::update() and reset() exactly as they were before the observer was
/// added (git show 73dc252:src/sapphirelib/control/pid.cpp), so the new code
/// can be held to producing the same bits. Only the packaging differs (a class
/// in this file, with the constant as a member, and one line re-wrapped for
/// the deeper indent); every expression of the math is as it was.
class OldPid {
public:
    explicit OldPid(PID::Config config) : config_(config) {}

    double update(double target, double measurement) {
        return update(target, measurement, config_.nominalDtS);
    }

    double update(double target, double measurement, double dtS) {
        if (!(dtS > 0.0) || dtS > kMaxPlausibleDtS) dtS = config_.nominalDtS;

        const double error = target - measurement;

        // Integral of error over time, so kI carries per-second units and stays
        // valid across a change of loop period — see PIDGains' comment.
        const double integralDelta = error * dtS;
        integral_ += integralDelta;
        if (config_.integralLimit > 0.0) {
            integral_ = std::clamp(integral_, -config_.integralLimit, config_.integralLimit);
        }

        // Rate of change per second, for the same reason.
        double derivative = 0.0;
        if (hasPrev_) {
            derivative = (config_.derivativeOnMeasurement ? -(measurement - prevMeasurement_)
                                                          : (error - prevError_)) /
                         dtS;
        }

        double output =
            config_.gains.kP * error + config_.gains.kI * integral_ + config_.gains.kD * derivative;

        // Conditional-integration anti-windup: if the output is already pinned
        // at the limit and this tick's error only pushes it further out, that
        // integration can't affect the plant — it just accumulates charge that
        // has to be paid back as overshoot once the error finally reverses. Roll
        // it back and recompute instead. Only meaningful when an output limit
        // exists to saturate against.
        if (config_.outputLimit > 0.0 && integralDelta != 0.0 &&
            std::fabs(output) > config_.outputLimit && (output > 0.0) == (integralDelta > 0.0)) {
            integral_ -= integralDelta;
            output = config_.gains.kP * error + config_.gains.kI * integral_ +
                     config_.gains.kD * derivative;
        }

        if (config_.slewRate > 0.0 && hasPrev_) {
            const double delta =
                std::clamp(output - prevOutput_, -config_.slewRate, config_.slewRate);
            output = prevOutput_ + delta;
        }

        if (config_.outputLimit > 0.0) {
            output = std::clamp(output, -config_.outputLimit, config_.outputLimit);
        }

        prevError_ = error;
        prevMeasurement_ = measurement;
        prevOutput_ = output;
        hasPrev_ = true;

        return output;
    }

    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
        prevMeasurement_ = 0.0;
        prevOutput_ = 0.0;
        hasPrev_ = false;
    }

    void setGains(PIDGains gains) { config_.gains = gains; }

private:
    static constexpr double kMaxPlausibleDtS = 0.5;

    PID::Config config_;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    double prevMeasurement_ = 0.0;
    double prevOutput_ = 0.0;
    bool hasPrev_ = false;
};

PIDGains randomGains(std::mt19937& rng) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    return {.kP = 5.0 * unit(rng),
            .kI = unit(rng) < 0.3 ? 0.0 : 5.0 * unit(rng),
            .kD = unit(rng) < 0.3 ? 0.0 : unit(rng)};
}

void testMatchesTheOldUpdateBitForBit() {
    std::mt19937 rng(96671);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    RecordingObserver observer;
    for (int run = 0; run < 2000; ++run) {
        const double dtChoices[] = {0.01, 0.02, 0.005, 0.001 + 0.05 * unit(rng)};
        const PID::Config config{
            .gains = randomGains(rng),
            .integralLimit = unit(rng) < 0.5 ? 0.0 : 0.1 + 10.0 * unit(rng),
            .outputLimit = unit(rng) < 0.5 ? 0.0 : 1.0 + 11.7 * unit(rng),
            .slewRate = unit(rng) < 0.5 ? 0.0 : 0.05 + 3.0 * unit(rng),
            .derivativeOnMeasurement = unit(rng) < 0.5,
            .nominalDtS = dtChoices[rng() % 4],
        };
        OldPid reference(config);
        PID pid(config);
        // Half the runs observed: attaching an observer must not change the
        // math either.
        const bool observed = run % 2 == 0;
        pid.setObserver(observed ? &observer : nullptr);

        double target = 0.0;
        double measurement = 0.0;
        for (int tick = 0; tick < 300; ++tick) {
            // Target steps now and then; the measurement chases it noisily.
            if (unit(rng) < 0.05) target = 200.0 * unit(rng) - 100.0;
            measurement += 0.1 * (target - measurement) + (unit(rng) - 0.5);

            double expected;
            double actual;
            const double roll = unit(rng);
            if (roll < 0.5) {
                expected = reference.update(target, measurement);
                actual = pid.update(target, measurement);
            } else {
                // Explicit timesteps, including ones that must fall back.
                const double dtS = roll < 0.8    ? 0.001 + 0.1 * unit(rng)
                                   : roll < 0.85 ? 0.0
                                   : roll < 0.9  ? -0.01
                                   : roll < 0.95 ? 0.5 + unit(rng)
                                                 : std::nan("");
                expected = reference.update(target, measurement, dtS);
                actual = pid.update(target, measurement, dtS);
            }
            if (!sameBits(expected, actual)) {
                std::printf("FAIL run %d tick %d: old update() gave %.17g, new gave %.17g\n", run,
                            tick, expected, actual);
                assert(false);
            }
            assert(sameBits(pid.lastStep().output, actual));
            if (observed) assert(sameBits(observer.steps.back().output, actual));

            if (unit(rng) < 0.01) {
                reference.reset();
                pid.reset();
            }
            if (unit(rng) < 0.01) {
                const PIDGains gains = randomGains(rng);
                reference.setGains(gains);
                pid.setGains(gains);
            }
        }
        observer.steps.clear();
        observer.updatedPids.clear();
        observer.resetPids.clear();
    }
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
    testObserverSeesEveryStepWithItsTerms();
    testDerivativeOnMeasurementTerm();
    testDtFallbackFlag();
    testSaturatedAndIntegralHeldFlags();
    testSlewLimitedFlag();
    testOnPidResetOnlyWhenThereWasState();
    testLastStep();
    testConfigReflectsConstructionAndGains();
    testDetachingAndCopying();
    testMatchesTheOldUpdateBitForBit();
    std::puts("pid_test: all assertions passed");
    return 0;
}
