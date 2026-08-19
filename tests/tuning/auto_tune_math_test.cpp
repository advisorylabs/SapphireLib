// Host-side unit test for sapphirelib::tuning's overshoot measurement and
// Twiddle search — no PROS/embedded dependencies, so it builds and runs
// with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/tuning/auto_tune_math_test.cpp \
//       src/sapphirelib/tuning/auto_tune_math.cpp \
//       -o auto_tune_math_test && ./auto_tune_math_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sapphirelib/tuning/auto_tune_math.hpp"

using sapphirelib::PIDGains;
using sapphirelib::tuning::autoTuneCost;
using sapphirelib::tuning::currentGains;
using sapphirelib::tuning::makeTwiddleState;
using sapphirelib::tuning::measureOvershoot;
using sapphirelib::tuning::OvershootResult;
using sapphirelib::tuning::reportCost;
using sapphirelib::tuning::totalStepSize;
using sapphirelib::tuning::TwiddleState;

namespace {

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-6) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
        assert(false);
    }
}

void testOvershootPastIncreasingTarget() {
    // Drove toward 24in, peaked at 26 (2in overshoot), settled back to 24.
    const OvershootResult result = measureOvershoot({0.0, 10.0, 22.0, 26.0, 25.0, 24.0}, 24.0);
    expectNear(result.overshoot, 2.0, "increasing: overshoot");
    expectNear(result.finalError, 0.0, "increasing: finalError");
}

void testOvershootPastDecreasingTarget() {
    // Turned from 90 down toward 30, undershot to 25 (5deg overshoot past
    // 30 in the direction of travel), settled at 30.
    const OvershootResult result = measureOvershoot({90.0, 60.0, 25.0, 28.0, 30.0}, 30.0);
    expectNear(result.overshoot, 5.0, "decreasing: overshoot");
    expectNear(result.finalError, 0.0, "decreasing: finalError");
}

void testNoOvershootWhenMonotonic() {
    const OvershootResult result = measureOvershoot({0.0, 5.0, 12.0, 18.0, 24.0}, 24.0);
    expectNear(result.overshoot, 0.0, "monotonic: overshoot");
    expectNear(result.finalError, 0.0, "monotonic: finalError");
}

void testFinalErrorWhenTargetNotReached() {
    // Timed out short of the target — no overshoot, but a real final error.
    const OvershootResult result = measureOvershoot({0.0, 8.0, 15.0, 20.0}, 24.0);
    expectNear(result.overshoot, 0.0, "short: overshoot");
    expectNear(result.finalError, 4.0, "short: finalError");
}

void testAutoTuneCostWeighting() {
    const OvershootResult result{.overshoot = 2.0, .finalError = 0.5};
    expectNear(autoTuneCost(result, 4.0), 2.0 + 4.0 * 0.5, "cost weighting");
}

void testTwiddleNeverProposesNegativeGains() {
    // Starting near zero with a large step should clamp instead of going
    // negative — a negative gain flips the controller's correction
    // direction, which must never be a value the search proposes to try.
    TwiddleState state = makeTwiddleState(PIDGains{.kP = 0.05, .kI = 0.0, .kD = 0.0},
                                          PIDGains{.kP = 1.0, .kI = 1.0, .kD = 1.0});
    reportCost(state, 10.0); // baseline
    for (int i = 0; i < 20; ++i) {
        const PIDGains gains = currentGains(state);
        assert(gains.kP >= 0.0);
        assert(gains.kI >= 0.0);
        assert(gains.kD >= 0.0);
        // A cost landscape that always "improves" by decreasing pushes the
        // search toward (and clamped at) zero as hard as possible.
        reportCost(state, currentGains(state).kP + currentGains(state).kI + currentGains(state).kD);
    }
    std::puts("twiddle never proposes negative gains: passed");
}

void testTwiddleConvergesOnKnownMinimum() {
    // A synthetic, noiseless quadratic "cost landscape" with a known
    // minimum at (kP=1.2, kI=0.05, kD=0.3) — the same shape of problem the
    // real auto-tuner solves (find gains minimizing a scalar cost), just
    // with a cost function whose answer we already know, so convergence can
    // actually be checked.
    constexpr double kTargetP = 1.2;
    constexpr double kTargetI = 0.05;
    constexpr double kTargetD = 0.3;

    auto cost = [&](PIDGains g) {
        const double dp = g.kP - kTargetP;
        const double di = g.kI - kTargetI;
        const double dd = g.kD - kTargetD;
        return dp * dp + di * di + dd * dd;
    };

    TwiddleState state = makeTwiddleState(PIDGains{.kP = 0.5, .kI = 0.2, .kD = 0.1},
                                          PIDGains{.kP = 0.2, .kI = 0.05, .kD = 0.1});
    reportCost(state, cost(currentGains(state)));

    constexpr int kMaxIterations = 500;
    int iterations = 0;
    while (totalStepSize(state) > 1e-4 && iterations < kMaxIterations) {
        reportCost(state, cost(currentGains(state)));
        ++iterations;
    }

    const PIDGains finalGains = currentGains(state);
    constexpr double kTolerance = 0.02;
    if (std::fabs(finalGains.kP - kTargetP) >= kTolerance ||
        std::fabs(finalGains.kI - kTargetI) >= kTolerance ||
        std::fabs(finalGains.kD - kTargetD) >= kTolerance) {
        std::printf("FAIL twiddle convergence: got (%.4f, %.4f, %.4f) after %d iterations, expected near "
                    "(%.4f, %.4f, %.4f)\n",
                    finalGains.kP, finalGains.kI, finalGains.kD, iterations, kTargetP, kTargetI, kTargetD);
        assert(false);
    }
    std::printf("twiddle convergence: passed in %d iterations, final (%.4f, %.4f, %.4f)\n", iterations,
                finalGains.kP, finalGains.kI, finalGains.kD);
}

} // namespace

int main() {
    testOvershootPastIncreasingTarget();
    testOvershootPastDecreasingTarget();
    testNoOvershootWhenMonotonic();
    testFinalErrorWhenTargetNotReached();
    testAutoTuneCostWeighting();
    testTwiddleNeverProposesNegativeGains();
    testTwiddleConvergesOnKnownMinimum();
    std::puts("auto_tune_math_test: all assertions passed");
    return 0;
}
