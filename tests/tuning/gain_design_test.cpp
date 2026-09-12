// Host-side unit test for sapphirelib::tuning's pole-placement gain design
// — no PROS/embedded dependencies, so it builds and runs with a normal
// desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/tuning/gain_design_test.cpp \
//       src/sapphirelib/tuning/gain_design.cpp \
//       src/sapphirelib/control/feedforward.cpp \
//       -o gain_design_test && ./gain_design_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sapphirelib/tuning/gain_design.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::PIDGains;
using sapphirelib::tuning::designPositionGains;
using sapphirelib::tuning::GainDesign;
using sapphirelib::tuning::normalizedSettleTime;
using sapphirelib::tuning::phaseMarginDeg;
using sapphirelib::tuning::ResponseSpec;

namespace {

constexpr MotorFeedforward kModel{.kS = 1.1, .kV = 0.14, .kA = 0.035};

void expectWithin(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %f, expected %f +- %f\n", label, actual, expected, tolerance);
        assert(false);
    }
}

void expectTrue(bool condition, const char* label) {
    if (!condition) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

struct StepResult {
    double overshootFraction;
    double settleTimeS;
};

/// Closes the designed PD loop around a frictionless simulated axis the way
/// the drivetrains do — 10ms ticks, derivative on error, output clamped to
/// 12V, each command reaching the axis `delayTicks` ticks late — and
/// measures a small (unsaturated) step.
StepResult simulateStep(const MotorFeedforward& truth, PIDGains gains, int delayTicks,
                        double stepSize) {
    constexpr double kTickS = 0.01;
    constexpr int kSubsteps = 20;
    std::vector<double> commands;
    double x = 0.0;
    double v = 0.0;
    double prevError = 0.0;
    double peak = 0.0;
    double lastOutsideS = 0.0;

    for (int i = 0; i < 400; ++i) {
        const double error = stepSize - x;
        const double derivative = i == 0 ? 0.0 : (error - prevError) / kTickS;
        prevError = error;
        commands.push_back(std::clamp(gains.kP * error + gains.kD * derivative, -12.0, 12.0));

        peak = std::max(peak, x);
        if (std::fabs(stepSize - x) > 0.02 * stepSize) lastOutsideS = i * kTickS;

        const double applied = i - delayTicks >= 0 ? commands[i - delayTicks] : 0.0;
        for (int s = 0; s < kSubsteps; ++s) {
            v += (applied - truth.kV * v) / truth.kA * (kTickS / kSubsteps);
            x += v * (kTickS / kSubsteps);
        }
    }
    return StepResult{(peak - stepSize) / stepSize, lastOutsideS + kTickS};
}

void testNormalizedSettleTime() {
    // Critical damping's 2% settle solves (1 + t)·e^−t = 0.02.
    const double critical = normalizedSettleTime(1.0);
    expectWithin((1.0 + critical) * std::exp(-critical), 0.02, 1e-4, "critical damping");
    expectWithin(normalizedSettleTime(0.5), 8.08, 0.02, "underdamped rings longer");
    expectTrue(normalizedSettleTime(2.0) > critical, "overdamped creeps in slower");
    expectTrue(normalizedSettleTime(0.7) > critical * 0.95,
               "slight underdamping is about as fast as it gets");
}

void testPolePlacementFormulas() {
    const GainDesign design = designPositionGains(kModel, ResponseSpec{.settleTimeS = 0.6});
    expectTrue(design.ok, "valid design");
    expectTrue(!design.limitedByDelay, "no delay, nothing limits it");

    const double omega = normalizedSettleTime(1.0) / 0.6;
    expectWithin(design.naturalFrequency, omega, 1e-9, "natural frequency from settle time");
    expectWithin(design.gains.kP, kModel.kA * omega * omega, 1e-9, "kP = kA·ω²");
    expectWithin(design.gains.kD, 2.0 * omega * kModel.kA - kModel.kV, 1e-9, "kD = 2ζω·kA − kV");
    expectWithin(design.gains.kI, 0.0, 0.0, "no integral");
    expectWithin(design.settleTimeS, 0.6, 1e-9, "achieves the requested settle time");
}

void testNegativeKdIsClamped() {
    // kV this large already overdamps a slow response on its own.
    const MotorFeedforward sluggish{.kS = 0.5, .kV = 2.0, .kA = 0.01};
    const GainDesign design = designPositionGains(sluggish, ResponseSpec{.settleTimeS = 2.0});
    expectTrue(design.ok, "still a valid design");
    expectWithin(design.gains.kD, 0.0, 0.0, "kD clamped at 0");
}

void testDelayLimitsTheDesign() {
    const ResponseSpec aggressive{.settleTimeS = 0.3, .minPhaseMarginDeg = 50.0};
    const GainDesign design = designPositionGains(kModel, aggressive, 0.05);
    expectTrue(design.ok, "slowed, but still a valid design");
    expectTrue(design.limitedByDelay, "50ms of delay can't support a 0.3s settle");
    expectTrue(design.settleTimeS > 0.3, "reports the slower settle time");
    expectWithin(design.phaseMarginDeg, 50.0, 0.5, "backs off to just the margin asked for");
}

void testPhaseMarginFallsWithDelay() {
    const PIDGains gains{.kP = 3.0, .kI = 0.0, .kD = 0.5};
    const double none = phaseMarginDeg(kModel, gains, 0.0);
    const double some = phaseMarginDeg(kModel, gains, 0.03);
    const double lots = phaseMarginDeg(kModel, gains, 0.2);
    expectTrue(none > some && some > lots, "more delay, less margin");
    expectTrue(lots < 0.0, "enough delay makes it unstable");
    expectWithin(phaseMarginDeg(kModel, PIDGains{}, 0.05), 180.0, 0.0, "no gain, no crossover");
}

void testDesignBehavesInClosedLoop() {
    // The point of all of it: the designed gains, run the way the robot runs
    // them, settle when promised without overshooting.
    struct Case {
        double settleTimeS;
        int delayTicks;
    };
    const MotorFeedforward frictionless{.kS = 0.0, .kV = kModel.kV, .kA = kModel.kA};
    for (const Case c : {Case{0.6, 0}, Case{0.6, 3}, Case{0.3, 5}}) {
        const GainDesign design = designPositionGains(
            frictionless, ResponseSpec{.settleTimeS = c.settleTimeS}, c.delayTicks * 0.01);
        const StepResult step = simulateStep(frictionless, design.gains, c.delayTicks, 2.0);
        expectTrue(step.overshootFraction < 0.01, "no overshoot at critical damping");
        expectWithin(step.settleTimeS, design.settleTimeS, 0.15 * design.settleTimeS,
                     "settles when the design says it will");
    }
}

void testRejectsNonsense() {
    expectTrue(!designPositionGains(MotorFeedforward{}, ResponseSpec{}).ok, "unmeasured model");
    expectTrue(!designPositionGains(kModel, ResponseSpec{.settleTimeS = 0.0}).ok,
               "zero settle time");
}

} // namespace

int main() {
    testNormalizedSettleTime();
    testPolePlacementFormulas();
    testNegativeKdIsClamped();
    testDelayLimitsTheDesign();
    testPhaseMarginFallsWithDelay();
    testDesignBehavesInClosedLoop();
    testRejectsNonsense();
    std::puts("gain_design_test: all assertions passed");
    return 0;
}
