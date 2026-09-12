// Host-side unit test for sapphirelib::tuning's system identification math
// — no PROS/embedded dependencies, so it builds and runs with a normal
// desktop compiler.
//
// Every test drives a simulated axis with known kS/kV/kA and delay through
// the same ramps and steps tuning::runCharacterization() uses on the robot,
// then checks the fit recovers what the simulation was built with.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/tuning/characterization_math_test.cpp \
//       src/sapphirelib/tuning/characterization_math.cpp \
//       src/sapphirelib/control/feedforward.cpp \
//       -o characterization_math_test && ./characterization_math_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <vector>

#include "sapphirelib/tuning/characterization_math.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::tuning::AxisCharacterization;
using sapphirelib::tuning::characterizeAxis;
using sapphirelib::tuning::CharacterizationData;
using sapphirelib::tuning::CharacterizationRun;
using sapphirelib::tuning::CharacterizationSample;
using sapphirelib::tuning::estimateResponseDelayS;
using sapphirelib::tuning::fitFeedforward;

namespace {

constexpr double kTickS = 0.01;
constexpr int kSubsteps = 20;

/// A forward-axis-sized plant: about 78 in/s at 12V.
constexpr MotorFeedforward kForwardTruth{.kS = 1.1, .kV = 0.14, .kA = 0.035};

/// A turn-axis-sized plant: about 238 deg/s at 12V.
constexpr MotorFeedforward kTurnTruth{.kS = 1.3, .kV = 0.045, .kA = 0.006};

double signOf(double value) { return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0); }

void expectWithin(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %f, expected %f +- %f\n", label, actual, expected, tolerance);
        assert(false);
    }
}

void expectRelative(double actual, double expected, double fraction, const char* label) {
    expectWithin(actual, expected, std::fabs(expected) * fraction, label);
}

void expectTrue(bool condition, const char* label) {
    if (!condition) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

/// Simulates `ticks` 10ms ticks of an axis obeying `truth`, runner-style:
/// read position, then command. Each command reaches the axis
/// `delayTicks` ticks later. Static friction holds the axis still until the
/// applied voltage exceeds kS. `sign` flips the measurement, to model a
/// sensor mounted backwards.
CharacterizationRun simulate(const MotorFeedforward& truth, int delayTicks,
                             const std::function<double(double)>& voltsAt, int ticks,
                             double noise, std::mt19937& rng, double sign = 1.0) {
    std::normal_distribution<double> jitter(0.0, noise > 0.0 ? noise : 1.0);
    std::vector<double> commands;
    CharacterizationRun run;
    double x = 0.0;
    double v = 0.0;

    for (int i = 0; i < ticks; ++i) {
        const double volts = voltsAt(i * kTickS);
        const double reading = sign * x + (noise > 0.0 ? jitter(rng) : 0.0);
        run.push_back(CharacterizationSample{
            .timeMs = static_cast<std::uint32_t>(i * 10), .volts = volts, .position = reading});
        commands.push_back(volts);

        const double applied = i - delayTicks >= 0 ? commands[i - delayTicks] : 0.0;
        const double h = kTickS / kSubsteps;
        for (int s = 0; s < kSubsteps; ++s) {
            const double friction = std::fabs(v) > 1e-3
                                        ? truth.kS * signOf(v)
                                        : signOf(applied) * std::min(std::fabs(applied), truth.kS);
            v += (applied - friction - truth.kV * v) / truth.kA * h;
            x += v * h;
        }
    }
    return run;
}

/// The runner's schedule: a ramp each way, then a step each way, each with a
/// 100ms pre-roll at 0V.
CharacterizationData collect(const MotorFeedforward& truth, int delayTicks, double noise,
                             unsigned seed) {
    std::mt19937 rng(seed);
    const auto ramp = [](double direction) {
        return [direction](double t) {
            return t < 0.1 ? 0.0 : direction * std::min(4.0 * (t - 0.1), 8.0);
        };
    };
    const auto step = [](double direction) {
        return [direction](double t) { return t < 0.1 ? 0.0 : direction * 6.0; };
    };

    CharacterizationData data;
    data.ramps.push_back(simulate(truth, delayTicks, ramp(1.0), 220, noise, rng));
    data.ramps.push_back(simulate(truth, delayTicks, ramp(-1.0), 220, noise, rng));
    data.steps.push_back(simulate(truth, delayTicks, step(1.0), 100, noise, rng));
    data.steps.push_back(simulate(truth, delayTicks, step(-1.0), 100, noise, rng));
    return data;
}

void expectRecovers(const AxisCharacterization& result, const MotorFeedforward& truth,
                    double delayS, const char* label) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s: ok", label);
    expectTrue(result.ok, buf);
    std::snprintf(buf, sizeof(buf), "%s: kS", label);
    expectRelative(result.fit.model.kS, truth.kS, 0.12, buf);
    std::snprintf(buf, sizeof(buf), "%s: kV", label);
    expectRelative(result.fit.model.kV, truth.kV, 0.03, buf);
    std::snprintf(buf, sizeof(buf), "%s: kA", label);
    expectRelative(result.fit.model.kA, truth.kA, 0.08, buf);
    std::snprintf(buf, sizeof(buf), "%s: delay", label);
    expectWithin(result.delayS, delayS, 0.01, buf);
    std::snprintf(buf, sizeof(buf), "%s: R2", label);
    expectTrue(result.fit.rSquared > 0.95, buf);
}

void testRecoversACleanAxis() {
    expectRecovers(characterizeAxis(collect(kForwardTruth, 0, 0.0, 1), 2.0), kForwardTruth, 0.0,
                   "clean, no delay");
}

void testRecoversAnAxisWithDelay() {
    expectRecovers(characterizeAxis(collect(kForwardTruth, 3, 0.0, 1), 2.0), kForwardTruth, 0.03,
                   "clean, 30ms delay");
}

void testRecoversANoisyAxisWithDelay() {
    // Several seeds, since a single lucky noise draw proves little.
    for (unsigned seed = 1; seed <= 5; ++seed) {
        expectRecovers(characterizeAxis(collect(kForwardTruth, 3, 0.005, seed), 2.0),
                       kForwardTruth, 0.03, "0.005in noise, 30ms delay");
    }
}

void testRecoversATurnSizedAxis() {
    expectRecovers(characterizeAxis(collect(kTurnTruth, 4, 0.0, 1), 5.0), kTurnTruth, 0.04,
                   "turn axis, 40ms delay");
}

void testIgnoringDelayWouldHaveBiasedTheFit() {
    // The reason characterizeAxis() bothers with a second pass: fitting
    // delayed data as if it weren't. This pins down that the unshifted fit
    // really is worse, so the second pass is doing real work.
    const CharacterizationData data = collect(kForwardTruth, 6, 0.0, 1);
    std::vector<CharacterizationRun> all = data.ramps;
    all.insert(all.end(), data.steps.begin(), data.steps.end());

    const double naiveError =
        std::fabs(fitFeedforward(all, 2.0, 5, 0).model.kS - kForwardTruth.kS);
    const double shiftedError =
        std::fabs(fitFeedforward(all, 2.0, 5, 6).model.kS - kForwardTruth.kS);
    expectTrue(shiftedError < naiveError, "shifting by the delay improves kS");
}

void testStationaryAxisFails() {
    // Voltage too low to break friction: nothing moves, nothing to fit.
    std::mt19937 rng(1);
    CharacterizationData data;
    const auto weak = [](double t) { return t < 0.1 ? 0.0 : 0.8; };
    data.ramps.push_back(simulate(kForwardTruth, 0, weak, 200, 0.001, rng));
    data.steps.push_back(simulate(kForwardTruth, 0, weak, 100, 0.001, rng));

    const AxisCharacterization result = characterizeAxis(data, 2.0);
    expectTrue(!result.ok, "stationary axis is rejected");
    expectTrue(result.fit.samplesUsed < 20, "and reports why");
}

void testBackwardsSensorFails() {
    // Positive volts making the reading shrink can't be a physical axis; the
    // fit must refuse rather than hand back negative gains.
    std::mt19937 rng(1);
    CharacterizationData data;
    const auto ramp = [](double t) { return t < 0.1 ? 0.0 : std::min(4.0 * (t - 0.1), 8.0); };
    const auto step = [](double t) { return t < 0.1 ? 0.0 : 6.0; };
    data.ramps.push_back(simulate(kForwardTruth, 0, ramp, 220, 0.0, rng, -1.0));
    data.steps.push_back(simulate(kForwardTruth, 0, step, 100, 0.0, rng, -1.0));

    expectTrue(!characterizeAxis(data, 2.0).ok, "reversed sensor is rejected");
}

void testDelayNeedsAStep() {
    const CharacterizationRun idle(50, CharacterizationSample{});
    expectTrue(estimateResponseDelayS(idle, kForwardTruth) < 0.0, "no step, no delay estimate");
    expectTrue(estimateResponseDelayS(idle, MotorFeedforward{}) < 0.0, "invalid model");
}

} // namespace

int main() {
    testRecoversACleanAxis();
    testRecoversAnAxisWithDelay();
    testRecoversANoisyAxisWithDelay();
    testRecoversATurnSizedAxis();
    testIgnoringDelayWouldHaveBiasedTheFit();
    testStationaryAxisFails();
    testBackwardsSensorFails();
    testDelayNeedsAStep();
    std::puts("characterization_math_test: all assertions passed");
    return 0;
}
