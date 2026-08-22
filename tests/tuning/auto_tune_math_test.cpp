// Host-side unit test for sapphirelib::tuning's Ziegler-Nichols relay
// auto-tune math — no PROS/embedded dependencies, so it builds and runs
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
using sapphirelib::tuning::analyzeRelayOscillation;
using sapphirelib::tuning::gainsFromUltimate;
using sapphirelib::tuning::TuningRule;
using sapphirelib::tuning::RelayOscillation;
using sapphirelib::tuning::RelaySample;
using sapphirelib::tuning::UltimateParams;
using sapphirelib::tuning::ultimateParamsFromRelay;
using sapphirelib::tuning::zieglerNicholsGains;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (std::fabs(actual - expected) >= tolerance) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
        assert(false);
    }
}

/// Synthesizes a clean relay-experiment sample series: a triangle wave with
/// the given period/amplitude/sample rate, the same shape a real relay
/// experiment's process variable traces out once it's settled into a
/// sustained oscillation.
std::vector<RelaySample> makeTriangleWave(double periodMs, double amplitude, int cycles,
                                          std::uint32_t samplePeriodMs) {
    std::vector<RelaySample> samples;
    const std::uint32_t totalMs = static_cast<std::uint32_t>(periodMs * cycles);
    for (std::uint32_t t = 0; t <= totalMs; t += samplePeriodMs) {
        // Triangle wave via a folded sawtooth: phase in [0, 1) maps to
        // value in [-amplitude, +amplitude], ramping up then down.
        const double phase = std::fmod(static_cast<double>(t), periodMs) / periodMs;
        const double value =
            phase < 0.5 ? (-amplitude + 4.0 * amplitude * phase) : (3.0 * amplitude - 4.0 * amplitude * phase);
        samples.push_back(RelaySample{.timeMs = t, .value = value});
    }
    return samples;
}

void testAnalyzeRelayOscillationOnCleanTriangleWave() {
    constexpr double kPeriodMs = 400.0;
    constexpr double kAmplitude = 3.0;
    const std::vector<RelaySample> samples = makeTriangleWave(kPeriodMs, kAmplitude, /*cycles=*/6, 10);

    const RelayOscillation result = analyzeRelayOscillation(samples);
    assert(result.ok);
    // A handful of samples' worth of quantization error is expected from the
    // 10ms sample grid discretizing the triangle wave's peaks/troughs.
    expectNear(result.periodMs, kPeriodMs, 15.0, "clean triangle: period");
    expectNear(result.amplitude, kAmplitude, 0.1, "clean triangle: amplitude");
    assert(result.cycleCount >= 3);
    std::puts("analyzeRelayOscillation on a clean triangle wave: passed");
}

void testAnalyzeRelayOscillationIgnoresNoise() {
    constexpr double kPeriodMs = 400.0;
    constexpr double kAmplitude = 3.0;
    std::vector<RelaySample> samples = makeTriangleWave(kPeriodMs, kAmplitude, /*cycles=*/6, 10);

    // Superimpose tiny alternating jitter well under the default noise
    // floor — a real sensor's read noise, which shouldn't be mistaken for
    // extra oscillation cycles.
    for (std::size_t i = 0; i < samples.size(); ++i) {
        samples[i].value += (i % 2 == 0) ? 0.001 : -0.001;
    }

    const RelayOscillation result = analyzeRelayOscillation(samples);
    assert(result.ok);
    expectNear(result.amplitude, kAmplitude, 0.1, "noisy triangle: amplitude");
    std::puts("analyzeRelayOscillation ignores sub-noise-floor jitter: passed");
}

/// Synthesizes a settled relay oscillation as a sine wave — a better model
/// than a triangle for what the noise floor has to cope with, since a real
/// plant decelerates smoothly into each reversal, leaving the signal nearly
/// flat around every peak and trough. `noise` is deterministic (a fixed
/// LCG), so this test can assert exact tolerances.
std::vector<RelaySample> makeNoisySineWave(double periodMs, double amplitude, int cycles,
                                           std::uint32_t samplePeriodMs, double noise) {
    std::vector<RelaySample> samples;
    const std::uint32_t totalMs = static_cast<std::uint32_t>(periodMs * cycles);
    std::uint32_t lcg = 12345;
    for (std::uint32_t t = 0; t <= totalMs; t += samplePeriodMs) {
        lcg = lcg * 1103515245u + 12345u;
        const double unitNoise = 2.0 * (static_cast<double>((lcg >> 16) & 0x7fff) / 32767.0) - 1.0;
        const double value = amplitude * std::sin(2.0 * kPi * static_cast<double>(t) / periodMs);
        samples.push_back(RelaySample{.timeMs = t, .value = value + noise * unitNoise});
    }
    return samples;
}

void testAnalyzeRelayOscillationSurvivesRealisticSensorNoise() {
    // +-0.15 units of read noise on a 5-unit swing — the scale of a real V5
    // IMU's heading jitter, and far above the 0.01 absolute floor. Around
    // each peak the true signal is nearly flat, so noise at this scale
    // *does* look like a real direction change to a fixed small floor:
    // measured directly below, a fixed floor reads this exact series as a
    // 194ms period and a 2.46 amplitude against a true 400ms/5.00, and
    // reports ok while doing it, so a caller gets no signal that anything
    // went wrong. Both errors push the derived gains too aggressive — a
    // halved amplitude doubles Ku, and a halved period halves Tu, which
    // inflates kI twice over. The adaptive floor scales to the swing it
    // actually measured, which is the only thing that can separate jitter
    // from signal here.
    constexpr double kPeriodMs = 400.0;
    constexpr double kAmplitude = 5.0;
    const std::vector<RelaySample> samples =
        makeNoisySineWave(kPeriodMs, kAmplitude, /*cycles=*/8, 10, /*noise=*/0.15);

    const RelayOscillation adaptive = analyzeRelayOscillation(samples);
    assert(adaptive.ok);
    expectNear(adaptive.periodMs, kPeriodMs, 20.0, "noisy sine: adaptive period");
    expectNear(adaptive.amplitude, kAmplitude, 0.3, "noisy sine: adaptive amplitude");

    // Same series, adaptation disabled (noiseFloorFraction = 0) — pins down
    // what the adaptive floor is actually buying, so this doesn't silently
    // become a test that would pass either way.
    const RelayOscillation fixedFloor =
        analyzeRelayOscillation(samples, /*minCycles=*/3, /*minNoiseFloor=*/0.01,
                                /*noiseFloorFraction=*/0.0);
    assert(fixedFloor.ok); // no failure reported, just wrong numbers
    assert(fixedFloor.periodMs < kPeriodMs * 0.9);
    assert(fixedFloor.amplitude < kAmplitude * 0.9);

    std::puts("analyzeRelayOscillation adapts its noise floor to the swing: passed");
}

void testAnalyzeRelayOscillationFailsWithoutEnoughCycles() {
    // Only two full cycles' worth of data, but minCycles defaults to 3.
    const std::vector<RelaySample> samples = makeTriangleWave(400.0, 3.0, /*cycles=*/2, 10);
    const RelayOscillation result = analyzeRelayOscillation(samples);
    assert(!result.ok);
    std::puts("analyzeRelayOscillation reports failure below minCycles: passed");
}

void testAnalyzeRelayOscillationFailsWithFlatSignal() {
    // The relay never actually moved the plant (e.g. amplitude too small to
    // overcome friction) — no extrema at all to measure.
    std::vector<RelaySample> samples;
    for (std::uint32_t t = 0; t <= 2000; t += 10) samples.push_back(RelaySample{.timeMs = t, .value = 0.0});
    const RelayOscillation result = analyzeRelayOscillation(samples);
    assert(!result.ok);
    std::puts("analyzeRelayOscillation reports failure on a flat signal: passed");
}

void testUltimateParamsFromRelay() {
    // Known describing-function relationship: Ku = 4d / (pi * a).
    constexpr double kRelayAmplitude = 4.0;
    const RelayOscillation oscillation{.ok = true, .periodMs = 500.0, .amplitude = 2.0, .cycleCount = 4};

    const UltimateParams ultimate = ultimateParamsFromRelay(kRelayAmplitude, oscillation);
    expectNear(ultimate.ultimateGain, (4.0 * kRelayAmplitude) / (kPi * oscillation.amplitude), 1e-9,
              "ultimate gain");
    expectNear(ultimate.ultimatePeriodMs, oscillation.periodMs, 1e-9, "ultimate period");
    std::puts("ultimateParamsFromRelay: passed");
}

void testUltimateParamsFromRelayCorrectsForHysteresis() {
    // A relay with a switching deadband overshoots each crossing before it
    // flips, so the raw swing overstates how weak the plant is. The
    // describing function accounts for that with sqrt(a^2 - h^2), which
    // must come out to a strictly higher Ku than the uncorrected form.
    constexpr double kRelayAmplitude = 4.0;
    constexpr double kHysteresis = 1.2;
    const RelayOscillation oscillation{.ok = true, .periodMs = 500.0, .amplitude = 2.0, .cycleCount = 4};

    const UltimateParams corrected =
        ultimateParamsFromRelay(kRelayAmplitude, oscillation, kHysteresis);
    const double effective =
        std::sqrt(oscillation.amplitude * oscillation.amplitude - kHysteresis * kHysteresis);
    expectNear(corrected.ultimateGain, (4.0 * kRelayAmplitude) / (kPi * effective), 1e-9,
              "hysteresis-corrected ultimate gain");
    assert(corrected.ultimateGain > ultimateParamsFromRelay(kRelayAmplitude, oscillation).ultimateGain);

    // A hysteresis that can't be reconciled with the measured swing falls
    // back to the uncorrected amplitude rather than rooting a negative.
    const UltimateParams fallback = ultimateParamsFromRelay(kRelayAmplitude, oscillation,
                                                            /*relayHysteresis=*/5.0);
    expectNear(fallback.ultimateGain, (4.0 * kRelayAmplitude) / (kPi * oscillation.amplitude), 1e-9,
              "hysteresis fallback ultimate gain");
    std::puts("ultimateParamsFromRelay corrects for relay hysteresis: passed");
}

void testUltimateParamsFromRelayRejectsFailedOscillation() {
    const RelayOscillation oscillation{.ok = false};
    const UltimateParams ultimate = ultimateParamsFromRelay(4.0, oscillation);
    expectNear(ultimate.ultimateGain, 0.0, 1e-9, "rejected: ultimate gain");
    expectNear(ultimate.ultimatePeriodMs, 0.0, 1e-9, "rejected: ultimate period");
    std::puts("ultimateParamsFromRelay rejects a failed oscillation: passed");
}

void testZieglerNicholsGains() {
    // Known values: Ku=10, Tu=1000ms (1s) -> kP=6, Ti=0.5s -> kI=12,
    // Td=0.125s -> kD=0.75.
    const UltimateParams ultimate{.ultimateGain = 10.0, .ultimatePeriodMs = 1000.0};
    const PIDGains gains = zieglerNicholsGains(ultimate);
    expectNear(gains.kP, 6.0, 1e-9, "ZN gains: kP");
    expectNear(gains.kI, 12.0, 1e-9, "ZN gains: kI");
    expectNear(gains.kD, 0.75, 1e-9, "ZN gains: kD");
    std::puts("zieglerNicholsGains: passed");
}

void testTuningRulesGetProgressivelyGentler() {
    // Ku=10, Tu=1000ms. Every rule shares the same Ku/Tu, so the ordering
    // of their kP values is the whole point: less proportional gain means
    // less overshoot and a slower approach.
    const UltimateParams ultimate{.ultimateGain = 10.0, .ultimatePeriodMs = 1000.0};

    const PIDGains classic = gainsFromUltimate(ultimate, TuningRule::classicZN);
    const PIDGains less = gainsFromUltimate(ultimate, TuningRule::lessOvershoot);
    const PIDGains none = gainsFromUltimate(ultimate, TuningRule::noOvershoot);
    assert(classic.kP > less.kP && less.kP > none.kP);

    // lessOvershoot: kP = 0.33*10 = 3.3, Ti = 0.5s -> kI = 6.6,
    // Td = 1/3s -> kD = 1.1.
    expectNear(less.kP, 3.3, 1e-9, "lessOvershoot: kP");
    expectNear(less.kI, 6.6, 1e-9, "lessOvershoot: kI");
    expectNear(less.kD, 1.1, 1e-9, "lessOvershoot: kD");

    // noOvershoot: kP = 0.2*10 = 2, Ti = 0.5s -> kI = 4, Td = 1/3s -> kD = 2/3.
    expectNear(none.kP, 2.0, 1e-9, "noOvershoot: kP");
    expectNear(none.kI, 4.0, 1e-9, "noOvershoot: kI");
    expectNear(none.kD, 2.0 / 3.0, 1e-9, "noOvershoot: kD");

    std::puts("gainsFromUltimate rule ordering and values: passed");
}

void testPdOnlyRuleHasNoIntegralTerm() {
    const UltimateParams ultimate{.ultimateGain = 10.0, .ultimatePeriodMs = 1000.0};
    const PIDGains gains = gainsFromUltimate(ultimate, TuningRule::pdOnly);
    expectNear(gains.kP, 8.0, 1e-9, "pdOnly: kP");   // 0.8 * Ku
    expectNear(gains.kI, 0.0, 1e-9, "pdOnly: kI");   // no integrator at all
    expectNear(gains.kD, 1.0, 1e-9, "pdOnly: kD");   // kP * Tu/8
    std::puts("gainsFromUltimate pdOnly drops the integral term: passed");
}

void testDefaultRuleIsNoOvershoot() {
    // The default matters: classic Ziegler-Nichols targets ~25% overshoot,
    // which is the wrong trade for a drive/turn positioning loop.
    const UltimateParams ultimate{.ultimateGain = 10.0, .ultimatePeriodMs = 1000.0};
    const PIDGains defaulted = gainsFromUltimate(ultimate);
    const PIDGains explicitRule = gainsFromUltimate(ultimate, TuningRule::noOvershoot);
    expectNear(defaulted.kP, explicitRule.kP, 1e-9, "default rule: kP");
    expectNear(defaulted.kI, explicitRule.kI, 1e-9, "default rule: kI");
    expectNear(defaulted.kD, explicitRule.kD, 1e-9, "default rule: kD");
    std::puts("gainsFromUltimate defaults to the no-overshoot rule: passed");
}

void testZieglerNicholsGainsZeroedWithoutValidPeriod() {
    const PIDGains gains = zieglerNicholsGains(UltimateParams{});
    expectNear(gains.kP, 0.0, 1e-9, "zeroed: kP");
    expectNear(gains.kI, 0.0, 1e-9, "zeroed: kI");
    expectNear(gains.kD, 0.0, 1e-9, "zeroed: kD");
    std::puts("zieglerNicholsGains zeroed without a valid period: passed");
}

} // namespace

int main() {
    testAnalyzeRelayOscillationOnCleanTriangleWave();
    testAnalyzeRelayOscillationIgnoresNoise();
    testAnalyzeRelayOscillationSurvivesRealisticSensorNoise();
    testAnalyzeRelayOscillationFailsWithoutEnoughCycles();
    testAnalyzeRelayOscillationFailsWithFlatSignal();
    testUltimateParamsFromRelay();
    testUltimateParamsFromRelayCorrectsForHysteresis();
    testUltimateParamsFromRelayRejectsFailedOscillation();
    testZieglerNicholsGains();
    testTuningRulesGetProgressivelyGentler();
    testPdOnlyRuleHasNoIntegralTerm();
    testDefaultRuleIsNoOvershoot();
    testZieglerNicholsGainsZeroedWithoutValidPeriod();
    std::puts("auto_tune_math_test: all assertions passed");
    return 0;
}
