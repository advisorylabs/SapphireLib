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
    testAnalyzeRelayOscillationFailsWithoutEnoughCycles();
    testAnalyzeRelayOscillationFailsWithFlatSignal();
    testUltimateParamsFromRelay();
    testUltimateParamsFromRelayRejectsFailedOscillation();
    testZieglerNicholsGains();
    testZieglerNicholsGainsZeroedWithoutValidPeriod();
    std::puts("auto_tune_math_test: all assertions passed");
    return 0;
}
