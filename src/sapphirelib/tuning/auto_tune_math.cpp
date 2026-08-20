#include "sapphirelib/tuning/auto_tune_math.hpp"

#include <cmath>

namespace sapphirelib::tuning {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Extremum {
    std::uint32_t timeMs;
    double value;
    bool isPeak;
};

/// Zig-zag extrema finder: tracks the running high (while rising) or low
/// (while falling) and commits it as a peak/trough once the signal reverses
/// by at least `noiseFloor`. Extrema returned strictly alternate peak/
/// trough by construction.
std::vector<Extremum> findExtrema(const std::vector<RelaySample>& samples, double noiseFloor) {
    std::vector<Extremum> extrema;
    if (samples.size() < 2) return extrema;

    std::size_t extremeIdx = 0;
    bool directionKnown = false;
    bool rising = false;

    for (std::size_t i = 1; i < samples.size(); ++i) {
        if (!directionKnown) {
            const double delta = samples[i].value - samples[extremeIdx].value;
            if (std::fabs(delta) < noiseFloor) continue;
            rising = delta > 0.0;
            directionKnown = true;
            extremeIdx = i;
            continue;
        }

        if (rising) {
            if (samples[i].value >= samples[extremeIdx].value) {
                extremeIdx = i;
            } else if (samples[extremeIdx].value - samples[i].value >= noiseFloor) {
                extrema.push_back(Extremum{
                    samples[extremeIdx].timeMs, samples[extremeIdx].value, /*isPeak=*/true});
                rising = false;
                extremeIdx = i;
            }
        } else {
            if (samples[i].value <= samples[extremeIdx].value) {
                extremeIdx = i;
            } else if (samples[i].value - samples[extremeIdx].value >= noiseFloor) {
                extrema.push_back(Extremum{
                    samples[extremeIdx].timeMs, samples[extremeIdx].value, /*isPeak=*/false});
                rising = true;
                extremeIdx = i;
            }
        }
    }

    return extrema;
}

} // namespace

RelayOscillation analyzeRelayOscillation(const std::vector<RelaySample>& samples, int minCycles,
                                         double noiseFloor) {
    const std::vector<Extremum> extrema = findExtrema(samples, noiseFloor);

    RelayOscillation result;

    // Each same-side extremum pair (peak-to-peak or trough-to-trough) with
    // the opposite-side extremum between them gives one full cycle's period
    // and amplitude.
    int cycles = 0;
    double periodSumMs = 0.0;
    double amplitudeSum = 0.0;
    for (std::size_t i = 2; i < extrema.size(); ++i) {
        const double periodMs = static_cast<double>(extrema[i].timeMs - extrema[i - 2].timeMs);
        const double swing1 = std::fabs(extrema[i - 1].value - extrema[i - 2].value);
        const double swing2 = std::fabs(extrema[i].value - extrema[i - 1].value);
        // Average of the two half-swings (peak-to-trough, trough-to-peak),
        // halved again: each half-swing already spans a full peak-to-peak
        // range, so this is the oscillation's amplitude (half its
        // peak-to-peak extent).
        amplitudeSum += (swing1 + swing2) / 4.0;
        periodSumMs += periodMs;
        ++cycles;
    }

    if (cycles < minCycles) return result;

    result.ok = true;
    result.cycleCount = cycles;
    result.periodMs = periodSumMs / cycles;
    result.amplitude = amplitudeSum / cycles;
    return result;
}

UltimateParams ultimateParamsFromRelay(double relayAmplitude, const RelayOscillation& oscillation) {
    if (!oscillation.ok || oscillation.amplitude <= 0.0) return UltimateParams{};
    return UltimateParams{
        .ultimateGain = (4.0 * relayAmplitude) / (kPi * oscillation.amplitude),
        .ultimatePeriodMs = oscillation.periodMs,
    };
}

PIDGains zieglerNicholsGains(UltimateParams ultimate) {
    if (ultimate.ultimatePeriodMs <= 0.0) return PIDGains{};

    const double tuS = ultimate.ultimatePeriodMs / 1000.0;
    const double kp = 0.6 * ultimate.ultimateGain;
    return PIDGains{
        .kP = kp,
        .kI = 1.2 * ultimate.ultimateGain / tuS,
        .kD = kp * (tuS / 8.0),
    };
}

} // namespace sapphirelib::tuning
