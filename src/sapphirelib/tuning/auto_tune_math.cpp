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

/// Averages the period/amplitude of every complete cycle in an already-
/// extracted extrema series. Each same-side extremum pair (peak-to-peak or
/// trough-to-trough) with the opposite-side extremum between them gives one
/// full cycle.
RelayOscillation averageCycles(const std::vector<Extremum>& extrema, int minCycles) {
    RelayOscillation result;

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

} // namespace

RelayOscillation analyzeRelayOscillation(const std::vector<RelaySample>& samples, int minCycles,
                                         double minNoiseFloor, double noiseFloorFraction) {
    // First pass at the caller's floor, only to find out roughly how far
    // this plant swings — see the header comment for why the floor can't
    // just be a fixed constant.
    const RelayOscillation coarse =
        averageCycles(findExtrema(samples, minNoiseFloor), /*minCycles=*/1);

    double noiseFloor = minNoiseFloor;
    if (coarse.ok && noiseFloorFraction > 0.0) {
        noiseFloor = std::fmax(minNoiseFloor, coarse.amplitude * noiseFloorFraction);
    }

    return averageCycles(findExtrema(samples, noiseFloor), minCycles);
}

UltimateParams ultimateParamsFromRelay(double relayAmplitude, const RelayOscillation& oscillation,
                                       double relayHysteresis) {
    if (!oscillation.ok || oscillation.amplitude <= 0.0) return UltimateParams{};

    // Describing-function amplitude, corrected for the relay's switching
    // deadband (see the header comment). Falls back to the uncorrected
    // amplitude if the hysteresis isn't consistent with the measured swing,
    // which would otherwise put a negative number under the square root.
    double effectiveAmplitude = oscillation.amplitude;
    const double h = std::fabs(relayHysteresis);
    if (h > 0.0 && h < oscillation.amplitude) {
        effectiveAmplitude = std::sqrt(oscillation.amplitude * oscillation.amplitude - h * h);
    }

    return UltimateParams{
        .ultimateGain = (4.0 * relayAmplitude) / (kPi * effectiveAmplitude),
        .ultimatePeriodMs = oscillation.periodMs,
    };
}

PIDGains gainsFromUltimate(UltimateParams ultimate, TuningRule rule) {
    if (ultimate.ultimatePeriodMs <= 0.0) return PIDGains{};

    const double ku = ultimate.ultimateGain;
    const double tuS = ultimate.ultimatePeriodMs / 1000.0;

    // Every rule below is stated as (kP factor, integral time Ti, derivative
    // time Td), the form the tuning literature uses; the conversion to the
    // kI/kD this library's PIDGains carries is the same for all of them.
    double kpFactor = 0.0;
    double tiS = 0.0; // 0 means "no integral term"
    double tdS = 0.0;
    switch (rule) {
        case TuningRule::classicZN:
            kpFactor = 0.6;
            tiS = tuS / 2.0;
            tdS = tuS / 8.0;
            break;
        case TuningRule::lessOvershoot:
            kpFactor = 0.33;
            tiS = tuS / 2.0;
            tdS = tuS / 3.0;
            break;
        case TuningRule::noOvershoot:
            kpFactor = 0.2;
            tiS = tuS / 2.0;
            tdS = tuS / 3.0;
            break;
        case TuningRule::pdOnly:
            kpFactor = 0.8;
            tiS = 0.0;
            tdS = tuS / 8.0;
            break;
    }

    const double kp = kpFactor * ku;
    return PIDGains{
        .kP = kp,
        .kI = tiS > 0.0 ? kp / tiS : 0.0,
        .kD = kp * tdS,
    };
}

PIDGains zieglerNicholsGains(UltimateParams ultimate) {
    return gainsFromUltimate(ultimate, TuningRule::classicZN);
}

} // namespace sapphirelib::tuning
