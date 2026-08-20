/**
 * \file sapphirelib/tuning/auto_tune_math.hpp
 *
 * Ziegler-Nichols relay-feedback auto-tuning — pure math, no PROS
 * dependency, so it can be unit-tested on a desktop compiler (see
 * tests/tuning/auto_tune_math_test.cpp). tuning::runRelayExperiment() and
 * the gui::PidTunerPage auto-tune flow wrap this with the actual relay
 * actuation and sensor sampling.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <vector>

#include "sapphirelib/control/pid.hpp"

namespace sapphirelib::tuning {

/// One sample from a relay-feedback auto-tune experiment: the process
/// variable's reading (e.g. distance traveled, or heading) at a given
/// millisecond timestamp, both relative to the experiment's own start.
struct RelaySample {
    std::uint32_t timeMs = 0;
    double value = 0.0;
};

/// Averaged oscillation parameters extracted from a relay experiment's
/// samples by analyzeRelayOscillation(). `ok` is false if too few complete
/// cycles were found to trust periodMs/amplitude/cycleCount.
struct RelayOscillation {
    bool ok = false;
    double periodMs = 0.0;
    double amplitude = 0.0; // half the peak-to-peak swing
    int cycleCount = 0;
};

/// Finds complete oscillation cycles in `samples` (a time-ordered series of
/// the process variable recorded during a relay experiment — see
/// runRelayExperiment()) and averages their period/amplitude. A relay
/// experiment should run for several oscillation cycles; averaging across
/// all complete ones found evens out sensor noise and the settling
/// transient the first half-cycle usually includes.
///
/// Extrema (peaks and troughs) are found with a simple noise-floor filter:
/// a reversal only counts once the signal has moved at least `noiseFloor`
/// from the running extremum candidate, so single-sample sensor jitter
/// doesn't get mistaken for a real direction change. The default is
/// intentionally tiny (a hundredth of an inch or degree) — a relay
/// experiment's forced oscillation should swing far more than that
/// regardless of which axis (distance or heading) it's tuning.
///
/// Needs at least `minCycles` complete cycles to return ok=true — a relay
/// experiment that never oscillates (e.g. relay amplitude too small to
/// overcome friction, so the plant just sits at one relay extreme) can't
/// identify Ku/Tu at all, and this reports that honestly rather than
/// guessing from noise.
RelayOscillation analyzeRelayOscillation(const std::vector<RelaySample>& samples, int minCycles = 3,
                                         double noiseFloor = 0.01);

/// A plant's ultimate gain (Ku) and ultimate period (Tu) — the classic
/// Ziegler-Nichols closed-loop tuning method's two inputs.
struct UltimateParams {
    double ultimateGain = 0.0;
    double ultimatePeriodMs = 0.0;
};

/// Derives Ku/Tu from one relay experiment via the Åström–Hägglund
/// describing-function approximation: a relay of amplitude `relayAmplitude`
/// forces a roughly sinusoidal oscillation of amplitude
/// `oscillation.amplitude`, at which point the plant's effective gain at
/// that oscillation's frequency is `oscillation.amplitude / (4 *
/// relayAmplitude / pi)` — Ku is that gain's reciprocal, the proportional
/// gain at which pure-P control would sustain the same oscillation.
/// `oscillation` must be ok (see analyzeRelayOscillation()); returns a
/// zeroed UltimateParams otherwise.
UltimateParams ultimateParamsFromRelay(double relayAmplitude, const RelayOscillation& oscillation);

/// Classic Ziegler-Nichols closed-loop PID tuning formulas: kP = 0.6*Ku,
/// Ti = Tu/2 (so kI = kP/Ti = 1.2*Ku/Tu), Td = Tu/8 (so kD = kP*Td =
/// 0.075*Ku*Tu). Returns a zeroed PIDGains if `ultimate.ultimatePeriodMs`
/// is 0 (no valid period to derive kI/kD from).
PIDGains zieglerNicholsGains(UltimateParams ultimate);

} // namespace sapphirelib::tuning
