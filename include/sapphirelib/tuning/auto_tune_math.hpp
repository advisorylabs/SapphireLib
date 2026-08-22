/**
 * \file sapphirelib/tuning/auto_tune_math.hpp
 *
 * Relay-feedback auto-tuning — pure math, no PROS dependency, so it can be
 * unit-tested on a desktop compiler (see
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
/// Extrema (peaks and troughs) are found with a noise-floor filter: a
/// reversal only counts once the signal has moved at least the floor from
/// the running extremum candidate, so sensor jitter near a peak — where the
/// signal flattens out and noise is most likely to look like a real
/// direction change — doesn't get counted as an extra cycle.
///
/// The floor adapts to the signal rather than being a fixed absolute
/// number, because the right value depends entirely on how far the plant
/// actually swings, which isn't known until it's measured: a first pass
/// using `minNoiseFloor` estimates the swing, then a second pass runs with
/// a floor of `noiseFloorFraction` of that estimate (never below
/// `minNoiseFloor`). This is what lets the same defaults work for a heading
/// loop swinging several degrees and a drive loop swinging several inches.
///
/// The default fraction is deliberately generous. A floor that's too small
/// is silently catastrophic — noise around the flat top of each peak reads
/// as extra reversals, which shortens the reported period and shrinks the
/// reported amplitude together, and both errors push the derived gains too
/// aggressive while still reporting ok=true. A floor that's too large costs
/// essentially nothing by comparison, since a genuine cycle swings twice
/// the amplitude and clears any fraction this side of ~0.5 easily. Measured
/// against a synthetic 400ms/5-unit oscillation, 0.2 tracks the true period
/// to within 1% up through ~5% relative sensor noise, where the default of
/// a twentieth would already be off by more than half. Beyond roughly 8%
/// relative noise no floor recovers the signal — that's a plant that needs
/// a larger relay amplitude, and analyzeRelayOscillation() can't tell you
/// so, which is worth knowing when a tune comes out strangely hot.
///
/// Needs at least `minCycles` complete cycles to return ok=true — a relay
/// experiment that never oscillates (e.g. relay amplitude too small to
/// overcome friction, so the plant just sits at one relay extreme) can't
/// identify Ku/Tu at all, and this reports that honestly rather than
/// guessing from noise.
RelayOscillation analyzeRelayOscillation(const std::vector<RelaySample>& samples, int minCycles = 3,
                                         double minNoiseFloor = 0.01,
                                         double noiseFloorFraction = 0.2);

/// A plant's ultimate gain (Ku) and ultimate period (Tu) — the two inputs
/// every closed-loop tuning rule in gainsFromUltimate() is built on.
struct UltimateParams {
    double ultimateGain = 0.0;
    double ultimatePeriodMs = 0.0;
};

/// Derives Ku/Tu from one relay experiment via the Åström–Hägglund
/// describing-function approximation: a relay of amplitude `relayAmplitude`
/// forces a roughly sinusoidal oscillation of amplitude
/// `oscillation.amplitude`, at which point the plant's effective gain at
/// that oscillation's frequency is the reciprocal of Ku.
///
/// `relayHysteresis` is the relay's switching deadband
/// (RelayTuneConfig::hysteresis), in the same units as the oscillation's
/// amplitude. A relay with hysteresis switches late, which inflates the
/// observed swing; the describing function corrects for this by using
/// sqrt(a^2 - h^2) in place of the raw amplitude, so passing the real
/// hysteresis keeps Ku from coming out biased low (and every derived gain
/// with it). Ignored if it's zero or too large to be consistent with the
/// measured amplitude.
///
/// `oscillation` must be ok (see analyzeRelayOscillation()); returns a
/// zeroed UltimateParams otherwise.
UltimateParams ultimateParamsFromRelay(double relayAmplitude, const RelayOscillation& oscillation,
                                       double relayHysteresis = 0.0);

/// Which closed-loop tuning rule gainsFromUltimate() applies to Ku/Tu.
///
/// These differ only in how aggressively they trade settling speed against
/// overshoot. The classic Ziegler-Nichols rule targets quarter-amplitude
/// decay, which means roughly 25% overshoot by design — appropriate for the
/// process-control loops it was derived for, but usually the wrong trade
/// for a robot positioning loop, where overshooting a heading or a distance
/// and coming back costs more time than approaching it slightly slower.
/// That's why noOvershoot, not classicZN, is the default.
enum class TuningRule {
    /// kP = 0.6Ku, Ti = Tu/2, Td = Tu/8. Fast, ~25% overshoot.
    classicZN,
    /// Pessen "some overshoot": kP = 0.33Ku, Ti = Tu/2, Td = Tu/3.
    lessOvershoot,
    /// kP = 0.2Ku, Ti = Tu/2, Td = Tu/3. Slowest to settle, but approaches
    /// the target from one side — the sane default for drive/turn loops.
    noOvershoot,
    /// Classic Ziegler-Nichols PD: kP = 0.8Ku, Td = Tu/8, kI = 0. For loops
    /// where steady-state error isn't worth an integrator's risk at all.
    pdOnly,
};

/// Applies `rule` to a plant's measured Ku/Tu and returns the resulting
/// gains, in PIDGains' continuous-time convention (kI per second, kD per
/// second) — so they can be handed straight to PID::setGains() with no
/// per-loop-period conversion. Returns a zeroed PIDGains if
/// `ultimate.ultimatePeriodMs` is 0 (no valid period to derive kI/kD from).
PIDGains gainsFromUltimate(UltimateParams ultimate, TuningRule rule = TuningRule::noOvershoot);

/// gainsFromUltimate() with TuningRule::classicZN — the textbook
/// Ziegler-Nichols closed-loop formulas, kept as a named entry point since
/// they're the reference every other rule here is stated relative to.
PIDGains zieglerNicholsGains(UltimateParams ultimate);

} // namespace sapphirelib::tuning
