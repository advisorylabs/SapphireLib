/**
 * \file sapphirelib/tuning/auto_tune_runner.hpp
 *
 * Runs a Ziegler-Nichols relay-feedback experiment — the piece
 * analyzeRelayOscillation() needs that the pure math half of
 * sapphirelib::tuning can't provide on its own.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "sapphirelib/tuning/auto_tune_math.hpp"

namespace sapphirelib::tuning {

/// Configuration for one relay-feedback auto-tune experiment — see
/// runRelayExperiment().
struct RelayTuneConfig {
    /// Applies a raw open-loop output directly to the plant, bypassing the
    /// PID being tuned entirely. Same units as that PID's own output (e.g.
    /// volts).
    std::function<void(double)> actuate;

    /// Reads the current process variable, in the same units as the PID's
    /// target/measurement (e.g. inches of distance traveled along a fixed
    /// reference direction, or *cumulative*, unwrapped heading degrees —
    /// see sensors::Imu::getCumulativeHeadingDeg() — so the relay's
    /// target-past-start-by-setpointDelta math doesn't break at the 0/360
    /// seam the way a raw wrapped heading would).
    std::function<double()> measure;

    /// Relay output magnitude — the only thing bounding how hard the
    /// experiment drives the plant, so keep this modest relative to the
    /// PID's own full output range. This is what makes relay tuning safe
    /// where a naive "keep raising kP until it oscillates" gain sweep
    /// isn't: the drive strength is fixed up front instead of climbing
    /// toward instability.
    double relayAmplitude = 0.0;

    /// How far past measure()'s reading at the start of the experiment to
    /// place the relay's setpoint — the experiment drives measure() back
    /// and forth across (start + setpointDelta).
    double setpointDelta = 0.0;

    /// Deadband around the setpoint, in measure()'s units, within which the
    /// relay holds its current sign instead of switching — filters out
    /// switching chatter from sensor noise right at the crossing point.
    /// Keep this small relative to setpointDelta.
    double hysteresis = 0.0;

    /// Hard cap on the experiment's duration — the failsafe against a
    /// relay that never settles into a clean oscillation (e.g. amplitude
    /// too small to overcome friction, so the plant just sits at one relay
    /// extreme).
    std::uint32_t timeoutMs = 8000;

    std::uint32_t samplePeriodMs = 10;
};

/// Runs a relay-feedback experiment (see gui::PidTunerPage's class comment
/// for the overall Ziegler-Nichols flow this feeds): drives
/// `config.actuate` with +-config.relayAmplitude based on which side of the
/// setpoint (config.measure()'s starting reading + config.setpointDelta)
/// config.measure() is currently on, sampling every config.samplePeriodMs,
/// until config.timeoutMs elapses. Always calls `config.actuate(0.0)`
/// before returning, even on timeout. Returns every sample collected, in
/// time order (timestamps relative to the experiment's own start) — feed
/// these into tuning::analyzeRelayOscillation().
std::vector<RelaySample> runRelayExperiment(const RelayTuneConfig& config);

} // namespace sapphirelib::tuning
