/**
 * \file sapphirelib/tuning/characterization_math.hpp
 *
 * System identification for one drive axis — pure math, no PROS dependency,
 * so it can be unit-tested on a desktop compiler (see
 * tests/tuning/characterization_math_test.cpp). tuning::runCharacterization()
 * collects the samples on the robot; characterizeAxis() turns them into a
 * MotorFeedforward model (kS/kV/kA) plus the axis's response delay, which is
 * everything tuning::designPositionGains() needs.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <vector>

#include "sapphirelib/control/feedforward.hpp"

namespace sapphirelib::tuning {

/// One tick of a characterization run: the voltage commanded on that tick
/// and the axis position read just before commanding it, at a millisecond
/// timestamp relative to the run's own start.
struct CharacterizationSample {
    std::uint32_t timeMs = 0;
    double volts = 0.0;
    double position = 0.0;
};

/// One continuous segment of driving. Velocity is differentiated within a
/// run, never across the gap between two of them.
using CharacterizationRun = std::vector<CharacterizationSample>;

/// Everything one axis's characterization collected. Ramps (voltage rising
/// slowly) mostly pin down kS and kV; steps (voltage jumping from 0 and
/// held) pin down kA and the response delay. Both are needed.
struct CharacterizationData {
    std::vector<CharacterizationRun> ramps;
    std::vector<CharacterizationRun> steps;
};

/// Result of fitFeedforward(). `ok` is false if the data couldn't support a
/// trustworthy model — too few moving samples, an ill-conditioned fit (no
/// acceleration content, say), a non-physical result, or an R² below the
/// requested floor. `rSquared` and `samplesUsed` are filled in either way,
/// to help diagnose a failure.
struct FeedforwardFit {
    bool ok = false;
    MotorFeedforward model;

    /// How much of the variation in commanded voltage the model explains,
    /// 0-1. Above ~0.95 is a clean fit; below ~0.8 is rejected.
    double rSquared = 0.0;

    int samplesUsed = 0;
};

/// Fits kS/kV/kA across `runs`.
///
/// Rather than differentiating position twice to get acceleration — which
/// amplifies sensor noise so badly that it drags kA toward zero (a noisy
/// regressor always biases its own coefficient down) — this fits the
/// exact discrete-time solution of the model over short intervals:
///
///     v[k+m] = α·v[k] + β·u + γ·sign(v[k])
///
/// then recovers kV = (1−α)/β, kS = −γ/β, kA = −kV·T/ln α. Only velocity is
/// needed, differentiated once with central differences `halfWindow`
/// samples wide, and the interval m is chosen so the two velocity estimates
/// in each row share no samples — so their noise is independent and
/// doesn't masquerade as dynamics.
///
/// `delayTicks` shifts each commanded voltage that many samples later
/// before fitting, to line the command up with when it actually moved the
/// axis; see characterizeAxis() for how it's chosen. Rows moving slower
/// than `minSpeed` are skipped: there, sign(v) is decided by noise and the
/// axis is stuck in static friction the model doesn't describe.
FeedforwardFit fitFeedforward(const std::vector<CharacterizationRun>& runs, double minSpeed,
                              int halfWindow = 5, int delayTicks = 0, double minRSquared = 0.8);

/// Estimates how long the axis takes to start responding to a command — the
/// sum of loop, motor-controller, and sensor latency — from a step run
/// (starting at rest, then a constant voltage).
///
/// Compares when the measured velocity first reaches half of the model's
/// steady-state speed against when an ideal, delay-free axis with the same
/// model would: kS/kV/kA say it should take kA/kV·ln 2 seconds, and
/// whatever extra it actually took is delay. Half, rather than something
/// like 10%, because the early part of the rise is exactly where sensor
/// noise is proportionally largest.
///
/// Returns a negative number if `stepRun` has no step or never reaches half
/// speed (e.g. it was cut short by its travel limit).
double estimateResponseDelayS(const CharacterizationRun& stepRun, const MotorFeedforward& model,
                              int halfWindow = 5);

/// Result of characterizeAxis().
struct AxisCharacterization {
    bool ok = false;
    FeedforwardFit fit;

    /// Measured response delay in seconds; 0 if no step run produced one.
    double delayS = 0.0;
};

/// The whole identification: fits a first model ignoring delay, measures
/// the delay against it on the step runs, then refits with the commands
/// shifted by that delay and re-measures. The second pass matters —
/// fitting delayed data as if it weren't inflates kS and shrinks kA.
AxisCharacterization characterizeAxis(const CharacterizationData& data, double minSpeed,
                                      int halfWindow = 5, double minRSquared = 0.8);

} // namespace sapphirelib::tuning
