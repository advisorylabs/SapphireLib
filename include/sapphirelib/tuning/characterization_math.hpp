/**
 * \file sapphirelib/tuning/characterization_math.hpp
 *
 * System identification for one axis — pure math, no PROS dependency, so it
 * can be unit-tested on a desktop compiler (see
 * tests/tuning/characterization_math_test.cpp). tuning::runCharacterization()
 * and tuning::runMechanismCharacterization() collect the samples on the
 * robot; characterizeAxis() turns a drive axis's samples into a
 * MotorFeedforward model (kS/kV/kA) plus the axis's response delay, and
 * characterizeMechanism() does the same for a lift or an arm, with a gravity
 * term (kG) on top. That's everything tuning::designPositionGains() needs.
 *
 * tools/analyzer/js/model.js is a line-for-line JavaScript port, so the
 * telemetry analyzer refits logged runs with exactly this math; the two are
 * held to the same numbers by a shared golden case (see
 * testGoldenCaseMatchesTheAnalyzer() in the test).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <vector>

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/mechanism/position_control.hpp"

namespace sapphirelib::tuning {

/// One tick of a characterization run: the voltage commanded on that tick
/// and the axis position read just before commanding it, at a millisecond
/// timestamp relative to the run's own start.
///
/// `volts` is NaN for a tick the axis wasn't under voltage control — a
/// mechanism held on its motors' brake (see
/// MechanismCharacterizationConfig::hold). The fit skips every interval that
/// touches one, since nobody knows what the brake applied.
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

    /// True if the run was stopped before it finished (the tuner's Stop
    /// button, the robot being disabled). What was collected is kept, but a
    /// tuner shouldn't act on it.
    bool aborted = false;
};

/// How gravity loads an axis. A drive axis has none; an elevator lift carries
/// the same weight at every height (constant); an arm's load falls off as it
/// swings toward vertical (cosine). This is only the load's shape — its size,
/// kG, is what characterizeMechanism() fits.
enum class GravityKind : std::uint8_t { none, constant, cosine };

struct GravityShape {
    GravityKind kind = GravityKind::none;

    /// Cosine only: the position reading at which the arm is horizontal, and
    /// arm degrees per unit of position reading — the same meaning as
    /// mechanism::GravityFeedforward's fields of the same names.
    double horizontalPosition = 0.0;
    double armDegreesPerUnit = 1.0;

    /// The load at `position` as a fraction of kG: 0 for none, 1 for
    /// constant, cos(arm angle) for cosine.
    double factor(double position) const;
};

/// A lift's or an arm's model: a drive axis's kS/kV/kA plus gravity,
///
///     V = kS·sign(v) + kG·g(x) + kV·v + kA·a
///
/// where g(x) is gravity.factor(x). kG is the volts it takes to hold the
/// mechanism still against gravity where g = 1 (anywhere, for a lift; level,
/// for an arm), friction aside.
struct MechanismModel {
    MotorFeedforward motion{};
    double kG = 0.0;
    GravityShape gravity{};

    /// The model's kV and kA are what a controller design needs; gravity and
    /// friction don't enter it (feedforward cancels one, the other only sets
    /// how close the loop can get — see GainDesign::staticErrorBound).
    bool valid() const { return motion.valid(); }

    /// kG·g(position): the volts gravity costs at `position`.
    double gravityVolts(double position) const;

    /// The feedforward that cancels this model's gravity, for a
    /// mechanism::PositionMechanism (PositionConfig::gravity, or
    /// PositionMechanism::setGravity() on a running one).
    mechanism::GravityFeedforward gravityFeedforward() const;
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

/// Result of fitMechanism() — FeedforwardFit with gravity.
struct MechanismFit {
    bool ok = false;
    MechanismModel model{};
    double rSquared = 0.0;
    int samplesUsed = 0;
};

/// fitFeedforward() with one more term, for gravity:
///
///     v[k+m] = α·v[k] + β·u + γ·sign(v[k]) + δ·g
///
/// where g is `gravity`'s factor averaged over the interval, and kG = −δ/β.
/// Separating kG from kS takes motion both ways — friction flips sign with
/// the direction of travel, gravity doesn't — which is why the mechanism
/// runner drives up and down. With GravityKind::none this is exactly
/// fitFeedforward(), kG = 0.
MechanismFit fitMechanism(const std::vector<CharacterizationRun>& runs, GravityShape gravity,
                          double minSpeed, int halfWindow = 5, int delayTicks = 0,
                          double minRSquared = 0.8);

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

/// estimateResponseDelayS() for a mechanism: the step's steady-state speed is
/// what's left of its volts after gravity (at the step's starting position)
/// and friction. The step is the first tick commanding finite, nonzero volts,
/// so held (NaN) pre-roll samples count as "at rest".
double estimateMechanismDelayS(const CharacterizationRun& stepRun, const MechanismModel& model,
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

/// Result of characterizeMechanism().
struct MechanismCharacterization {
    bool ok = false;
    MechanismFit fit{};
    double delayS = 0.0;
};

/// characterizeAxis() with gravity: the same two passes, through
/// fitMechanism() and estimateMechanismDelayS().
MechanismCharacterization characterizeMechanism(const CharacterizationData& data,
                                                GravityShape gravity, double minSpeed,
                                                int halfWindow = 5, double minRSquared = 0.8);

} // namespace sapphirelib::tuning
