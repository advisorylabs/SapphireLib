#pragma once

#include <cstdint>
#include <vector>

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/mechanism/position_control.hpp"

// system identification: turn the samples a characterization run collects into a feedforward model
// and response delay, which is everything designPositionGains() needs. tools/analyzer/js/model.js
// is a line for line port, held to the same numbers by a shared golden test case

namespace sapphirelib::tuning {

/**
 * @brief One tick of a characterization run
 */
struct CharacterizationSample {
    /** time since the run started, in milliseconds */
    std::uint32_t timeMs = 0;
    /** voltage commanded this tick. NaN while held on the brake, which the fit skips */
    double volts = 0.0;
    /** position read just before commanding it */
    double position = 0.0;
};

/**
 * @brief One continuous segment of driving. Velocity is never taken across two runs
 */
using CharacterizationRun = std::vector<CharacterizationSample>;

/**
 * @brief Everything one axis's characterization collected
 */
struct CharacterizationData {
    /** runs where the voltage rose slowly. Mostly pin down kS and kV */
    std::vector<CharacterizationRun> ramps;
    /** runs where the voltage jumped from 0. Pin down kA and the delay */
    std::vector<CharacterizationRun> steps;

    /** whether the run was stopped early (Stop button, robot disabled). Don't act on it */
    bool aborted = false;
};

/**
 * @brief How gravity loads an axis: none (a drive axis), constant (an elevator lift), or cosine
 * (an arm)
 */
enum class GravityKind : std::uint8_t { none, constant, cosine };

/**
 * @brief The shape of gravity's load on an axis. Its size, kG, is what gets fitted
 */
struct GravityShape {
    /** none, constant, or cosine */
    GravityKind kind = GravityKind::none;

    /** cosine only: the position reading where the arm is horizontal */
    double horizontalPosition = 0.0;
    /** cosine only: arm degrees per unit of position reading */
    double armDegreesPerUnit = 1.0;

    /**
     * @brief Get the load at a position, as a fraction of kG
     *
     * @return double 0 for none, 1 for constant, cos(arm angle) for cosine
     */
    double factor(double position) const;
};

/**
 * @brief A lift's or arm's model: V = kS * sign(v) + kG * g(x) + kV * v + kA * a
 *
 * kG is the volts it takes to hold the mechanism still where g = 1 (anywhere for a lift, level for
 * an arm)
 */
struct MechanismModel {
    /** kS, kV, and kA */
    MotorFeedforward motion{};
    /** volts to hold against gravity */
    double kG = 0.0;
    /** how gravity loads it */
    GravityShape gravity{};

    /**
     * @brief Whether the model can be used to design a controller (kV and kA are positive)
     */
    bool valid() const { return motion.valid(); }

    /**
     * @brief Get the volts gravity costs at a position: kG * g(position)
     */
    double gravityVolts(double position) const;

    /**
     * @brief Get the feedforward that cancels this model's gravity, for a PositionMechanism
     *
     * @b Example
     * @code {.cpp}
     * lift.setGravity(result.fit.model.gravityFeedforward());
     * @endcode
     */
    mechanism::GravityFeedforward gravityFeedforward() const;
};

/**
 * @brief Result of fitFeedforward()
 */
struct FeedforwardFit {
    /**
     * whether the model can be trusted. false for too few moving samples, no acceleration in the
     * data, a non-physical result, or a low R^2
     */
    bool ok = false;
    /** the fitted model */
    MotorFeedforward model;

    /** how much of the voltage the model explains, 0-1. Above 0.95 is clean; below 0.8 fails */
    double rSquared = 0.0;

    /** samples the fit used */
    int samplesUsed = 0;
};

/**
 * @brief Fit kS, kV, and kA to some runs
 *
 * Differentiating position twice amplifies noise and drags kA toward zero, so instead this fits
 * the model's exact discrete-time solution, v[k+m] = a * v[k] + b * u + c * sign(v[k]), then
 * recovers kV = (1 - a) / b, kS = -c / b, and kA = -kV * T / ln(a). The interval m is picked so the
 * two velocity estimates in each row share no samples
 *
 * @param runs the runs to fit
 * @param minSpeed slowest speed that counts, in units/s. Slower samples are stuck in friction
 * @param halfWindow half width of the velocity difference, in samples. 5 by default
 * @param delayTicks how many samples later each command took effect. 0 by default
 * @param minRSquared lowest R^2 that counts as ok. 0.8 by default
 * @return FeedforwardFit the fit
 */
FeedforwardFit fitFeedforward(const std::vector<CharacterizationRun>& runs, double minSpeed,
                              int halfWindow = 5, int delayTicks = 0, double minRSquared = 0.8);

/**
 * @brief Result of fitMechanism()
 */
struct MechanismFit {
    /** whether the model can be trusted */
    bool ok = false;
    /** the fitted model */
    MechanismModel model{};
    /** how much of the voltage the model explains, 0-1 */
    double rSquared = 0.0;
    /** samples the fit used */
    int samplesUsed = 0;
};

/**
 * @brief fitFeedforward() with a gravity term: v[k+m] = a * v[k] + b * u + c * sign(v[k]) + d * g
 *
 * kG = -d / b. Telling kG from kS takes motion both ways, since friction flips with direction and
 * gravity doesn't. With GravityKind::none this is exactly fitFeedforward()
 *
 * @param runs the runs to fit
 * @param gravity how gravity loads the axis
 * @param minSpeed slowest speed that counts, in units/s
 * @param halfWindow half width of the velocity difference, in samples. 5 by default
 * @param delayTicks how many samples later each command took effect. 0 by default
 * @param minRSquared lowest R^2 that counts as ok. 0.8 by default
 * @return MechanismFit the fit
 */
MechanismFit fitMechanism(const std::vector<CharacterizationRun>& runs, GravityShape gravity,
                          double minSpeed, int halfWindow = 5, int delayTicks = 0,
                          double minRSquared = 0.8);

/**
 * @brief Estimate how long an axis takes to start responding to a command
 *
 * Compares when the step reaches half the model's steady speed against when a delay-free axis
 * would (kA / kV * ln 2 seconds). Anything extra is delay
 *
 * @param stepRun a step run, starting at rest
 * @param model the axis model
 * @param halfWindow half width of the velocity difference, in samples. 5 by default
 * @return double the delay, in seconds. Negative if the run has no step or never reaches half speed
 */
double estimateResponseDelayS(const CharacterizationRun& stepRun, const MotorFeedforward& model,
                              int halfWindow = 5);

/**
 * @brief estimateResponseDelayS() for a mechanism
 *
 * The steady speed is what's left of the step's volts after gravity and friction. Held (NaN)
 * samples before the step count as at rest
 */
double estimateMechanismDelayS(const CharacterizationRun& stepRun, const MechanismModel& model,
                               int halfWindow = 5);

/**
 * @brief Result of characterizeAxis()
 */
struct AxisCharacterization {
    /** whether the model can be trusted */
    bool ok = false;
    /** the fit */
    FeedforwardFit fit;

    /** response delay, in seconds. 0 if no step run gave one */
    double delayS = 0.0;
};

/**
 * @brief Measure an axis's model and delay from a characterization run
 *
 * Fits a model ignoring delay, measures the delay against it, then refits with the commands
 * shifted by the delay. The second pass matters, since fitting delayed data as if it weren't
 * inflates kS and shrinks kA
 *
 * @param data what the run collected
 * @param minSpeed slowest speed that counts, in units/s
 * @param halfWindow half width of the velocity difference, in samples. 5 by default
 * @param minRSquared lowest R^2 that counts as ok. 0.8 by default
 * @return AxisCharacterization the model and delay
 *
 * @b Example
 * @code {.cpp}
 * auto data = sapphirelib::tuning::runCharacterization(turnConfig);
 * auto result = sapphirelib::tuning::characterizeAxis(data, turnConfig.minSpeed);
 * if (result.ok) printf("kV %f kA %f\n", result.fit.model.kV, result.fit.model.kA);
 * @endcode
 */
AxisCharacterization characterizeAxis(const CharacterizationData& data, double minSpeed,
                                      int halfWindow = 5, double minRSquared = 0.8);

/**
 * @brief Result of characterizeMechanism()
 */
struct MechanismCharacterization {
    /** whether the model can be trusted */
    bool ok = false;
    /** the fit */
    MechanismFit fit{};
    /** response delay, in seconds */
    double delayS = 0.0;
};

/**
 * @brief characterizeAxis() for a lift or arm, with gravity
 *
 * @param data what the run collected
 * @param gravity how gravity loads it
 * @param minSpeed slowest speed that counts, in units/s
 * @param halfWindow half width of the velocity difference, in samples. 5 by default
 * @param minRSquared lowest R^2 that counts as ok. 0.8 by default
 * @return MechanismCharacterization the model and delay
 */
MechanismCharacterization characterizeMechanism(const CharacterizationData& data,
                                                GravityShape gravity, double minSpeed,
                                                int halfWindow = 5, double minRSquared = 0.8);

} // namespace sapphirelib::tuning
