#pragma once

#include <cstdint>
#include <functional>

#include "sapphirelib/tuning/characterization_math.hpp"

namespace sapphirelib::tuning {

/**
 * @brief Settings for characterizing one axis. See runCharacterization()
 */
struct CharacterizationConfig {
    /**
     * applies a voltage to this axis alone, before wheel mixing, e.g.
     * holonomicVolts(v, 0, 0) for forward. Must be the same kind of command the axis's PIDs output
     */
    std::function<void(double)> actuate;

    /**
     * reads the axis's position in the units its PIDs use: inches for driving, unwrapped degrees
     * for turning. A reading that isn't finite ends the segment
     */
    std::function<double()> measure;

    /** voltage each step jumps to. High enough to reach a good part of top speed. 6 by default */
    double stepVolts = 6.0;

    /** how fast each ramp raises the voltage, in volts per second. 4 by default */
    double rampVoltsPerS = 4.0;
    /** the most a ramp goes to, in volts. 8 by default */
    double rampMaxVolts = 8.0;

    /**
     * how far each segment can move from where it started, in measure()'s units, to keep the
     * robot on the field. Leave room for coasting. 0 for no limit (fine for turning)
     */
    double maxTravel = 0.0;

    /** longest any one segment can last, in milliseconds. 2500 by default */
    std::uint32_t maxSegmentMs = 2500;

    /** time recorded at 0V before each segment, in milliseconds. 100 by default */
    std::uint32_t preRollMs = 100;

    /**
     * slowest speed that counts toward the fit, in units/s. Pick something above what noise reads
     * while sitting still: a few in/s for driving, a few deg/s for turning. 2 by default
     */
    double minSpeed = 2.0;

    /** longest to wait for the axis to stop between segments, in milliseconds. 1500 by default */
    std::uint32_t settleTimeoutMs = 1500;

    /** time between samples, in milliseconds. 10 by default */
    std::uint32_t samplePeriodMs = 10;

    /**
     * checked before every sample. Returning true ends the run right away with
     * CharacterizationData::aborted set. For a Stop button, or the robot being disabled
     */
    std::function<bool()> shouldAbort{};

    /**
     * called once before the run commands anything. For taking the motors from whatever else
     * drives them, e.g. PositionMechanism::beginExternalControl(). Runs on the characterization
     * task
     */
    std::function<void()> start{};
    /** called once after the run's last command, however it ended */
    std::function<void()> finish{};
};

/**
 * @brief Drive an axis through the ramps and steps characterizeAxis() needs, and record it
 *
 * Runs a ramp forward, a ramp back, a step forward, and a step back, so a drive axis ends up near
 * where it started. Each segment ends at its travel limit or time cap, and the axis stops between
 * segments. Always ends at 0V
 *
 * @note blocks for several seconds
 *
 * @param config the axis and run settings
 * @return CharacterizationData the recorded samples
 *
 * @b Example
 * @code {.cpp}
 * auto data = sapphirelib::tuning::runCharacterization({
 *     .actuate = [](double v) { drivetrain().holonomicVolts(0, 0, v); },
 *     .measure = [] { return drivetrain().imu().getCumulativeHeadingDeg(); },
 *     .minSpeed = 5.0,
 * });
 * @endcode
 */
CharacterizationData runCharacterization(const CharacterizationConfig& config);

/**
 * @brief Settings for characterizing a lift, an arm, or anything else gravity loads
 */
struct MechanismCharacterizationConfig {
    /**
     * actuate, measure, the voltages, and the timing, as for a drive axis. A positive voltage
     * must raise the measurement. maxTravel is ignored; the limits below are used instead
     */
    CharacterizationConfig axis{};

    /**
     * lowest position, in measure()'s units. A downward segment ends here. Keep it short of the
     * hard stop, since the mechanism coasts while braking. Start the run near this limit
     */
    double lowerLimit = 0.0;
    /** highest position, in measure()'s units. An upward segment ends here */
    double upperLimit = 0.0;

    /**
     * the downward step's volts, as a positive number. 0 uses axis.stepVolts. Gravity helps a
     * lift down, so it's often worth asking for less
     */
    double downStepVolts = 0.0;

    /**
     * holds the mechanism still between segments and at the end, e.g. braking with the brake mode
     * set to hold. Empty commands 0V, which only suits a mechanism that stays put unpowered. Held
     * samples record NaN volts
     */
    std::function<void()> hold{};

    /** how gravity loads it */
    GravityShape gravity{};
};

/**
 * @brief Drive a mechanism through the ramps and steps characterizeMechanism() needs
 *
 * Ramp up, ramp down, step up, step down, each from rest at the end of the last, each ending at
 * its limit or time cap. Moving both ways is what separates kS from kG. Holds the mechanism between
 * segments and at the end. Feed the result to characterizeMechanism() with the same gravity
 *
 * @note blocks for several seconds
 *
 * @param config the mechanism and run settings
 * @return CharacterizationData the recorded samples
 */
CharacterizationData runMechanismCharacterization(const MechanismCharacterizationConfig& config);

} // namespace sapphirelib::tuning
