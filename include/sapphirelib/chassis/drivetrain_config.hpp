#pragma once

#include <cstdint>

#include "sapphirelib/control/feedforward.hpp"

namespace sapphirelib::chassis {

/**
 * @brief How driver control stick input becomes motor output
 */
enum class DriverInputMode {
    /**
     * stick position is a fraction of 12V, so half stick is 6V. The first bit of stick travel does
     * nothing until the voltage beats friction
     */
    voltage,

    /**
     * stick position is a fraction of the axis's top speed, turned into volts through the
     * measured model (see HolonomicAxisModels). Axes without a valid model fall back to voltage
     */
    velocity,
};

/**
 * @brief Measured feedforward models for a holonomic chassis's three axes
 *
 * Forward and strafe in inches, turn in degrees, against the axis voltage before wheel mixing.
 * Auto-Tune measures these (see tuning::characterizeAxis())
 */
struct HolonomicAxisModels {
    /** forward/backward axis model */
    MotorFeedforward forward;
    /** sideways axis model */
    MotorFeedforward strafe;
    /** rotation axis model */
    MotorFeedforward turn;
};

/**
 * @brief Physical settings for a drivetrain
 */
struct DrivetrainConfig {
    /** drive wheel diameter, in inches. Required */
    double wheelDiameterIn = 0.0;

    /** motor rotations per wheel rotation. 1 for direct drive */
    double externalGearRatio = 1.0;

    /** kP for keeping driveDistance() straight with the IMU. 0 disables it */
    double headingCorrectionKP = 0.0;
};

/**
 * @brief Settings for "Asterisk" center wheels on a HolonomicDrivetrain
 *
 * Asterisk wheels are two motors between the corners facing straight forward. They add power when
 * driving forward and turning, sit idle during a pure strafe, and can correct forward/backward
 * drift while strafing (see HolonomicDrivetrain::setDriftSource()). Leave out the drivetrain's
 * asterisk argument for a standard 4 motor chassis
 */
struct AsteriskConfig {
    /** left center motor port. Negative reverses it. Uses the corners' gearset */
    std::int8_t middleLeftPort;
    /** right center motor port. Negative reverses it */
    std::int8_t middleRightPort;

    /**
     * kP for correcting forward/backward drift while strafing, in volts per in/s. Needs a vertical
     * tracking wheel from setDriftSource(). 0 disables drift correction (the center wheels still
     * drive forward and turn)
     */
    double driftCorrectionKP = 0.0;

    /**
     * how much of each turn the center wheels help with, driven left forward and right backward.
     * 1 (the default) gives them the same authority as the corners, 0 lets them coast through
     * turns. Turn it down if the center motors are weaker than the corners and saturate first
     */
    double turnContribution = 1.0;

    /**
     * how much of the corners' thermal power loss the center wheels make up. 1 by default, 0
     * disables it. V5 motors cut their own power as they heat, and uneven corners make a strafe
     * creep and twist; the center wheels can cancel that forward/backward error. Works alongside
     * driftCorrectionKP: this corrects ahead of time, drift correction catches the rest. See
     * chassis::centerThermalCorrection()
     */
    double thermalCompensation = 1.0;

    /**
     * the most the thermal correction can add to a center wheel, in volts. 6 by default. The
     * forward and turn parts are capped separately. 0 disables the correction
     */
    double maxThermalCorrectionVolts = 6.0;
};

/**
 * @brief The voltage a drivetrain last commanded on each axis, before wheel mixing
 */
struct AxisVolts {
    /** forward/backward, in volts */
    double forward = 0.0;
    /** sideways, in volts. Always 0 on a tank drive */
    double strafe = 0.0;
    /** rotation, in volts */
    double turn = 0.0;
};

/**
 * @brief When a blocking motion ends
 *
 * @b Example
 * @code {.cpp}
 * // settle within half an inch for 150ms, and give up after 2 seconds
 * chassis.driveDistance(24, {.errorThreshold = 0.5, .settleTimeMs = 150, .timeoutMs = 2000});
 * @endcode
 */
struct ExitConditions {
    /**
     * the error that counts as arrived: inches for driveDistance() and moveToPoint(), degrees for
     * turnToHeading(). 1 by default (turnToHeading()'s default argument uses 2 degrees)
     */
    double errorThreshold = 1.0;

    /** how long the error must stay within errorThreshold, in milliseconds. 200 by default */
    std::uint32_t settleTimeMs = 200;

    /** longest time the motion can take, in milliseconds. 3000 by default. 0 disables it */
    std::uint32_t timeoutMs = 3000;
};

} // namespace sapphirelib::chassis
