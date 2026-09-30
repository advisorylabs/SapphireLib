#pragma once

#include <cstdint>

#include "sapphirelib/chassis/drivetrain_config.hpp"

namespace sapphirelib::motion {

/**
 * @brief When moveToPose() ends
 *
 * Settles once the position and heading errors are both within their thresholds at the same time
 *
 * @b Example
 * @code {.cpp}
 * // settle within 0.5" and 1 degree, and give up after 2.5 seconds
 * drivetrain().moveToPose(24, 48, 90, {.positionErrorThresholdIn = 0.5,
 *                                      .headingErrorThresholdDeg = 1.0,
 *                                      .timeoutMs = 2500});
 * @endcode
 */
struct PoseExitConditions {
    /** distance from the point that counts as arrived, in inches. 1 by default */
    double positionErrorThresholdIn = 1.0;

    /** heading error that counts as arrived, in degrees. 2 by default */
    double headingErrorThresholdDeg = 2.0;

    /** how long both errors must stay within threshold, in milliseconds. 200 by default */
    std::uint32_t settleTimeMs = 200;

    /** longest time the motion can take, in milliseconds. 4000 by default. 0 disables it */
    std::uint32_t timeoutMs = 4000;

    /**
     * boomerang lead, as a fraction of the remaining distance. TankDrivetrain only. Smaller makes
     * a tighter curve into the final heading, larger a wider one. Must be above 0. 0.6 by default
     */
    double boomerangLeadPct = 0.6;
};

/**
 * @brief Settings for followPath()
 */
struct PursuitConfig {
    /**
     * how far ahead on the path to aim, in inches. Larger cuts corners more but is smoother;
     * smaller tracks the path tighter but can oscillate
     */
    double lookaheadIn;

    /** voltage to follow the path at, clamped to +-12. Constant until the final approach */
    double cruiseVoltage = 8.0;

    /**
     * distance from the last waypoint where followPath() switches to moveToPoint(), so it slows
     * down and settles instead of circling the end, in inches. 6 by default
     */
    double finalApproachIn = 6.0;

    /** when the final moveToPoint() ends */
    chassis::ExitConditions finalExit = chassis::ExitConditions{1.0};

    /**
     * longest time the pursuit can take before the final approach, in milliseconds. 10000 by
     * default. On timeout the robot stops without trying the final approach. 0 disables it
     */
    std::uint32_t timeoutMs = 10000;
};

} // namespace sapphirelib::motion
