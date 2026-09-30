#pragma once

namespace sapphirelib::sensors {

/**
 * @brief Get the signed change between two raw IMU headings
 *
 * Assumes the robot turned less than 180 degrees between the readings
 *
 * @param lastRawHeadingDeg the earlier heading, 0-360 degrees
 * @param rawHeadingDeg the later heading, 0-360 degrees
 * @return double the change, (-180, 180] degrees
 */
double rawHeadingDeltaDeg(double lastRawHeadingDeg, double rawHeadingDeg);

/**
 * @brief Wrap an angle to [0, 360)
 *
 * @param degrees the angle, any size
 * @return double the angle, [0, 360) degrees
 */
double wrapDegrees360(double degrees);

/**
 * @brief Get the field heading for an unwrapped rotation plus the field offset
 *
 * @param cumulativeDeg unwrapped rotation, in degrees
 * @param headingOffsetDeg the offset from setHeadingDeg(), in degrees
 * @return double heading, 0-360 degrees
 */
double fieldHeadingDeg(double cumulativeDeg, double headingOffsetDeg);

/**
 * @brief Get the offset that makes the field heading read a target right now
 *
 * @param targetHeadingDeg the heading to read, in degrees. Any value
 * @param cumulativeDeg the current unwrapped rotation, in degrees
 * @return double the offset, (-180, 180] degrees
 */
double headingOffsetFor(double targetHeadingDeg, double cumulativeDeg);

/**
 * @brief Work out the IMU's headingScale
 *
 * With an Imu built with a scale of 1, spin the robot a known number of full turns (more turns
 * average out the error better), then compare with what the IMU measured
 *
 * @param actualTurns how many turns the robot really made, e.g. 10
 * @param measuredTurns how many the IMU measured, e.g. imu.getCumulativeHeadingDeg() / 360
 * @return double the headingScale to build this robot's Imu with
 *
 * @b Example
 * @code {.cpp}
 * // after spinning the robot exactly 10 turns
 * double measured = imu.getCumulativeHeadingDeg() / 360;
 * double scale = sapphirelib::sensors::calibrateHeadingScale(10, measured);
 * @endcode
 */
double calibrateHeadingScale(double actualTurns, double measuredTurns);

} // namespace sapphirelib::sensors
