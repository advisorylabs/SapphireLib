#pragma once

#include <atomic>
#include <cstdint>

#include "pros/imu.hpp"

namespace sapphirelib::sensors {

/**
 * @brief V5 IMU with multi-turn drift correction
 *
 * The V5 IMU over or under reports rotation by a small, fairly consistent percentage, which adds
 * up to tens of degrees over a match. This tracks the unwrapped rotation, multiplies it by a
 * calibrated headingScale, and wraps it back to 0-360. Calibrate once per robot with
 * calibrateHeadingScale(). A scale of 1 (the default) does no correction
 *
 * Safe to read from several tasks at once. A read the sensor can't answer (unplugged, or
 * recalibrating after a brownout) is skipped instead of turning the heading into NaN
 *
 * @b Example
 * @code {.cpp}
 * // IMU on port 10 that reads 0.4% low
 * sapphirelib::sensors::Imu imu(10, 1.004);
 * double heading = imu.getHeadingDeg();
 * @endcode
 */
class Imu {
public:
    /**
     * @brief Construct a new Imu
     *
     * Blocks until calibration finishes, so headings are valid right away. Logs an error if
     * calibration fails; see calibrated()
     *
     * @param port IMU port
     * @param headingScale drift correction. 1 by default, which does nothing
     */
    explicit Imu(std::uint8_t port, double headingScale = 1.0);

    /**
     * @brief Whether calibration succeeded
     *
     * False usually means there's no IMU on the port. Headings stay at 0 until it answers, so check
     * this once at startup
     */
    bool calibrated() const;

    /**
     * @brief Get the field heading
     *
     * Like pros::Imu::get_heading(), with headingScale applied and shifted by setHeadingDeg()
     *
     * @note not const: every call advances the rotation tracking, so call this (or
     * getCumulativeHeadingDeg()) at your loop's rate. With too long between calls, a fast spin
     * could look like a smaller turn the other way
     *
     * @return double heading, 0-360 degrees, clockwise positive
     */
    double getHeadingDeg();

    /**
     * @brief Get the total rotation since construction, not wrapped
     *
     * 3.5 clockwise turns reads 1260, not 180. Not shifted by setHeadingDeg(), so re-framing the
     * field heading never makes it jump. Same call rate note as getHeadingDeg()
     *
     * @return double rotation, in degrees, with headingScale applied
     */
    double getCumulativeHeadingDeg();

    /**
     * @brief Set the field heading the robot has right now
     *
     * Turns nothing and changes no rotation reading; it only picks which direction is heading 0.
     * odom::Odometry::setPose() calls this, which is why odometry should share the drivetrain's
     * imu(). Safe from any task
     *
     * @param headingDeg the heading the robot has now, in degrees. Wrapped to 0-360
     */
    void setHeadingDeg(double headingDeg);

    /**
     * @brief Get what getHeadingDeg() adds to the rotation, to convert between the two
     *
     * @return double offset, (-180, 180] degrees. 0 until setHeadingDeg() is called
     */
    double headingOffsetDeg() const;

    /**
     * @brief Set the drift correction
     *
     * @param headingScale the new scale
     */
    void setHeadingScale(double headingScale);

    /**
     * @brief Get the drift correction
     */
    double headingScale() const;

private:
    static_assert(std::atomic<double>::is_always_lock_free,
                  "Imu's cross-task heading tracking relies on lock-free atomic doubles");

    pros::Imu imu_;
    std::atomic<double> headingScale_;

    // the raw reading the next delta is measured from. NaN until the sensor has answered once, so
    // the first good reading becomes the baseline instead of a jump
    std::atomic<double> lastRawHeadingDeg_;
    std::atomic<double> rawCumulativeDeg_{0.0};

    // see setHeadingDeg(). Separate from the rotation tracking, so re-framing never touches it
    std::atomic<double> headingOffsetDeg_{0.0};
    bool calibrated_ = false;

    void updateCumulative();
};

} // namespace sapphirelib::sensors
