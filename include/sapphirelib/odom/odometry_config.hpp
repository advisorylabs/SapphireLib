#pragma once

namespace sapphirelib::odom {

/**
 * @brief Tracking wheel offsets for odometry
 *
 * Each offset is how far the wheel rolls per radian of clockwise turn in place, which is what
 * odometry subtracts to tell turning from driving. For the horizontal wheel that's its distance
 * ahead of center. For the vertical wheel the sign flips: a wheel right of center rolls backward
 * on a clockwise turn (like a tank's right side), so a right-side wheel has a negative offset.
 * calibrateTrackingWheelOffsetIn() (the Odometry page's "Calibrate Offsets") measures both with the
 * right sign; to measure by hand, use the signs below
 *
 * @b Example
 * @code {.cpp}
 * // vertical wheel 3.5in right of center, horizontal wheel 2in behind it
 * sapphirelib::odom::OdometryConfig config{.verticalOffsetIn = -3.5, .horizontalOffsetIn = -2.0};
 * @endcode
 */
struct OdometryConfig {
    /**
     * distance from the vertical tracking wheel to the tracking center, in inches. Positive is
     * LEFT of center, negative is right (the opposite of
     * localization::DistanceSensorMount::rightIn). 0 for drive encoders. Measure it with
     * calibrateTrackingWheelOffsetIn()
     */
    double verticalOffsetIn = 0.0;

    /**
     * distance from the horizontal tracking wheel to the tracking center, in inches. Positive is
     * ahead of center, negative is behind. Unused without a horizontal wheel
     */
    double horizontalOffsetIn = 0.0;
};

} // namespace sapphirelib::odom
