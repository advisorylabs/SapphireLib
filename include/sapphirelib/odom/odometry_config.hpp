#pragma once

namespace sapphirelib::odom {

/**
 * @brief Tracking wheel offsets for odometry
 */
struct OdometryConfig {
    /**
     * distance from the vertical tracking wheel to the tracking center, in inches. Positive is
     * right of center. 0 for drive encoders. Measure it with calibrateTrackingWheelOffsetIn()
     */
    double verticalOffsetIn = 0.0;

    /**
     * distance from the horizontal tracking wheel to the tracking center, in inches. Positive is
     * ahead of center. Unused without a horizontal wheel
     */
    double horizontalOffsetIn = 0.0;
};

} // namespace sapphirelib::odom
