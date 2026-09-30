#pragma once

namespace sapphirelib::odom {

/**
 * @brief The robot's position and heading on the field
 *
 * The origin and axes are whatever the odometry's start pose or a later setPose() makes them.
 * x increases to the right, y increases downfield, and heading is 0-360 degrees clockwise, matching
 * the IMU (0 faces +y, 90 faces +x)
 */
struct Pose {
    /** x position, in inches */
    double xIn = 0.0;
    /** y position, in inches */
    double yIn = 0.0;
    /** heading, 0-360 degrees */
    double headingDeg = 0.0;
};

} // namespace sapphirelib::odom
