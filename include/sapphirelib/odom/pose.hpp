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

/**
 * @brief How fast the robot is moving on the field, in the same frame as Pose
 */
struct Velocity {
    /** x speed, in inches per second */
    double xInPerS = 0.0;
    /** y speed, in inches per second */
    double yInPerS = 0.0;
    /** turn rate, in degrees per second, clockwise positive */
    double headingDegPerS = 0.0;
};

} // namespace sapphirelib::odom
