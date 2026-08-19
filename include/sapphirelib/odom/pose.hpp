/**
 * \file sapphirelib/odom/pose.hpp
 *
 * The robot's estimated position and heading on the field.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::odom {

/// Field-coordinate pose: position in inches, heading in degrees.
///
/// Coordinate convention: the origin and axis orientation are whatever
/// Odometry's startPose (or a later setPose()) defines them to be — but
/// once fixed, x increases to the right, y increases "downfield" from that
/// origin, and heading is 0-360 degrees, clockwise-positive, matching
/// pros::Imu::get_heading() (0 = facing +y, 90 = facing +x).
struct Pose {
    double xIn = 0.0;
    double yIn = 0.0;
    double headingDeg = 0.0;
};

} // namespace sapphirelib::odom
