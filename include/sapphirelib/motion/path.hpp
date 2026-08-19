/**
 * \file sapphirelib/motion/path.hpp
 *
 * An ordered sequence of field waypoints for followPath() to pursue.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <vector>

namespace sapphirelib::motion {

/// A single point on a field path, in the same x/y coordinate frame as
/// odom::Pose.
struct Waypoint {
    double xIn = 0.0;
    double yIn = 0.0;
};

/// Points aren't smoothed or interpolated — for widely spaced waypoints,
/// densify the path yourself (e.g. linearly interpolate extra points
/// between them) for a smoother pursuit line. Must contain at least one
/// waypoint.
class Path {
public:
    explicit Path(std::vector<Waypoint> waypoints);

    const std::vector<Waypoint>& waypoints() const;

private:
    std::vector<Waypoint> waypoints_;
};

} // namespace sapphirelib::motion
