#pragma once

#include <vector>

namespace sapphirelib::motion {

/**
 * @brief A point on a path, in the same field frame as odom::Pose
 */
struct Waypoint {
    /** x position, in inches */
    double xIn = 0.0;
    /** y position, in inches */
    double yIn = 0.0;
};

/**
 * @brief A list of waypoints for followPath()
 *
 * Points aren't smoothed or filled in, so add points between far-apart waypoints yourself for a
 * smoother line
 *
 * @b Example
 * @code {.cpp}
 * const sapphirelib::motion::Path path({{0, 0}, {0, 24}, {12, 36}, {24, 48}});
 * @endcode
 */
class Path {
public:
    /**
     * @brief Construct a new Path
     *
     * @param waypoints the points, in order. Must have at least one
     */
    explicit Path(std::vector<Waypoint> waypoints);

    /**
     * @brief Get the waypoints
     */
    const std::vector<Waypoint>& waypoints() const;

private:
    std::vector<Waypoint> waypoints_;
};

} // namespace sapphirelib::motion
