#pragma once

#include <cstddef>

#include "sapphirelib/motion/path.hpp"

namespace sapphirelib::motion {

/**
 * @brief A field offset in the robot's frame
 */
struct LocalOffset {
    /** distance ahead of the robot, in inches */
    double forwardIn = 0.0;
    /** distance to the robot's right, in inches */
    double lateralIn = 0.0;
};

/**
 * @brief Rotate a field offset into the robot's frame
 *
 * @param dxIn field x offset, in inches
 * @param dyIn field y offset, in inches
 * @param headingDeg robot heading, 0-360 degrees, clockwise positive
 * @return LocalOffset the offset ahead of and to the right of the robot
 */
LocalOffset toLocalFrame(double dxIn, double dyIn, double headingDeg);

/**
 * @brief Result of a lookahead search
 */
struct LookaheadResult {
    /** the point to steer toward */
    Waypoint point;
    /** the path segment it's on. Pass it back as fromIndex so the search never goes backward */
    std::size_t segmentIndex;
};

/**
 * @brief Find the pure pursuit lookahead point
 *
 * Intersects a circle of radius lookaheadIn around the robot with each path segment from
 * fromIndex on, and keeps the furthest along the stretch of path the circle reaches: once the
 * path has left the circle, a later segment that comes back within reach (a closed lap's last
 * side, beside its first) isn't skipped ahead to. Once the robot is within lookaheadIn of every
 * remaining segment, returns the last waypoint
 *
 * @param xIn robot x, in inches
 * @param yIn robot y, in inches
 * @param path the path. Must have at least one waypoint
 * @param lookaheadIn lookahead distance, in inches
 * @param fromIndex the segment to search from
 * @return LookaheadResult the lookahead point and its segment
 */
LookaheadResult findLookaheadPoint(double xIn, double yIn, const Path& path, double lookaheadIn,
                                   std::size_t fromIndex);

/**
 * @brief Whether followPath() should stop pursuing and drive its final approach
 *
 * Only once pursuit has reached the path's last segment, not just whenever the robot is near the
 * last waypoint: a closed path (a lap that ends where it started) begins right on top of its own
 * end, and the distance alone would finish it before the robot moved
 *
 * @param distanceToFinalIn distance from the robot to the last waypoint, in inches
 * @param finalApproachIn PursuitConfig::finalApproachIn, in inches
 * @param segmentIndex the segment pursuit is on: the last LookaheadResult::segmentIndex, 0 before
 * the first search
 * @param waypointCount how many waypoints the path has, at least 1
 * @return true to switch to the final approach
 */
bool reachedFinalApproach(double distanceToFinalIn, double finalApproachIn,
                          std::size_t segmentIndex, std::size_t waypointCount);

} // namespace sapphirelib::motion
