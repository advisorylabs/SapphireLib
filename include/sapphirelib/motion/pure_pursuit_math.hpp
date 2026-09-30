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
 * fromIndex on, and keeps the furthest along. Once the robot is within lookaheadIn of every
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

} // namespace sapphirelib::motion
