/**
 * \file sapphirelib/motion/pure_pursuit_math.hpp
 *
 * Coordinate-frame and lookahead-point math shared by moveToPoint()/
 * moveToPose()/followPath() on both drivetrains. Pure math, no PROS
 * dependency — see tests/motion/pure_pursuit_math_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>

#include "sapphirelib/motion/path.hpp"

namespace sapphirelib::motion {

/// A field-frame displacement rotated into the chassis's local frame: +
/// forward is the direction the chassis is currently facing, + lateral is
/// to its right.
struct LocalOffset {
    double forwardIn = 0.0;
    double lateralIn = 0.0;
};

/// Rotates a field-frame displacement (`dxIn`, `dyIn`) into the chassis's
/// local frame at `headingDeg` (0-360, clockwise-positive, matching
/// pros::Imu::get_heading() and odom::Pose::headingDeg). Used to aim a
/// field-frame position error (moveToPoint()/moveToPose()) or a lookahead
/// point (followPath()) as a drivable local vector.
LocalOffset toLocalFrame(double dxIn, double dyIn, double headingDeg);

/// A lookahead search result: the target point to steer toward, and the
/// path segment index it was found on — feed back in as `fromIndex` on the
/// next call so the search never regresses to an earlier point on the path.
struct LookaheadResult {
    Waypoint point;
    std::size_t segmentIndex;
};

/// Finds the point on `path` at distance `lookaheadIn` from (`xIn`, `yIn`),
/// searching forward from `fromIndex` — the standard pure-pursuit lookahead
/// search: intersect a circle of radius `lookaheadIn` centered on the
/// chassis with each path segment at or after `fromIndex`, keeping the
/// furthest-along valid intersection found across the whole remaining path.
/// Falls back to the path's final waypoint once the chassis is within
/// `lookaheadIn` of every remaining segment (so pursuit converges on the
/// endpoint instead of endlessly searching for an intersection that no
/// longer exists). `path` must contain at least one waypoint.
LookaheadResult findLookaheadPoint(double xIn, double yIn, const Path& path, double lookaheadIn,
                                    std::size_t fromIndex);

} // namespace sapphirelib::motion
