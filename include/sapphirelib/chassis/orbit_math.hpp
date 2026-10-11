#pragma once

#include "sapphirelib/odom/pose.hpp"

namespace sapphirelib::chassis {

/**
 * @brief The point an orbit circles, in the same frame as the poses it's measured against
 */
struct OrbitPivot {
    /** x position, in inches */
    double xIn = 0.0;
    /** y position, in inches */
    double yIn = 0.0;
};

/**
 * @brief Place a pivot straight behind a pose, to start an orbit
 *
 * @param pose where the chassis is
 * @param radiusIn how far behind the tracking center, in inches
 * @param pivotAhead true to place it ahead instead, for driving with front and back swapped
 * @return OrbitPivot the pivot
 */
OrbitPivot pivotBehind(odom::Pose pose, double radiusIn, bool pivotAhead);

/**
 * @brief Where a chassis stands relative to the pivot it's orbiting
 */
struct OrbitFrame {
    /** the pivot, pushed back along the line to the chassis if it came closer than the minimum */
    OrbitPivot pivot;

    /** distance from the pivot to the tracking center, in inches */
    double radiusIn = 0.0;

    /**
     * how far the chassis has to turn to face straight away from the pivot, its back (or with
     * pivotAhead its front) at it, in degrees. Clockwise positive, -180 to 180
     */
    double headingErrorDeg = 0.0;
};

/**
 * @brief Measure an orbit: the radius, and the turn that would put the chassis's back to the pivot
 *
 * @param pivot the pivot
 * @param pose where the chassis is, in the pivot's frame
 * @param pivotAhead true if the chassis's front faces the pivot rather than its back
 * @param minRadiusIn the closest the chassis can get, in inches. Closer, and the pivot is pushed
 * back to this distance, so the turn per inch of travel around it stays bounded
 * @return OrbitFrame the radius, the heading error, and the (possibly pushed) pivot
 */
OrbitFrame measureOrbit(OrbitPivot pivot, odom::Pose pose, bool pivotAhead, double minRadiusIn);

/**
 * @brief Translation in the chassis's own frame, in whatever units it was asked for in
 */
struct OrbitTranslation {
    /** forward, positive ahead */
    double forward = 0.0;
    /** sideways, positive right */
    double strafe = 0.0;
};

/**
 * @brief Turn an orbit request into the chassis's forward and strafe
 *
 * The request is in the orbit's own frame: straight away from the pivot, and around it, positive
 * clockwise (to the right, with the pivot behind). It's rotated by the heading error first, so the
 * chassis travels along the real circle even while its heading still lags the turn: a lagging
 * heading then only means it isn't quite facing away yet, never that the circle moves
 *
 * @param radial away from the pivot (negative toward it), e.g. volts
 * @param around around the pivot, in the same units
 * @param headingErrorDeg OrbitFrame::headingErrorDeg
 * @param pivotAhead true if the chassis's front faces the pivot rather than its back
 * @return OrbitTranslation forward and strafe, in the same units
 */
OrbitTranslation orbitTranslation(double radial, double around, double headingErrorDeg,
                                  bool pivotAhead);

} // namespace sapphirelib::chassis
