/**
 * \file sapphirelib/odom/odometry_config.hpp
 *
 * Config shared across all four supported odometry sensor combinations.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::odom {

/// Which of the four supported sensor combinations is active is determined
/// purely by which TrackingWheel pointers you pass to Odometry::Sensors —
/// this struct only holds the offsets those wheels need.
struct OdometryConfig {
    /// Perpendicular distance from the vertical tracking wheel (or drive
    /// encoders, if used as the forward-distance fallback) to the robot's
    /// tracking center, in inches. Positive = wheel is to the right of
    /// center. Leave at 0 when using the drive-encoder fallback. Determine
    /// for a real tracking wheel via calibrateTrackingWheelOffsetIn().
    double verticalOffsetIn = 0.0;

    /// Perpendicular distance from the horizontal tracking wheel to the
    /// robot's tracking center, in inches. Positive = wheel is ahead of
    /// center. Unused when there's no horizontal wheel.
    double horizontalOffsetIn = 0.0;
};

} // namespace sapphirelib::odom
