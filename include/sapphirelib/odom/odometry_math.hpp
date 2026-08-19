/**
 * \file sapphirelib/odom/odometry_math.hpp
 *
 * The tracking-wheel-and-IMU ("arc") odometry algorithm, factored out as
 * pure math with no PROS dependency, so it can be unit-tested on a desktop
 * compiler (see tests/odom/odometry_math_test.cpp). Odometry (odometry.hpp)
 * wraps this with the actual sensor reads and background task.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::odom {

/// Field-frame displacement for one odometry update.
struct PoseDelta {
    double dxIn = 0.0;
    double dyIn = 0.0;
};

/// Computes one odometry update's field-frame displacement from raw sensor
/// deltas.
///
/// `lastHeadingDeg`/`headingDeg` are absolute IMU headings (0-360,
/// clockwise-positive) before/after this update. `verticalDeltaIn`/
/// `horizontalDeltaIn` are the vertical/horizontal tracking wheels' (or, for
/// vertical, a drive-encoder fallback's) signed distance traveled since the
/// last update, in inches — pass 0 for whichever axis has no sensor.
/// `verticalOffsetIn`/`horizontalOffsetIn` are each wheel's perpendicular
/// distance from the robot's tracking center, in inches (0 for an absent
/// wheel, or for a drive-encoder fallback, which approximates the tracking
/// center directly rather than an offset wheel).
///
/// This is the standard two-wheel-and-IMU ("arc") tracking algorithm: each
/// wheel's raw delta includes an arc-length contribution purely from
/// rotation (offset * radians turned), which is subtracted out to leave the
/// wheel's true translational contribution; that local (forward, lateral)
/// displacement is then rotated into the field frame using the *average*
/// heading across the update — more accurate than using either endpoint
/// alone for anything but an infinitesimal update.
PoseDelta computeOdometryDelta(double lastHeadingDeg, double headingDeg, double verticalDeltaIn,
                                double horizontalDeltaIn, double verticalOffsetIn,
                                double horizontalOffsetIn);

/// Calibration helper for a tracking wheel's offset (OdometryConfig::
/// verticalOffsetIn / horizontalOffsetIn): spin the chassis in place through
/// a known total rotation — e.g. 10 full turns, tracked by summing
/// *unwrapped* IMU heading deltas (not wrapDegrees180'd, since the rotation
/// is cumulative and normally exceeds +-180) — and record how far the
/// tracking wheel moved over that same rotation. `wheelDistanceIn` is the
/// wheel's total signed distance traveled; `rotatedRadians` is the total
/// angle turned, in radians (e.g. 10 turns = 10 * 2 * pi). Returns the
/// wheel's perpendicular offset from the tracking center, in inches.
double calibrateTrackingWheelOffsetIn(double wheelDistanceIn, double rotatedRadians);

} // namespace sapphirelib::odom
