/**
 * \file sapphirelib/odom/tracking_wheel.hpp
 *
 * Abstract distance source for odometry. Implemented by
 * RotationTrackingWheel (a dedicated tracking wheel) and
 * MotorGroupTrackingWheel (drive motor encoders, used as a forward-distance
 * fallback when no dedicated tracking wheel exists — see OdometryConfig).
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::odom {

/// Something that reports cumulative signed distance traveled, in inches,
/// along one axis — the common interface Odometry reads from, regardless of
/// whether that axis is backed by a dedicated tracking wheel or a
/// drivetrain's own encoders.
class TrackingWheel {
public:
    virtual ~TrackingWheel() = default;

    /// Cumulative signed distance traveled, in inches, since construction or
    /// the last reset().
    virtual double getDistanceIn() const = 0;

    /// Zeroes getDistanceIn().
    virtual void reset() = 0;
};

} // namespace sapphirelib::odom
