/**
 * \file sapphirelib/odom/rotation_tracking_wheel.hpp
 *
 * TrackingWheel backed by a dedicated VEX Rotation Sensor — the vertical or
 * horizontal wheel in the "IMU + vertical", "IMU + horizontal", or "IMU +
 * both" odometry configs.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "pros/rotation.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"

namespace sapphirelib::odom {

class RotationTrackingWheel : public TrackingWheel {
public:
    /// `port` follows pros::Rotation's convention (negative = reversed).
    /// `wheelDiameterIn` is the tracking wheel's diameter, not the sensor's.
    RotationTrackingWheel(std::int8_t port, double wheelDiameterIn, double externalGearRatio = 1.0);

    /// While the sensor isn't answering (unplugged, a loose cable), holds
    /// the last good reading — "didn't move" — instead of converting PROS's
    /// error value into a distance. Safe to call from several tasks (the
    /// odometry task and an Asterisk drivetrain's drift correction both
    /// read the same wheel).
    double getDistanceIn() const override;
    void reset() override;

private:
    pros::Rotation rotation_;
    double wheelDiameterIn_;
    double externalGearRatio_;

    /// The last reading that wasn't PROS_ERR, in centidegrees. Atomic
    /// because getDistanceIn() updates it from whichever task reads.
    mutable std::atomic<std::int32_t> lastGoodCentidegrees_{0};
};

} // namespace sapphirelib::odom
