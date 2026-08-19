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

#include <cstdint>

#include "pros/rotation.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"

namespace sapphirelib::odom {

class RotationTrackingWheel : public TrackingWheel {
public:
    /// `port` follows pros::Rotation's convention (negative = reversed).
    /// `wheelDiameterIn` is the tracking wheel's diameter, not the sensor's.
    RotationTrackingWheel(std::int8_t port, double wheelDiameterIn, double externalGearRatio = 1.0);

    double getDistanceIn() const override;
    void reset() override;

private:
    pros::Rotation rotation_;
    double wheelDiameterIn_;
    double externalGearRatio_;
};

} // namespace sapphirelib::odom
