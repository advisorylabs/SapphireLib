/**
 * \file sapphirelib/odom/motor_group_tracking_wheel.hpp
 *
 * Adapts a drivetrain's MotorGroup into a TrackingWheel, for the "IMU +
 * drive encoders only" odometry config (no dedicated tracking wheel).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"

namespace sapphirelib::odom {

/// Less accurate than a real tracking wheel — drive wheels can slip,
/// especially on a holonomic chassis or during collisions — but works with
/// zero extra hardware. Always used as the *vertical* (forward) source with
/// a 0 OdometryConfig::verticalOffsetIn (it approximates the tracking
/// center directly, not an offset wheel); there's no meaningful
/// drive-encoder substitute for a horizontal wheel, so configs without one
/// simply can't track lateral motion.
class MotorGroupTrackingWheel : public TrackingWheel {
public:
    MotorGroupTrackingWheel(chassis::MotorGroup& motors, double wheelDiameterIn,
                             double externalGearRatio = 1.0);

    double getDistanceIn() const override;
    void reset() override;

private:
    chassis::MotorGroup& motors_;
    double wheelDiameterIn_;
    double externalGearRatio_;
};

} // namespace sapphirelib::odom
