#pragma once

#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"

namespace sapphirelib::odom {

/**
 * @brief A tracking wheel made from a drivetrain's motor encoders
 *
 * For odometry with no dedicated tracking wheels. Less accurate, since drive wheels slip, but needs
 * no extra hardware. Only works as the vertical (forward) wheel, with a verticalOffsetIn of 0
 *
 * @b Example
 * @code {.cpp}
 * // left side motors, 3.25" wheels
 * sapphirelib::odom::MotorGroupTrackingWheel forward(leftMotors, 3.25);
 * @endcode
 */
class MotorGroupTrackingWheel : public TrackingWheel {
public:
    /**
     * @brief Construct a new MotorGroupTrackingWheel
     *
     * @param motors the motors to read. Must outlive the tracking wheel
     * @param wheelDiameterIn drive wheel diameter, in inches
     * @param externalGearRatio motor rotations per wheel rotation. 1 by default
     */
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
