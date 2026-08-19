#include "sapphirelib/odom/motor_group_tracking_wheel.hpp"

namespace sapphirelib::odom {

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

MotorGroupTrackingWheel::MotorGroupTrackingWheel(chassis::MotorGroup& motors, double wheelDiameterIn,
                                                   double externalGearRatio)
    : motors_(motors), wheelDiameterIn_(wheelDiameterIn), externalGearRatio_(externalGearRatio) {
    motors_.tarePosition();
}

double MotorGroupTrackingWheel::getDistanceIn() const {
    const double wheelCircumferenceIn = wheelDiameterIn_ * kPi;
    return (motors_.getPositionDegrees() / 360.0 / externalGearRatio_) * wheelCircumferenceIn;
}

void MotorGroupTrackingWheel::reset() { motors_.tarePosition(); }

} // namespace sapphirelib::odom
