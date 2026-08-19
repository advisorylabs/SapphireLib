#include "sapphirelib/odom/rotation_tracking_wheel.hpp"

namespace sapphirelib::odom {

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

RotationTrackingWheel::RotationTrackingWheel(std::int8_t port, double wheelDiameterIn,
                                              double externalGearRatio)
    : rotation_(port), wheelDiameterIn_(wheelDiameterIn), externalGearRatio_(externalGearRatio) {
    rotation_.reset_position();
}

double RotationTrackingWheel::getDistanceIn() const {
    // get_position() is cumulative (non-wrapping) centidegrees of sensor
    // rotation, unlike get_angle() which wraps every revolution — cumulative
    // is what odometry needs to compute a delta since the last update.
    const double sensorDegrees = rotation_.get_position() / 100.0;
    const double wheelCircumferenceIn = wheelDiameterIn_ * kPi;
    return (sensorDegrees / 360.0 / externalGearRatio_) * wheelCircumferenceIn;
}

void RotationTrackingWheel::reset() { rotation_.reset_position(); }

} // namespace sapphirelib::odom
