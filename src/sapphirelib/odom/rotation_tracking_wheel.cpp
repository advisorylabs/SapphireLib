#include "sapphirelib/odom/rotation_tracking_wheel.hpp"

#include "pros/error.h"

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
    // get_position() is cumulative centidegrees, unlike get_angle() which wraps every turn
    std::int32_t centidegrees = rotation_.get_position();

    // PROS_ERR while unplugged, which would teleport the pose ~515,000in. Hold the last good
    // reading instead, which reads as "didn't move"
    if (centidegrees == PROS_ERR) {
        centidegrees = lastGoodCentidegrees_.load();
    } else {
        lastGoodCentidegrees_.store(centidegrees);
    }

    const double sensorDegrees = centidegrees / 100.0;
    const double wheelCircumferenceIn = wheelDiameterIn_ * kPi;
    return (sensorDegrees / 360.0 / externalGearRatio_) * wheelCircumferenceIn;
}

void RotationTrackingWheel::reset() {
    rotation_.reset_position();
    // so a reset while unplugged still reads 0
    lastGoodCentidegrees_.store(0);
}

} // namespace sapphirelib::odom
