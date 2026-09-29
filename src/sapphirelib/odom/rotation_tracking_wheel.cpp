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
    // get_position() is cumulative (non-wrapping) centidegrees of sensor
    // rotation, unlike get_angle() which wraps every revolution — cumulative
    // is what odometry needs to compute a delta since the last update.
    std::int32_t centidegrees = rotation_.get_position();

    // PROS_ERR (INT32_MAX) while unplugged. Reading that as a position
    // teleports the pose ~515,000in (2.75in wheel); holding the last good
    // value reads as "didn't move" until the sensor answers again.
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
    // Keeps a reset that lands while the sensor is unplugged reading 0, as
    // reset() promises, rather than the pre-reset position.
    lastGoodCentidegrees_.store(0);
}

} // namespace sapphirelib::odom
