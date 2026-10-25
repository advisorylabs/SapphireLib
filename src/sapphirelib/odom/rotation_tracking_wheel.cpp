#include "sapphirelib/odom/rotation_tracking_wheel.hpp"

#include "pros/error.h"

namespace sapphirelib::odom {

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

RotationTrackingWheel::RotationTrackingWheel(std::int8_t port, double wheelDiameterIn,
                                             double externalGearRatio)
    : rotation_(port), externalGearRatio_(externalGearRatio),
      scale_(Scale{.wheelDiameterIn = wheelDiameterIn}) {
    rotation_.reset_position();
}

std::int32_t RotationTrackingWheel::readCentidegrees() const {
    // get_position() is cumulative centidegrees, unlike get_angle() which wraps every turn
    const std::int32_t centidegrees = rotation_.get_position();

    // PROS_ERR while unplugged, which would teleport the pose ~515,000in. Hold the last good
    // reading instead, which reads as "didn't move"
    if (centidegrees == PROS_ERR) return lastGoodCentidegrees_.load();
    lastGoodCentidegrees_.store(centidegrees);
    return centidegrees;
}

double RotationTrackingWheel::inchesFor(std::int32_t centidegrees, double wheelDiameterIn) const {
    const double sensorDegrees = centidegrees / 100.0;
    const double wheelCircumferenceIn = wheelDiameterIn * kPi;
    return (sensorDegrees / 360.0 / externalGearRatio_) * wheelCircumferenceIn;
}

double RotationTrackingWheel::getDistanceIn() const {
    // read with the scale locked, so a setWheelDiameterIn() can't move the base between the
    // reading and the sum
    pros::MutexVarLock<Scale> scale = scale_.lock();
    return scale->baseIn +
           inchesFor(readCentidegrees() - scale->baseCentidegrees, scale->wheelDiameterIn);
}

void RotationTrackingWheel::reset() {
    pros::MutexVarLock<Scale> scale = scale_.lock();
    rotation_.reset_position();
    // so a reset while unplugged still reads 0
    lastGoodCentidegrees_.store(0);
    scale->baseIn = 0.0;
    scale->baseCentidegrees = 0;
}

double RotationTrackingWheel::wheelDiameterIn() const { return scale_.lock()->wheelDiameterIn; }

void RotationTrackingWheel::setWheelDiameterIn(double wheelDiameterIn) {
    pros::MutexVarLock<Scale> scale = scale_.lock();
    const std::int32_t centidegrees = readCentidegrees();
    scale->baseIn += inchesFor(centidegrees - scale->baseCentidegrees, scale->wheelDiameterIn);
    scale->baseCentidegrees = centidegrees;
    scale->wheelDiameterIn = wheelDiameterIn;
}

} // namespace sapphirelib::odom
