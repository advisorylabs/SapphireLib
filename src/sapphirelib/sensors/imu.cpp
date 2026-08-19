#include "sapphirelib/sensors/imu.hpp"

#include "sapphirelib/sensors/imu_scale_math.hpp"

namespace sapphirelib::sensors {

Imu::Imu(std::uint8_t port, double headingScale) : imu_(port), headingScale_(headingScale) {
    imu_.reset(true);
    lastRawHeadingDeg_ = imu_.get_heading();
}

void Imu::updateCumulative() {
    const double rawHeadingDeg = imu_.get_heading();
    rawCumulativeDeg_ += rawHeadingDeltaDeg(lastRawHeadingDeg_, rawHeadingDeg);
    lastRawHeadingDeg_ = rawHeadingDeg;
}

double Imu::getCumulativeHeadingDeg() {
    updateCumulative();
    return rawCumulativeDeg_ * headingScale_;
}

double Imu::getHeadingDeg() { return wrapDegrees360(getCumulativeHeadingDeg()); }

void Imu::setHeadingScale(double headingScale) { headingScale_ = headingScale; }

double Imu::headingScale() const { return headingScale_; }

} // namespace sapphirelib::sensors
