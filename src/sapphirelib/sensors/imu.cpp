#include "sapphirelib/sensors/imu.hpp"

#include <cmath>
#include <limits>

#include "pros/error.h"
#include "sapphirelib/sensors/imu_scale_math.hpp"
#include "sapphirelib/util/log.hpp"

namespace sapphirelib::sensors {

Imu::Imu(std::uint8_t port, double headingScale)
    : imu_(port), headingScale_(headingScale),
      lastRawHeadingDeg_(std::numeric_limits<double>::quiet_NaN()) {
    // reset(true) returns PROS_ERR right away with nothing on the port, and after a timeout when
    // calibration never finishes
    calibrated_ = imu_.reset(true) != PROS_ERR;
    if (!calibrated_) {
        SAPPHIRELIB_LOG_ERROR("imu", "port %u: calibration failed", static_cast<unsigned>(port));
    }
    updateCumulative();
}

bool Imu::calibrated() const { return calibrated_; }

void Imu::updateCumulative() {
    const double rawHeadingDeg = imu_.get_heading();
    // infinity while unplugged or recalibrating after a brownout, which would make the sum NaN
    // forever, so skip it. Also forget the baseline: a sensor that dropped out usually lost power
    // and comes back near 0, which isn't a real turn
    if (!std::isfinite(rawHeadingDeg)) {
        lastRawHeadingDeg_.store(std::numeric_limits<double>::quiet_NaN());
        return;
    }

    // called from several tasks at once. exchange() pairs every reading with the one before it, so
    // the deltas always add up to latest - first however the calls interleave, with no lock
    const double previousDeg = lastRawHeadingDeg_.exchange(rawHeadingDeg);

    // no baseline yet (no answer at construction, or back from a dropout): this reading becomes it
    if (!std::isfinite(previousDeg)) return;

    rawCumulativeDeg_.fetch_add(rawHeadingDeltaDeg(previousDeg, rawHeadingDeg));
}

double Imu::getCumulativeHeadingDeg() {
    updateCumulative();
    return rawCumulativeDeg_.load() * headingScale_.load();
}

double Imu::getHeadingDeg() {
    return fieldHeadingDeg(getCumulativeHeadingDeg(), headingOffsetDeg_.load());
}

void Imu::setHeadingDeg(double headingDeg) {
    // against a fresh reading, so it means "here, now"
    headingOffsetDeg_.store(headingOffsetFor(headingDeg, getCumulativeHeadingDeg()));
}

double Imu::headingOffsetDeg() const { return headingOffsetDeg_.load(); }

void Imu::setHeadingScale(double headingScale) { headingScale_.store(headingScale); }

double Imu::headingScale() const { return headingScale_.load(); }

} // namespace sapphirelib::sensors
