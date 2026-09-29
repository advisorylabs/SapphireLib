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
    // reset(true) returns PROS_ERR straight away when nothing is on the
    // port, and after a timeout when calibration never finishes. Say so,
    // rather than letting initialize() go on to log "IMU calibrated".
    calibrated_ = imu_.reset(true) != PROS_ERR;
    if (!calibrated_) {
        SAPPHIRELIB_LOG_ERROR("imu", "port %u: calibration failed", static_cast<unsigned>(port));
    }
    updateCumulative();
}

bool Imu::calibrated() const { return calibrated_; }

void Imu::updateCumulative() {
    const double rawHeadingDeg = imu_.get_heading();
    // PROS_ERR_F (infinity) while the IMU is unplugged or recalibrating
    // after a brownout. Folding one of those into the sum would make it NaN
    // for the rest of the program, so a bad read just skips this update.
    //
    // It also forgets the baseline. A sensor that dropped out has usually
    // lost power, and it comes back reading near 0 wherever the chassis now
    // points — measuring from the pre-dropout reading would count that
    // restart as a real turn and leave heading wrong by the old reading for
    // the rest of the match. Re-baselining on the next good read (below)
    // loses only whatever the chassis actually turned during the gap.
    if (!std::isfinite(rawHeadingDeg)) {
        lastRawHeadingDeg_.store(std::numeric_limits<double>::quiet_NaN());
        return;
    }

    // Called concurrently from odometry, the drive loop, HomePage and
    // tuning runs. exchange() pairs every reading with exactly the one
    // stored before it, so the deltas still add up to (latest - first)
    // however the calls interleave — the unsynchronized read-add-write this
    // replaced could count one delta twice and leave a permanent offset.
    // And there's no lock for a deleted competition task to orphan.
    const double previousDeg = lastRawHeadingDeg_.exchange(rawHeadingDeg);

    // No baseline (the sensor didn't answer at construction, or just came
    // back from a dropout): this reading becomes it, rather than a jump of up
    // to 180 degrees.
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
    // Measured against a fresh reading, so it's "here, now". A turn landing
    // between this read and the store is lost from the new frame, but that's
    // one control tick's rotation at most, and only at the moment of a
    // re-frame.
    headingOffsetDeg_.store(headingOffsetFor(headingDeg, getCumulativeHeadingDeg()));
}

double Imu::headingOffsetDeg() const { return headingOffsetDeg_.load(); }

void Imu::setHeadingScale(double headingScale) { headingScale_.store(headingScale); }

double Imu::headingScale() const { return headingScale_.load(); }

} // namespace sapphirelib::sensors
