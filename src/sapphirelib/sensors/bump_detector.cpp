#include "sapphirelib/sensors/bump_detector.hpp"

#include <cmath>

#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::sensors {

namespace {

// calls further apart than this start the average over: nobody was calling update() (the caller
// paused, or a task stalled), so the average is stale and the next reading could look like a jump
constexpr std::uint32_t kMaxGapMs = 100;

} // namespace

BumpDetector::BumpDetector(BumpConfig config) : config_(config) {}

bool BumpDetector::update(double xG, double yG, double zG, std::uint32_t nowMs) {
    const double reading[3] = {xG, yG, zG};
    if (!std::isfinite(xG) || !std::isfinite(yG) || !std::isfinite(zG)) {
        hasAverage_ = false;
        lastJoltG_ = 0.0;
        return false;
    }

    const std::uint32_t dtMs = elapsedMs(lastMs_, nowMs);
    lastMs_ = nowMs;
    if (!hasAverage_ || dtMs > kMaxGapMs) {
        // this reading becomes the average
        for (int axis = 0; axis < 3; ++axis) averageG_[axis] = reading[axis];
        hasAverage_ = true;
        lastJoltG_ = 0.0;
        return false;
    }

    // measured against the average before this reading joins it, or a jolt would partly hide
    // itself
    double squaredG = 0.0;
    const double weight = config_.averageMs > 0.0
                              ? 1.0 - std::exp(-static_cast<double>(dtMs) / config_.averageMs)
                              : 1.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double offG = reading[axis] - averageG_[axis];
        squaredG += offG * offG;
        averageG_[axis] += weight * offG;
    }
    lastJoltG_ = std::sqrt(squaredG);

    if (holding_ && elapsedMs(bumpMs_, nowMs) < config_.holdoffMs) return false;
    holding_ = false;
    if (lastJoltG_ < config_.thresholdG) return false;
    holding_ = true;
    bumpMs_ = nowMs;
    return true;
}

double BumpDetector::lastJoltG() const { return lastJoltG_; }

void BumpDetector::reset() {
    hasAverage_ = false;
    lastJoltG_ = 0.0;
    holding_ = false;
}

const BumpConfig& BumpDetector::config() const { return config_; }

} // namespace sapphirelib::sensors
