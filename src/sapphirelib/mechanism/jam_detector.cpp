#include "sapphirelib/mechanism/jam_detector.hpp"

#include <cmath>

namespace sapphirelib::mechanism {

namespace {

// calls further apart than this start the stall timing over: nobody was calling update()
// (autonomous, disabled), so old timing means nothing
constexpr std::uint32_t kMaxGapMs = 100;

} // namespace

JamDetector::JamDetector(JamConfig config) : config_(config), gap_(kMaxGapMs) {}

double JamDetector::update(double commandVolts, double velocityRpm, std::uint32_t nowMs) {
    if (!config_.enabled) return commandVolts;

    if (gap_.update(nowMs)) reset();

    // >= is false for a NaN command, which counts as not watched
    const int direction =
        std::fabs(commandVolts) >= config_.minCommandVolts ? (commandVolts > 0.0 ? 1 : -1) : 0;
    if (direction != direction_) {
        // let go or turned around: drop any pulse, and wait for a fresh spin-up
        pulse_.stop();
        slow_ = TimedFlag(false, nowMs);
        direction_ = direction;
    }

    if (pulse_.running()) {
        if (!pulse_.hasElapsed(config_.reverseMs, nowMs)) return pulseVolts();
        // pulse over: back to the command, with a full stallMs before the next one
        pulse_.stop();
        slow_ = TimedFlag(false, nowMs);
    }

    // < is false for NaN and infinity (an unplugged motor), so neither looks like a stall
    slow_.set(direction_ != 0 && std::fabs(velocityRpm) < config_.stallRpm, nowMs);
    if (slow_.trueFor(config_.stallMs, nowMs)) {
        pulse_.restart(nowMs);
        return pulseVolts();
    }
    return commandVolts;
}

bool JamDetector::reversing() const { return pulse_.running(); }

void JamDetector::reset() {
    pulse_.stop();
    slow_ = TimedFlag();
    direction_ = 0;
}

const JamConfig& JamDetector::config() const { return config_; }

double JamDetector::pulseVolts() const {
    // opposite the direction that jammed. fabs() in case reverseVolts was written negative
    return -direction_ * std::fabs(config_.reverseVolts);
}

} // namespace sapphirelib::mechanism
