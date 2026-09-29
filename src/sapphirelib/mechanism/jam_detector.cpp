#include "sapphirelib/mechanism/jam_detector.hpp"

#include <cmath>

namespace sapphirelib::mechanism {

namespace {

/// Calls further apart than this start the stall timing over. A driver loop
/// ticks every 10-20ms, so a longer gap means nobody was calling update() —
/// autonomous ran, the robot was disabled — and "slow since 3 seconds ago"
/// would fire a pulse the instant the roller is next commanded. The same
/// 100ms the controller's own restart detection uses.
constexpr std::uint32_t kMaxGapMs = 100;

} // namespace

JamDetector::JamDetector(JamConfig config) : config_(config), gap_(kMaxGapMs) {}

double JamDetector::update(double commandVolts, double velocityRpm, std::uint32_t nowMs) {
    if (!config_.enabled) return commandVolts;

    if (gap_.update(nowMs)) reset();

    // `>=` is false for a NaN command, which then counts as not watched.
    const int direction =
        std::fabs(commandVolts) >= config_.minCommandVolts ? (commandVolts > 0.0 ? 1 : -1) : 0;
    if (direction != direction_) {
        // Let go, or turned around: a pulse against the old direction is no
        // longer wanted, and the roller needs a fresh spin-up before being
        // slow means anything.
        pulse_.stop();
        slow_ = TimedFlag(false, nowMs);
        direction_ = direction;
    }

    if (pulse_.running()) {
        if (!pulse_.hasElapsed(config_.reverseMs, nowMs)) return pulseVolts();
        // Pulse over: back to the command, with a full stallMs before the
        // next one — the roller has to turn around again first.
        pulse_.stop();
        slow_ = TimedFlag(false, nowMs);
    }

    // `<` is false for NaN and for the infinity PROS reports from an
    // unplugged motor, so neither can look like a stall.
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
    // Opposite the direction that jammed. fabs() so a config written with a
    // negative number still pulses the right way.
    return -direction_ * std::fabs(config_.reverseVolts);
}

} // namespace sapphirelib::mechanism
