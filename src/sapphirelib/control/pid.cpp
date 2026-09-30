#include "sapphirelib/control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib {

namespace {

// longer timesteps are scheduling hiccups (a preempted task, a paused motion), not control time
constexpr double kMaxPlausibleDtS = 0.5;

} // namespace

PID::PID(Config config) : config_(config) {}

double PID::update(double target, double measurement) {
    return update(target, measurement, config_.nominalDtS);
}

double PID::update(double target, double measurement, double dtS) {
    std::uint8_t flags = hasPrev_ ? 0 : PidStep::kFirstStep;
    if (!(dtS > 0.0) || dtS > kMaxPlausibleDtS) {
        dtS = config_.nominalDtS;
        flags |= PidStep::kDtFallback;
    }

    const double error = target - measurement;

    // integrate over time, so kI is per second and stays valid if the loop period changes
    const double integralDelta = error * dtS;
    integral_ += integralDelta;
    if (config_.integralLimit > 0.0) {
        integral_ = std::clamp(integral_, -config_.integralLimit, config_.integralLimit);
    }

    // derivative per second, for the same reason
    double derivative = 0.0;
    if (hasPrev_) {
        derivative = (config_.derivativeOnMeasurement ? -(measurement - prevMeasurement_)
                                                      : (error - prevError_)) /
                     dtS;
    }

    double output =
        config_.gains.kP * error + config_.gains.kI * integral_ + config_.gains.kD * derivative;

    // anti-windup: if the output is already past the limit and this tick's error pushes it
    // further, undo this tick's integration. It can't affect anything, and would only come back
    // as overshoot
    if (config_.outputLimit > 0.0 && integralDelta != 0.0 &&
        std::fabs(output) > config_.outputLimit && (output > 0.0) == (integralDelta > 0.0)) {
        integral_ -= integralDelta;
        output = config_.gains.kP * error + config_.gains.kI * integral_ +
                 config_.gains.kD * derivative;
        flags |= PidStep::kIntegralHeld;
    }

    const double rawOutput = output;
    if (config_.slewRate > 0.0 && hasPrev_) {
        const double delta = std::clamp(output - prevOutput_, -config_.slewRate, config_.slewRate);
        // compare against the same expression it was clamped from, so rounding can't flag it
        if (delta != output - prevOutput_) flags |= PidStep::kSlewLimited;
        output = prevOutput_ + delta;
    }

    if (config_.outputLimit > 0.0) {
        const double unclamped = output;
        output = std::clamp(output, -config_.outputLimit, config_.outputLimit);
        if (output != unclamped) flags |= PidStep::kSaturated;
    }

    prevError_ = error;
    prevMeasurement_ = measurement;
    prevOutput_ = output;
    hasPrev_ = true;

    // work out the terms separately from the output expression above, so observing a PID can't
    // change its math by even a rounding step. They can differ from rawOutput in the last bit
    lastStep_ = PidStep{
        .target = target,
        .measurement = measurement,
        .error = error,
        .pTerm = config_.gains.kP * error,
        .iTerm = config_.gains.kI * integral_,
        .dTerm = config_.gains.kD * derivative,
        .rawOutput = rawOutput,
        .output = output,
        .dtS = dtS,
        .flags = flags,
    };
    if (observer_ != nullptr) observer_->onPidUpdate(*this, lastStep_);

    return output;
}

void PID::reset() {
    const bool hadState = hasPrev_;
    integral_ = 0.0;
    prevError_ = 0.0;
    prevMeasurement_ = 0.0;
    prevOutput_ = 0.0;
    hasPrev_ = false;
    if (hadState && observer_ != nullptr) observer_->onPidReset(*this);
}

void PID::setGains(PIDGains gains) {
    config_.gains = gains;
}

const PIDGains& PID::gains() const {
    return config_.gains;
}

const PID::Config& PID::config() const {
    return config_;
}

const PidStep& PID::lastStep() const {
    return lastStep_;
}

void PID::setObserver(PidObserver* observer) {
    observer_ = observer;
}

PidObserver* PID::observer() const {
    return observer_;
}

} // namespace sapphirelib
