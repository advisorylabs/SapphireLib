#include "sapphirelib/control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib {

namespace {

/// Any timestep beyond this is treated as a scheduling hiccup (a preempted
/// task, a paused motion) rather than real elapsed control time — folding
/// it in would dump a huge lump of integral in and divide the derivative by
/// a meaninglessly long interval.
constexpr double kMaxPlausibleDtS = 0.5;

} // namespace

PID::PID(Config config) : config_(config) {}

double PID::update(double target, double measurement) {
    return update(target, measurement, config_.nominalDtS);
}

double PID::update(double target, double measurement, double dtS) {
    if (!(dtS > 0.0) || dtS > kMaxPlausibleDtS) dtS = config_.nominalDtS;

    const double error = target - measurement;

    // Integral of error over time, so kI carries per-second units and stays
    // valid across a change of loop period — see PIDGains' comment.
    const double integralDelta = error * dtS;
    integral_ += integralDelta;
    if (config_.integralLimit > 0.0) {
        integral_ = std::clamp(integral_, -config_.integralLimit, config_.integralLimit);
    }

    // Rate of change per second, for the same reason.
    double derivative = 0.0;
    if (hasPrev_) {
        derivative = (config_.derivativeOnMeasurement ? -(measurement - prevMeasurement_)
                                                      : (error - prevError_)) /
                     dtS;
    }

    double output =
        config_.gains.kP * error + config_.gains.kI * integral_ + config_.gains.kD * derivative;

    // Conditional-integration anti-windup: if the output is already pinned
    // at the limit and this tick's error only pushes it further out, that
    // integration can't affect the plant — it just accumulates charge that
    // has to be paid back as overshoot once the error finally reverses. Roll
    // it back and recompute instead. Only meaningful when an output limit
    // exists to saturate against.
    if (config_.outputLimit > 0.0 && integralDelta != 0.0 &&
        std::fabs(output) > config_.outputLimit && (output > 0.0) == (integralDelta > 0.0)) {
        integral_ -= integralDelta;
        output = config_.gains.kP * error + config_.gains.kI * integral_ +
                 config_.gains.kD * derivative;
    }

    if (config_.slewRate > 0.0 && hasPrev_) {
        const double delta = std::clamp(output - prevOutput_, -config_.slewRate, config_.slewRate);
        output = prevOutput_ + delta;
    }

    if (config_.outputLimit > 0.0) {
        output = std::clamp(output, -config_.outputLimit, config_.outputLimit);
    }

    prevError_ = error;
    prevMeasurement_ = measurement;
    prevOutput_ = output;
    hasPrev_ = true;

    return output;
}

void PID::reset() {
    integral_ = 0.0;
    prevError_ = 0.0;
    prevMeasurement_ = 0.0;
    prevOutput_ = 0.0;
    hasPrev_ = false;
}

void PID::setGains(PIDGains gains) {
    config_.gains = gains;
}

const PIDGains& PID::gains() const {
    return config_.gains;
}

} // namespace sapphirelib
