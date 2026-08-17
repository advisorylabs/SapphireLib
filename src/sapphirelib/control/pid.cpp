#include "sapphirelib/control/pid.hpp"

#include <algorithm>

namespace sapphirelib {

PID::PID(Config config) : config_(config) {}

double PID::update(double target, double measurement) {
    const double error = target - measurement;

    integral_ += error;
    if (config_.integralLimit > 0.0) {
        integral_ = std::clamp(integral_, -config_.integralLimit, config_.integralLimit);
    }

    double derivative = 0.0;
    if (hasPrev_) {
        derivative = config_.derivativeOnMeasurement ? -(measurement - prevMeasurement_)
                                                       : (error - prevError_);
    }

    double output =
        config_.gains.kP * error + config_.gains.kI * integral_ + config_.gains.kD * derivative;

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
