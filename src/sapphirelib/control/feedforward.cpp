#include "sapphirelib/control/feedforward.hpp"

#include <cmath>

namespace sapphirelib {

namespace {

double signOf(double value) { return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0); }

} // namespace

bool MotorFeedforward::valid() const { return kV > 0.0 && kA > 0.0; }

double MotorFeedforward::volts(double velocity, double acceleration) const {
    const double direction = velocity != 0.0 ? signOf(velocity) : signOf(acceleration);
    return kS * direction + kV * velocity + kA * acceleration;
}

double MotorFeedforward::maxVelocity(double availableVolts) const {
    if (!(kV > 0.0)) return 0.0;
    return std::fmax(0.0, (std::fabs(availableVolts) - kS) / kV);
}

} // namespace sapphirelib
