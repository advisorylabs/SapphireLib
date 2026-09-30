#include "sapphirelib/mechanism/position_control.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib::mechanism {

double GravityFeedforward::volts(double position) const {
    // exactly constantVolts for a plain lift, so no cos() rounding sneaks in
    if (cosineVolts == 0.0) return constantVolts;
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    return constantVolts +
           cosineVolts * std::cos((position - horizontalPosition) * armDegreesPerUnit * kDegToRad);
}

const char* positionLawName(PositionLaw law) {
    switch (law) {
        case PositionLaw::track: return "track";
        case PositionLaw::seat: return "seat";
        case PositionLaw::rest: return "rest";
        case PositionLaw::sensorLost: return "no sensor";
        case PositionLaw::manual: return "manual";
        case PositionLaw::off: return "off";
        case PositionLaw::external: return "external";
    }
    return "?";
}

PositionCommand computePositionCommand(PID& pid, const PositionConfig& config, double target,
                                       double position) {
    // each branch is the same expression, in the same order, as the robot's original lift loop,
    // so a mechanism sends the very same millivolts (checked in position_control_test.cpp)
    if (!std::isfinite(position)) {
        // chasing a missing reading could run into either end, so brake and start fresh once it's
        // back
        pid.reset();
        return {.law = PositionLaw::sensorLost, .volts = 0.0, .brake = true};
    }
    double volts;
    PositionLaw law;
    if (!config.seat.enabled || target > config.seat.floor) {
        volts = pid.update(target, position) + config.gravity.volts(position);
        law = PositionLaw::track;
    } else if (position > config.seat.floor + config.seat.restBand) {
        // heading for the floor. No gravity feedforward, which would hold it up off the stop
        volts = std::min(pid.update(target, position), -config.seat.seatVolts);
        law = PositionLaw::seat;
    } else {
        volts = 0.0;
        pid.reset();
        law = PositionLaw::rest;
    }
    const double clamped = std::clamp(volts, -config.maxVolts, config.maxVolts);
    return {.law = law, .volts = clamped, .brake = false};
}

} // namespace sapphirelib::mechanism
