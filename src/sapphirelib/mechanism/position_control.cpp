#include "sapphirelib/mechanism/position_control.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib::mechanism {

double GravityFeedforward::volts(double position) const {
    // Exactly constantVolts for a plain lift, so "loop + this" is bit-for-bit
    // "loop + a bare constant" — no cos() rounding sneaks into a mechanism
    // that never asked for it.
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
    // Every branch below is written as the same expression, in the same
    // order, as the lift loop in the robot's original driver macros, so a
    // mechanism moved onto this law sends the very same millivolts
    // (tests/mechanism/position_control_test.cpp checks that bit for bit).
    if (!std::isfinite(position)) {
        // Chasing a missing reading could run the mechanism into either end,
        // so hold it where it is on the motors' own brake instead, and start
        // the loop fresh once the reading comes back.
        pid.reset();
        return {.law = PositionLaw::sensorLost, .volts = 0.0, .brake = true};
    }
    double volts;
    PositionLaw law;
    if (!config.seat.enabled || target > config.seat.floor) {
        volts = pid.update(target, position) + config.gravity.volts(position);
        law = PositionLaw::track;
    } else if (position > config.seat.floor + config.seat.restBand) {
        // Heading for the floor: see SeatConfig. No gravity feedforward here
        // — it would only hold the mechanism up off the stop it's seating on.
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
