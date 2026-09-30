#include "sapphirelib/mechanism/piston.hpp"

namespace sapphirelib::mechanism {

Piston::Piston(std::uint8_t adiPort, bool extendedAtStart, bool extendedIsLow)
    : pneumatics_(adiPort, extendedAtStart, extendedIsLow), state_(extendedAtStart) {}

Piston::Piston(pros::adi::ext_adi_port_pair_t expanderPort, bool extendedAtStart,
               bool extendedIsLow)
    : pneumatics_(expanderPort, extendedAtStart, extendedIsLow), state_(extendedAtStart) {}

bool Piston::set(bool extended, std::uint32_t nowMs) {
    // only a real change reaches the valve, so callers can pass the wanted state every tick
    if (!state_.set(extended, nowMs)) return false;
    if (extended) {
        pneumatics_.extend();
    } else {
        pneumatics_.retract();
    }
    return true;
}

bool Piston::toggle(std::uint32_t nowMs) { return set(!state_.value(), nowMs); }

bool Piston::extended() const { return state_.value(); }

std::uint32_t Piston::msInState(std::uint32_t nowMs) const { return state_.msInState(nowMs); }

bool Piston::extendedFor(std::uint32_t ms, std::uint32_t nowMs) const {
    return state_.trueFor(ms, nowMs);
}

bool Piston::retractedFor(std::uint32_t ms, std::uint32_t nowMs) const {
    return state_.falseFor(ms, nowMs);
}

} // namespace sapphirelib::mechanism
