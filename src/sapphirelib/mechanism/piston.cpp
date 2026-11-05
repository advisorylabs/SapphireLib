#include "sapphirelib/mechanism/piston.hpp"

#include "pros/adi.h"
#include "pros/ext_adi.h"

namespace sapphirelib::mechanism {

Piston::Piston(std::uint8_t adiPort, bool extendedAtStart, bool extendedIsLow)
    : pneumatics_(adiPort, extendedAtStart, extendedIsLow), state_(extendedAtStart),
      smartPort_(INTERNAL_ADI_PORT), adiPort_(adiPort) {}

Piston::Piston(pros::adi::ext_adi_port_pair_t expanderPort, bool extendedAtStart,
               bool extendedIsLow)
    : pneumatics_(expanderPort, extendedAtStart, extendedIsLow), state_(extendedAtStart),
      smartPort_(expanderPort.first), adiPort_(expanderPort.second) {}

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

void Piston::resend() {
    // the port first, in case the brain dropped the constructor's, then the level: extend() and
    // retract() always write, whatever pneumatics_ last sent
    pros::c::ext_adi_port_set_config(smartPort_, adiPort_, pros::E_ADI_DIGITAL_OUT);
    if (state_.value()) {
        pneumatics_.extend();
    } else {
        pneumatics_.retract();
    }
}

bool Piston::extended() const { return state_.value(); }

std::uint32_t Piston::msInState(std::uint32_t nowMs) const { return state_.msInState(nowMs); }

bool Piston::extendedFor(std::uint32_t ms, std::uint32_t nowMs) const {
    return state_.trueFor(ms, nowMs);
}

bool Piston::retractedFor(std::uint32_t ms, std::uint32_t nowMs) const {
    return state_.falseFor(ms, nowMs);
}

} // namespace sapphirelib::mechanism
