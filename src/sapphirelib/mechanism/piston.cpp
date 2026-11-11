#include "sapphirelib/mechanism/piston.hpp"

#include "pros/adi.h"
#include "pros/ext_adi.h"

namespace sapphirelib::mechanism {

Piston::Piston(std::uint8_t adiPort, bool extendedAtStart, bool extendedIsLow)
    : pneumatics_(adiPort, extendedAtStart, extendedIsLow), state_(extendedAtStart),
      smartPort_(INTERNAL_ADI_PORT), adiPort_(adiPort), extendedIsLow_(extendedIsLow) {}

Piston::Piston(pros::adi::ext_adi_port_pair_t expanderPort, bool extendedAtStart,
               bool extendedIsLow)
    : pneumatics_(expanderPort, extendedAtStart, extendedIsLow), state_(extendedAtStart),
      smartPort_(expanderPort.first), adiPort_(expanderPort.second),
      extendedIsLow_(extendedIsLow) {}

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
    // the port set up again only if the brain doesn't have it as an output (the constructor's
    // dropped): setting it up again takes it low, even as what it already is, and the level
    // written straight after can be lost with it. SL000066: both side toggles, held out, dropped
    // at the end of a routine as this resent them, and stayed in. So not every time
    if (pros::c::ext_adi_port_get_config(smartPort_, adiPort_) != pros::E_ADI_DIGITAL_OUT) {
        pros::c::ext_adi_port_set_config(smartPort_, adiPort_, pros::E_ADI_DIGITAL_OUT);
    }
    // then the level, straight to the port. Not through pneumatics_: its extend() and retract()
    // write nothing when it thinks it's there already. The level is the one pneumatics_ thinks it
    // sent, so it stays in step
    pros::c::ext_adi_digital_write(smartPort_, adiPort_, state_.value() != extendedIsLow_);
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
