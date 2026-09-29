/**
 * \file sapphirelib/mechanism/piston.hpp
 *
 * A pneumatic piston that remembers how long it has been in its current
 * state ("outtake once the claw has had 400ms to deploy"), on top of
 * pros::adi::Pneumatics.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

#include "pros/adi.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::mechanism {

/// Drive it only through this class: calling the underlying Pneumatics
/// directly would desync the time-in-state. Safe at namespace scope, as
/// pros::adi::Pneumatics itself is. Not thread-safe: drive it from one task.
///
///   Piston claw('A');
///   ...every tick:
///   claw.set(clawWanted, now);                   // level-triggered is fine
///   if (claw.extendedFor(400, now)) outtake();   // it's had time to deploy
class Piston {
public:
    /// `adiPort` is 'A'-'H' (or 1-8) on the brain. Set `extendedIsLow` for a
    /// valve plumbed so that extended is the port's low state. The
    /// time-in-state starts counting from program start.
    explicit Piston(std::uint8_t adiPort, bool extendedAtStart = false, bool extendedIsLow = false);

    /// On a 3-wire expander: {smart port, 'A'-'H'}.
    explicit Piston(pros::adi::ext_adi_port_pair_t expanderPort, bool extendedAtStart = false,
                    bool extendedIsLow = false);

    /// Extends or retracts. A no-op when it's already there, and the
    /// time-in-state keeps counting from the real change, so calling this
    /// every tick with a level-triggered value is fine. True if it moved.
    bool set(bool extended, std::uint32_t nowMs);

    /// Flips it. Always moves, so always true.
    bool toggle(std::uint32_t nowMs);

    bool extended() const;

    /// Time since the last actual extend or retract.
    std::uint32_t msInState(std::uint32_t nowMs) const;

    /// Extended, and for at least `ms`, e.g. "it's had time to deploy".
    bool extendedFor(std::uint32_t ms, std::uint32_t nowMs) const;

    bool retractedFor(std::uint32_t ms, std::uint32_t nowMs) const;

private:
    pros::adi::Pneumatics pneumatics_;
    /// The piston's state and when it last changed. It and pneumatics_ start
    /// from the same extendedAtStart and only ever change together, in set().
    TimedFlag state_;
};

} // namespace sapphirelib::mechanism
