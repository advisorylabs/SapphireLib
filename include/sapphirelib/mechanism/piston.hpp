#pragma once

#include <cstdint>

#include "pros/adi.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::mechanism {

/**
 * @brief A pneumatic piston that remembers how long it's been extended or retracted
 *
 * For "outtake once the claw has had 400ms to deploy". Only drive it through this class, or the
 * time in state gets out of sync. Safe to construct at namespace scope
 *
 * @note not thread-safe, so drive it from one task
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::mechanism::Piston claw('A');
 *
 * // every tick
 * claw.set(clawWanted, now); // calling it every tick with the same value is fine
 * if (claw.extendedFor(400, now)) outtake(); // it's had time to deploy
 * @endcode
 */
class Piston {
public:
    /**
     * @brief Construct a new Piston on a brain ADI port
     *
     * @param adiPort 'A'-'H' (or 1-8)
     * @param extendedAtStart whether it starts extended. false by default
     * @param extendedIsLow whether extended is the port's low state, for a valve plumbed that way.
     * false by default
     */
    explicit Piston(std::uint8_t adiPort, bool extendedAtStart = false, bool extendedIsLow = false);

    /**
     * @brief Construct a new Piston on a 3-wire expander
     *
     * @param expanderPort {smart port, 'A'-'H'}
     * @param extendedAtStart whether it starts extended. false by default
     * @param extendedIsLow whether extended is the port's low state. false by default
     */
    explicit Piston(pros::adi::ext_adi_port_pair_t expanderPort, bool extendedAtStart = false,
                    bool extendedIsLow = false);

    /**
     * @brief Extend or retract the piston
     *
     * Does nothing if it's already there, and the time in state keeps counting from the real
     * change, so calling this every tick is fine
     *
     * @param extended true to extend, false to retract
     * @param nowMs the current time, in milliseconds
     * @return true the piston moved
     */
    bool set(bool extended, std::uint32_t nowMs);

    /**
     * @brief Flip the piston
     *
     * @param nowMs the current time, in milliseconds
     * @return true always, since it always moves
     */
    bool toggle(std::uint32_t nowMs);

    /**
     * @brief Set the port up as an output again, and send the valve the state the piston is in
     *
     * A piston built at namespace scope sets its port up from its constructor, before
     * initialize(), and the brain may not take it: the valve can then sit at the port's low state,
     * whatever `extendedAtStart` said, while extended() says otherwise. Call this once the program
     * is running, at each enable, or every tick, so the valve matches. Always writes the level,
     * even when it hasn't changed, but sets the port up only when the brain doesn't have it as an
     * output: setting a port up again takes it low, and the write right after can be lost. When it
     * did set the port up, call it again a tick later. The piston doesn't move as far as the class
     * knows, so its time in state keeps counting
     */
    void resend();

    /**
     * @brief Whether the piston is extended
     */
    bool extended() const;

    /**
     * @brief Get the time since the piston last moved
     *
     * @param nowMs the current time, in milliseconds
     * @return std::uint32_t time in its current state, in milliseconds
     */
    std::uint32_t msInState(std::uint32_t nowMs) const;

    /**
     * @brief Whether the piston is extended, and has been for at least `ms`
     */
    bool extendedFor(std::uint32_t ms, std::uint32_t nowMs) const;

    /**
     * @brief Whether the piston is retracted, and has been for at least `ms`
     */
    bool retractedFor(std::uint32_t ms, std::uint32_t nowMs) const;

private:
    pros::adi::Pneumatics pneumatics_;
    // the piston's state and when it last changed. Only ever changes together with pneumatics_
    TimedFlag state_;
    // where pneumatics_ is, for resend(): INTERNAL_ADI_PORT for the brain's own ports
    std::uint8_t smartPort_;
    std::uint8_t adiPort_;
    // whether extended is the port's low state, for resend()'s write
    bool extendedIsLow_;
};

} // namespace sapphirelib::mechanism
