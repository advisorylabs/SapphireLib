/**
 * \file sapphirelib/mechanism/roller.hpp
 *
 * An intake, conveyor, or any other open-loop roller, with optional anti-jam
 * (see jam_detector.hpp). Which way it spins and when is your robot's
 * policy — usually an `if` chain over buttons — so this adds only the one
 * genuinely reusable roller behavior on top of a motor group.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <initializer_list>

#include "pros/motor_group.hpp"
#include "sapphirelib/mechanism/jam_detector.hpp"

namespace sapphirelib::mechanism {

///   Roller intake({-18, 11}, {.enabled = true});
///   ...every tick:
///   intake.spin(controller.held(Button::r1) ? 12.0 : 0.0, controller.now());
///
/// Call spin() every tick, even when the answer is 0V: jam detection only
/// works while it's being fed. The constructor only builds the PROS motor
/// group (no device commands), so namespace scope is safe. Not thread-safe:
/// drive it from one task.
class Roller {
public:
    explicit Roller(std::initializer_list<std::int8_t> ports, JamConfig jam = {});

    /// Spins at `volts` (clamped to ±12; positive is the motors' forward), or
    /// at the jam detector's reverse pulse while it's clearing a jam. `nowMs`
    /// is the tick's one "now". A NaN command sends 0V.
    void spin(double volts, std::uint32_t nowMs);

    /// 0V, and forgets any jam in progress.
    void stop();

    /// Reversing to clear a jam right now — e.g. to rumble the controller.
    bool jammed() const;

    /// The motors: for brake mode, current, temperature, and so on.
    pros::MotorGroup& motors();

private:
    /// Mean speed of the motors that are answering, in RPM (NaN if none
    /// are). Read per motor rather than with get_actual_velocity_all(),
    /// which would allocate a vector every tick.
    double speedRpm() const;

    pros::MotorGroup motors_;
    JamDetector jam_;
};

} // namespace sapphirelib::mechanism
