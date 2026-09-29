/**
 * \file sapphirelib/mechanism/position_control.hpp
 *
 * The control law behind mechanism::PositionMechanism, kept pure so it can be
 * checked against a simulated lift on a desktop compiler
 * (tests/mechanism/position_control_test.cpp). Given a target and a position
 * reading, it decides what to command: PID plus gravity feedforward, the
 * optional seat-and-rest at a hard stop, and what to do when the reading is
 * missing.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

#include "sapphirelib/control/pid.hpp"

namespace sapphirelib::mechanism {

/// Volts added to the loop's output to carry the mechanism's weight, so the
/// PID only has to correct error rather than also hold the load up. Without
/// it a proportional loop settles short of every target by exactly the error
/// whose kP·error equals the holding voltage.
struct GravityFeedforward {
    /// Added at every position. For an elevator-style lift, whose load is the
    /// same at every height: raise it until the lift stops settling below its
    /// targets.
    double constantVolts = 0.0;

    /// Volts needed to hold an arm level, scaled by cos(arm angle), since
    /// gravity's torque on an arm falls off as it swings toward vertical.
    double cosineVolts = 0.0;

    /// The position reading at which the arm is horizontal.
    double horizontalPosition = 0.0;

    /// Arm degrees per unit of position reading: 1 for a degrees sensor on the
    /// arm's own pivot, 1/ratio for one on the motor side of a ratio:1
    /// reduction.
    double armDegreesPerUnit = 1.0;

    /// constantVolts + cosineVolts·cos(arm angle). Returns exactly
    /// constantVolts when cosineVolts is 0, so a plain lift's output is
    /// bit-for-bit "loop + constant".
    double volts(double position) const;
};

/// Seat-and-rest at a hard stop. With plain PID, a mechanism resting on its
/// stop settles a few units short (gravity feedforward pushes up, and the
/// little kP gives that close can't beat friction), and a target past the
/// stop has the loop pushing into it forever. With this enabled, a target at
/// or below `floor` drives down with at least `seatVolts` until the
/// mechanism is within `restBand` of the floor, then cuts to 0V and resets
/// the loop, resting on the stop instead of pushing into it.
struct SeatConfig {
    bool enabled = false;

    /// The hard stop's position. Targets `<= floor` use this law instead of
    /// the loop.
    double floor = 0.0;

    /// Minimum downward volts while seating (a positive number; applied as
    /// "at most -seatVolts"). No gravity feedforward is added while seating.
    double seatVolts = 0.0;

    /// Once `position <= floor + restBand`, rest at 0V.
    double restBand = 0.0;
};

/// Everything the law needs to know about one mechanism. At namespace scope
/// (not nested in PositionMechanism) so it can be a `= {}` default argument
/// — GCC can't use a nested struct with default member initializers that way
/// inside its enclosing class. Every member has a default (the `{}`s too), so
/// a designated initializer can leave out any of them without a
/// -Wmissing-field-initializers warning.
struct PositionConfig {
    /// Error in position units, output in volts. Set nominalDtS to how often
    /// update() actually runs: 0.02 from a 20ms opcontrol tick, 0.01 for
    /// startTask(10).
    PID::Config pid{};

    GravityFeedforward gravity{};

    SeatConfig seat{};

    /// Final clamp on the commanded volts (after feedforward), applied before
    /// the conversion to millivolts.
    double maxVolts = 12.0;

    /// isNear()/atTarget() band, in position units.
    double tolerance = 1.0;

    /// settled() needs this long continuously within tolerance. 0 = settled as
    /// soon as it's within tolerance on an update.
    std::uint32_t settleTimeMs = 0;

    /// update() calls further apart than this reset the PID first, so its
    /// derivative doesn't kick off a reading from before the gap (e.g. the
    /// lift was moved during autonomous). 0 disables.
    std::uint32_t resetAfterGapMs = 100;
};

/// Which branch produced a command. Also useful as a screen or telemetry
/// field (its numeric value is stable: new laws only ever get added at the
/// end).
enum class PositionLaw : std::uint8_t {
    track,      ///< PID on the target plus gravity feedforward
    seat,       ///< heading for the floor, driving down at least seatVolts
    rest,       ///< at the floor: 0V, loop reset
    sensorLost, ///< no position reading: brake, loop reset
    manual,     ///< open-loop volts (PositionMechanism::setVolts())
    off,        ///< brake: PositionMechanism::stop(), or its task while disabled
    external,   ///< something else has the motors: a tuning run (see
                ///< PositionMechanism::beginExternalControl())
};

/// A short printable name ("track", "seat", "rest", "no sensor", "manual",
/// "off", "external"), at most 9 characters so it fits on a controller line.
const char* positionLawName(PositionLaw law);

struct PositionCommand {
    PositionLaw law = PositionLaw::off;

    /// Volts to command, already clamped to ±maxVolts. 0 when `brake`.
    double volts = 0.0;

    /// Call brake() (the motors' configured brake mode) instead of commanding
    /// volts.
    bool brake = false;

    /// `volts` as the millivolts pros::Motor::move_voltage() takes: scaled
    /// and truncated toward zero, the same conversion
    /// chassis::MotorGroup::moveVoltage() (and every hand-written lift loop
    /// before this one) uses. Kept here, next to the law, so the exact
    /// integer a mechanism sends is host-tested along with it.
    std::int32_t millivolts() const { return static_cast<std::int32_t>(volts * 1000.0); }
};

/// One closed-loop step. Exactly, checked in this order:
///
///   position NaN or infinite         -> pid.reset(); {sensorLost, 0V, brake}
///   !seat.enabled || target > floor  -> track: v = pid.update(target, position)
///                                                  + gravity.volts(position)
///   position > floor + restBand      -> seat:  v = min(pid.update(target, position),
///                                                      -seatVolts)
///   otherwise                        -> rest:  v = 0; pid.reset()
///
/// and then volts = clamp(v, -maxVolts, maxVolts).
///
/// Calls pid.update() (two-argument form, config.pid.nominalDtS) at most
/// once. `target` must be finite; PositionMechanism::setTarget() ignores one
/// that isn't. `pid` is taken separately from `config.pid` because it's the
/// live controller, whose gains a tuner may have changed since.
PositionCommand computePositionCommand(PID& pid, const PositionConfig& config, double target,
                                       double position);

} // namespace sapphirelib::mechanism
