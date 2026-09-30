/**
 * \file robot/driver.hpp
 *
 * Driver control: the controller, the stick drive, and the developer
 * shortcuts that only work off the field.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <cstdint>

#include "sapphirelib/input/controller.hpp"

namespace robot {

/// opcontrol()'s loop period. The macros' lift PID assumes it (its
/// nominalDtS is 0.02), and so does the controller's restart detection.
inline constexpr std::uint32_t kDriverLoopMs = 20;

/// The driver's controller. It lives at namespace scope, never as a local in
/// opcontrol(), so its button history survives opcontrol() restarting, see
/// sapphirelib::input::Controller's class comment.
sapphirelib::input::Controller& controller();

/// One tick of driving from the sticks. opcontrol() skips it while a GUI
/// routine is driving the chassis (see screenBusy()), so the two never fight
/// over the motors.
void drive(sapphirelib::input::Controller& controller);

/// Button shortcuts that aren't driving, checked every tick, including while
/// a GUI routine is busy, since a button press is only seen on the one tick
/// it happens:
///   - A redefines "forward" for field-centric driving as whichever way the
///     chassis faces now (it never moves the chassis, so it's safe any time).
///   - B+DOWN (pressed together, either order) runs the autonomous routine
///     picked on the brain screen, but only with no competition switch or
///     field connected, so it can't fire during a real match, and not while a
///     GUI routine is driving the chassis. It stops the intake, claw, and lift
///     first (macros::stop()), and blocks until the routine ends.
void shortcuts(sapphirelib::input::Controller& controller);

} // namespace robot
