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

/// One tick of driving from the sticks, and the controller screen's line 1
/// (the drive mode). opcontrol() skips it while the auto cycle (A, see
/// autocycle.hpp) or a GUI routine (see screenBusy()) is driving the chassis,
/// so the two never fight over the motors.
///
/// Holonomic split arcade, robot-centric: left stick Y drives forward and
/// back, left stick X strafes, right stick X turns.
///   - B inverts: front and back swap, so forward and strafe flip and turning
///     stays the same. Stays on until pressed again, restarts included.
///   - The higher the lift, the slower the drive: no wheel gets more than
///     macros::driveMaxVolts(), 12V with the lift down and 7V at its full
///     height, and 5.75V while clawSpeed is on (DOWN, see macros.hpp). Past
///     that, more stick goes no faster.
///   - In practice mode (setPracticeMode()), any drive stick past
///     kPracticeMaxThrottlePct (config.hpp) cuts the drive motors' power,
///     with a short rumble as it cuts. Power comes back once every drive
///     stick is under it again.
void drive(sapphirelib::input::Controller& controller);

/// Turns practice mode on or off, from any task (the Home page's Practice
/// button). Off at startup.
void setPracticeMode(bool on);

/// Whether practice mode is on.
bool practiceMode();

/// Button shortcuts that aren't driving, checked every tick, including while
/// a GUI routine is busy, since a button press is only seen on the one tick
/// it happens:
///   - X+UP (pressed together, either order) runs the autonomous routine
///     picked on the brain screen, but only with no competition switch or
///     field connected, so it can't fire during a real match, and not while a
///     GUI routine is driving the chassis. It ends the auto cycle and brakes
///     the lift first (macros::stop()), and blocks until the routine ends.
///
/// A alone, the auto cycle, is autocycle::update()'s (robot/autocycle.hpp).
void shortcuts(sapphirelib::input::Controller& controller);

} // namespace robot
