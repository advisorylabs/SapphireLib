/**
 * \file robot/macros.hpp
 *
 * Driver macros for the lift, the claw, and the extender, built on
 * SapphireLib's input/, mechanism/, and util/ primitives. Robot-specific, so
 * it sits outside include/sapphirelib and is kept out of the library archive
 * (see EXCLUDE_SRC_FROM_LIB in the Makefile).
 *
 * The robot starts with the claw closed on a preload, the extender in, and
 * the lift at the bottom. Scoring beats every other macro, and nothing
 * interrupts a score once it's started.
 *
 * Controls:
 *   R1         Open or close the claw. Ignored during a score, which opens it
 *              itself.
 *   R2         Score, only with the claw closed: dips halfway to the level
 *              below, opens the claw, gives the pin a moment to come off,
 *              then lowers the lift to the bottom. At the bottom there's no
 *              level below to dip to, so it just opens the claw.
 *   L1         Up a level. Hold to keep stepping up.
 *   L2         Down a level, all the way to the bottom. Hold to keep stepping
 *              down.
 *   Y          Push the extender out, or pull it back in. Works at any time,
 *              a score included.
 *   RIGHT      Cycle goal (ALLIANCE, MEDIUM, CENTER), each with its own
 *              level heights and level count. The lift keeps its level,
 *              moving to the new goal's height for it.
 *   LEFT       Cycle the controller's bottom line: lift position and target,
 *              then each lift motor's volts, current, and temperature.
 *
 * autoClaw: with the claw open and the lift down on its bottom switch, a pin
 * the claw's distance sensor sees closes the claw, on the first reading.
 * Driving onto the pin, it closes further out the faster the robot goes, so
 * the claw is shut by the time the pin is in it rather than closing behind
 * it. Once the claw opens on a pin (R1, or a score at the bottom), autoClaw
 * waits for the sensor to see the pin gone before it closes again, so opening
 * the claw to drop a pin doesn't snap it shut on the same one.
 *
 * The lift: its motors (config.hpp's kLiftMotors, three of them) hold each
 * level on a PID loop around their own encoders, in motor degrees. A limit
 * switch at the bottom zeroes them each time it's pressed, and a lift heading
 * for the bottom drives down until it's pressed, so the encoders can't drift
 * for long.
 *
 * Debug mode (the Home page's Debug button), for bringing the lift up: the
 * lift ignores its levels, and L1 and L2 power its motors directly, up and
 * down for as long as they're held, braking wherever it's let go, encoders or
 * no encoders. L2 stops at the bottom switch. Nothing else moves the lift: R2
 * never scores, and autoClaw is off. R1 and Y still work the pistons. Leaving
 * debug mode puts the lift back on its level, so expect it to move.
 *
 * Controller screen: line 0 is the goal and level ("DBG" and the pistons in
 * debug mode), line 2 the readout LEFT picks, its label "BTM" instead of
 * "LIFT" while the bottom switch is pressed. Line 1 belongs to driver.cpp.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include "sapphirelib/input/controller.hpp"
#include "sapphirelib/mechanism/position_mechanism.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace robot::macros {

/// Call once from initialize(), with the lift all the way down: this zeroes
/// the lift's encoders, and every level height is measured up from there. The
/// first press of the bottom switch zeroes them again anyway. (initDevices()
/// does.)
void initialize();

/// Call every opcontrol() loop tick, right after controller.update(). Reads
/// the controller and drives the lift, the claw and the extender, and sets
/// the controller screen's lines 0 and 2; opcontrol() flushes the screen once
/// at the end of the tick.
void update(sapphirelib::input::Controller& controller);

/// Brakes the lift, right away. For disabled(), and for just before something
/// runs a blocking routine from opcontrol() (driver.cpp's X+UP): nothing
/// calls update() until that routine ends, and the lift would otherwise keep
/// its last command for the whole routine, a lift climbing at 12V would stay
/// at 12V into its hard stop.
///
/// Nothing else changes: the pistons stay where they are, and the lift's
/// level and the goal are kept, and the next update() commands everything
/// from them again, with the lift back on its PID loop at its current level.
/// (After a pause long enough to count as a restart, update() also drops a
/// half-finished score, as it always has.)
void stop();

/// Turns debug mode (see the file comment) on or off. Safe from any task; the
/// Home page's button calls it from the GUI's. update() picks it up on its
/// next tick, dropping a score that was running.
void setDebug(bool on);

/// Whether debug mode is on (or about to be, on update()'s next tick). Safe
/// from any task.
bool debug();

/// Whether the claw's distance sensor sees a pin: what an autonomous routine
/// branches on (autons.cpp's Branch routines). Reads the sensor, so it's
/// current; safe from any task, including a trajectory's next.
bool pinInClaw();

/// Whether the lift is pressing its bottom switch. Reads the switch, so it's
/// current; safe from any task.
bool liftAtBottom();

/// Logs the lift to `logger`: its PID's every step (channel "lift") and
/// what each update actually commanded (channel "lift.act": target and
/// position in motor degrees, the volts sent after gravity feedforward and
/// the clamp, and which branch of the control law ran, see
/// docs/TELEMETRY_FORMAT.md), and the macros' own state every tick (channel
/// "mech"). Call from initialize(), before opcontrol can run: it attaches the
/// PID's observer and the lift's step listener, neither of which is
/// synchronized with update(). Without it, nothing is logged and nothing else
/// changes.
void attachTelemetry(sapphirelib::telemetry::Logger& logger);

// --- The lift, for the PID page (tuning.cpp) and the Lift Testing auton ---

/// The lift mechanism itself: its pid() is the PID page's Lift entry, and
/// Auto-Tune installs the gravity feedforward it measures with setGravity().
/// Its motors() are every motor in config.hpp's kLiftMotors.
sapphirelib::mechanism::PositionMechanism& liftMechanism();

/// Auto-Tune's lift experiment: ramps and steps between two heights well
/// inside the lift's travel, with the motors braked between segments. It
/// takes the motors from the lift's own loop for the run
/// (PositionMechanism::beginExternalControl()), update() keeps running, and
/// logs law "external", and hands them back after. Start it with the lift
/// all the way down.
sapphirelib::tuning::MechanismCharacterizationConfig liftExperiment();

/// Run Test for the Lift entry: raises the lift to a test height and back
/// down to another, through its normal loop (the same PID, gravity volts and
/// update() the macros run), then returns it to its current level. Blocks;
/// the PID page runs it on a task of its own. The lift only moves while
/// driver control is running update().
void liftTuningTest();

} // namespace robot::macros
