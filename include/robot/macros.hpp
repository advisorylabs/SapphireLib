/**
 * \file robot/macros.hpp
 *
 * Driver macros for the lift, the claw, and the extender, built on
 * SapphireLib's input/, mechanism/, and util/ primitives. Robot-specific, so
 * it sits outside include/sapphirelib and is kept out of the library archive
 * (see EXCLUDE_SRC_FROM_LIB in the Makefile).
 *
 * The robot starts with the claw closed on a preload, the extender in, and
 * the lift at the bottom. Scoring beats every other macro, and once a score
 * has started nothing interrupts it, but for L2 while it waits on the robot
 * to back off (see L2).
 *
 * Controls:
 *   R1         Open or close the claw. Ignored during a score, which opens it
 *              itself.
 *   R2         Score, only with the claw closed: dips halfway to the level
 *              below and opens the claw, then holds the lift there until the
 *              vertical tracking wheel has seen the robot back 3in off the
 *              goal, and only then lowers it to the bottom. At the bottom
 *              there's no level below to dip to, so it just opens the claw.
 *              The face winks as each score opens the claw.
 *   L1         Up a level. Hold to keep stepping up.
 *   L2         Down a level, all the way to the bottom. Hold to keep stepping
 *              down. While a score waits for the robot to back off, L2 lowers
 *              the lift to the bottom now instead.
 *   Y          Push the extender out, or pull it back in. Works at any time,
 *              a score included.
 *   RIGHT      Cycle goal (ALLIANCE, MEDIUM, CENTER), each with its own
 *              level heights and level count. The lift keeps its level,
 *              moving to the new goal's height for it.
 *   LEFT       Cycle the controller's bottom line: lift position and target,
 *              then each lift motor's volts, current, and temperature.
 *
 * autoClaw: with the claw open and the lift down at its bottom level, a pin
 * the claw's distance sensor sees closes the claw, on the first reading.
 * Driving onto the pin, it closes further out the faster the robot goes, so
 * the claw is shut by the time the pin is in it rather than closing behind
 * it. Once the claw opens on a pin (R1, or a score at the bottom), autoClaw
 * waits for the sensor to see the pin gone before it closes again, so opening
 * the claw to drop a pin doesn't snap it shut on the same one. It's off while
 * the auto cycle (A, autocycle.hpp) runs, which closes the claw itself.
 *
 * The lift: its motors (config.hpp's kLiftMotors, three of them) hold each
 * level on a PID loop around their own encoders, in motor degrees. At driver
 * control's start, before taking any level, the lift homes: it drives down at
 * 45% power until the limit switch at the bottom is pressed (unless it
 * already is), and zeroes the encoders there, so it needn't start the program
 * all the way down. That's the only time the switch zeroes them: after
 * homing, level 0 is encoder 0, switch pressed or not.
 *
 * Debug mode (the Home page's Debug button), for bringing the lift up: the
 * lift ignores its levels, and L1 and L2 power its motors directly, up and
 * down for as long as they're held, braking wherever it's let go, encoders or
 * no encoders. L2 stops at the bottom switch. Nothing else moves the lift: R2
 * never scores, and autoClaw is off. R1 and Y still work the pistons. Leaving
 * debug mode puts the lift back on its level, so expect it to move.
 *
 * Autonomous: update() only runs in driver control (and teach mode's
 * replays), so routines move the claw and the extender with setClawClosed()
 * and setExtended(), or from a path's actions (paths.hpp). Driver control
 * carries on with both wherever autonomous left them.
 *
 * Controller screen: line 0 is the goal and level ("DBG" and the pistons in
 * debug mode), line 2 the readout LEFT picks, its label "BTM" instead of
 * "LIFT" while the bottom switch is pressed. Line 1 belongs to driver.cpp.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <cstdint>

#include "sapphirelib/input/controller.hpp"
#include "sapphirelib/mechanism/position_mechanism.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace robot::macros {

/// Call once from initialize(): this zeroes the lift's encoders where the lift
/// is, and update()'s homing then drives it down onto the bottom switch, which
/// zeroes them again at the real bottom, once. (initDevices() does.)
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

// --- The claw and the extender, for autonomous ---
//
// Each moves its piston now, since update() isn't running to: for autonomous
// routines, and for anything else that runs while it isn't (X+UP's routines).
// Call them from the task running the routine; the macros' state has no lock,
// so never while update() runs on another task.

/// Closes the claw (true) or opens it (false). Opening disarms autoClaw until
/// the sensor sees the pin gone, as R1 does, so driver control's first ticks
/// don't close it again on a pin autonomous just let go of.
void setClawClosed(bool closed);

/// Whether the claw is closed, or about to be: what it was last told.
bool clawClosed();

/// Pushes the extender out (true) or pulls it in (false).
void setExtended(bool out);

/// Whether the extender is out, or about to be: what it was last told.
bool extended();

/// How many scores have opened the claw since the program started: the
/// face winks each time it goes up (screen.cpp). Safe from any task.
std::uint32_t scoreCount();

/// Whether the claw's distance sensor sees a pin: what an autonomous routine
/// branches on (autons.cpp's Branch routines). Reads the sensor, so it's
/// current; safe from any task, including a trajectory's next.
bool pinInClaw();

/// Whether the lift is pressing its bottom switch. Reads the switch, so it's
/// current; safe from any task.
bool liftAtBottom();

// --- For the auto cycle (autocycle.hpp), on opcontrol's task ---

/// Whether the claw's distance sensor reads a pin within `mm`. Reads the
/// sensor, so it's current; unplugged, it never does.
bool pinWithinMm(std::int32_t mm);

/// Whether the lift is at its bottom level and there, by its encoders (the
/// switch needn't be pressed): what the cycle waits on before driving into
/// the loader. Reads the macros' state, so call it from opcontrol's task.
bool liftDown();

/// Stops autoClaw closing the claw (true) until it's let go again (false),
/// for the cycle, which closes it itself. R1 and a score still work.
void setAutoClawPaused(bool paused);

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
