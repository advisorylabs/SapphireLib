/**
 * \file robot/macros.hpp
 *
 * Driver macros for the intake, roller claw, and lift, built on SapphireLib's
 * input/, mechanism/, and util/ primitives. Robot-specific, so it sits
 * outside include/sapphirelib and is kept out of the library archive (see
 * EXCLUDE_SRC_FROM_LIB in the Makefile).
 *
 * The robot starts stowed: claw piston retracted, lift at the bottom (where
 * it intakes from, below level 1). Scoring beats every other macro, and
 * nothing interrupts a score once it's started.
 *
 * Controls:
 *   R1 (hold)  Intake. Stows first, retracting the claw and bringing the
 *              lift to the bottom, and only once it's there spins the intake
 *              and roller claw in. Waits for a score to finish rather than
 *              interrupting it.
 *   R2         Stowed: reverses the intake and roller claw for as long as
 *              it's held. Deployed: scores (dips halfway to the level below,
 *              outtakes the roller claw for a moment, then raises the lift a
 *              level), and reverses the intake while held.
 *   L1         Stowed: deploys the claw and raises the lift to level 1.
 *              Deployed: up a level. Hold to keep stepping up.
 *   L2         Down a level, stopping at level 1. Hold to keep stepping
 *              down. Only intaking stows.
 *   Y (hold)   Claw intake: deploys the claw at level 1 and spins the roller
 *              claw in (intake idle). On release, the lift drops all the way
 *              to the bottom and comes back up to level 1, the roller claw
 *              still spinning in.
 *   RIGHT      Cycle goal (ALLIANCE, MEDIUM, CENTER), each with its own
 *              level heights and level count. The lift keeps its level,
 *              moving to the new goal's height for it.
 *   LEFT       Cycle the controller's bottom line: lift position and target,
 *              then each lift motor's volts, current, and temperature.
 *
 * The roller claw also spins in on its own whenever its distance sensor sees
 * a pin, to keep hold of it. The lift holds each level on a PID loop around
 * its rotation sensor.
 *
 * Two pistons (ports in config.hpp). The claw piston, on G, deploys the
 * moment L1 or Y takes the lift off the bottom, and only R1's stow retracts
 * it. The top piston, on H, deploys once the lift has reached its goal's top
 * level, stays out through a score there, and retracts the moment the lift is
 * sent to any other level. RIGHT at the top level retracts it until the lift
 * reaches the new goal's top, unless it's already there.
 *
 * Debug mode (the Home page's Debug button), for bringing the lift up: the
 * lift ignores its levels, and L1 and L2 power its motors directly, up and
 * down for as long as they're held, braking wherever it's let go, sensor or
 * no sensor. Nothing else moves the lift. A press of A flips the claw piston
 * (G) and a press of X the top piston (H), and nothing else moves either:
 * R1 intakes at once, R2 only reverses the intake and roller claw (it never
 * scores), Y spins the roller claw in, and releasing Y doesn't dip. X and A
 * are also halves of shortcuts, so in debug mode the X+UP auton shortcut
 * flips H too, and teach mode's X+A (teach.hpp) flips both. Leaving debug
 * mode puts the lift back on its level (level 1 or up with the claw
 * deployed, the bottom with it retracted), so expect it to move, and
 * retracts the top piston until the lift is at the top level again.
 *
 * Controller screen: line 0 is the goal and level ("STOW" when stowed, or
 * "DBG" and each piston in debug mode), line 2 the readout LEFT picks. Line 1
 * belongs to driver.cpp.
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
/// the lift's rotation sensor, and every level height is measured up from
/// there. (initDevices() does.)
void initialize();

/// Call every opcontrol() loop tick, right after controller.update(). Reads
/// the controller and drives the intake, roller claw, and lift, and sets the
/// controller screen's lines 0 and 2; opcontrol() flushes the screen once at
/// the end of the tick.
void update(sapphirelib::input::Controller& controller);

/// Stops the claw and intake and brakes the lift, right away. For disabled(),
/// and for just before something runs a blocking routine from opcontrol()
/// (driver.cpp's X+UP): nothing calls update() until that routine ends, and
/// every motor would otherwise keep its last command for the whole routine,
/// a lift climbing at 12V would stay at 12V into its hard stop.
///
/// Nothing else changes: the claw's deploy state, the lift's level and the
/// scoring mode are kept, and the next update() commands everything from
/// them again, with the lift back on its PID loop at its current level. (After
/// a pause long enough to count as a restart, update() also drops any
/// half-finished sequence, as it always has.)
void stop();

/// Turns debug mode (see the file comment) on or off. Safe from any task; the
/// Home page's button calls it from the GUI's. update() picks it up on its
/// next tick, dropping whatever sequence was running.
void setDebug(bool on);

/// Whether debug mode is on (or about to be, on update()'s next tick). Safe
/// from any task.
bool debug();

/// Whether the roller claw's distance sensor sees a pin: what an autonomous
/// routine branches on (autons.cpp's Branch routines). Reads the sensor, so
/// it's current; safe from any task, including a trajectory's next.
bool pinInClaw();

/// Logs the lift to `logger`: its PID's every step (channel "lift") and
/// what each update actually commanded (channel "lift.act": target and
/// position in degrees, the volts sent after gravity feedforward and the
/// clamp, and which branch of the control law ran, see
/// docs/TELEMETRY_FORMAT.md). Call from initialize(), before opcontrol can
/// run: it attaches the PID's observer and the lift's step listener, neither
/// of which is synchronized with update(). Without it, nothing is logged and
/// nothing else changes.
void attachTelemetry(sapphirelib::telemetry::Logger& logger);

// --- The lift, for the PID page (tuning.cpp) ---

/// The lift mechanism itself: its pid() is the PID page's Lift entry, and
/// Auto-Tune installs the gravity feedforward it measures with setGravity().
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
