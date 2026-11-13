/**
 * \file robot/macros.hpp
 *
 * Driver macros for the lift, the claw, and the side piston, built on
 * SapphireLib's input/, mechanism/, and util/ primitives. Robot-specific, so
 * it sits outside include/sapphirelib and is kept out of the library archive
 * (see EXCLUDE_SRC_FROM_LIB in the Makefile).
 *
 * The robot starts with the claw open, the side piston in, and the lift at the
 * bottom, and autonomous's start closes the claw on the preload (see Bottom
 * lift). Scoring beats every other macro, and once
 * a score has started nothing interrupts it, but for L2 while it waits on the
 * robot to back off (see L2).
 *
 * Controls:
 *   R1         Open or close the claw. Ignored during a score, which opens it
 *              itself. At the bottom, closing it lifts the lift to 75
 *              degrees, and opening it lowers it back (see Bottom lift).
 *              Acts 80ms after the press, or as it's let go if sooner, in
 *              case R2 follows (see R1+R2); so does R2.
 *   R2         Score, only with the claw closed: dips 30 degrees past halfway
 *              to the level below (never past that level) and opens the claw
 *              as soon as the lift gets there, or 100ms after the press
 *              (kScoreReleaseMs) if it hasn't yet, the lift carrying on down
 *              meanwhile. Then holds the lift there until the vertical
 *              tracking wheel has seen the robot back 3in off the goal, and
 *              only then lowers it to the bottom. At the bottom there's no level below to dip
 *              to, so it just opens the claw, and the lift comes back down
 *              from 75 degrees. The face winks as each score opens the claw,
 *              and the LEDs show Scoring while one above the bottom runs
 *              (leds.hpp). A score above the bottom also sets the lift's
 *              memory (see Lift memory).
 *   L1         Up a level, or straight to the lift's memory if it's higher
 *              (see Lift memory). Hold to keep stepping up. Only ALLIANCE has
 *              an L1: on MEDIUM and CENTER, the first level up from the
 *              bottom is L2.
 *   L2         Down a level, all the way to the bottom. Hold to keep stepping
 *              down. While a score waits for the robot to back off, L2 lowers
 *              the lift to the bottom now instead.
 *   L1+L2      Pressed together: forget the lift's memory. The lift stays
 *              where it was (a step the first of the two took is undone),
 *              and neither moves it again until both are let go. The LEDs
 *              flash purple for half a second (MemoryReset, leds.hpp).
 *   R1+R2      Pressed together, in either order: turn autoClaw on or off
 *              (clawSpeed, see there), a toggle: press both once for on,
 *              again for off. Neither does its own job. The controller
 *              rumbles long for on and short-short for off. Works at any
 *              time, a score and debug mode included. The second pressed
 *              within 80ms of the first is clean; pressed later while the
 *              first is still held, it still toggles autoClaw, but the first
 *              has already acted on its own.
 *   Y          Push both side toggles out, or pull them both back in (both in
 *              if either was out). Works at any time, a score included. Each
 *              one out also fires on its own (see Side toggles).
 *   RIGHT      Cycle goal (ALLIANCE, MEDIUM, CENTER), each with its own
 *              level heights and level count. The lift keeps its level (its
 *              L number), moving to the new goal's height for it: from
 *              ALLIANCE's L1 to the others' L2, and from a level higher than
 *              the new goal has to its top one. Forgets the lift's memory.
 *              The LEDs show the new goal's color for a second (leds.hpp).
 *   LEFT       Cycle the controller's bottom line: lift position and target,
 *              the claw's sensor (CLAW: its reading, and how fast the robot
 *              drove onto the last pin autoClaw grabbed, in in/s), then each
 *              lift motor's volts, current, and temperature, then each side
 *              toggle's sensors, left then right: "LY* 48 62%   40"
 *              is the left optical seeing yellow (R, Y, B, or - for no
 *              color), '*' while they'd fire it, the optical's hue (degrees)
 *              and saturation, and the distance sensor (mm).
 *
 * Side toggles: two pistons, left (H) and right (G). Y moves both together.
 * Each one out also fires on its own, in every mode, to flip what it's next
 * to round to our alliance's color (alliance.hpp, picked on the controller
 * before the match, red off the field): when its distance sensor (config.hpp:
 * the localizer's west or east one) reads under 65mm and its optical (17
 * left, 1 right) sees a color that isn't ours, yellow (or the green yellow
 * sometimes reads as) or blue on red, red or yellow on blue, it goes in for
 * 250ms, then back out for 250ms, and only then can it fire again; still
 * seeing the wrong color, it does. A toggle Y has pulled in stays in and
 * doesn't fire. The numbers and the colors' hue ranges are kToggle* and
 * kRedHues and the rest in macros.cpp. Driver control only: autonomous moves
 * them, together or one at a time, with setExtended(). Driver control starts
 * with both out (main.cpp's opcontrol()), wherever autonomous left them, so
 * both fire on their own from its start until Y pulls them in.
 *
 * autoClaw: only while clawSpeed is on (R1+R2). With the claw open and the
 * lift down at its bottom level, the claw closes on a pin by itself, early
 * enough for the jaws to shut with the pin at kGrabMm (macros.cpp) from its
 * distance sensor: the faster the robot drives onto the pin, the further out
 * it fires, by odometry's speed times kClawLeadMs, the time from a reading to
 * the jaws shut (see kGrabMm). Once the claw opens on a pin (R1,
 * or a score), autoClaw waits for the sensor to see the claw empty before it
 * closes again, so opening the claw to drop a pin doesn't snap it shut on
 * the same one. Opening an empty claw leaves autoClaw ready for the next
 * pin. The AI Vision sensor (vision.hpp) plays no part in driver control's
 * autoClaw: it sits idle outside autonomous's runs onto the Pins.
 *
 * Bottom lift: at the bottom level, the lift sits at 75 motor degrees
 * whenever the claw is closed, and all the way down whenever it's open. So a
 * grab (autoClaw or R1) lifts it to 75, opening the claw (R1, or R2's score)
 * sends it back down, and the lift coming down to the bottom with the claw
 * closed stops at 75. A grab waits 65ms after the claw closes before it
 * lifts, so the jaws are shut before the pin leaves the floor. The claw
 * starts open, so the preload goes in with the program running, and each
 * start of autonomous is a grab on it: the claw closes and the lift rises to
 * 75 (grabPreload(), ahead of the routine). Driver control's start leaves the
 * claw as it is, open if autonomous never ran, so a match's driver control
 * carries on from wherever autonomous left it. At any other level the claw
 * doesn't move the lift.
 *
 * clawSpeed: while it's on, no drive wheel gets more than kClawSpeedVolts
 * (macros.cpp, see driveMaxVolts()), so the robot drives slower, and autoClaw
 * is on; the lift
 * isn't touched. It turns itself off 350ms after the claw closes (autoClaw or
 * R1) with the lift down at the bottom, so the robot drives off with the pin
 * at full speed, with the same short-short rumble as R1+R2 turning it off,
 * and autoClaw is off again until R1+R2. If the claw opens again within those
 * 350ms, it stays on. Off at startup. The LEDs show AutoClaw while it's on,
 * outside debug mode (leds.hpp), and so does setAutoClaw()'s mode.
 *
 * Lift memory: R2 scoring at a level remembers the level above it (scoring at
 * L5 remembers L6; at the goal's top level, the top level itself). From then
 * on, L1 goes straight there from any level below it, the bottom included,
 * and steps up one as usual from it or above. It lasts until L1+L2 or RIGHT
 * forgets it, or the next score replaces it. With no memory, L1 steps up one.
 * A score at the bottom (R2 just dropping the pin) leaves it as it was.
 *
 * The lift: its motors (config.hpp's kLiftMotors, three of them) hold each
 * level on a PID loop around their own encoders, in motor degrees. Those
 * drift, so the limit switch at the bottom zeroes them every time it's
 * pressed. Every trip down to the bottom creeps at 4V for its last 150
 * degrees and stops where the encoders put the bottom. If the switch still
 * isn't pressed 2 seconds later, it creeps on to find it, but no more than
 * 100 degrees past, once per trip: a lift held up off the switch rests where
 * it is rather than unwinding. Driver control's start counts as a trip down,
 * from wherever the lift was when the program started, so start it with the
 * lift down.
 *
 * Debug mode (the Home page's Debug button), for bringing the lift up: the
 * lift ignores its levels, and L1 and L2 power its motors directly, up and
 * down for as long as they're held, braking wherever it's let go, encoders or
 * no encoders. L2 stops at the bottom switch. Nothing else moves the lift: R2
 * never scores, and autoClaw is off. R1 and Y still work the pistons. Leaving
 * debug mode puts the lift back on its level, so expect it to move.
 *
 * Autonomous: update() only runs in driver control, so routines move the
 * claw and the side piston with setClawClosed() and setExtended(), and the lift
 * with setLiftLevel() and setLiftDeg(), and setAutoClaw() is clawSpeed and
 * autoClaw's toggle. The lift runs its PID loop on a task
 * of its own (startLift()), so a routine can send it up and drive on while
 * it climbs, then waitForLift() before scoring. Driver control carries on
 * with the claw, the side piston, and the lift's goal and level wherever
 * autonomous left them. Only driver control creeps onto the bottom switch
 * and zeroes the encoders there: in autonomous the lift comes down to the
 * bottom on its loop alone, to where the encoders put it.
 *
 * Controller screen: line 0 is the goal and level ("DBG" and the pistons in
 * debug mode), with an arrow to the lift's memory while L1 would jump there:
 * "ALLIANCE BTM>L6" at the bottom, "ALLIANCE L3>L6" at L3. At or above the
 * remembered level, or with no memory, it shows "ALLIANCE BOTTOM" or
 * "ALLIANCE L3/6" as usual. Line 2 is the readout LEFT picks, its label "BTM"
 * instead of "LIFT" while the bottom switch is pressed. Line 1 belongs to
 * driver.cpp, or to the auto cycle while it runs (autocycle.hpp).
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <cstdint>
#include <optional>

#include "sapphirelib/input/controller.hpp"
#include "sapphirelib/mechanism/position_mechanism.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace robot::macros {

/// Call once from initialize(): this zeroes the lift's encoders where the lift
/// is, until the bottom switch zeroes them again at the real bottom, as every
/// press of it does. (initDevices() does.)
void initialize();

/// Call every opcontrol() loop tick, right after controller.update(). Reads
/// the controller and drives the lift, the claw and the side piston, and sets
/// the controller screen's lines 0 and 2; opcontrol() flushes the screen once
/// at the end of the tick.
void update(sapphirelib::input::Controller& controller);

/// Brakes the lift, from its task's next step (within 20ms). For disabled(),
/// and for just before something runs a blocking routine from opcontrol()
/// (driver.cpp's X+UP): nothing calls update() until that routine ends, and
/// the lift would otherwise keep its last command for the whole routine, a
/// lift climbing at 12V in debug mode would stay at 12V into its hard stop.
/// The routine can still move it (setLiftLevel()).
///
/// Nothing else changes: the pistons stay where they are, and the lift's
/// level and the goal are kept, and the next update() commands everything
/// from them again, with the lift back on its PID loop at its current level.
/// (After a pause long enough to count as a restart, update() also drops a
/// half-finished score, as it always has.)
void stop();

/// Starts the lift's own task, which runs its PID loop every 20ms from then
/// on, in autonomous and driver control alike; until then nothing drives
/// the lift. Call once from initialize(), after attachTelemetry() (main.cpp
/// calls it after startTelemetry()): the step listener that attaches must be
/// in place before the task starts.
void startLift();

/// Turns debug mode (see the file comment) on or off. Safe from any task; the
/// Home page's button calls it from the GUI's. update() picks it up on its
/// next tick, dropping a score that was running.
void setDebug(bool on);

/// Whether debug mode is on (or about to be, on update()'s next tick). Safe
/// from any task.
bool debug();

// --- The claw and the side piston, for autonomous ---
//
// Each moves its piston now, since update() isn't running to: for autonomous
// routines, and for anything else that runs while it isn't (X+UP's routines).
// Call them from the task running the routine; the macros' state has no lock,
// so never while update() runs on another task.

/// Closes the claw (true) or opens it (false). Opening disarms autoClaw until
/// the sensor sees the pin gone, as R1 does, so driver control's first ticks
/// (or setAutoClaw()'s) don't close it again on a pin autonomous just let go
/// of.
void setClawClosed(bool closed);

/// Whether the claw is closed, or about to be: what it was last told.
bool clawClosed();

/// Sends each valve (the claw, both side toggles) the state the macros have it
/// in, setting its port up again only if the brain doesn't have it as an
/// output (Piston::resend()). update() also resends every valve every tick,
/// so one the brain took low is back within a tick. The pistons are built before initialize(),
/// and the brain can drop what they send then: a valve whose starting state
/// is the port's high one would go to its low one once the 3-wire ports come
/// up (the LEDs' setup, at the end of initialize()), while the macros still
/// think otherwise. Both start low today (the claw open, the side piston in),
/// so this is insurance. main.cpp calls this at the end of initialize() and as
/// autonomous starts, and update() on driver control's first tick after any
/// gap. Safe from any task.
void resendPistons();

/// The claw starts open, and each start of autonomous closes it on the
/// preload: main.cpp's autonomous() calls this ahead of the routine. Closes
/// the claw, waits kGrabLiftDelayMs (65ms) for the jaws to shut, and sends
/// the lift up to the bottom level's 75 degrees, as a grab does in driver
/// control; the routine's first setLiftLevel() or setLiftDeg() takes over
/// from there. Above the bottom level it only closes the claw. Driver
/// control's start doesn't grab, so the claw stays open there until R1 or
/// autoClaw closes it. Blocks for those 65ms, so autonomous only.
void grabPreload();

/// autoClaw for autonomous, a toggle like R1+R2's clawSpeed, so a routine
/// drives up to a pin without timing setClawClosed(). While it's on, every
/// motion from the next one on runs at no more than clawSpeed's volts
/// (kClawSpeedVolts in macros.cpp, through auton::setSpeedLimit()), and the
/// claw closes on a pin by itself, firing as early as the robot's speed says
/// it must, as autoClaw does: the claw open and armed, and the lift sent to
/// the bottom (setLiftLevel(goal, 0)) and down there. A task of its own
/// watches the sensor every 10ms, so it grabs in the middle of a motion. The
/// lift stays down after
/// a grab (driver control's 75 degrees is update()'s): raise it with
/// setLiftLevel().
///
/// Without grabMm it grabs on autoClaw's lead, as driver control does: early
/// enough at the robot's speed for the jaws to shut with the pin at kGrabMm
/// (macros.cpp). grabMm, if given, is a fixed place to grab instead: the claw
/// closes the first time the sensor reads that many millimetres or closer,
/// whatever the speed, for a routine tuned at its own approach speed.
/// Further out grabs sooner, closer in later. Capped at 2000, the sensor's
/// range. Only the grab moves: what re-arms it after the claw opens on a pin
/// is still kPieceDetectMm. Each call sets it, so a call while it's on moves
/// it, and one without it goes back to the lead. Ignored by
/// setAutoClaw(false).
///
/// Turned on with the claw open and empty, opened kAutoClawClearMs (400ms)
/// or more ago, it's armed at once: a routine may turn it on just short of
/// the Pin (autons.cpp's ontoPin() does, for the last third of the run).
/// Its speed limit holds only motions that start after it, and never
/// followTrajectory(), whose plan sets its own speeds: ontoPin() starts its
/// approach() right after it, and turns it off again after the grab, so the
/// turn after that isn't held to it.
///
/// It turns itself off kClawSpeedOffDelayMs after the grab, as clawSpeed
/// does, so the motions after it are full speed again; the claw opening
/// before then keeps it on. It also turns off with setAutoClaw(false), when
/// autonomous ends, when the routine returns, and on driver control's first
/// update(), which has the claw from then on.
///
///   macros::setLiftLevel(macros::Goal::alliance, 0); // down to pick it up
///   macros::setAutoClaw(true);                        // slow, and grab it
///   moveToPose(-27.5, -20, 133);                      // up to the pin
///   if (!macros::clawClosed()) return;                // missed it: stop
///
///   macros::setAutoClaw(true, 90);                    // the same, at 90mm
void setAutoClaw(bool on, std::optional<std::int32_t> grabMm = std::nullopt);

/// Whether setAutoClaw()'s mode is on. Safe from any task.
bool autoClaw();

/// The two side toggles: left on H, right on G (config.hpp).
enum class Side : std::uint8_t { left, right };

/// Pushes both side toggles out (true) or pulls them in (false), as Y does.
/// Their auto pulses don't run in autonomous (see Side toggles).
void setExtended(bool out);

/// Pushes one side toggle out (true) or pulls it in (false), leaving the
/// other where it is:
///
///   macros::setExtended(macros::Side::left, true);   // just the left one out
///   macros::setExtended(macros::Side::right, false); // the right one in
void setExtended(Side side, bool out);

/// Whether either side toggle is out, or about to be: what Y or
/// setExtended() last told them, not an auto pulse.
bool extended();

/// Whether one side toggle is out, or about to be, as extended() above.
bool extended(Side side);

// --- The lift, for autonomous ---
//
// Each returns at once while the lift's task (startLift()) drives it, so the
// routine can drive on meanwhile; waitForLift() waits for it to get there.
// Like the claw and the side piston: call them from the task running the
// routine, never while update() runs on another task, which would override
// them on its next tick anyway.
//
//   macros::setLiftLevel(macros::Goal::alliance, 3); // start it up...
//   drive(24);                                       // ...while driving there
//   macros::waitForLift(1500);
//   macros::setLiftDeg(macros::liftScoreDeg());      // dip as R2 does
//   macros::waitForLift(1000);
//   macros::setClawClosed(false);

/// The goals RIGHT cycles through, in its order, each with its own level
/// heights (macros.cpp's ladder).
enum class Goal : std::uint8_t { alliance, medium, center };

/// Sends the lift to a goal's level. Levels go by the numbers the controller
/// shows: 0 is the bottom, then ALLIANCE's L1 to L6, MEDIUM's L2 to L6 and
/// CENTER's L2 to L5 (neither has an L1). A level the goal doesn't have goes
/// to the nearest one it does, L1 on MEDIUM to its L2 and L9 to any goal's
/// top, with a warning on the terminal. Driver control carries on from this
/// goal and level, as if R1, L1, L2 and RIGHT had picked them. At the
/// bottom, the lift goes all the way down whether the claw is closed or not
/// (driver control then lifts it to 75 degrees if it is, see Bottom lift).
/// Sending it up shows the LEDs' LiftGauge for a second (leds.hpp), as L1
/// does in driver control.
void setLiftLevel(Goal goal, int level);

/// Sends the lift to `deg` motor degrees up from the bottom switch, for a
/// height between levels: a score's dip (liftScoreDeg()), say. A `deg` that
/// isn't a number is ignored. Driver control goes back to the level the last
/// setLiftLevel() picked (the bottom if none did).
void setLiftDeg(double deg);

/// Where the level the last setLiftLevel() picked sits, in motor degrees up
/// from the bottom switch (the bottom if none did): for a setLiftDeg() a set
/// amount above or below it.
double liftLevelDeg();

/// Where R2's score dips to from the level the last setLiftLevel() picked,
/// in motor degrees: 30 degrees past halfway to the level below, but never
/// past it. At the bottom, the bottom. For setLiftDeg().
double liftScoreDeg();

/// Waits until the lift is within 15 motor degrees of where setLiftLevel()
/// or setLiftDeg() last sent it, or `timeoutMs` runs out (0 for no limit,
/// never in a match). Returns whether it got there: false if it's held up,
/// or every lift motor is unplugged. Blocks, so autonomous only.
bool waitForLift(std::uint32_t timeoutMs);

/// How many scores have opened the claw since the program started: the
/// face winks each time it goes up (screen.cpp). Safe from any task.
std::uint32_t scoreCount();

/// Whether the claw's distance sensor sees a pin: what an autonomous routine
/// branches on (autons.cpp's Branch routines). Reads the sensor, so it's
/// current; safe from any task, including a trajectory's next.
bool pinInClaw();

/// What the claw's distance sensor reads, in mm: for a routine that wants a
/// closer check than pinInClaw()'s (autons.cpp's fromLoader() wants a Pin
/// within 50mm after a grab). 9999 when it sees nothing, and INT32_MAX
/// (PROS_ERR) unplugged, so a "this close or closer" test fails on both.
/// Reads the sensor, so it's current; safe from any task.
std::int32_t clawSensorMm();

/// Whether the lift is pressing its bottom switch. Reads the switch, so it's
/// current; safe from any task.
bool liftAtBottom();

/// The most volts driver control may put on any drive wheel with the lift
/// where it is now, so the robot drives slower the higher the lift is: 12V
/// with it at the bottom, falling in a straight line to 7V at its full height
/// (1520 motor degrees, MEDIUM's top level), and 7V above that. 12V with no
/// lift reading. Never more than kClawSpeedVolts (macros.cpp) while clawSpeed
/// is on (R1+R2). Reads the
/// lift's motors, so it's current; driver.cpp's drive() calls it every tick,
/// on the same task as update().
double driveMaxVolts();

/// Logs the lift to `logger`: its PID's every step (channel "lift") and
/// what each update actually commanded (channel "lift.act": target and
/// position in motor degrees, the volts sent after gravity feedforward and
/// the clamp, and which branch of the control law ran, see
/// docs/TELEMETRY_FORMAT.md), and the macros' own state every tick (channel
/// "mech"). Call from initialize(), before startLift(): it attaches the PID's
/// observer and the lift's step listener, neither of which is synchronized
/// with the lift's task. Without it, nothing is logged and nothing else
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
/// (PositionMechanism::beginExternalControl()), the lift's task keeps
/// running, and logs law "external", and hands them back after. Start it with the lift
/// all the way down.
sapphirelib::tuning::MechanismCharacterizationConfig liftExperiment();

/// Run Test for the Lift entry: raises the lift to a test height and back
/// down to another, through its normal loop (the same PID, gravity volts and
/// update() the macros run), then returns it to its current level. Blocks;
/// the PID page runs it on a task of its own. The lift only moves while
/// driver control is running update().
void liftTuningTest();

} // namespace robot::macros
