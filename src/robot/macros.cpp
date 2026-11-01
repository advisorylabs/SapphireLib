/**
 * \file macros.cpp
 *
 * Driver macros for the lift, the claw, and the extender, see macros.hpp for
 * the controls. Everything meant to be tuned is in the constants at the top;
 * the ports are in config.hpp.
 *
 * The mechanics (button edges, controller-screen throttling, piston state,
 * the lift's control law, and the score's steps) come from SapphireLib's
 * input/, mechanism/, and util/ primitives. What's left here is this robot's
 * numbers and rules.
 *
 * Team 96671H: Hitmen
 */

#include "robot/macros.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "robot/config.hpp"
#include "robot/devices.hpp"
#include "sapphirelib/auton/auton.hpp"
#include "sapphirelib/mechanism/piston.hpp"
#include "sapphirelib/mechanism/position_mechanism.hpp"
#include "sapphirelib/mechanism/preset_ladder.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/telemetry/logger.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"
#include "sapphirelib/util/sequence.hpp"
#include "sapphirelib/util/timing.hpp"
#include "sapphirelib/util/wait.hpp"

namespace robot::macros {

namespace {

using sapphirelib::Sequence;
using sapphirelib::TimedFlag;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
using sapphirelib::input::ControllerScreen;
using sapphirelib::mechanism::Piston;
using sapphirelib::mechanism::PositionMechanism;
using sapphirelib::mechanism::PositionStep;
using sapphirelib::mechanism::PresetLadder;
using sapphirelib::tuning::GravityKind;
using sapphirelib::tuning::MechanismCharacterizationConfig;

// --- Claw and autoClaw ---
// autoClaw only works while clawSpeed is on (DOWN, see kClawSpeedVolts), so
// it's off again once a grab turns clawSpeed off.
// Where autoClaw grabs: it closes the claw the first time the claw's distance
// sensor reads this or closer, at any speed. It's also what counts as a pin
// in the claw, for re-arming autoClaw (kAutoClawClearMs) and for pinInClaw()
// (the Branch autons), so keep it between what the sensor reads with a pin
// loaded and without one.
constexpr std::int32_t kPieceDetectMm = 125;
// Once the claw opens on a pin, autoClaw waits for the sensor to have seen
// the claw empty (nothing at kPieceDetectMm or closer) for this long before
// it can close again, so one reading that misses the pin still sitting in
// the open claw doesn't count as it gone. A claw that had already been empty
// this long when it opened can close again at once.
constexpr std::uint32_t kAutoClawClearMs = 400;

// --- Scoring ---
// Once a score opens the claw, the lift holds where it dipped to until the
// robot has backed this far off the goal, by the vertical tracking wheel, and
// only then heads for the bottom: lowering with the claw still around the
// pin drags it back off the goal. Measured from where the robot was as the
// claw opened, so backing up during the dip doesn't count. Relies on the
// wheel's sign being right (config.hpp: driving forward must raise it), as
// odometry does: backwards, the lift waits for the robot to drive forward.
constexpr double kScoreBackOffIn = 3.0;
// A score dips this many motor degrees further than halfway to the level
// below, on every goal and level, but never past the level below itself.
constexpr double kScoreExtraDipDeg = 30.0;
// At the bottom level, the lift holds this many motor degrees whenever the
// claw is closed, and goes down to the bottom whenever it's open: a grab
// (autoClaw or R1) lifts it here, opening the claw sends it back down, and
// coming down to the bottom with the claw closed (the preload at driver
// control's start included) stops here. Other levels ignore the claw.
constexpr double kBottomClosedLiftDeg = 75.0;
// A grab waits this long after the claw closes before lifting, so the jaws
// are shut on the pin before it leaves the floor. Only the claw closing
// starts the wait: a lift coming down with the claw long since closed stops
// at kBottomClosedLiftDeg without one.
constexpr std::uint32_t kGrabLiftDelayMs = 65;

// --- Timing ---
// L1/L2 held: one level on the press, another after the delay, then one every
// period for as long as it's held.
constexpr std::uint32_t kLevelRepeatDelayMs = 300;
constexpr std::uint32_t kLevelRepeatPeriodMs = 150;

// --- Lift ---
// Position loop on the lift motors' encoders, in volts per motor degree:
// Auto-Tune's gains, designed from its measured lift model (kS 0.71, kV
// 0.0096, kA 0.0004, kG 1.08, in volts per motor degree, per degree/s and per
// degree/s^2). Only kG is used outside the design, as kLiftGravityVolts
// below. kP = 0.049 is full power at about 245 degrees of error. Raising kP
// won't make a lift that stalls far below its target climb higher: that far
// out it's already getting the full 12V.
constexpr sapphirelib::PIDGains kLiftGains{.kP = 0.1, .kI = 0.0, .kD = 0.0};
// The most the lift is ever driven down with: 75% of full power (12V), so
// it comes down no harder than this however far it has to go. Its loop, the
// seat, the seek and debug mode's L2 all stop here, and so do Auto-Tune's
// ramps and the Lift Testing auton. Going up is still full power. It caps
// volts, not speed: gravity helps the lift down, so it can still come down
// a little faster than 75% of what full power lifts it at.
constexpr double kLiftMaxDownVolts = 9.0;
// Constant upward volts added to the loop's output to carry the lift's
// weight: Auto-Tune's kG. If the lift settles a little below each level,
// raise this until it doesn't.
constexpr double kLiftGravityVolts = 1.08;
// The bottom sits on the lift's hard stop, where the loop alone settles a few
// degrees short (the gravity volts push up, and what little kP gives that
// close can't beat friction). So heading for the bottom, the loop drives down
// with at least kLiftSeatVolts until the creep below takes over, and once its
// encoders read within kLiftRestDeg of zero (or below), it cuts power and
// rests on the stop rather than pushing into it.
constexpr double kLiftSeatVolts = 0.0;
constexpr double kLiftRestDeg = 2.0;
// The encoders drift, so every press of the bottom switch zeroes them there.
// Heading for the bottom, below kLiftSlowDeg the lift leaves its loop and
// creeps down at kLiftSlowVolts: slow enough to stop at the switch, where at
// full speed it carries on down past the zero the switch just set. It stops
// where the encoders put the bottom, switch or no switch.
constexpr double kLiftSlowDeg = 150.0;
constexpr double kLiftSlowVolts = 4.0;
// Stopped there for kLiftSeekWaitMs with the switch still not pressed, it
// creeps on down to find it, but no more than kLiftSeekBelowDeg past the
// encoders' zero (and for no more than kLiftSeekTimeoutMs, so with the switch
// unplugged it isn't pushing into its hard stop all match), then rests where
// it is. A lift held up off the switch (fighting another robot, or the claw
// pushed up) gets the wait to come down onto it, and after that unwinds no
// further than kLiftSeekBelowDeg. Once per trip down: once the switch is
// found or given up on, the lift doesn't look again until it's sent up to a
// level. Driver control's start counts as a trip down, from wherever
// initialize() zeroed the encoders, so start the program with the lift down.
// TODO: tune on the robot. Each zero logs the encoders' reading at the
// switch (event lift_zeroed): keep kLiftSeekBelowDeg a little more than the
// furthest below 0 they read.
constexpr std::uint32_t kLiftSeekWaitMs = 2000;
constexpr double kLiftSeekBelowDeg = 100.0;
constexpr std::uint32_t kLiftSeekTimeoutMs = 1000;
// How close the lift must get before a score's dip moves on to opening the
// claw, and how long the dip waits for that before moving on anyway.
constexpr double kLiftToleranceDeg = 15.0;
constexpr std::uint32_t kLiftDescendTimeoutMs = 1000;
// Debug mode: what L1 and L2 put on the lift motors while held. L2's is
// capped at kLiftMaxDownVolts.
constexpr double kLiftDebugVolts = 12.0;
// The lift runs its loop on a task of its own this often (startLift()), in
// autonomous and driver control alike. kI and kD assume it (pid.nominalDtS).
constexpr std::uint32_t kLiftPeriodMs = 20;

// --- Drive speed with the lift up ---
// The higher the lift, the slower driver control may drive (driveMaxVolts()):
// the most any drive wheel gets falls in a straight line from full power with
// the lift at the bottom to kDriveTopVolts at kDriveTopLiftDeg (MEDIUM's top
// level, the lift's full height), and stays there above it. Past the limit,
// more stick doesn't go any faster. By where the lift is, not where it's
// headed, so the drive slows as it climbs and speeds up as it comes down.
constexpr double kDriveFullVolts = 12.0;
constexpr double kDriveTopVolts = 7.0;
constexpr double kDriveTopLiftDeg = 1520.0;

// --- clawSpeed (DOWN) ---
// While clawSpeed is on, no drive wheel gets more than this many volts,
// and less if the lift's height says so (kDriveTopVolts), and autoClaw can
// close the claw (it can't with clawSpeed off). DOWN turns it on
// and off, and the claw closing with the lift down at the bottom (autoClaw or
// R1) turns it off kClawSpeedOffDelayMs later, so the robot drives off with
// the pin at full speed. The claw opening again before then keeps it on.
constexpr double kClawSpeedVolts = 5.0;
constexpr std::uint32_t kClawSpeedOffDelayMs = 350;
// The controller rumbles as clawSpeed turns on (one long buzz) and off (two
// short ones), by DOWN or by a grab, since nothing on its screen shows it.
constexpr const char* kClawSpeedOnRumble = "-";
constexpr const char* kClawSpeedOffRumble = "..";
// setAutoClaw(), autonomous's clawSpeed, caps the drive at the same
// kClawSpeedVolts and ends kClawSpeedOffDelayMs after its grab too. Its task
// reads the claw's sensor this often while it's on.
constexpr std::uint32_t kAutonClawPollMs = 10;

// --- Lift Auto-Tune and Run Test (the PID page's Lift entry) ---
// Auto-Tune drives the lift up and down between these two heights, stopping
// each segment once it passes one, so keep both well inside the lift's real
// travel (it takes a few degrees to brake), and start with the lift down.
constexpr double kLiftTuneLowerDeg = 60.0;
constexpr double kLiftTuneUpperDeg = 1300.0;
// Up steps fight gravity, so they get more volts than down steps, which it
// helps.
constexpr double kLiftTuneUpVolts = 8.0;
constexpr double kLiftTuneDownVolts = 4.0;
// Run Test: up to the first height, back down to the second, waiting for each
// to settle (or the timeout), then back to wherever the level says.
constexpr double kLiftTestHighDeg = 450.0;
constexpr double kLiftTestLowDeg = 150.0;
constexpr std::uint32_t kLiftTestMoveTimeoutMs = 2500;
constexpr std::uint32_t kLiftTestDwellMs = 500;

// Goals, one table per goal height, in macros.hpp's Goal order (autonomous's
// setLiftLevel() picks them by it): lift target at each level, in motor
// degrees up from the bottom switch. Entry 0 is the bottom, where the lift
// rests on its switch and autoClaw picks up pins, the same in every table;
// entries 1 and up are that goal's scoring levels, one per L1 notch, so the
// entry count sets how many levels the goal has. Spacing needn't be even: a
// score dips kScoreExtraDipDeg past halfway to the level below, however big
// that gap is. Names are shown on the controller; 8 characters max.
//
// Only ALLIANCE is scored on at L1. MEDIUM's and CENTER's lowest scoring
// level is L2, so their entry 1 is their L2 (the third number below), and the
// controller shows it as L2, entry 2 as L3, and so on.
//
// All measured on the motor encoders. To measure one again: the controller's
// third line shows the lift's live reading (then its current target), so run
// the lift to each height in debug mode and copy the number. Switching goals
// keeps the L number, so from ALLIANCE's L1 the lift goes to the others' L2,
// and from a level higher than the new goal has, to its top one.
PresetLadder ladder({
    {"ALLIANCE", {0, 225, 370, 635, 910, 1170, 1430}},
    {"MEDIUM", {0, 425, 705, 975, 1245, 1520}, /*firstLevelNumber=*/2},
    {"CENTER", {0, 630, 915, 1170, 1450}, /*firstLevelNumber=*/2},
});
constexpr int kBottomLevel = 0; // on the bottom switch, where autoClaw picks up pins

// Extended is closed: the claw starts closed, on a preload. Its valve is
// plumbed so the port's low state closes it, hence extendedIsLow: without it,
// the claw started open, R1 and R2 were backward, and autoClaw opened the
// claw on a pin instead of closing it. If the air lines are ever swapped,
// make this false again.
Piston claw(ports::kClawPiston, /*extendedAtStart=*/true, /*extendedIsLow=*/true);
Piston extender(ports::kExtenderPiston, /*extendedAtStart=*/false);
pros::Distance clawSensor(ports::kClawDistance);
pros::adi::DigitalIn bottomSwitch(ports::kLiftBottomSwitch);

// The lift's position: the median of its motors' encoders, in motor degrees.
// A motor that doesn't answer is left out, so the lift keeps holding its
// levels on the rest. The median rather than the average, so one motor that
// browned out and came back, its encoder restarted from 0, can't drag the
// others' number with it. NaN only with every motor gone. Declared here for
// `lift`, which reads it.
double liftPositionDeg();

// Runs its loop on a task of its own every kLiftPeriodMs (startLift()), so it
// keeps moving and holding while an autonomous routine blocks on a drive
// motion. update() and the autonomous functions only tell it where to go.
PositionMechanism lift(std::vector<std::int8_t>(ports::kLiftMotors.begin(),
                                                ports::kLiftMotors.end()),
                       [] { return liftPositionDeg(); },
                       {
                           .pid = {.gains = kLiftGains,
                                   .outputLimit = 12.7,
                                   // Level changes step the target, which would
                                   // otherwise kick the output.
                                   .derivativeOnMeasurement = true,
                                   .nominalDtS = kLiftPeriodMs / 1000.0}, // its task's period
                           .gravity = {.constantVolts = kLiftGravityVolts},
                           .seat = {.enabled = true,
                                    .floor = 0.0,
                                    .seatVolts = kLiftSeatVolts,
                                    .restBand = kLiftRestDeg},
                           .maxDownVolts = kLiftMaxDownVolts,
                           .tolerance = kLiftToleranceDeg,
                       });

double liftPositionDeg() {
    std::array<double, ports::kLiftMotors.size()> readings{};
    std::size_t count = 0;
    for (std::size_t i = 0; i < readings.size(); ++i) {
        // PROS_ERR_F (infinity) from a motor that doesn't answer.
        const double deg = lift.motors().get_position(static_cast<std::uint8_t>(i));
        if (std::isfinite(deg)) readings[count++] = deg;
    }
    if (count == 0) return std::numeric_limits<double>::quiet_NaN();
    std::sort(readings.begin(), readings.begin() + count);
    const std::size_t middle = count / 2;
    return count % 2 == 1 ? readings[middle] : (readings[middle - 1] + readings[middle]) / 2.0;
}

// What LEFT cycles the controller's bottom line through: 0 is the lift, and
// 1 and up each lift motor in turn, A first.
constexpr int kReadoutCount = 1 + static_cast<int>(ports::kLiftMotors.size());

struct State {
    // The claw: closed holds a pin. R1 flips it, a score opens it, and
    // autoClaw closes it.
    bool clawClosed = true;
    bool extended = false; // the extender, Y's alone
    bool clawSpeed = false; // the drive capped at kClawSpeedVolts, see there
    // When a grab at the bottom closed the claw with clawSpeed on, which ends
    // it kClawSpeedOffDelayMs later. Dropped if the claw opens first, or
    // DOWN gets there first.
    std::optional<std::uint32_t> clawSpeedGrabMs;
    // Whether autoClaw may close the claw: false from the moment the claw
    // opens on a pin until the sensor has seen the claw empty for
    // kAutoClawClearMs, so opening the claw to drop a pin doesn't snap it
    // shut on the same one.
    bool autoClawArmed = false;
    // The claw piston has been closed for kGrabLiftDelayMs, as of this tick's
    // start: until then, a grab at the bottom leaves the lift down.
    bool clawGripping = false;
    // This trip down's search for the bottom switch (see kLiftSeekWaitMs):
    // when the lift got down to the encoders' zero without it, and whether
    // the search is over, the switch found or given up on. Both start over
    // whenever the lift is sent up to a level.
    std::optional<std::uint32_t> atZeroSinceMs;
    bool seekOver = false;
    // The bottom switch has zeroed the lift's encoders for this press (see
    // zeroAtBottom()).
    bool zeroedThisPress = false;
    // The vertical tracking wheel's reading as the running score opened the
    // claw, for kScoreBackOffIn.
    double releaseWheelIn = 0.0;
    // The lift's memory: the level one above the last score's, where L1 goes
    // next from anywhere below it (levelUp()). Forgotten by L1+L2 together
    // and by RIGHT, and replaced by the next score.
    std::optional<int> nextLevel;
    // The level the lift was at before the L1 or L2 press that last moved it,
    // while that button is still held, so L1+L2 pressed a tick or two apart
    // can undo the first one's step. Dropped once neither is held.
    std::optional<int> levelBeforePress;
    // L1+L2 were pressed together: neither moves the lift again until both
    // are let go.
    bool levelButtonsLocked = false;
    bool debug = false; // the debug mode update() is running, see debugMode
    int readout = 0;    // see kReadoutCount
};

State state;

// Debug mode as asked for by setDebug(), from any task. update() copies it
// into state.debug once a tick, so a whole tick runs in one mode.
std::atomic<bool> debugMode{false};

// Whether the claw's distance sensor sees a pin in the claw (kPieceDetectMm),
// and since when, for kAutoClawClearMs.
TimedFlag clawLoaded;

// Every score that has opened the claw, for the face's wink (scoreCount()).
std::atomic<std::uint32_t> scoresReleased{0};

// The score: dip kScoreExtraDipDeg past halfway to the level below, open the
// claw, hold there until the robot has backed kScoreBackOffIn off the goal,
// then down to the bottom.
// What each step *does* stays level-triggered below (claw, lift target); the
// sequence only says which step it is and when to move on.
enum class Phase { scoreDescend, scoreRelease };

bool liftArrived();
bool backedOff();

void beginRelease() {
    state.releaseWheelIn = verticalWheel().getDistanceIn();
    scoresReleased.fetch_add(1);
}

// No timeout on the release: the lift waits for the robot to back off however
// long that takes, and L2 is the way out (see update()).
const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{
    {.id = Phase::scoreDescend, .until = liftArrived, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::scoreRelease, .until = backedOff, .onEnter = beginRelease},
}};
const Sequence<Phase>::Program kScore{.steps = kScoreSteps,
                                      .onFinish = [] { ladder.setLevel(kBottomLevel); }};

Sequence<Phase> sequence;

// Set by liftTuningTest() from the PID page's task, read by update(): while
// it's a number, the lift heads there instead of to its level.
std::atomic<double> liftTestTarget{std::numeric_limits<double>::quiet_NaN()};

// The "mech" telemetry channel, once attachTelemetry() has made it.
sapphirelib::telemetry::Channel* mechLog = nullptr;

// setAutoClaw()'s mode. Its task and the routine's (setClawClosed()) both
// move the claw while it's on, so both hold autonClawMutex() to, and so does
// update() turning it off on its first tick: from then on the claw, and
// State, are update()'s alone again.
struct AutonClaw {
    bool on = false;
    // pros::competition::get_status() as it turned on, so autonomous ending
    // turns it off (motion::competitionModeChanged()).
    std::uint8_t startStatus = 0;
    // When it closed the claw on a pin, which ends it kClawSpeedOffDelayMs
    // later. Dropped if the claw opens first.
    std::optional<std::uint32_t> grabMs;
};

AutonClaw autonClaw;

// Built on first use, at runtime, like every PROS object here that needs the
// kernel running.
pros::Mutex& autonClawMutex() {
    static pros::Mutex instance;
    return instance;
}

// setAutoClaw()'s task, from its first call on. Never deleted, as the lift's
// isn't.
std::unique_ptr<pros::Task> autonClawTask;

bool pieceInClaw(std::int32_t distanceMm) {
    // get_distance() reads 9999 when it sees nothing and PROS_ERR (INT32_MAX)
    // when unplugged, so neither counts as a pin.
    return distanceMm <= kPieceDetectMm;
}

bool switchPressed() { return bottomSwitch.get_value() == 1; }

/// Where a score dips to: kScoreExtraDipDeg past halfway to the level below,
/// but no lower than that level. At the bottom, the bottom.
double scoreDipDeg() {
    const int level = ladder.level();
    return std::max(ladder.midpointBelow(level) - kScoreExtraDipDeg,
                    ladder.levelPosition(level - 1));
}

double liftTargetDeg() {
    const double testTarget = liftTestTarget.load();
    if (std::isfinite(testTarget)) return testTarget;
    // Ahead of the score, so one at the bottom opens the claw where the lift
    // is, and the lift heads down once it has.
    if (ladder.level() == kBottomLevel && state.clawClosed) {
        return state.clawGripping ? kBottomClosedLiftDeg : ladder.levelPosition(kBottomLevel);
    }
    if (sequence.running(kScore)) return scoreDipDeg();
    return ladder.levelPosition();
}

/// Never true with every lift motor unplugged (isNear() is false with no
/// reading), so a score waiting on this falls through to its timeout.
bool liftArrived() { return lift.isNear(liftTargetDeg()); }

/// Whether the robot has backed kScoreBackOffIn off the goal since the claw
/// opened. At the bottom there's no lift to bring down, so no wait. An
/// unplugged wheel holds its last reading, so this never comes true: L2 ends
/// the wait instead.
bool backedOff() {
    if (ladder.level() == kBottomLevel) return true;
    return state.releaseWheelIn - verticalWheel().getDistanceIn() >= kScoreBackOffIn;
}

/// Zeroes the lift's encoders on the first tick of each press of the bottom
/// switch, so every level is measured from the real bottom however far the
/// encoders had drifted, and logs how far that was. Not while Auto-Tune has
/// the motors: a jump in the middle of its run would spoil the fit, so with
/// the switch still pressed it zeroes once the run is over instead.
void zeroAtBottom(bool atBottom) {
    if (!atBottom) {
        state.zeroedThisPress = false;
        return;
    }
    if (state.zeroedThisPress || lift.externalControl()) return;
    const double driftDeg = lift.position();
    lift.motors().tare_position_all();
    // The reading just jumped: start the loop fresh, or its derivative kicks.
    lift.resetController();
    state.zeroedThisPress = true;
    sapphirelib::telemetry::event("macro", "lift_zeroed,deg=%.0f", driftDeg);
}

/// Whether the lift creeps down at kLiftSlowVolts this tick instead of running
/// its loop (see kLiftSlowDeg): heading for the bottom, off the switch, and
/// below kLiftSlowDeg, down to where the encoders put the bottom, then, after
/// kLiftSeekWaitMs resting there, on past it while this trip's search for the
/// switch lasts.
bool creepingDown(std::uint32_t now, bool atBottom) {
    if (liftTargetDeg() > ladder.levelPosition(kBottomLevel)) {
        // Sent up: the next trip down searches for the switch again.
        state.atZeroSinceMs.reset();
        state.seekOver = false;
        return false;
    }
    if (atBottom) {
        state.seekOver = true;
        return false;
    }
    const double deg = lift.position();
    // Above the slow zone the loop brings it down, and with no reading (NaN)
    // the loop brakes.
    if (!(deg <= kLiftSlowDeg)) return false;
    if (deg > kLiftRestDeg) return true;
    // Where the encoders put the bottom, but the switch isn't pressed.
    if (state.seekOver) return false;
    if (!state.atZeroSinceMs) state.atZeroSinceMs = now;
    const std::uint32_t atZeroMs = sapphirelib::elapsedMs(*state.atZeroSinceMs, now);
    // Rest at zero for the wait first.
    if (atZeroMs < kLiftSeekWaitMs) return false;
    if (deg > -kLiftSeekBelowDeg && atZeroMs < kLiftSeekWaitMs + kLiftSeekTimeoutMs) return true;
    state.seekOver = true;
    SAPPHIRELIB_LOG_WARN("macros", "lift didn't reach its bottom switch (port %c), resting at %.0f",
                         ports::kLiftBottomSwitch, deg);
    sapphirelib::telemetry::event("macro", "seek_failed,deg=%.0f", deg);
    return false;
}

/// Whether the lift is down at its bottom level's height, by its encoders,
/// switch or no switch: at or below it, since a search for the switch that
/// gives up leaves them reading below. False with no reading.
bool liftDown() {
    return lift.position() <= ladder.levelPosition(kBottomLevel) + kLiftToleranceDeg;
}

/// Whether autoClaw would close the claw on this reading, whichever mode turns
/// it on (clawSpeed or setAutoClaw()): the claw is open and armed, the lift is
/// down (liftDown()) and meant to be there, and the reading has a pin at
/// kPieceDetectMm or closer. On the first such reading, since any wait only
/// makes it later.
bool grabReady(std::int32_t pieceMm) {
    return !state.clawClosed && state.autoClawArmed && ladder.level() == kBottomLevel &&
           liftDown() && pieceInClaw(pieceMm);
}

/// autoClaw in driver control (see macros.hpp): clawSpeed is on, and
/// grabReady().
bool autoClawWants(std::int32_t pieceMm) { return state.clawSpeed && grabReady(pieceMm); }

/// Turns setAutoClaw()'s mode off, and the speed limit with it. With
/// autonClawMutex() held.
void endAutonClaw(const char* why) {
    if (!autonClaw.on) return;
    autonClaw.on = false;
    autonClaw.grabMs.reset();
    sapphirelib::auton::setSpeedLimit(kDriveFullVolts);
    sapphirelib::telemetry::event("macro", "auto_claw,on=0,%s", why);
}

/// One look at the claw for setAutoClaw()'s mode, from its task, with
/// autonClawMutex() held: autoClaw's arming and grab as update() does them,
/// then the mode's end.
void autonClawStep(std::uint32_t now) {
    if (!autonClaw.on) return;
    if (sapphirelib::motion::competitionModeChanged(autonClaw.startStatus,
                                                    pros::competition::get_status())) {
        endAutonClaw("mode");
        return;
    }
    const std::int32_t pieceMm = clawSensor.get_distance();
    clawLoaded.set(pieceInClaw(pieceMm), now);
    if (clawLoaded.falseFor(kAutoClawClearMs, now)) state.autoClawArmed = true;
    if (!autonClaw.grabMs && grabReady(pieceMm)) {
        state.clawClosed = true;
        claw.set(true, now);
        autonClaw.grabMs = now;
        sapphirelib::telemetry::event("macro", "autoclaw,piece_mm=%d", static_cast<int>(pieceMm));
    }
    if (autonClaw.grabMs && sapphirelib::elapsedMs(*autonClaw.grabMs, now) >= kClawSpeedOffDelayMs) {
        endAutonClaw("grabbed");
    }
}

/// R2's score remembers the level above the one it scores at, for L1's next
/// press, or the top level for a score at the top. Not a score at the bottom,
/// which drops the pin rather than scoring it.
void rememberNextLevel() {
    const int level = ladder.level();
    if (level == kBottomLevel) return;
    state.nextLevel = std::min(level + 1, ladder.maxLevel());
    sapphirelib::telemetry::event("macro", "lift_memory,level=%d", *state.nextLevel);
}

void forgetNextLevel() {
    if (!state.nextLevel) return;
    state.nextLevel.reset();
    sapphirelib::telemetry::event("macro", "lift_memory,level=0");
}

/// Sends the lift to `deg` for autonomous (setLiftLevel(), setLiftDeg()).
/// Sent up off the bottom, the next trip down searches for the switch again
/// and the next press of it zeroes the encoders, as creepingDown() and
/// zeroAtBottom() would have arranged had update() been running to see it.
void sendLift(double deg) {
    if (deg > ladder.levelPosition(kBottomLevel)) {
        state.atZeroSinceMs.reset();
        state.seekOver = false;
        state.zeroedThisPress = false;
    }
    lift.setTarget(deg);
}

/// L1: straight to the remembered level (state.nextLevel) while it's above the
/// lift's, otherwise up one.
void levelUp() {
    if (state.nextLevel && *state.nextLevel > ladder.level()) {
        ladder.setLevel(*state.nextLevel);
    } else {
        ladder.up();
    }
}

void setClawSpeed(Controller& controller, bool on) {
    state.clawSpeed = on;
    state.clawSpeedGrabMs.reset();
    controller.rumble(on ? kClawSpeedOnRumble : kClawSpeedOffRumble);
    sapphirelib::telemetry::event("macro", "claw_speed,on=%d", on ? 1 : 0);
}

void showStatus(Controller& controller, bool atBottom) {
    ControllerScreen& screen = controller.screen();
    if (state.debug) {
        // No level to show: the lift is L1/L2's. Each piston, to check it
        // moves the way the line says.
        screen.setLine(0, "DBG %s EX:%s", state.clawClosed ? "SHUT" : "OPEN",
                       state.extended ? "OUT" : "IN");
    } else if (state.nextLevel && *state.nextLevel > ladder.level()) {
        // Where L1 jumps to (levelUp()), in the goal's own level numbers:
        // "ALLIANCE BTM>L6" just fits the 15 columns.
        const int target = ladder.levelNumber(*state.nextLevel);
        if (ladder.level() == kBottomLevel) {
            screen.setLine(0, "%-8s BTM>L%d", ladder.tableName(), target);
        } else {
            screen.setLine(0, "%-8s L%d>L%d", ladder.tableName(), ladder.levelNumber(), target);
        }
    } else if (ladder.level() == kBottomLevel) {
        screen.setLine(0, "%-8s BOTTOM", ladder.tableName());
    } else {
        // The goal's own level numbers: MEDIUM's first scoring level reads L2.
        screen.setLine(0, "%-8s L%d/%d", ladder.tableName(), ladder.levelNumber(),
                       ladder.levelNumber(ladder.maxLevel()));
    }
    if (state.readout == 0) {
        // Lift reading, then target, in whole motor degrees, for measuring
        // level heights and seeing whether the lift is short of its target.
        // In debug mode there's no target, just the reading. "BTM" while the
        // bottom switch is pressed, for checking it.
        const char* label = atBottom ? "BTM " : "LIFT";
        const double liftDeg = lift.position();
        if (std::isnan(liftDeg)) {
            screen.setLine(2, "%s NO MOTORS", label);
        } else if (state.debug) {
            screen.setLine(2, "%s %5.0f", label, liftDeg);
        } else {
            screen.setLine(2, "%s %5.0f/%4.0f", label, liftDeg, liftTargetDeg());
        }
    } else {
        // One lift motor (A, then B, C... on each LEFT): the volts it's
        // actually applying, its current draw, and its temperature. A motor
        // stalled at 12V should be drawing its full 2.5A; much less than
        // that, or 55C and up (where it starts cutting its own current),
        // means it's being held back rather than out-muscled.
        const auto i = static_cast<std::uint8_t>(state.readout - 1);
        screen.setLine(2, "%c %3.0fV %3.1fA %2.0fC", 'A' + i, lift.motors().get_voltage(i) / 1000.0,
                       lift.motors().get_current_draw(i) / 1000.0,
                       lift.motors().get_temperature(i));
    }
}

} // namespace

void initialize() {
    lift.motors().set_encoder_units_all(pros::v5::MotorUnits::degrees);
    // Wherever the lift is now is the bottom until the switch says otherwise:
    // driver control's first trip down searches for the switch from here (see
    // kLiftSeekWaitMs). Already on it, this is that press's zero.
    lift.motors().tare_position_all();
    state.zeroedThisPress = switchPressed();
    // Only matters when the lift falls back to brake(), with no encoders, or
    // after stop().
    lift.motors().set_brake_mode_all(pros::v5::MotorBrake::hold);
}

void update(Controller& controller) {
    const std::uint32_t now = controller.now();
    // Driver control has the claw: setAutoClaw()'s mode, left on by an
    // autonomous routine (or X+UP's), ends here, and its task leaves the claw
    // alone from then on.
    {
        std::lock_guard<pros::Mutex> lock(autonClawMutex());
        endAutonClaw("driver");
    }
    // After autonomous, a disable, or a blocking routine run from opcontrol:
    // drop a half-finished score (its timing is stale) and start the lift
    // loop fresh. The controller already forgot what its screen showed, so
    // the text gets resent.
    if (controller.resumed()) {
        sequence.cancel();
        lift.resetController();
        // A search for the switch cut short by a disable starts its clock
        // again, rather than counting the pause toward its wait and timeout.
        state.atZeroSinceMs.reset();
    }

    // One reading of each sensor for the whole tick.
    const std::int32_t pieceMm = clawSensor.get_distance();
    const bool atBottom = switchPressed();
    zeroAtBottom(atBottom);
    // Extended is closed. The piston as last tick left it, so a claw closed
    // this tick waits the full kGrabLiftDelayMs from the next.
    state.clawGripping = claw.extendedFor(kGrabLiftDelayMs, now);
    clawLoaded.set(pieceInClaw(pieceMm), now);
    // Armed here, ahead of autoClaw below, so the tick the claw has been
    // empty long enough is one it can close on.
    if (clawLoaded.falseFor(kAutoClawClearMs, now)) state.autoClawArmed = true;

    // controller.update() sampled every button once for this tick, and edges
    // come from comparing samples, so reading (or not reading) a button
    // changes nothing.
    if (controller.pressed(Button::left)) state.readout = (state.readout + 1) % kReadoutCount;

    if (controller.pressed(Button::right)) {
        // The lift keeps its L number on the new goal (moving to that goal's
        // height for it), capped if the new goal has fewer levels, and raised
        // to L2 from ALLIANCE's L1. A new goal starts without the lift's
        // memory.
        ladder.nextTable();
        forgetNextLevel();
        // TODO: LED library's mode-change call goes here.
    }

    // L1 and L2 pressed together, in either order, forget the lift's memory
    // in every mode, and leave the lift where it is: neither moves it again
    // until both are let go (below).
    const bool levelReset = controller.combo(Button::l1, Button::l2);
    if (levelReset) {
        forgetNextLevel();
        state.levelButtonsLocked = true;
    }
    if (!controller.held(Button::l1) && !controller.held(Button::l2)) {
        state.levelButtonsLocked = false;
        state.levelBeforePress.reset();
    }

    // The extender is Y's alone: in every mode, and through a score.
    if (controller.pressed(Button::y)) state.extended = !state.extended;
    // So is clawSpeed DOWN's, though a grab at the bottom also ends it
    // (below).
    if (controller.pressed(Button::down)) setClawSpeed(controller, !state.clawSpeed);

    // Debug mode switched on or off (from the Home page): each mode starts
    // from a clean slate, no score carried across.
    const bool debugging = debugMode.load();
    if (debugging != state.debug) {
        state.debug = debugging;
        sequence.cancel();
        if (!debugging) {
            // Back on the levels, with the lift loop starting fresh, and from
            // wherever debug mode left the lift, the next trip down searches
            // for the switch.
            lift.resetController();
            state.atZeroSinceMs.reset();
            state.seekOver = false;
        }
    }

    const bool wasClosed = state.clawClosed;

    // One branch per tick, in priority order. Debug mode first, since it
    // leaves out everything that moves the lift. Then scoring: once started,
    // nothing interrupts it, and starting one beats everything else.
    if (state.debug) {
        // No levels, no score, no autoClaw: the claw is R1's, and the lift is
        // L1 and L2's alone (below).
        if (controller.pressed(Button::r1)) state.clawClosed = !state.clawClosed;
    } else if (sequence.running(kScore)) {
        // R1 and L1 wait for it to finish. It opens the claw itself (below),
        // and its onFinish sends the lift to the bottom once the robot has
        // backed off. With the claw open, L2 sends it there now instead: the
        // way out when the robot can't back up, or the vertical wheel is
        // unplugged and never sees it. (Checked before update(), so the claw
        // has been open for at least a tick.)
        if (sequence.is(Phase::scoreRelease) && controller.pressed(Button::l2)) {
            sequence.cancel();
            ladder.setLevel(kBottomLevel);
            sapphirelib::telemetry::event("macro", "score_lowered_early");
        } else {
            sequence.update(now);
        }
    } else if (controller.pressed(Button::r2) && state.clawClosed) {
        rememberNextLevel();
        sequence.start(kScore, now);
    } else {
        if (controller.pressed(Button::r1)) {
            state.clawClosed = !state.clawClosed;
        } else if (autoClawWants(pieceMm)) {
            state.clawClosed = true;
            sapphirelib::telemetry::event("macro", "autoclaw,piece_mm=%d", static_cast<int>(pieceMm));
        }
        if (state.levelButtonsLocked) {
            // L1+L2 rarely land on the same tick, so the first of them has
            // usually stepped the lift already: put it back, if that step was
            // its press's alone (held too briefly to have repeated).
            const std::uint32_t firstHeldMs =
                std::max(controller.heldMs(Button::l1), controller.heldMs(Button::l2));
            if (levelReset && state.levelBeforePress && firstHeldMs < kLevelRepeatDelayMs) {
                ladder.setLevel(*state.levelBeforePress);
            }
            state.levelBeforePress.reset();
        } else {
            if (controller.pressed(Button::l1) || controller.pressed(Button::l2)) {
                state.levelBeforePress = ladder.level();
            }
            if (controller.repeated(Button::l1, kLevelRepeatDelayMs, kLevelRepeatPeriodMs)) {
                levelUp();
            }
            if (controller.repeated(Button::l2, kLevelRepeatDelayMs, kLevelRepeatPeriodMs)) {
                ladder.down();
            }
        }
    }

    // A score's release opens the claw, and it stays open after: autoClaw, or
    // R1, closes it on the next pin.
    if (sequence.is(Phase::scoreRelease)) state.clawClosed = false;

    // Opening the claw on a pin disarms autoClaw until the sensor has seen
    // the claw empty for kAutoClawClearMs (armed again above). One already
    // empty that long stays armed, so it closes on the next pin.
    if (wasClosed && !state.clawClosed) {
        state.autoClawArmed = clawLoaded.falseFor(kAutoClawClearMs, now);
        // A grab let go of before clawSpeed ended leaves it on.
        state.clawSpeedGrabMs.reset();
    }
    // The claw closing with the lift down ends clawSpeed kClawSpeedOffDelayMs
    // later.
    if (!wasClosed && state.clawClosed && state.clawSpeed && liftDown()) {
        state.clawSpeedGrabMs = now;
    }
    if (state.clawSpeedGrabMs &&
        sapphirelib::elapsedMs(*state.clawSpeedGrabMs, now) >= kClawSpeedOffDelayMs) {
        setClawSpeed(controller, false);
    }

    claw.set(state.clawClosed, now);
    extender.set(state.extended, now);

    // What the lift should do this tick. Its own task (startLift()) does it.
    if (state.debug) {
        // Straight to the motors, whatever the encoders say (or don't): up
        // for L1, down for L2, and braked where it's let go (hold mode, see
        // initialize()). Both at once cancel out to braked. L2 stops at the
        // bottom switch rather than driving into the hard stop under it.
        const bool up = controller.held(Button::l1);
        const bool down = controller.held(Button::l2) && !atBottom;
        if (up != down) {
            lift.setVolts(up ? kLiftDebugVolts : -kLiftDebugVolts);
        } else {
            lift.stop();
        }
    } else if (creepingDown(now, atBottom)) {
        lift.setVolts(-kLiftSlowVolts);
    } else {
        lift.setTarget(liftTargetDeg());
    }
    showStatus(controller, atBottom);

    if (mechLog != nullptr) {
        // What every mechanism was told this tick, for the analyzer's replay
        // (the lift's own rows are in "lift"/"lift.act"). get_distance()
        // reads PROS_ERR unplugged, which is logged as a gap.
        const std::optional<Phase> phase = sequence.current();
        mechLog->record({claw.extended() ? 1.0 : 0.0, extender.extended() ? 1.0 : 0.0,
                         pieceMm == PROS_ERR ? std::numeric_limits<double>::quiet_NaN()
                                             : static_cast<double>(pieceMm),
                         static_cast<double>(ladder.level()), static_cast<double>(ladder.tableIndex()),
                         phase ? static_cast<double>(*phase) : -1.0, atBottom ? 1.0 : 0.0});
    }
}

void stop() {
    // "Brake every update", from the lift's task's next one (within
    // kLiftPeriodMs). The next update() from opcontrol sets a target again,
    // which puts the lift back on its loop, and so does a routine's
    // setLiftLevel() or setLiftDeg(). The pistons hold where they are on
    // their own.
    lift.stop();
}

void startLift() { lift.startTask(kLiftPeriodMs); }

void attachTelemetry(sapphirelib::telemetry::Logger& logger) {
    // Three decimals: the lift works in whole degrees and volts, and at
    // kD = 0.01 its derivative term is already in hundredths of a volt.
    // Capacity 128 is 2.5s at 20ms, the lift task's period and the opcontrol
    // tick alike.
    logger.pid("lift", lift.pid(), {.capacity = 128, .decimals = 3});
    // What the motors were actually sent, after gravity volts and the clamp,
    // the "lift" channel's "out" is only the loop's share. The listener runs
    // inside each update, on the lift's own task (startLift(), which comes
    // after this), and reuses that update's one position read; recording
    // never blocks.
    sapphirelib::telemetry::Channel& act = logger.channel(
        "lift.act", {"target", "pos", "volts", "law"}, {.capacity = 128, .decimals = 3});
    lift.setStepListener([channel = &act](const PositionStep& step) {
        channel->record({step.target, step.position, step.volts, static_cast<double>(step.law)});
    });
    // The claw, extender and lift state machine, every opcontrol tick: the
    // claw piston (1 closed) and the extender (1 out), the claw's distance
    // reading (mm), the lift's level (0 the bottom; its entry in the goal's
    // table, so MEDIUM's L2 logs 1) and goal (0 ALLIANCE, 1 MEDIUM, 2
    // CENTER), the running score step (-1 none; else Phase's order:
    // scoreDescend, scoreRelease), and the bottom switch (1 pressed).
    mechLog = &logger.channel("mech",
                              {"claw", "extender", "piece_mm", "level", "mode", "phase", "bottom"},
                              {.capacity = 128, .decimals = 2});
}

void setDebug(bool on) { debugMode.store(on); }

bool debug() { return debugMode.load(); }

void setClawClosed(bool closed) {
    // setAutoClaw()'s task moves the claw too.
    std::lock_guard<pros::Mutex> lock(autonClawMutex());
    const std::uint32_t now = sapphirelib::millis();
    // Opening disarms autoClaw, so driver control's first ticks (or
    // setAutoClaw()'s) don't snap the claw shut on a pin autonomous just let
    // go of: update() or setAutoClaw()'s task arms it again once the sensor
    // has seen the claw empty for kAutoClawClearMs, counted from a reading
    // taken now rather than whenever one was last taken.
    if (state.clawClosed && !closed) {
        state.autoClawArmed = false;
        clawLoaded.set(pieceInClaw(clawSensor.get_distance()), now);
        // A grab let go of before setAutoClaw()'s delay leaves it on.
        autonClaw.grabMs.reset();
    }
    state.clawClosed = closed;
    claw.set(closed, now);
    sapphirelib::telemetry::event("macro", "claw,closed=%d", closed ? 1 : 0);
}

bool clawClosed() { return state.clawClosed; }

void setAutoClaw(bool on) {
    std::lock_guard<pros::Mutex> lock(autonClawMutex());
    if (!on) {
        endAutonClaw("off");
        return;
    }
    if (autonClawTask == nullptr) {
        autonClawTask = std::make_unique<pros::Task>(
            [] {
                std::uint32_t wake = pros::millis();
                while (true) {
                    {
                        std::lock_guard<pros::Mutex> stepLock(autonClawMutex());
                        autonClawStep(sapphirelib::millis());
                    }
                    pros::Task::delay_until(&wake, kAutonClawPollMs);
                }
            },
            "autoClaw");
    }
    // Under this routine's mode, if it was already on under another's.
    autonClaw.startStatus = pros::competition::get_status();
    if (autonClaw.on) return;
    autonClaw.on = true;
    autonClaw.grabMs.reset();
    sapphirelib::auton::setSpeedLimit(kClawSpeedVolts);
    sapphirelib::telemetry::event("macro", "auto_claw,on=1");
}

bool autoClaw() {
    std::lock_guard<pros::Mutex> lock(autonClawMutex());
    return autonClaw.on;
}

void setExtended(bool out) {
    state.extended = out;
    extender.set(out, sapphirelib::millis());
    sapphirelib::telemetry::event("macro", "extender,out=%d", out ? 1 : 0);
}

bool extended() { return state.extended; }

void setLiftLevel(Goal goal, int level) {
    const auto table = static_cast<std::size_t>(goal);
    if (table != ladder.tableIndex()) {
        ladder.selectTable(table);
        // As RIGHT does: a new goal starts without the lift's memory.
        forgetNextLevel();
    }
    ladder.setLevelNumber(level);
    if (ladder.levelNumber() != level) {
        SAPPHIRELIB_LOG_WARN("macros", "setLiftLevel(): %s has no L%d, so L%d", ladder.tableName(),
                             level, ladder.levelNumber());
    }
    sendLift(ladder.levelPosition());
    // As "mech" logs them: the goal's index and its table's entry.
    sapphirelib::telemetry::event("macro", "lift_level,mode=%d,level=%d", static_cast<int>(table),
                                  ladder.level());
}

void setLiftDeg(double deg) {
    sendLift(deg);
    sapphirelib::telemetry::event("macro", "lift_deg,deg=%.0f", deg);
}

double liftScoreDeg() { return scoreDipDeg(); }

bool waitForLift(std::uint32_t timeoutMs) { return lift.waitUntilSettled(timeoutMs); }

std::uint32_t scoreCount() { return scoresReleased.load(); }

bool pinInClaw() { return pieceInClaw(clawSensor.get_distance()); }

bool liftAtBottom() { return switchPressed(); }

double driveMaxVolts() {
    const double ceiling = state.clawSpeed ? kClawSpeedVolts : kDriveFullVolts;
    const double deg = lift.position();
    // With every lift motor gone there's no telling, and nothing holding the
    // lift up either: full speed, rather than a whole match slow.
    if (std::isnan(deg)) return ceiling;
    const double raised = std::clamp(deg / kDriveTopLiftDeg, 0.0, 1.0);
    return std::min(ceiling, kDriveFullVolts - raised * (kDriveFullVolts - kDriveTopVolts));
}

sapphirelib::mechanism::PositionMechanism& liftMechanism() { return lift; }

MechanismCharacterizationConfig liftExperiment() {
    return MechanismCharacterizationConfig{
        .axis =
            {
                // Straight to the motors, at the characterization's own 10ms
                // rate: the lift's loop (its own task, at 20ms) is told to
                // keep its hands off for the run.
                .actuate =
                    [](double volts) {
                        lift.motors().move_voltage(static_cast<std::int32_t>(volts * 1000.0));
                    },
                .measure = [] { return lift.position(); },
                .stepVolts = kLiftTuneUpVolts,
                .rampVoltsPerS = 6.0,
                // The same both ways, so the down ramp's cap is the up
                // ramp's too.
                .rampMaxVolts = std::min(10.0, kLiftMaxDownVolts),
                // A segment still short of kLiftTuneLowerDeg/UpperDeg by
                // then ends anyway: raise this if they keep ending early.
                .maxSegmentMs = 4000,
                .minSpeed = 10.0, // motor degrees/s
                .start = [] { lift.beginExternalControl(); },
                .finish = [] { lift.endExternalControl(); },
            },
        .lowerLimit = kLiftTuneLowerDeg,
        .upperLimit = kLiftTuneUpperDeg,
        .downStepVolts = kLiftTuneDownVolts,
        // initialize() sets the brake mode to hold, so this keeps the lift
        // where it is between segments, which 0V wouldn't.
        .hold = [] { lift.motors().brake(); },
        .gravity = {.kind = GravityKind::constant},
    };
}

void liftTuningTest() {
    for (const double target : {kLiftTestHighDeg, kLiftTestLowDeg}) {
        liftTestTarget.store(target);
        // settled() only counts updates made for this target, so this can't
        // return on the old one's result.
        sapphirelib::waitUntil([] { return lift.settled(); }, kLiftTestMoveTimeoutMs, 20);
        sapphirelib::delayMs(kLiftTestDwellMs);
    }
    liftTestTarget.store(std::numeric_limits<double>::quiet_NaN());
}

} // namespace robot::macros
