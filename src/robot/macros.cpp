/**
 * \file macros.cpp
 *
 * Driver macros for the lift, the claw, and the extender, see macros.hpp for
 * the controls. Everything meant to be tuned is in the constants at the top;
 * the ports are in config.hpp.
 *
 * The mechanics (button edges, controller-screen throttling, piston state,
 * the lift's control law, and the timed score) come from SapphireLib's
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
#include <optional>
#include <vector>

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/motors.hpp"
#include "robot/config.hpp"
#include "robot/devices.hpp"
#include "sapphirelib/mechanism/piston.hpp"
#include "sapphirelib/mechanism/position_mechanism.hpp"
#include "sapphirelib/mechanism/preset_ladder.hpp"
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
using sapphirelib::mechanism::PositionLaw;
using sapphirelib::mechanism::PositionMechanism;
using sapphirelib::mechanism::PositionStep;
using sapphirelib::mechanism::PresetLadder;
using sapphirelib::tuning::GravityKind;
using sapphirelib::tuning::MechanismCharacterizationConfig;

constexpr double kPi = 3.14159265358979323846;

// --- Claw and autoClaw ---
// Anything the claw's distance sensor reads at or closer than this counts as
// a pin in the claw: between what the sensor reads with a pin loaded and
// without one.
constexpr std::int32_t kPieceDetectMm = 80;
// autoClaw closes the claw once the sensor has seen a pin for this long
// without a break, so one stray reading doesn't close it on nothing. 0
// closes it on the first reading.
constexpr std::uint32_t kAutoClawDetectMs = 0;
// Once the claw opens on a pin, autoClaw waits for the sensor to have seen
// nothing for this long before it can close again, so one reading that
// misses the pin still sitting in the open claw doesn't count as it gone. 0
// arms it again on the first reading that sees nothing: if the claw ever
// snaps shut on a pin it was just opened to drop, raise this (it was 200).
constexpr std::uint32_t kAutoClawClearMs = 0;
// Driving onto a pin, autoClaw closes early: between the reading that sees
// the pin and the claw actually shut, the robot carries the pin this many ms
// further in, so at speed a claw that waits for kPieceDetectMm closes behind
// the pin, or on it. The reach grows by how fast the robot is driving the
// claw's way (odometry's velocity, in mm per ms) times this: at 1 m/s (39
// in/s), 100ms reaches 100mm further. Standing still it's kPieceDetectMm.
// TODO: tune on the robot. A pin taken at speed that still ends up past the
// claw or pushed out of it wants more; one that closes before the pin gets
// there, less. The log's autoclaw events have the reading and reach of each
// grab.
constexpr std::uint32_t kAutoClawLeadMs = 100;
// The furthest out the lead reaches, however fast the robot goes, so driving
// at a wall or another robot doesn't close the claw half a field away.
constexpr std::int32_t kAutoClawMaxReachMm = 200;
// Which way the claw's mouth faces, in degrees clockwise from the robot's
// front (0 the front, 180 the back): only driving that way counts toward the
// lead. TODO: check on the robot. If it's wrong, the lead never kicks in
// driving onto a pin, and autoClaw closes at kPieceDetectMm as before.
constexpr double kClawFacingDeg = 0.0;

// --- Timing ---
// How long a score waits after opening the claw before lowering the lift:
// the claw has to open all the way and let go first, or the lift drags the
// pin back off the goal on its way down.
constexpr std::uint32_t kScoreReleaseMs = 400;
// L1/L2 held: one level on the press, another after the delay, then one every
// period for as long as it's held.
constexpr std::uint32_t kLevelRepeatDelayMs = 300;
constexpr std::uint32_t kLevelRepeatPeriodMs = 150;

// --- Lift ---
// Position loop on the lift motors' encoders, in volts per motor degree.
// kP = 0.2 is full power at 60 degrees of error; anything much higher is
// effectively on/off and oscillates at each level. Raising kP won't make a
// lift that stalls far below its target climb higher: that far out it's
// already getting the full 12V. TODO: these were tuned on the old lift's
// rotation sensor, whose degrees aren't these motor degrees: Auto-Tune the
// lift again (docs/NEW_CHASSIS.md, step 2).
constexpr sapphirelib::PIDGains kLiftGains{.kP = 0.2, .kI = 0.0, .kD = 0.01};
// The most the lift is ever driven down with: 75% of full power (12V), so
// it comes down no harder than this however far it has to go. Its loop, the
// seat, the seek and debug mode's L2 all stop here, and so do Auto-Tune's
// ramps and the Lift Testing auton. Going up is still full power. It caps
// volts, not speed: gravity helps the lift down, so it can still come down
// a little faster than 75% of what full power lifts it at.
constexpr double kLiftMaxDownVolts = 9.0;
// Constant upward volts added to the loop's output to carry the lift's
// weight. If the lift settles a little below each level, raise this until it
// doesn't.
constexpr double kLiftGravityVolts = 0.0;
// The bottom sits on the lift's hard stop, where the loop alone settles a few
// degrees short (the gravity volts push up, and what little kP gives that
// close can't beat friction). So heading for the bottom, the lift drives down
// with at least this many volts until its encoders read within kLiftRestDeg
// of zero, then cuts power and rests on the stop rather than pushing into it.
constexpr double kLiftSeatVolts = 0.0;
constexpr double kLiftRestDeg = 2.0;
// If the bottom switch isn't pressed once the lift rests, the encoders have
// drifted (a skipped tooth, or a restart with the lift up) and the real
// bottom is further down: the lift drives on down at kLiftSeekVolts until the
// switch is pressed, which zeroes the encoders there. With the switch
// unplugged or broken, it gives up after kLiftSeekTimeoutMs and rests where
// it is, rather than pushing into its hard stop all match. TODO: tune on the
// robot. Enough to get the lift down onto the switch, not so much that it
// slams.
constexpr double kLiftSeekVolts = 3.0;
constexpr std::uint32_t kLiftSeekTimeoutMs = 1000;
// How close the lift must get before a score's dip moves on to opening the
// claw, and how long the dip waits for that before moving on anyway.
constexpr double kLiftToleranceDeg = 15.0;
constexpr std::uint32_t kLiftDescendTimeoutMs = 1000;
// Debug mode: what L1 and L2 put on the lift motors while held. L2's is
// capped at kLiftMaxDownVolts.
constexpr double kLiftDebugVolts = 12.0;

// --- Lift Auto-Tune and Run Test (the PID page's Lift entry) ---
// Auto-Tune drives the lift up and down between these two heights, stopping
// each segment once it passes one, so keep both well inside the lift's real
// travel (it takes a few degrees to brake), and start with the lift down.
// TODO: these assume the placeholder level heights below; set them from the
// real ones once those are measured.
constexpr double kLiftTuneLowerDeg = 60.0;
constexpr double kLiftTuneUpperDeg = 720.0;
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

// Goals, one table per goal height: lift target at each level, in motor
// degrees up from the bottom switch. Entry 0 is the bottom, where the lift
// rests on its switch and autoClaw picks up pins, the same in every table;
// entries 1 and up are that goal's scoring levels, one per L1 notch, so the
// entry count sets how many levels the goal has. Spacing needn't be even: a
// score dips halfway to the level below, however big that gap is. Names are
// shown on the controller; 8 characters max.
//
// TODO: placeholder heights, from the old lift's rotation sensor. Measure
// each goal's real heights on the motor encoders and put them here: the
// controller's third line shows the lift's live reading (then its current
// target), so run the lift to each height in debug mode and copy the number.
PresetLadder ladder({
    {"ALLIANCE", {0, 150, 300, 450, 600, 750, 900}},
    {"MEDIUM", {0, 180, 360, 540, 720, 900}},
    {"CENTER", {0, 225, 450, 675, 900}},
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

// Driven from update(), once per opcontrol tick, with that tick's `now`,
// not on a task of its own, so nothing moves the lift while opcontrol isn't
// running.
PositionMechanism lift(std::vector<std::int8_t>(ports::kLiftMotors.begin(),
                                                ports::kLiftMotors.end()),
                       [] { return liftPositionDeg(); },
                       {
                           .pid = {.gains = kLiftGains,
                                   .outputLimit = 12.7,
                                   // Level changes step the target, which would
                                   // otherwise kick the output.
                                   .derivativeOnMeasurement = true,
                                   .nominalDtS = 0.02}, // update() runs every 20ms opcontrol tick
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
    // Whether autoClaw may close the claw: false from the moment the claw
    // opens on a pin until the sensor has seen it gone for kAutoClawClearMs,
    // so opening the claw to drop a pin doesn't snap it shut on the same one.
    bool autoClawArmed = false;
    // The bottom switch has zeroed the lift's encoders for this press (see
    // zeroAtBottom()).
    bool zeroedThisPress = false;
    // Heading for the bottom with the switch not yet pressed (see
    // seekingBottom()): when the lift started driving down to find it, and
    // whether it gave up.
    std::optional<std::uint32_t> seekStartMs;
    bool seekGaveUp = false;
    bool debug = false; // the debug mode update() is running, see debugMode
    int readout = 0;    // see kReadoutCount
};

State state;

// Debug mode as asked for by setDebug(), from any task. update() copies it
// into state.debug once a tick, so a whole tick runs in one mode.
std::atomic<bool> debugMode{false};

// Whether the claw's distance sensor has seen a pin within autoClaw's reach
// (see autoClawReachMm()), and since when, for kAutoClawDetectMs and
// kAutoClawClearMs.
TimedFlag pinSeen;

// The score: dip halfway to the level below, open the claw and give it
// kScoreReleaseMs to let go, then down to the bottom. What each step *does*
// stays level-triggered below (claw, lift target); the sequence only says
// which step it is and when to move on.
enum class Phase { scoreDescend, scoreRelease };

bool liftArrived();

const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{
    {.id = Phase::scoreDescend, .until = liftArrived, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::scoreRelease, .timeoutMs = kScoreReleaseMs},
}};
const Sequence<Phase>::Program kScore{.steps = kScoreSteps,
                                      .onFinish = [] { ladder.setLevel(kBottomLevel); }};

Sequence<Phase> sequence;

// Set by liftTuningTest() from the PID page's task, read by update(): while
// it's a number, the lift heads there instead of to its level.
std::atomic<double> liftTestTarget{std::numeric_limits<double>::quiet_NaN()};

// The "mech" telemetry channel, once attachTelemetry() has made it.
sapphirelib::telemetry::Channel* mechLog = nullptr;

bool pieceInClaw(std::int32_t distanceMm, std::int32_t reachMm = kPieceDetectMm) {
    // get_distance() reads 9999 when it sees nothing and PROS_ERR (INT32_MAX)
    // when unplugged, so neither counts as a pin.
    return distanceMm <= reachMm;
}

/// How fast the robot is driving the claw's mouth onto whatever is in front
/// of it, in mm per ms (the same number as m/s): odometry's field velocity
/// along the way the claw faces. 0 backing away from it or moving across it.
double clawApproachMmPerMs() {
    const sapphirelib::odom::Odometry::Snapshot snapshot = odometry().snapshot();
    // Heading 0 faces +y and 90 faces +x (see Pose). The raw pose, since the
    // velocity is the raw pose's.
    const double facingRad = (snapshot.rawPose.headingDeg + kClawFacingDeg) * (kPi / 180.0);
    const double inPerS = snapshot.rawVelocity.xInPerS * std::sin(facingRad) +
                          snapshot.rawVelocity.yInPerS * std::cos(facingRad);
    return std::max(0.0, inPerS * 25.4 / 1000.0);
}

/// Where autoClaw closes on a pin at this approach speed: kPieceDetectMm,
/// plus kAutoClawLeadMs of travel, up to kAutoClawMaxReachMm.
std::int32_t autoClawReachMm(double approachMmPerMs) {
    const double reachMm = kPieceDetectMm + approachMmPerMs * kAutoClawLeadMs;
    return static_cast<std::int32_t>(std::min(reachMm, static_cast<double>(kAutoClawMaxReachMm)));
}

bool switchPressed() { return bottomSwitch.get_value() == 1; }

double liftTargetDeg() {
    const double testTarget = liftTestTarget.load();
    if (std::isfinite(testTarget)) return testTarget;
    if (sequence.running(kScore)) return ladder.midpointBelow(ladder.level());
    return ladder.levelPosition();
}

/// Never true with every lift motor unplugged (isNear() is false with no
/// reading), so a score waiting on this falls through to its timeout.
bool liftArrived() { return lift.isNear(liftTargetDeg()); }

/// Zeroes the lift's encoders on the first tick of each press of the bottom
/// switch, so every level is measured from the real bottom however far the
/// encoders had drifted. Not while Auto-Tune has the motors: a jump in the
/// middle of its run would spoil the fit, so with the switch still pressed
/// it zeroes once the run is over instead.
void zeroAtBottom(bool atBottom) {
    if (!atBottom) {
        state.zeroedThisPress = false;
        return;
    }
    if (state.zeroedThisPress || lift.externalControl()) return;
    lift.motors().tare_position_all();
    // The reading just jumped: start the loop fresh, or its derivative kicks.
    lift.resetController();
    state.zeroedThisPress = true;
}

/// Whether the lift should drive straight down onto its bottom switch this
/// tick: it's heading for the bottom and its loop has brought it to rest
/// there (the encoders read 0), but the switch isn't pressed, so the real
/// bottom is lower. Keeps going until the switch is pressed or
/// kLiftSeekTimeoutMs passes, then gives up until the lift next leaves the
/// bottom.
bool seekingBottom(std::uint32_t now, bool atBottom) {
    const bool toBottom = liftTargetDeg() <= ladder.levelPosition(kBottomLevel);
    if (!toBottom || atBottom) {
        state.seekStartMs.reset();
        state.seekGaveUp = false;
        return false;
    }
    if (state.seekGaveUp) return false;
    if (!state.seekStartMs) {
        // Still on its way down, on the loop: the seek only takes over once
        // the loop thinks it's there. (Never with no encoders: that's law
        // "no sensor", braked.)
        if (lift.law() != PositionLaw::rest) return false;
        state.seekStartMs = now;
    }
    if (sapphirelib::elapsedMs(*state.seekStartMs, now) < kLiftSeekTimeoutMs) return true;
    state.seekGaveUp = true;
    SAPPHIRELIB_LOG_WARN("macros", "lift didn't reach its bottom switch (port %c) in %ums",
                         ports::kLiftBottomSwitch, static_cast<unsigned>(kLiftSeekTimeoutMs));
    sapphirelib::telemetry::event("macro", "seek_failed");
    return false;
}

/// autoClaw (see macros.hpp): the claw is open, the lift is down on its
/// bottom switch and meant to be there, and the sensor has seen a pin within
/// the reach for kAutoClawDetectMs.
bool autoClawWants(std::uint32_t now, bool atBottom) {
    return !state.clawClosed && state.autoClawArmed && atBottom &&
           ladder.level() == kBottomLevel && pinSeen.trueFor(kAutoClawDetectMs, now);
}

void showStatus(Controller& controller, bool atBottom) {
    ControllerScreen& screen = controller.screen();
    if (state.debug) {
        // No level to show: the lift is L1/L2's. Each piston, to check it
        // moves the way the line says.
        screen.setLine(0, "DBG %s EX:%s", state.clawClosed ? "SHUT" : "OPEN",
                       state.extended ? "OUT" : "IN");
    } else if (ladder.level() == kBottomLevel) {
        screen.setLine(0, "%-8s BOTTOM", ladder.tableName());
    } else {
        screen.setLine(0, "%-8s L%d/%d", ladder.tableName(), ladder.level(), ladder.maxLevel());
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
    // Wherever the lift is now is the bottom until the switch says otherwise,
    // so start the program with the lift down. Already on the switch, this
    // counts as its press.
    lift.motors().tare_position_all();
    state.zeroedThisPress = switchPressed();
    // Only matters when the lift falls back to brake(), with no encoders, or
    // after stop().
    lift.motors().set_brake_mode_all(pros::v5::MotorBrake::hold);
}

void update(Controller& controller) {
    const std::uint32_t now = controller.now();
    // After autonomous, a disable, or a blocking routine run from opcontrol:
    // drop a half-finished score (its timing is stale) and start the lift
    // loop fresh. The controller already forgot what its screen showed, so
    // the text gets resent.
    if (controller.resumed()) {
        sequence.cancel();
        lift.resetController();
        state.seekStartMs.reset();
        state.seekGaveUp = false;
    }

    // One reading of each sensor for the whole tick. The pin counts as seen
    // out at the reach, so autoClaw (and its arming) works from where the
    // claw has to start closing at this speed, not from where it ends up.
    const std::int32_t pieceMm = clawSensor.get_distance();
    const double approachMmPerMs = clawApproachMmPerMs();
    const std::int32_t reachMm = autoClawReachMm(approachMmPerMs);
    const bool pin = pieceInClaw(pieceMm, reachMm);
    const bool atBottom = switchPressed();
    pinSeen.set(pin, now);
    zeroAtBottom(atBottom);

    // controller.update() sampled every button once for this tick, and edges
    // come from comparing samples, so reading (or not reading) a button
    // changes nothing.
    if (controller.pressed(Button::left)) state.readout = (state.readout + 1) % kReadoutCount;

    if (controller.pressed(Button::right)) {
        // The lift keeps its level on the new goal (moving to that goal's
        // height for it), capped if the new goal has fewer levels.
        ladder.nextTable();
        // TODO: LED library's mode-change call goes here.
    }

    // The extender is Y's alone: in every mode, and through a score.
    if (controller.pressed(Button::y)) state.extended = !state.extended;

    // Debug mode switched on or off (from the Home page): each mode starts
    // from a clean slate, no score carried across.
    const bool debugging = debugMode.load();
    if (debugging != state.debug) {
        state.debug = debugging;
        sequence.cancel();
        // Back on the levels, with the lift loop starting fresh.
        if (!debugging) lift.resetController();
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
        // R1, L1 and L2 all wait for it to finish. It opens the claw itself
        // (below), and its onFinish sends the lift to the bottom.
        sequence.update(now);
    } else if (controller.pressed(Button::r2) && state.clawClosed) {
        sequence.start(kScore, now);
    } else {
        if (controller.pressed(Button::r1)) {
            state.clawClosed = !state.clawClosed;
        } else if (autoClawWants(now, atBottom)) {
            state.clawClosed = true;
            sapphirelib::telemetry::event("macro", "autoclaw,piece_mm=%d,reach_mm=%d,speed=%.2f",
                                          static_cast<int>(pieceMm), static_cast<int>(reachMm),
                                          approachMmPerMs);
        }
        if (controller.repeated(Button::l1, kLevelRepeatDelayMs, kLevelRepeatPeriodMs)) {
            ladder.up();
        }
        if (controller.repeated(Button::l2, kLevelRepeatDelayMs, kLevelRepeatPeriodMs)) {
            ladder.down();
        }
    }

    // A score's release opens the claw, and it stays open after: autoClaw, or
    // R1, closes it on the next pin.
    if (sequence.is(Phase::scoreRelease)) state.clawClosed = false;

    // Opening disarms autoClaw; the sensor seeing nothing for a while arms it
    // again. In that order, so a claw opened on nothing is armed again at
    // once.
    if (wasClosed && !state.clawClosed) state.autoClawArmed = false;
    if (pinSeen.falseFor(kAutoClawClearMs, now)) state.autoClawArmed = true;

    claw.set(state.clawClosed, now);
    extender.set(state.extended, now);

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
    } else if (seekingBottom(now, atBottom)) {
        lift.setVolts(-kLiftSeekVolts);
    } else {
        lift.setTarget(liftTargetDeg());
    }
    lift.update(now);
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
    // stop() alone only switches the lift to "brake every update"; the
    // update() here applies that right away, since nothing else will call
    // one until the blocking routine (or the disable) is over. The next
    // update() from opcontrol sets a target again, which puts the lift back
    // on its loop. The pistons hold where they are on their own.
    lift.stop();
    lift.update(sapphirelib::millis());
}

void attachTelemetry(sapphirelib::telemetry::Logger& logger) {
    // Three decimals: the lift works in whole degrees and volts, and at
    // kD = 0.01 its derivative term is already in hundredths of a volt.
    // Capacity 128 is 2.5s at the 20ms opcontrol tick.
    logger.pid("lift", lift.pid(), {.capacity = 128, .decimals = 3});
    // What the motors were actually sent, after gravity volts and the clamp,
    // the "lift" channel's "out" is only the loop's share. The listener runs
    // inside lift.update(), on the opcontrol task, and reuses that update's
    // one position read; recording never blocks.
    sapphirelib::telemetry::Channel& act = logger.channel(
        "lift.act", {"target", "pos", "volts", "law"}, {.capacity = 128, .decimals = 3});
    lift.setStepListener([channel = &act](const PositionStep& step) {
        channel->record({step.target, step.position, step.volts, static_cast<double>(step.law)});
    });
    // The claw, extender and lift state machine, every opcontrol tick: the
    // claw piston (1 closed) and the extender (1 out), the claw's distance
    // reading (mm), the lift's level (0 the bottom) and goal (0 ALLIANCE,
    // 1 MEDIUM, 2 CENTER), the running score step (-1 none; else Phase's
    // order: scoreDescend, scoreRelease), and the bottom switch (1 pressed).
    mechLog = &logger.channel(
        "mech", {"claw", "extender", "piece_mm", "level", "mode", "phase", "bottom"},
        {.capacity = 128, .decimals = 2});
}

void setDebug(bool on) { debugMode.store(on); }

bool debug() { return debugMode.load(); }

bool pinInClaw() { return pieceInClaw(clawSensor.get_distance()); }

bool liftAtBottom() { return switchPressed(); }

sapphirelib::mechanism::PositionMechanism& liftMechanism() { return lift; }

MechanismCharacterizationConfig liftExperiment() {
    return MechanismCharacterizationConfig{
        .axis =
            {
                // Straight to the motors, at the characterization's own 10ms
                // rate: the lift's loop (driven from opcontrol at 20ms) is
                // told to keep its hands off for the run.
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
                .maxSegmentMs = 2000,
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
