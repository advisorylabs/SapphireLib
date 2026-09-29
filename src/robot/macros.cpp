/**
 * \file macros.cpp
 *
 * Driver macros for the intake, claw, and lift — see macros.hpp for the
 * controls. Everything meant to be tuned is in the constants at the top; the
 * ports are in config.hpp.
 *
 * The mechanics (button edges, controller-screen throttling, piston timing,
 * the lift's control law, and the timed sequences) come from SapphireLib's
 * input/, mechanism/, and util/ primitives. What's left here is this robot's
 * numbers and rules.
 *
 * Team 96671H — Hitmen
 */

#include "robot/macros.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "robot/config.hpp"
#include "sapphirelib/mechanism/piston.hpp"
#include "sapphirelib/mechanism/position_mechanism.hpp"
#include "sapphirelib/mechanism/preset_ladder.hpp"
#include "sapphirelib/telemetry/logger.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/sequence.hpp"
#include "sapphirelib/util/wait.hpp"

namespace robot::macros {

namespace {

using sapphirelib::Sequence;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
using sapphirelib::input::ControllerScreen;
using sapphirelib::mechanism::Piston;
using sapphirelib::mechanism::PositionMechanism;
using sapphirelib::mechanism::PositionStep;
using sapphirelib::mechanism::PresetLadder;
using sapphirelib::tuning::GravityKind;
using sapphirelib::tuning::MechanismCharacterizationConfig;

// --- Speeds, in millivolts (12000 = full) ---
constexpr std::int32_t kIntakeMv = 12700;
constexpr std::int32_t kClawMv = 12700; // intaking and outtaking
// Claw spin while its distance sensor sees a piece and nothing else is
// commanding it. It's stalled against the piece most of that time, so if the
// claw motor runs hot over a match, turn this down first.
constexpr std::int32_t kClawHoldMv = 12700;

// Anything the claw's distance sensor reads at or closer than this counts as
// a piece in the claw. TODO: read the sensor with and without a piece loaded
// and pick a value in between.
constexpr std::int32_t kPieceDetectMm = 50;

// --- Timing ---
// How long the claw piston gets to deploy before anything outtakes past it.
constexpr std::uint32_t kPistonDeployMs = 400;
// How long the claw outtakes when scoring above level 0.
constexpr std::uint32_t kHeightOuttakeMs = 500;
// The re-seat after releasing Y, once the lift is back down at level 0:
// piston retracted for this long, then redeployed for this long before the
// claw goes back to holding on its own.
constexpr std::uint32_t kReseatRetractMs = 250;
constexpr std::uint32_t kReseatDeployMs = 250;

// --- Lift ---
// Position loop on the lift's rotation sensor, in volts per degree of sensor
// rotation. kP = 0.1 is full power at 120 degrees of error; anything much
// higher is effectively on/off and oscillates at each level. Raising kP won't
// make a lift that stalls far below its target climb higher: 120 degrees out
// it's already getting the full 12V. TODO: tune on the robot.
constexpr sapphirelib::PIDGains kLiftGains{.kP = 0.2, .kI = 0.0, .kD = 0.01};
// Constant upward volts added to the loop's output to carry the lift's
// weight. If the lift settles a little below each level, raise this until it
// doesn't.
constexpr double kLiftGravityVolts = 0.0;
// Level 0 sits on the lift's hard stop, where the loop alone settles a few
// degrees short (the gravity volts push up, and what little kP gives that
// close can't beat friction). So heading for level 0, the lift drives down
// with at least this many volts, until it's within kLiftRestDeg of the
// bottom, then cuts power and rests on the stop rather than pushing into it.
constexpr double kLiftSeatVolts = 0.0;
constexpr double kLiftRestDeg = 2.0;
// How close the lift must get before a sequence waiting on it moves on (a
// score above level 0 waiting to outtake, or a re-seat waiting for level 0),
// and how long it waits for that before moving on anyway.
constexpr double kLiftToleranceDeg = 15.0;
constexpr std::uint32_t kLiftDescendTimeoutMs = 1000;

// --- Lift Auto-Tune and Run Test (the PID page's Lift entry) ---
// Auto-Tune drives the lift up and down between these two heights, stopping
// each segment once it passes one — so keep both well inside the lift's real
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

// Scoring modes: lift target at each level, in degrees of the lift's rotation
// sensor up from where it was zeroed. Entry 0 is level 0 (claw piston
// deployed, lift down); each entry after it is one L1 notch, so the entry
// count sets how many levels the mode has. Spacing needn't be even: a score
// above level 0 dips halfway to the level below, however big that gap is.
// Names are shown on the controller; 8 characters max.
//
// TODO: placeholder heights. Measure each goal's real heights on the
// rotation sensor and put them here: the controller's third line shows the
// lift's live reading (then its current target), so run the lift to each
// height manually and copy the number.
PresetLadder ladder({
    {"ALLIANCE", {0, 150, 300, 450, 600, 750, 900}},
    {"MEDIUM", {0, 180, 360, 540, 720, 900}},
    {"CENTER", {0, 225, 450, 675, 900}},
});

pros::Motor intake(ports::kIntake);
pros::Motor claw(ports::kClawMotor);
pros::Rotation liftSensor(ports::kLiftSensor);
pros::Distance clawSensor(ports::kClawDistance);
Piston clawPiston(ports::kClawPiston, /*extendedAtStart=*/false);

// Driven from update(), once per opcontrol tick, with that tick's `now` —
// not on a task of its own — so nothing moves the lift while opcontrol isn't
// running, exactly as before these macros moved onto the library.
PositionMechanism lift({ports::kLiftA, ports::kLiftB},
                       [] { return sapphirelib::mechanism::readRotationDeg(liftSensor); },
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
                           .tolerance = kLiftToleranceDeg,
                       });

// Timed sequences, during which L1, L2, and R2 are ignored. A score above
// level 0 dips halfway to the level below, outtakes, then goes up a level.
// A re-seat, after releasing Y, lowers the lift back to level 0, then
// retracts and redeploys the claw piston. What each step *does* stays
// level-triggered below (claw volts, piston, lift target); the sequence only
// says which step it is and when to move on.
enum class Phase { scoreDescend, scoreOuttake, reseatLower, reseatRetract, reseatDeploy };

bool liftArrived();

const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{
    {.id = Phase::scoreDescend, .until = liftArrived, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::scoreOuttake, .timeoutMs = kHeightOuttakeMs},
}};
const Sequence<Phase>::Program kScore{.steps = kScoreSteps, .onFinish = [] { ladder.up(); }};

const std::array<Sequence<Phase>::Step, 3> kReseatSteps{{
    {.id = Phase::reseatLower, .until = liftArrived, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::reseatRetract, .timeoutMs = kReseatRetractMs},
    {.id = Phase::reseatDeploy, .timeoutMs = kReseatDeployMs},
}};
const Sequence<Phase>::Program kReseat{.steps = kReseatSteps};

Sequence<Phase> sequence;

// Set by liftTuningTest() from the PID page's task, read by update(): while
// it's a number, the lift heads there instead of to its level.
std::atomic<double> liftTestTarget{std::numeric_limits<double>::quiet_NaN()};

// The "mech" telemetry channel, once attachTelemetry() has made it.
sapphirelib::telemetry::Channel* mechLog = nullptr;

struct State {
    // false = stowed: piston retracted, lift at level 0. Only R1 stows.
    bool clawDeployed = false;
    bool bottomOuttake = false;  // R2 pressed at level 0 and still held
    bool deployedIntake = false; // Y held (and not overridden by R1)
    bool showLiftMotors = false; // controller lines 2-3 show lift motor readings
};

State state;

bool pieceInClaw() {
    // get_distance() reads 9999 when it sees nothing and PROS_ERR (INT32_MAX)
    // when unplugged, so neither counts as a piece.
    return clawSensor.get_distance() <= kPieceDetectMm;
}

double liftTargetDeg() {
    const double testTarget = liftTestTarget.load();
    if (std::isfinite(testTarget)) return testTarget;
    if (state.deployedIntake) return ladder.midpointBelow(1);
    if (sequence.running(kScore)) return ladder.midpointBelow(ladder.level());
    return ladder.levelPosition();
}

/// Never true while the rotation sensor is unplugged (isNear() is false with
/// no reading), so a sequence waiting on this falls through to its timeout.
bool liftArrived() { return lift.isNear(liftTargetDeg()); }

void showStatus(Controller& controller) {
    ControllerScreen& screen = controller.screen();
    if (state.clawDeployed) {
        screen.setLine(0, "%-8s L%d/%d", ladder.tableName(), ladder.level(), ladder.maxLevel());
    } else {
        screen.setLine(0, "%-8s STOW", ladder.tableName());
    }
    if (state.showLiftMotors) {
        // One line per lift motor (port A, then B): the volts it's actually
        // applying, its current draw, and its temperature. A motor stalled
        // at 12V should be drawing its full 2.5A; much less than that, or
        // 55C and up (where it starts cutting its own current), means it's
        // being held back rather than out-muscled.
        for (std::uint8_t i = 0; i < 2; ++i) {
            screen.setLine(1 + i, "%3.0fV %3.1fA %2.0fC", lift.motors().get_voltage(i) / 1000.0,
                           lift.motors().get_current_draw(i) / 1000.0,
                           lift.motors().get_temperature(i));
        }
    } else {
        // The piston itself, not state.clawDeployed, so this also shows the
        // brief retract during a re-seat.
        screen.setLine(1, "CLAW %s", clawPiston.extended() ? "DEPLOYED" : "RETRACTED");
        // Lift sensor reading, then target, in whole degrees — for measuring
        // level heights and seeing whether the lift is short of its target.
        const double liftDeg = lift.position();
        if (std::isnan(liftDeg)) {
            screen.setLine(2, "LIFT NO SENSOR");
        } else {
            screen.setLine(2, "LIFT %5.0f/%4.0f", liftDeg, liftTargetDeg());
        }
    }
    // At most one line per interval (see ControllerConfig::screenIntervalMs
    // in driver.cpp), so a change that arrives too soon after the last print
    // (or behind another line's change) waits for a later tick.
    controller.flushScreen();
}

} // namespace

void initialize() {
    liftSensor.reset_position();
    // Only matters when the lift falls back to brake() — with no sensor, or
    // after stop().
    lift.motors().set_brake_mode_all(pros::v5::MotorBrake::hold);
}

void update(Controller& controller) {
    const std::uint32_t now = controller.now();
    // After autonomous, a disable, or a blocking routine run from opcontrol:
    // drop any half-finished sequence (its timing is stale) and start the
    // lift loop fresh. The controller already forgot what its screen showed,
    // so the text gets resent.
    if (controller.resumed()) {
        sequence.cancel();
        lift.resetController();
    }

    // controller.update() sampled every button once for this tick, and edges
    // come from comparing samples — so reading (or not reading) a button
    // changes nothing. No more reading every new-press every tick so that a
    // press made while one went unread doesn't fire late.
    const bool intaking = controller.held(Button::r1);
    const bool deployedIntaking = controller.held(Button::y);
    const bool scoreHeld = controller.held(Button::r2);
    const bool scorePressed = controller.pressed(Button::r2);

    if (controller.pressed(Button::left)) state.showLiftMotors = !state.showLiftMotors;

    if (controller.pressed(Button::right)) {
        // The lift keeps its level in the new mode (moving to that mode's
        // height for it), capped if the new mode has fewer levels.
        ladder.nextTable();
        // TODO: LED library's mode-change call goes here.
    }

    if (intaking) {
        // Resets to stowed from any state, cancelling whatever was in progress.
        sequence.cancel();
        state.bottomOuttake = false;
        state.deployedIntake = false;
        state.clawDeployed = false;
        ladder.setLevel(0);
    } else if (deployedIntaking) {
        // The same reset, but with the claw deployed, and the lift up halfway
        // to level 1 (see liftTargetDeg()) until Y is released.
        sequence.cancel();
        state.bottomOuttake = false;
        state.deployedIntake = true;
        state.clawDeployed = true;
        ladder.setLevel(0);
    } else if (state.deployedIntake) {
        // Y was just released: lower to level 0 and re-seat the piece before
        // the claw goes back to holding it on its own.
        state.deployedIntake = false;
        sequence.start(kReseat, now);
    } else if (sequence.active()) {
        sequence.update(now);
    } else {
        if (controller.pressed(Button::l1)) {
            if (!state.clawDeployed) {
                state.clawDeployed = true;
            } else {
                ladder.up();
            }
        }
        // Never retracts the piston; only intaking does.
        if (controller.pressed(Button::l2)) ladder.down();
        if (scorePressed) {
            if (ladder.level() == 0) {
                state.clawDeployed = true;
                state.bottomOuttake = true;
            } else {
                sequence.start(kScore, now);
            }
        }
    }
    if (!scoreHeld) state.bottomOuttake = false;

    const bool reseating = sequence.running(kReseat);
    clawPiston.set(state.clawDeployed && !sequence.is(Phase::reseatRetract), now);

    // Waits out the rest of the piston's deploy time, which is none at all if
    // it deployed long ago.
    const bool bottomOuttaking = state.bottomOuttake && clawPiston.extendedFor(kPistonDeployMs, now);

    // Claw: outtaking beats intaking (R1, Y, or re-seating), which beats
    // holding a detected piece.
    std::int32_t clawMv = 0;
    if (bottomOuttaking || sequence.is(Phase::scoreOuttake)) {
        clawMv = -kClawMv;
    } else if (intaking || state.deployedIntake || reseating) {
        clawMv = kClawMv;
    } else if (pieceInClaw()) {
        clawMv = kClawHoldMv;
    }
    claw.move_voltage(clawMv);

    // Intake: runs in only for R1 (Y intakes with the claw alone), and
    // reverses with the claw only for a level-0 score. Above level 0 the
    // piece leaves from up on the lift, so the intake stays off.
    std::int32_t intakeMv = 0;
    if (intaking) {
        intakeMv = kIntakeMv;
    } else if (bottomOuttaking) {
        intakeMv = -kIntakeMv;
    }
    intake.move_voltage(intakeMv);

    lift.setTarget(liftTargetDeg());
    lift.update(now);
    showStatus(controller);

    if (mechLog != nullptr) {
        // What every mechanism was told this tick, for the analyzer's replay
        // (the lift's own rows are in "lift"/"lift.act"). get_distance()
        // reads PROS_ERR unplugged, which is logged as a gap.
        const std::int32_t pieceMm = clawSensor.get_distance();
        const std::optional<Phase> phase = sequence.current();
        mechLog->record({intakeMv / 1000.0, clawMv / 1000.0, clawPiston.extended() ? 1.0 : 0.0,
                         pieceMm == PROS_ERR ? std::numeric_limits<double>::quiet_NaN()
                                             : static_cast<double>(pieceMm),
                         static_cast<double>(ladder.level()), static_cast<double>(ladder.tableIndex()),
                         phase ? static_cast<double>(*phase) : -1.0,
                         state.clawDeployed ? 1.0 : 0.0});
    }
}

void stop() {
    claw.move_voltage(0);
    intake.move_voltage(0);
    // stop() alone only switches the lift to "brake every update"; the
    // update() here applies that right away, since nothing else will call
    // one until the blocking routine (or the disable) is over. The next
    // update() from opcontrol sets a target again, which puts the lift back
    // on its loop.
    lift.stop();
    lift.update(sapphirelib::millis());
}

void attachTelemetry(sapphirelib::telemetry::Logger& logger) {
    // Three decimals: the lift works in whole degrees and volts, and at
    // kD = 0.01 its derivative term is already in hundredths of a volt.
    // Capacity 128 is 2.5s at the 20ms opcontrol tick.
    logger.pid("lift", lift.pid(), {.capacity = 128, .decimals = 3});
    // What the motors were actually sent, after gravity volts and the clamp —
    // the "lift" channel's "out" is only the loop's share. The listener runs
    // inside lift.update(), on the opcontrol task, and reuses that update's
    // one sensor read; recording never blocks.
    sapphirelib::telemetry::Channel& act = logger.channel(
        "lift.act", {"target", "pos", "volts", "law"}, {.capacity = 128, .decimals = 3});
    lift.setStepListener([channel = &act](const PositionStep& step) {
        channel->record({step.target, step.position, step.volts, static_cast<double>(step.law)});
    });
    // The intake, claw and lift state machine, every opcontrol tick: volts
    // sent to the intake and claw, the piston, the claw's distance reading
    // (mm), the lift's level and scoring mode (0 ALLIANCE, 1 MEDIUM, 2
    // CENTER), the running sequence step (-1 none; else Phase's order:
    // scoreDescend, scoreOuttake, reseatLower, reseatRetract, reseatDeploy),
    // and whether the claw is deployed.
    mechLog = &logger.channel(
        "mech", {"intake_v", "claw_v", "piston", "piece_mm", "level", "mode", "phase", "deployed"},
        {.capacity = 128, .decimals = 2});
}

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
                .rampMaxVolts = 10.0,
                .maxSegmentMs = 2000,
                .minSpeed = 10.0, // deg/s of the rotation sensor
                .start = [] { lift.beginExternalControl(); },
                .finish = [] { lift.endExternalControl(); },
            },
        .lowerLimit = kLiftTuneLowerDeg,
        .upperLimit = kLiftTuneUpperDeg,
        .downStepVolts = kLiftTuneDownVolts,
        // initialize() sets the brake mode to hold, so this keeps the lift
        // where it is between segments — which 0V wouldn't.
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
