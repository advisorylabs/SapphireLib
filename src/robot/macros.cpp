/**
 * \file macros.cpp
 *
 * Driver macros for the intake, roller claw, and lift, see macros.hpp for the
 * controls. Everything meant to be tuned is in the constants at the top; the
 * ports are in config.hpp.
 *
 * The mechanics (button edges, controller-screen throttling, piston timing,
 * the lift's control law, and the timed sequences) come from SapphireLib's
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
constexpr std::int32_t kClawMv = 12700; // the roller claw, intaking and outtaking
// Roller claw spin while its distance sensor sees a pin and nothing else is
// commanding it. It's stalled against the pin most of that time, so if the
// claw motor runs hot over a match, turn this down first.
constexpr std::int32_t kClawHoldMv = 12700;

// Anything the claw's distance sensor reads at or closer than this counts as
// a pin in the claw. TODO: read the sensor with and without a pin loaded and
// pick a value in between.
constexpr std::int32_t kPieceDetectMm = 50;

// --- Timing ---
// How long the claw piston gets to finish moving before anything runs past
// it: a score's outtake waits for it to have deployed this long, and R1's
// intake for it to have retracted this long.
constexpr std::uint32_t kPistonMoveMs = 400;
// How long the roller claw outtakes when scoring.
constexpr std::uint32_t kScoreOuttakeMs = 500;
// L1/L2 held: one level on the press, another after the delay, then one every
// period for as long as it's held.
constexpr std::uint32_t kLevelRepeatDelayMs = 300;
constexpr std::uint32_t kLevelRepeatPeriodMs = 150;
// After releasing Y, how long the lift sits at the bottom before coming back
// up to level 1, so it really gets there rather than turning around inside
// kLiftToleranceDeg of it.
constexpr std::uint32_t kDipBottomMs = 100;

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
// The bottom sits on the lift's hard stop, where the loop alone settles a few
// degrees short (the gravity volts push up, and what little kP gives that
// close can't beat friction). So heading for the bottom, the lift drives down
// with at least this many volts, until it's within kLiftRestDeg of it, then
// cuts power and rests on the stop rather than pushing into it.
constexpr double kLiftSeatVolts = 0.0;
constexpr double kLiftRestDeg = 2.0;
// How close the lift must get before a sequence waiting on it moves on (a
// score waiting to outtake, the dip after Y waiting for the bottom, or R1
// waiting to intake), and how long each waits for that before moving on
// anyway. Stowing can start from the top level, so it gets longer.
constexpr double kLiftToleranceDeg = 15.0;
constexpr std::uint32_t kLiftDescendTimeoutMs = 1000;
constexpr std::uint32_t kStowTimeoutMs = 2000;
// Debug mode: what L1 and L2 put on the lift motors while held.
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

// Goals, one table per goal height: lift target at each level, in degrees of
// the lift's rotation sensor up from where it was zeroed. Entry 0 is the
// bottom, where the lift stows and intakes from, the same in every table;
// entries 1 and up are that goal's scoring levels, one per L1 notch, so the
// entry count sets how many levels the goal has. Spacing needn't be even: a
// score dips halfway to the level below, however big that gap is. Names are
// shown on the controller; 8 characters max.
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
constexpr int kBottomLevel = 0; // stowed, and where R1 intakes
constexpr int kFirstLevel = 1;  // the lowest scoring level: where L1 and Y deploy to

pros::Motor intake(ports::kIntake);
pros::Motor claw(ports::kClawMotor); // the roller claw
pros::Rotation liftSensor(ports::kLiftSensor);
pros::Distance clawSensor(ports::kClawDistance);
Piston clawPiston(ports::kClawPiston, /*extendedAtStart=*/false);

// Driven from update(), once per opcontrol tick, with that tick's `now`,
// not on a task of its own, so nothing moves the lift while opcontrol isn't
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

// What LEFT cycles the controller's bottom line through.
enum class Readout { lift, liftMotorA, liftMotorB };

struct State {
    // false = stowed: piston retracted, lift at the bottom. Only R1 stows.
    bool clawDeployed = false;
    bool clawIntake = false;  // Y held (and not overridden by R1 or a score)
    bool intakeReady = false; // R1's stow is done, so it intakes until R1 is let go
    bool debug = false;       // the debug mode update() is running, see debugMode
    Readout readout = Readout::lift;
};

State state;

// Debug mode as asked for by setDebug(), from any task. update() copies it
// into state.debug once a tick, so a whole tick runs in one mode.
std::atomic<bool> debugMode{false};

// This tick's time, for the sequences' until conditions to time the piston.
std::uint32_t tickMs = 0;

// Timed sequences. A score dips halfway to the level below, outtakes, then
// goes up a level. The dip, after releasing Y, drops the lift to the bottom
// and lets it come back up to level 1. The stow, while R1 is held, waits for
// the lift to reach the bottom and the claw to retract before R1 intakes.
// What each step *does* stays level-triggered below (claw volts, piston, lift
// target); the sequence only says which step it is and when to move on.
enum class Phase { scoreDescend, scoreOuttake, dipLower, dipBottom, stowLower };

bool liftArrived();
bool scoreReady();
bool stowReached();

const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{
    {.id = Phase::scoreDescend, .until = scoreReady, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::scoreOuttake, .timeoutMs = kScoreOuttakeMs},
}};
const Sequence<Phase>::Program kScore{.steps = kScoreSteps, .onFinish = [] { ladder.up(); }};

const std::array<Sequence<Phase>::Step, 2> kDipSteps{{
    {.id = Phase::dipLower, .until = liftArrived, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::dipBottom, .timeoutMs = kDipBottomMs},
}};
const Sequence<Phase>::Program kDip{.steps = kDipSteps};

const std::array<Sequence<Phase>::Step, 1> kStowSteps{{
    {.id = Phase::stowLower, .until = stowReached, .timeoutMs = kStowTimeoutMs},
}};
const Sequence<Phase>::Program kStow{.steps = kStowSteps,
                                     .onFinish = [] { state.intakeReady = true; }};

Sequence<Phase> sequence;

// Set by liftTuningTest() from the PID page's task, read by update(): while
// it's a number, the lift heads there instead of to its level.
std::atomic<double> liftTestTarget{std::numeric_limits<double>::quiet_NaN()};

// The "mech" telemetry channel, once attachTelemetry() has made it.
sapphirelib::telemetry::Channel* mechLog = nullptr;

bool pieceInClaw() {
    // get_distance() reads 9999 when it sees nothing and PROS_ERR (INT32_MAX)
    // when unplugged, so neither counts as a pin.
    return clawSensor.get_distance() <= kPieceDetectMm;
}

double liftTargetDeg() {
    const double testTarget = liftTestTarget.load();
    if (std::isfinite(testTarget)) return testTarget;
    if (sequence.running(kScore)) return ladder.midpointBelow(ladder.level());
    if (sequence.running(kDip)) return ladder.levelPosition(kBottomLevel);
    return ladder.levelPosition();
}

/// Never true while the rotation sensor is unplugged (isNear() is false with
/// no reading), so a sequence waiting on this falls through to its timeout.
bool liftArrived() { return lift.isNear(liftTargetDeg()); }

/// The dip is done and the claw has had time to deploy, so outtaking won't
/// throw the pin from a half-open claw.
bool scoreReady() { return liftArrived() && clawPiston.extendedFor(kPistonMoveMs, tickMs); }

/// The lift is at the bottom and the claw has had time to retract, so R1 can
/// intake.
bool stowReached() { return liftArrived() && clawPiston.retractedFor(kPistonMoveMs, tickMs); }

void showStatus(Controller& controller) {
    ControllerScreen& screen = controller.screen();
    if (state.debug) {
        // No level to show: the lift is L1/L2's.
        screen.setLine(0, "DEBUG CLAW %s", state.clawDeployed ? "OUT" : "IN");
    } else if (state.clawDeployed) {
        screen.setLine(0, "%-8s L%d/%d", ladder.tableName(), ladder.level(), ladder.maxLevel());
    } else {
        screen.setLine(0, "%-8s STOW", ladder.tableName());
    }
    if (state.readout == Readout::lift) {
        // Lift sensor reading, then target, in whole degrees, for measuring
        // level heights and seeing whether the lift is short of its target.
        // In debug mode there's no target, just the reading.
        const double liftDeg = lift.position();
        if (std::isnan(liftDeg)) {
            screen.setLine(2, "LIFT NO SENSOR");
        } else if (state.debug) {
            screen.setLine(2, "LIFT %5.0f", liftDeg);
        } else {
            screen.setLine(2, "LIFT %5.0f/%4.0f", liftDeg, liftTargetDeg());
        }
    } else {
        // One lift motor (A, then B on the next LEFT): the volts it's
        // actually applying, its current draw, and its temperature. A motor
        // stalled at 12V should be drawing its full 2.5A; much less than
        // that, or 55C and up (where it starts cutting its own current),
        // means it's being held back rather than out-muscled.
        const std::uint8_t i = state.readout == Readout::liftMotorA ? 0 : 1;
        screen.setLine(2, "%c %3.0fV %3.1fA %2.0fC", 'A' + i, lift.motors().get_voltage(i) / 1000.0,
                       lift.motors().get_current_draw(i) / 1000.0,
                       lift.motors().get_temperature(i));
    }
}

} // namespace

void initialize() {
    liftSensor.reset_position();
    // Only matters when the lift falls back to brake(), with no sensor, or
    // after stop().
    lift.motors().set_brake_mode_all(pros::v5::MotorBrake::hold);
}

void update(Controller& controller) {
    const std::uint32_t now = controller.now();
    tickMs = now;
    // After autonomous, a disable, or a blocking routine run from opcontrol:
    // drop any half-finished sequence (its timing is stale) and start the
    // lift loop fresh. R1 held through the gap stows again before intaking,
    // which takes no time if nothing moved. The controller already forgot
    // what its screen showed, so the text gets resent.
    if (controller.resumed()) {
        sequence.cancel();
        lift.resetController();
        state.intakeReady = false;
    }

    // controller.update() sampled every button once for this tick, and edges
    // come from comparing samples, so reading (or not reading) a button
    // changes nothing. No more reading every new-press every tick so that a
    // press made while one went unread doesn't fire late.
    const bool intaking = controller.held(Button::r1);
    const bool clawIntaking = controller.held(Button::y);
    const bool outtaking = controller.held(Button::r2);

    if (controller.pressed(Button::left)) {
        state.readout = state.readout == Readout::lift         ? Readout::liftMotorA
                        : state.readout == Readout::liftMotorA ? Readout::liftMotorB
                                                               : Readout::lift;
    }

    if (controller.pressed(Button::right)) {
        // The lift keeps its level on the new goal (moving to that goal's
        // height for it), capped if the new goal has fewer levels.
        ladder.nextTable();
        // TODO: LED library's mode-change call goes here.
    }

    // Debug mode switched on or off (from the Home page): each mode starts
    // from a clean slate, nothing half-done carried across.
    const bool debugging = debugMode.load();
    if (debugging != state.debug) {
        state.debug = debugging;
        sequence.cancel();
        state.intakeReady = false;
        state.clawIntake = false;
        if (!debugging) {
            // Back on the levels, at one that agrees with the claw (deployed
            // means level 1 and up), and with the lift loop starting fresh.
            ladder.setLevel(state.clawDeployed ? std::max(ladder.level(), kFirstLevel)
                                               : kBottomLevel);
            lift.resetController();
        }
    }

    // Letting go of R1 ends intaking, a stow still on its way down included.
    if (!intaking) {
        state.intakeReady = false;
        if (sequence.running(kStow)) sequence.cancel();
    }

    // One branch per tick, in priority order. Debug mode first, since it
    // leaves out everything that moves the lift. Then scoring: once started,
    // nothing interrupts it, and starting one beats everything else.
    if (state.debug) {
        // No levels and no sequences: R1 retracts the claw and intakes at
        // once, Y deploys it and spins the roller claw in, and the lift is
        // L1 and L2's alone (below).
        if (intaking) {
            state.clawDeployed = false;
        } else if (clawIntaking) {
            state.clawDeployed = true;
        }
        state.intakeReady = intaking;
        state.clawIntake = clawIntaking && !intaking;
    } else if (sequence.running(kScore)) {
        // R1, Y, L1 and L2 all wait for it to finish.
        sequence.update(now);
    } else if (controller.pressed(Button::r2) && state.clawDeployed) {
        // Takes over from Y, or the dip after it, too.
        state.clawIntake = false;
        sequence.start(kScore, now);
    } else if (intaking) {
        // Stows from any state: claw retracted and the lift to the bottom,
        // and the intake waits for both (kStow) before it runs.
        if (!sequence.running(kStow)) sequence.cancel();
        state.clawDeployed = false;
        state.clawIntake = false;
        ladder.setLevel(kBottomLevel);
        if (sequence.running(kStow)) {
            sequence.update(now);
        } else if (!state.intakeReady) {
            // Already stowed (intaking again at the bottom): no wait at all.
            if (stowReached()) {
                state.intakeReady = true;
            } else {
                sequence.start(kStow, now);
            }
        }
    } else if (clawIntaking) {
        // Deployed at level 1, roller claw spinning in, until Y is released.
        sequence.cancel();
        state.clawDeployed = true;
        state.clawIntake = true;
        ladder.setLevel(kFirstLevel);
    } else if (state.clawIntake) {
        // Y was just released: down to the bottom and back up to level 1.
        state.clawIntake = false;
        sequence.start(kDip, now);
    } else if (sequence.active()) {
        sequence.update(now);
    } else {
        if (controller.repeated(Button::l1, kLevelRepeatDelayMs, kLevelRepeatPeriodMs)) {
            if (state.clawDeployed) {
                ladder.up();
            } else {
                state.clawDeployed = true;
                ladder.setLevel(kFirstLevel);
            }
        }
        // Stops at level 1, deployed; only intaking stows.
        if (controller.repeated(Button::l2, kLevelRepeatDelayMs, kLevelRepeatPeriodMs) &&
            ladder.level() > kFirstLevel) {
            ladder.down();
        }
    }

    clawPiston.set(state.clawDeployed, now);

    // R2 held while stowed (or in debug mode, which never scores) reverses the
    // roller claw along with the intake. Deployed, an R2 press scores
    // instead, and the score runs the claw.
    const bool clawOuttaking = outtaking && (state.debug || !state.clawDeployed);

    // Roller claw: outtaking (a score, or R2 as above) beats intaking (R1 once
    // stowed, Y, or the dip after it), which beats holding a detected pin.
    std::int32_t clawMv = 0;
    if (sequence.is(Phase::scoreOuttake) || clawOuttaking) {
        clawMv = -kClawMv;
    } else if (state.intakeReady || state.clawIntake || sequence.running(kDip)) {
        clawMv = kClawMv;
    } else if (pieceInClaw()) {
        clawMv = kClawHoldMv;
    }
    claw.move_voltage(clawMv);

    // Intake: R2 reverses it whenever it's held, deployed or not, so a score's
    // press reverses it for as long as the button is down. Otherwise it runs
    // in only for R1, once stowed (state.intakeReady, never true without R1
    // held); Y leaves it off.
    std::int32_t intakeMv = 0;
    if (outtaking) {
        intakeMv = -kIntakeMv;
    } else if (state.intakeReady) {
        intakeMv = kIntakeMv;
    }
    intake.move_voltage(intakeMv);

    if (state.debug) {
        // Straight to the motors, whatever the sensor says (or doesn't): up
        // for L1, down for L2, and braked where it's let go (hold mode, see
        // initialize()). Both at once cancel out to braked.
        const bool up = controller.held(Button::l1);
        const bool down = controller.held(Button::l2);
        if (up != down) {
            lift.setVolts(up ? kLiftDebugVolts : -kLiftDebugVolts);
        } else {
            lift.stop();
        }
    } else {
        lift.setTarget(liftTargetDeg());
    }
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
    // What the motors were actually sent, after gravity volts and the clamp,
    // the "lift" channel's "out" is only the loop's share. The listener runs
    // inside lift.update(), on the opcontrol task, and reuses that update's
    // one sensor read; recording never blocks.
    sapphirelib::telemetry::Channel& act = logger.channel(
        "lift.act", {"target", "pos", "volts", "law"}, {.capacity = 128, .decimals = 3});
    lift.setStepListener([channel = &act](const PositionStep& step) {
        channel->record({step.target, step.position, step.volts, static_cast<double>(step.law)});
    });
    // The intake, claw and lift state machine, every opcontrol tick: volts
    // sent to the intake and roller claw, the piston, the claw's distance
    // reading (mm), the lift's level (0 stowed) and goal (0 ALLIANCE, 1
    // MEDIUM, 2 CENTER), the running sequence step (-1 none; else Phase's
    // order: scoreDescend, scoreOuttake, dipLower, dipBottom, stowLower), and
    // whether the claw is deployed.
    mechLog = &logger.channel(
        "mech", {"intake_v", "claw_v", "piston", "piece_mm", "level", "mode", "phase", "deployed"},
        {.capacity = 128, .decimals = 2});
}

void setDebug(bool on) { debugMode.store(on); }

bool debug() { return debugMode.load(); }

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
