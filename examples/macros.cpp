/**
 * \file examples/macros.cpp
 *
 * Phase 5 example: a small driver macro system for a hypothetical robot, built
 * on SapphireLib's input/, mechanism/, and util/ primitives. This is the
 * complete example docs/MACROS.md walks through. The robot has:
 *   - an arm on two motors, with a rotation sensor on its pivot, held up
 *     against gravity by cosine feedforward and resting on a hard stop at the
 *     bottom;
 *   - a clamp on a pneumatic piston;
 *   - an intake roller that briefly reverses itself when it jams.
 *
 * Controls:
 *   L1 / L2    Arm up / down one preset (hold to repeat).
 *   R1         Close / open the clamp.
 *   R2 (hold)  Intake in.
 *   Y (hold)   Intake out.
 *   A          Score (arm above STOW only): dip the arm, open the clamp, then
 *              stow the arm.
 *
 * Not built into the program (the Makefile only compiles src/), but
 * `make check-examples` compiles it against the current headers so it can't
 * silently fall out of date. Copy the pieces you need into your own files and
 * change the ports and numbers for your robot.
 *
 * Team 96671H — Hitmen
 */

#include <array>
#include <cstdint>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::Sequence;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
using sapphirelib::mechanism::Piston;
using sapphirelib::mechanism::PositionConfig;
using sapphirelib::mechanism::positionLawName;
using sapphirelib::mechanism::PositionMechanism;
using sapphirelib::mechanism::PresetLadder;
using sapphirelib::mechanism::readRotationDeg;
using sapphirelib::mechanism::Roller;

namespace {

// --- Ports ---
// Negate the sensor port if its reading goes down as the arm goes up.
constexpr std::int8_t kArmSensorPort = 4;
constexpr char kClampPort = 'A';

// --- Numbers to tune ---
// How far below its preset the arm dips to score, in degrees.
constexpr double kScoreDipDeg = 20.0;
// How long the clamp stays open before the arm stows.
constexpr std::uint32_t kScoreReleaseMs = 300;
constexpr double kIntakeVolts = 12.0;

// At namespace scope, never a local in opcontrol(): its button history has to
// survive opcontrol() being restarted, or a button held through a disable and
// re-enable reads as a fresh press (see input::Controller).
Controller master(pros::E_CONTROLLER_MASTER);

pros::Rotation armSensor(kArmSensorPort);

// Arm presets, in degrees on its rotation sensor, zeroed with the arm resting
// on its bottom stop: STOW, LOW, HIGH.
PresetLadder armLevels({{"ARM", {0.0, 60.0, 135.0}}});

const PositionConfig kArmConfig{
    .pid = {.gains = {.kP = 0.15, .kD = 0.005},
            .outputLimit = 12.0,
            // Preset changes step the target, which would otherwise kick the
            // output.
            .derivativeOnMeasurement = true,
            // arm.startTask(10) in initialize().
            .nominalDtS = 0.01},
    // The sensor reads 90 with the arm horizontal, where gravity pulls
    // hardest: 1.5V there, tapering to none with the arm hanging straight down.
    .gravity = {.cosineVolts = 1.5, .horizontalPosition = 90.0},
    // STOW sits on the bottom stop: drive down onto it, then rest at 0V
    // instead of pushing into it.
    .seat = {.enabled = true, .floor = 0.0, .seatVolts = 2.0, .restBand = 3.0},
    // Within 3 degrees for 100ms counts as settled.
    .tolerance = 3.0,
    .settleTimeMs = 100,
};

PositionMechanism arm({7, -8}, [] { return readRotationDeg(armSensor); }, kArmConfig);

// Extended = clamped shut.
Piston clamp(kClampPort);

Roller intake({11, -12}, {.enabled = true});

// Score: dip below the current preset, open the clamp, then stow. Each step
// commands its mechanism once, from onEnter, and the arm's own task does the
// rest. So the same program runs from opcontrol() (one update() per tick) and
// from autonomous() (runBlocking()).
enum class ScoreStep { dip, release, stow };

const std::array<Sequence<ScoreStep>::Step, 3> kScoreSteps{{
    {.id = ScoreStep::dip,
     .until = [] { return arm.settled(); },
     .timeoutMs = 800,
     .onEnter = [] { arm.setTarget(armLevels.levelPosition() - kScoreDipDeg); }},
    {.id = ScoreStep::release,
     .timeoutMs = kScoreReleaseMs,
     // onEnter isn't handed the tick's "now"; a later clock reading is harmless
     // here, since elapsed time never goes negative (util/timing.hpp).
     .onEnter = [] { clamp.set(false, sapphirelib::millis()); }},
    {.id = ScoreStep::stow,
     .until = [] { return arm.settled(); },
     .timeoutMs = 1000,
     .onEnter =
         [] {
             armLevels.setLevel(0);
             arm.setTarget(armLevels.levelPosition());
         }},
}};
// Referenced by the sequence, not copied, so it lives at namespace scope too.
const Sequence<ScoreStep>::Program kScore{.steps = kScoreSteps};

Sequence<ScoreStep> sequence;

bool wasJammed = false;

/// One driver-control tick, after master.update().
void updateMacros() {
    const std::uint32_t now = master.now();

    // After autonomous, a disable, or anything else that stopped this loop, a
    // half-finished sequence's timing is stale: drop it. (The arm needs
    // nothing here: its own task never stopped.)
    if (master.resumed()) sequence.cancel();

    // One chain, in priority order: a running sequence owns the arm and clamp,
    // then starting a score, then the manual buttons.
    if (sequence.active()) {
        sequence.update(now);
    } else if (master.pressed(Button::a) && armLevels.level() > 0) {
        sequence.start(kScore, now);
    } else {
        if (master.repeated(Button::l1, 400, 150)) armLevels.up();
        if (master.repeated(Button::l2, 400, 150)) armLevels.down();
        if (master.pressed(Button::r1)) clamp.toggle(now);
        // Setting the same target every tick is fine: only a new value restarts
        // settle timing.
        arm.setTarget(armLevels.levelPosition());
    }

    // The intake belongs to the driver whatever else is going on. spin() runs
    // every tick, 0V included, because jam detection only works while fed.
    double intakeVolts = 0.0;
    if (master.held(Button::r2)) {
        intakeVolts = kIntakeVolts;
    } else if (master.held(Button::y)) {
        intakeVolts = -kIntakeVolts;
    }
    intake.spin(intakeVolts, now);

    // One short buzz as each jam starts, not one per tick while it clears.
    const bool jammed = intake.jammed();
    if (jammed && !wasJammed) master.rumble(".");
    wasJammed = jammed;

    // Say what each line should show every tick; flushScreen() sends only what
    // changed, at most one write per 60ms.
    master.screen().setLine(0, "ARM %d %s", armLevels.level(), positionLawName(arm.law()));
    master.screen().setLine(1, "CLAMP %s", clamp.extended() ? "CLOSED" : "OPEN");
    master.screen().setLine(2, "%s", sequence.active() ? "SCORING" : jammed ? "INTAKE JAM" : "");
    master.flushScreen();
}

} // namespace

void initialize() {
    sapphirelib::initialize();

    // With the arm resting on its bottom stop: every preset is measured up
    // from here.
    armSensor.reset_position();
    // What the arm does whenever it brakes: sensor lost, stop(), and while
    // disabled.
    arm.motors().set_brake_mode_all(pros::v5::MotorBrake::hold);
    // The arm runs its own 10ms loop from here on (matching pid.nominalDtS),
    // so it keeps holding while autonomous blocks on drivetrain motions.
    arm.startTask(10);
}

void disabled() {
    // VEXos ignores motor commands while disabled; this just makes sure the
    // intake doesn't pick up where it left off when the robot is enabled again.
    intake.stop();
}

void autonomous() {
    clamp.set(true, sapphirelib::millis()); // grab the preload

    // Start the arm rising without waiting for it: its task keeps driving it
    // while autonomous moves on (to a drivetrain motion, say).
    armLevels.setLevel(1);
    arm.setTarget(armLevels.levelPosition());
    // ...drive to the goal here...

    // Block until it clears the goal's rim. Every wait has a timeout, so an
    // unplugged sensor costs a second rather than the rest of the match.
    sapphirelib::waitUntil([] { return arm.position() >= 45.0; }, 1000);

    // Go to a preset and wait for it to settle there, in one call. False if it
    // hadn't settled within 1.5s.
    armLevels.setLevel(2);
    arm.moveTo(armLevels.levelPosition(), 1500);

    // The same program the A button runs, run to completion.
    sequence.runBlocking(kScore, 3000);
}

void opcontrol() {
    while (true) {
        master.update(); // sample the controller once, at the top of the tick
        updateMacros();
        // Drive code goes here, reading the same sample: for example
        // drivetrain.arcade(master.axis(Axis::leftY), master.axis(Axis::rightX)).
        pros::delay(20);
    }
}
