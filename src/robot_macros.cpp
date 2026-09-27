/**
 * \file robot_macros.cpp
 *
 * Temporary driver macros for the intake, claw, and lift — see
 * robot_macros.hpp for the controls. Everything meant to be tuned is in the
 * constants at the top.
 *
 * Team 96671H — Hitmen
 */

#include "robot_macros.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/control/pid.hpp"

namespace robot_macros {

namespace {

// --- Ports ---
// TODO: only the lift motor ports are real (the same winch motors main.cpp's
// Lift Testing auton uses). Every other port is a placeholder — set it to
// where that device is actually plugged in.
constexpr std::int8_t kIntakePort = -18;  // reversed, so intaking (+) spins it counterclockwise
constexpr std::int8_t kClawMotorPort = 15;  // intaking (+) spins it clockwise
constexpr std::int8_t kLiftPortA = -20;
constexpr std::int8_t kLiftPortB = 19;
// Negate this if the sensor's reading goes down as the lift goes up.
constexpr std::int8_t kLiftSensorPort = 16;
constexpr std::uint8_t kClawDistancePort = 5;
constexpr char kClawPistonPort = 'A';

// --- Speeds, in millivolts (12000 = full) ---
constexpr std::int32_t kIntakeMv = 12700;
constexpr std::int32_t kClawMv = 12700;  // intaking and outtaking
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

struct ScoringMode {
    const char* name;  // shown on the controller; 8 characters max
    // Lift target at each level, in degrees of the lift's rotation sensor up
    // from where it was zeroed. Entry 0 is level 0 (claw piston deployed,
    // lift down); each entry after it is one L1 notch, so the entry count
    // sets how many levels the mode has. Spacing needn't be even: a score
    // above level 0 dips halfway to the level below, however big that gap is.
    std::vector<double> levelDeg;
};

// TODO: placeholder heights. Measure each goal's real heights on the
// rotation sensor and put them here: the controller's third line shows the
// lift's live reading (then its current target), so run the lift to each
// height manually and copy the number.
const std::array<ScoringMode, 3> kModes{{
    {"ALLIANCE", {0, 150, 300, 450, 600, 750, 900}},
    {"MEDIUM", {0, 180, 360, 540, 720, 900}},
    {"CENTER", {0, 225, 450, 675, 900}},
}};

// The controller drops text sent faster than about every 50ms, across all of
// its lines.
constexpr std::uint32_t kControllerPrintMs = 60;
// update() runs every opcontrol tick (20ms); a longer gap than this means
// opcontrol() stopped and restarted in between.
constexpr std::uint32_t kRestartGapMs = 100;

pros::Motor intake(kIntakePort);
pros::Motor claw(kClawMotorPort);
pros::MotorGroup lift({kLiftPortA, kLiftPortB});
pros::Rotation liftSensor(kLiftSensorPort);
pros::Distance clawSensor(kClawDistancePort);
pros::adi::Pneumatics clawPiston(kClawPistonPort, /*start_extended=*/false);

sapphirelib::PID liftPid(sapphirelib::PID::Config{
    .gains = kLiftGains,
    .outputLimit = 12.7,
    // Level changes step the target, which would otherwise kick the output.
    .derivativeOnMeasurement = true,
    .nominalDtS = 0.02,  // update() runs every 20ms opcontrol tick
});

// Timed sequences, during which L1, L2, and R2 are ignored. A score above
// level 0 dips halfway to the level below, outtakes, then goes up a level.
// A re-seat, after releasing Y, lowers the lift back to level 0, then
// retracts and redeploys the claw piston.
enum class Phase { none, scoreDescend, scoreOuttake, reseatLower, reseatRetract, reseatDeploy };

struct State {
    std::size_t mode = 0;  // index into kModes
    // false = stowed: piston retracted, lift at level 0. Only R1 stows.
    bool clawDeployed = false;
    int level = 0;
    std::uint32_t deployedAtMs = 0;  // when the piston last extended
    bool bottomOuttake = false;  // R2 pressed at level 0 and still held
    bool deployedIntake = false;  // Y held (and not overridden by R1)
    Phase phase = Phase::none;
    std::uint32_t phaseStartMs = 0;
    bool showLiftMotors = false;  // controller lines 2-3 show lift motor readings

    std::array<std::string, 3> shownLines;  // what each controller line last showed
    std::uint32_t lastPrintMs = 0;
    std::uint32_t lastUpdateMs = 0;
};

State state;

const ScoringMode& currentMode() { return kModes[state.mode]; }

int maxLevel() { return static_cast<int>(currentMode().levelDeg.size()) - 1; }

void enterPhase(Phase phase, std::uint32_t now) {
    state.phase = phase;
    state.phaseStartMs = now;
}

void setPiston(bool extended, std::uint32_t now) {
    if (extended == clawPiston.is_extended()) return;
    if (extended) {
        clawPiston.extend();
        state.deployedAtMs = now;
    } else {
        clawPiston.retract();
    }
}

bool pieceInClaw() {
    // get_distance() reads 9999 when it sees nothing and PROS_ERR (INT32_MAX)
    // when unplugged, so neither counts as a piece.
    return clawSensor.get_distance() <= kPieceDetectMm;
}

/// NaN if the rotation sensor isn't answering.
double liftPositionDeg() {
    const std::int32_t centidegrees = liftSensor.get_position();
    if (centidegrees == PROS_ERR) return std::numeric_limits<double>::quiet_NaN();
    return centidegrees / 100.0;
}

/// Halfway between `level` and the level below it.
double halfwayBelow(int level) {
    const std::vector<double>& levels = currentMode().levelDeg;
    return (levels[level] + levels[level - 1]) / 2.0;
}

double liftTargetDeg() {
    if (state.deployedIntake) return halfwayBelow(1);
    if (state.phase == Phase::scoreDescend || state.phase == Phase::scoreOuttake) {
        return halfwayBelow(state.level);
    }
    return currentMode().levelDeg[state.level];
}

/// Never true while the rotation sensor is unplugged (its position reads
/// NaN), so a sequence waiting on this falls through to its timeout.
bool liftArrived() { return std::abs(liftPositionDeg() - liftTargetDeg()) <= kLiftToleranceDeg; }

void driveLift(double targetDeg) {
    const double positionDeg = liftPositionDeg();
    if (std::isnan(positionDeg)) {
        // Chasing a missing reading could run the lift into either end, so
        // hold it where it is on the motors' own encoders instead.
        lift.brake();
        liftPid.reset();
        return;
    }
    double volts;
    if (targetDeg > 0.0) {
        volts = liftPid.update(targetDeg, positionDeg) + kLiftGravityVolts;
    } else if (positionDeg > kLiftRestDeg) {
        // Heading for level 0: see kLiftSeatVolts.
        volts = std::min(liftPid.update(targetDeg, positionDeg), -kLiftSeatVolts);
    } else {
        volts = 0.0;
        liftPid.reset();
    }
    lift.move_voltage(static_cast<std::int32_t>(std::clamp(volts, -12.0, 12.0) * 1000.0));
}

void advancePhase(std::uint32_t now) {
    const std::uint32_t elapsedMs = now - state.phaseStartMs;
    switch (state.phase) {
        case Phase::scoreDescend:
            if (liftArrived() || elapsedMs >= kLiftDescendTimeoutMs) {
                enterPhase(Phase::scoreOuttake, now);
            }
            break;
        case Phase::scoreOuttake:
            if (elapsedMs >= kHeightOuttakeMs) {
                state.phase = Phase::none;
                state.level = std::min(state.level + 1, maxLevel());
            }
            break;
        case Phase::reseatLower:
            if (liftArrived() || elapsedMs >= kLiftDescendTimeoutMs) {
                enterPhase(Phase::reseatRetract, now);
            }
            break;
        case Phase::reseatRetract:
            if (elapsedMs >= kReseatRetractMs) enterPhase(Phase::reseatDeploy, now);
            break;
        case Phase::reseatDeploy:
            if (elapsedMs >= kReseatDeployMs) state.phase = Phase::none;
            break;
        case Phase::none:
            break;
    }
}

void showStatus(pros::Controller& controller, std::uint32_t now) {
    char lines[3][16];
    if (state.clawDeployed) {
        std::snprintf(lines[0], sizeof(lines[0]), "%-8s L%d/%d", currentMode().name,
                      state.level, maxLevel());
    } else {
        std::snprintf(lines[0], sizeof(lines[0]), "%-8s STOW", currentMode().name);
    }
    if (state.showLiftMotors) {
        // One line per lift motor (port A, then B): the volts it's actually
        // applying, its current draw, and its temperature. A motor stalled
        // at 12V should be drawing its full 2.5A; much less than that, or
        // 55C and up (where it starts cutting its own current), means it's
        // being held back rather than out-muscled.
        for (std::uint8_t i = 0; i < 2; ++i) {
            std::snprintf(lines[1 + i], sizeof(lines[1 + i]), "%3.0fV %3.1fA %2.0fC",
                          lift.get_voltage(i) / 1000.0, lift.get_current_draw(i) / 1000.0,
                          lift.get_temperature(i));
        }
    } else {
        // The piston itself, not state.clawDeployed, so this also shows the
        // brief retract during a re-seat.
        std::snprintf(lines[1], sizeof(lines[1]), "CLAW %s",
                      clawPiston.is_extended() ? "DEPLOYED" : "RETRACTED");
        // Lift sensor reading, then target, in whole degrees — for measuring
        // level heights and seeing whether the lift is short of its target.
        const double liftDeg = liftPositionDeg();
        if (std::isnan(liftDeg)) {
            std::snprintf(lines[2], sizeof(lines[2]), "LIFT NO SENSOR");
        } else {
            std::snprintf(lines[2], sizeof(lines[2]), "LIFT %5.0f/%4.0f", liftDeg, liftTargetDeg());
        }
    }

    // At most one line per print interval, so a change that arrives too soon
    // after the last print (or behind another line's change) waits for a
    // later tick.
    if (now - state.lastPrintMs < kControllerPrintMs) return;
    for (std::uint8_t i = 0; i < state.shownLines.size(); ++i) {
        if (state.shownLines[i] == lines[i]) continue;
        state.lastPrintMs = now;
        // Padded to the screen's full 15 columns, so a shorter line fully
        // overwrites a longer one.
        if (controller.print(i, 0, "%-15s", lines[i]) != PROS_ERR) state.shownLines[i] = lines[i];
        return;
    }
}

}  // namespace

void initialize() {
    liftSensor.reset_position();
    // Only matters when driveLift() falls back to brake() with no sensor.
    lift.set_brake_mode_all(pros::v5::MotorBrake::hold);
}

void update(pros::Controller& controller) {
    const std::uint32_t now = pros::millis();
    // After autonomous, a disable, or a blocking routine run from opcontrol:
    // drop any half-finished sequence (its timing is stale), start the lift
    // loop fresh, and resend the controller text, which may have been lost.
    if (now - state.lastUpdateMs > kRestartGapMs) {
        state.phase = Phase::none;
        liftPid.reset();
        for (std::string& line : state.shownLines) line.clear();
    }
    state.lastUpdateMs = now;

    // Every new-press is read every tick, even ones about to be ignored. PROS
    // only clears a button's "new" flag when it's read, so a press made while
    // it went unread would fire late, the next time it was checked.
    const bool intaking = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    const bool deployedIntaking = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    const bool scoreHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    const bool scorePressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2);
    const bool upPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
    const bool downPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
    const bool modePressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
    const bool liftViewPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);

    if (liftViewPressed) state.showLiftMotors = !state.showLiftMotors;

    if (modePressed) {
        state.mode = (state.mode + 1) % kModes.size();
        // The lift keeps its level in the new mode (moving to that mode's
        // height for it), capped if the new mode has fewer levels.
        state.level = std::min(state.level, maxLevel());
        // TODO: LED library's mode-change call goes here.
    }

    if (intaking) {
        // Resets to stowed from any state, cancelling whatever was in progress.
        state.phase = Phase::none;
        state.bottomOuttake = false;
        state.deployedIntake = false;
        state.clawDeployed = false;
        state.level = 0;
    } else if (deployedIntaking) {
        // The same reset, but with the claw deployed, and the lift up halfway
        // to level 1 (see liftTargetDeg()) until Y is released.
        state.phase = Phase::none;
        state.bottomOuttake = false;
        state.deployedIntake = true;
        state.clawDeployed = true;
        state.level = 0;
    } else if (state.deployedIntake) {
        // Y was just released: lower to level 0 and re-seat the piece before
        // the claw goes back to holding it on its own.
        state.deployedIntake = false;
        enterPhase(Phase::reseatLower, now);
    } else if (state.phase != Phase::none) {
        advancePhase(now);
    } else {
        if (upPressed) {
            if (!state.clawDeployed) {
                state.clawDeployed = true;
            } else if (state.level < maxLevel()) {
                ++state.level;
            }
        }
        // Never retracts the piston; only intaking does.
        if (downPressed && state.level > 0) --state.level;
        if (scorePressed) {
            if (state.level == 0) {
                state.clawDeployed = true;
                state.bottomOuttake = true;
            } else {
                enterPhase(Phase::scoreDescend, now);
            }
        }
    }
    if (!scoreHeld) state.bottomOuttake = false;

    const bool reseating = state.phase == Phase::reseatLower ||
                           state.phase == Phase::reseatRetract ||
                           state.phase == Phase::reseatDeploy;
    setPiston(state.clawDeployed && state.phase != Phase::reseatRetract, now);

    // Waits out the rest of the piston's deploy time, which is none at all if
    // it deployed long ago.
    const bool bottomOuttaking =
        state.bottomOuttake && now - state.deployedAtMs >= kPistonDeployMs;

    // Claw: outtaking beats intaking (R1, Y, or re-seating), which beats
    // holding a detected piece.
    std::int32_t clawMv = 0;
    if (bottomOuttaking || state.phase == Phase::scoreOuttake) {
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

    driveLift(liftTargetDeg());
    showStatus(controller, now);
}

}  // namespace robot_macros
