/**
 * \file examples/holonomic_chassis.cpp
 *
 * Phase 1 example: closed-loop holonomic (mecanum/X-drive) drive using only
 * an IMU and drive motor encoders, no tracking wheels. Assumes one motor
 * per corner with green (200 RPM) gearing and a 4" wheel diameter.
 *
 * Not built into the program (the Makefile only compiles src/), but
 * `make check-examples` compiles it against the current headers so it can't
 * silently fall out of date. Copy the relevant pieces into your own
 * src/main.cpp and adjust ports/gains for your robot.
 *
 * Team 96671H: Hitmen
 */

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::input::Axis;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;

namespace {

constexpr std::uint8_t kImuPort = 10;
constexpr double kStickDeadband = 0.05; // ignores the count or two a stick reads at rest
constexpr double kJoystickCurve = 0.3;  // 0 = linear, 1 = full cubic

// At namespace scope, never a local in opcontrol(), so its button history
// survives opcontrol() restarting (see input::Controller).
Controller master(pros::E_CONTROLLER_MASTER);

/// Built on the first call, which initialize() makes, rather than at namespace
/// scope: constructing a drivetrain blocks for ~2-3s while its IMU calibrates,
/// and static initialization runs before initialize(), before the program has
/// even started.
HolonomicDrivetrain& drivetrain() {
    static HolonomicDrivetrain instance(
        /*frontLeftPort=*/1, /*frontRightPort=*/-2, /*backLeftPort=*/3, /*backRightPort=*/-4,
        Gearset::green, kImuPort,
        DrivetrainConfig{.wheelDiameterIn = 4.0, .externalGearRatio = 1.0,
                         .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0});
    return instance;
}

/// A stick reading, [-1, 1], shaped for driving: deadband first, so the curve
/// starts from a clean zero.
double shapeStick(double input) {
    return sapphirelib::curveJoystick(sapphirelib::applyDeadband(input, kStickDeadband),
                                      kJoystickCurve);
}

} // namespace

void initialize() {
    sapphirelib::initialize();
    drivetrain(); // constructs it, blocking while the IMU calibrates
}

void opcontrol() {
    while (true) {
        master.update(); // sample the controller once per tick

        if (master.pressed(Button::a)) {
            // Redefine "forward" as whichever way the chassis is facing now
            // handy after defense spins the robot, or to re-square against
            // a wall. Only relevant to the field-centric calls below.
            drivetrain().resetFieldHeading();
        }

        const double throttle = shapeStick(master.axis(Axis::leftY));
        const double strafe = shapeStick(master.axis(Axis::leftX));
        const double turn = shapeStick(master.axis(Axis::rightX));

        // Field-centric ("headless") driver control: throttle/strafe always
        // mean the same field direction regardless of chassis orientation.
        // For robot-centric control, call holonomic() with the same
        // arguments instead; for a turn stick that steers a held heading
        // rather than a turn rate, holonomicFieldCentricHeadingHold().
        drivetrain().holonomicFieldCentric(throttle, strafe, turn);
        pros::delay(20);
    }
}

void autonomous() {
    // Exit conditions default to a 1in threshold for drives and 2 degrees for
    // turns, each held for 200ms, with a 3s timeout.
    drivetrain().driveDistance(24.0);
    drivetrain().turnToHeading(90.0);

    // Override just the fields you care about. Every motion also returns a
    // MotionResult, so a routine can tell a settled motion from one that
    // timed out against something.
    const auto result = drivetrain().driveDistance(-24.0, {.timeoutMs = 1500});
    if (!result.settled()) return; // stuck: don't carry on from the wrong place
    drivetrain().turnToHeading(0.0);
}
