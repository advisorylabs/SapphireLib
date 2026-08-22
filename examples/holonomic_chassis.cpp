/**
 * \file examples/holonomic_chassis.cpp
 *
 * Phase 1 example: closed-loop holonomic (mecanum/X-drive) drive using only
 * an IMU and drive motor encoders — no tracking wheels. Assumes one motor
 * per corner with green (200 RPM) gearing and a 4" wheel diameter.
 *
 * This file is illustrative, not built by the project (the Makefile only
 * compiles src/). Copy the relevant pieces into your own src/main.cpp and
 * adjust ports/gains for your robot.
 *
 * Team 96671H — Hitmen
 */

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::ExitConditions;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;

namespace {

constexpr std::uint8_t kImuPort = 10;
constexpr double kJoystickCurve = 0.3; // 0 = linear, 1 = full cubic

HolonomicDrivetrain* drivetrain = nullptr;

} // namespace

void initialize() {
    sapphirelib::initialize();

    static HolonomicDrivetrain chassis(
        /*frontLeftPort=*/1, /*frontRightPort=*/-2, /*backLeftPort=*/3, /*backRightPort=*/-4,
        Gearset::green, kImuPort,
        DrivetrainConfig{.wheelDiameterIn = 4.0, .externalGearRatio = 1.0,
                          .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0});
    drivetrain = &chassis;
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        if (master.get_digital_new_press(DIGITAL_A)) {
            // Redefine "forward" as whichever way the chassis is facing now
            // — handy after defense spins the robot, or to re-square against
            // a wall. Only relevant to holonomicFieldCentric() below.
            drivetrain->resetFieldHeading();
        }

        const double throttle =
            sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_Y) / 127.0, kJoystickCurve);
        const double strafe =
            sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_X) / 127.0, kJoystickCurve);
        const double turn =
            sapphirelib::curveJoystick(master.get_analog(ANALOG_RIGHT_X) / 127.0, kJoystickCurve);

        // Field-centric ("headless") driver control: throttle/strafe always
        // mean the same field direction regardless of chassis orientation.
        // For robot-centric control instead, call drivetrain->holonomic(...)
        // directly with the same arguments.
        drivetrain->holonomicFieldCentric(throttle, strafe, turn);
        pros::delay(20);
    }
}

void autonomous() {
    drivetrain->driveDistance(24.0, ExitConditions{.errorThreshold = 1.0});
    drivetrain->turnToHeading(90.0, ExitConditions{.errorThreshold = 2.0});
    drivetrain->driveDistance(-24.0, ExitConditions{.errorThreshold = 1.0});
}
