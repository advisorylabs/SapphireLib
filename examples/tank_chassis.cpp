/**
 * \file examples/tank_chassis.cpp
 *
 * Phase 1 example: closed-loop tank drive using only an IMU and drive motor
 * encoders — no tracking wheels. Assumes a 2-motor-per-side drivetrain with
 * green (200 RPM) gearing and a 3.25" wheel diameter.
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
using sapphirelib::chassis::TankDrivetrain;

namespace {

constexpr std::uint8_t kImuPort = 10;
constexpr double kJoystickCurve = 0.3; // 0 = linear, 1 = full cubic

TankDrivetrain* drivetrain = nullptr;

} // namespace

void initialize() {
    sapphirelib::initialize();

    static TankDrivetrain chassis(
        /*leftPorts=*/{1, -2, 3}, /*rightPorts=*/{-4, 5, -6}, Gearset::green, kImuPort,
        DrivetrainConfig{.wheelDiameterIn = 3.25, .externalGearRatio = 1.0,
                          .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.1}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.02}, .outputLimit = 12.0});
    drivetrain = &chassis;
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        const double throttle =
            sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_Y) / 127.0, kJoystickCurve);
        const double turn =
            sapphirelib::curveJoystick(master.get_analog(ANALOG_RIGHT_X) / 127.0, kJoystickCurve);

        drivetrain->arcade(throttle, turn);
        pros::delay(20);
    }
}

void autonomous() {
    drivetrain->driveDistance(24.0, ExitConditions{.errorThreshold = 1.0});
    drivetrain->turnToHeading(90.0, ExitConditions{.errorThreshold = 2.0});
    drivetrain->driveDistance(-24.0, ExitConditions{.errorThreshold = 1.0});
}
