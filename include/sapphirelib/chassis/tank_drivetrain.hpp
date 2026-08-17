/**
 * \file sapphirelib/chassis/tank_drivetrain.hpp
 *
 * Closed-loop differential (tank) drivetrain built on IMU heading + drive
 * motor encoders — no tracking wheels required. Also provides tank()/
 * arcade() for driver control.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <initializer_list>

#include "pros/imu.hpp"
#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/control/pid.hpp"

namespace sapphirelib::chassis {

/// Closed-loop differential (2-side) drivetrain: driveDistance()/
/// turnToHeading() using only an IMU and drive motor encoders. Also
/// provides tank()/arcade() for driver control. For mecanum/X-drive
/// chassis, see HolonomicDrivetrain instead.
class TankDrivetrain {
public:
    /// Constructs the drivetrain's motor groups and IMU directly from ports
    /// (rather than accepting already-built MotorGroup/pros::Imu objects) —
    /// pros::MotorGroup holds a non-copyable, non-movable mutex, so it can
    /// only be constructed in place, never passed by value.
    TankDrivetrain(std::initializer_list<std::int8_t> leftPorts,
                   std::initializer_list<std::int8_t> rightPorts, Gearset gearset,
                   std::uint8_t imuPort, DrivetrainConfig config, PID::Config drivePIDConfig,
                   PID::Config turnPIDConfig);

    /// Driver control: `left`/`right` are normalized [-1, 1] joystick
    /// positions, already curved by the caller if desired (see
    /// sapphirelib::curveJoystick()).
    void tank(double left, double right);

    /// Driver control: `throttle`/`turn` are normalized [-1, 1].
    void arcade(double throttle, double turn);

    /// Drives straight for `inches` (signed: negative reverses) using
    /// drive-encoder position PID with IMU-based heading correction.
    /// Blocks until settled or timed out, then stops.
    void driveDistance(double inches, ExitConditions exit = ExitConditions{1.0});

    /// Turns in place to `headingDeg` (absolute heading, matching
    /// pros::Imu::get_heading()'s 0-360 range) using IMU heading PID.
    /// Blocks until settled or timed out, then stops.
    void turnToHeading(double headingDeg, ExitConditions exit = ExitConditions{2.0});

    void stop(BrakeMode mode = BrakeMode::brake);

private:
    MotorGroup left_;
    MotorGroup right_;
    pros::Imu imu_;
    DrivetrainConfig config_;
    PID drivePID_;
    PID turnPID_;

    double degreesToInches(double degrees) const;
};

} // namespace sapphirelib::chassis
