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

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/motion_config.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::chassis {

/// Closed-loop differential (2-side) drivetrain: driveDistance()/
/// turnToHeading() using only an IMU and drive motor encoders. Also
/// provides tank()/arcade() for driver control. For mecanum/X-drive
/// chassis, see HolonomicDrivetrain instead.
class TankDrivetrain {
public:
    /// Constructs the drivetrain's motor groups and IMU directly from ports
    /// (rather than accepting already-built MotorGroup/sensors::Imu
    /// objects) — pros::MotorGroup holds a non-copyable, non-movable mutex,
    /// so it can only be constructed in place, never passed by value.
    /// `imuHeadingScale` corrects for the V5 IMU's multi-turn drift — see
    /// sensors::Imu's class comment; leave at the default 1.0 until you've
    /// run sensors::calibrateHeadingScale() for this robot.
    TankDrivetrain(std::initializer_list<std::int8_t> leftPorts,
                   std::initializer_list<std::int8_t> rightPorts, Gearset gearset,
                   std::uint8_t imuPort, DrivetrainConfig config, PID::Config drivePIDConfig,
                   PID::Config turnPIDConfig, double imuHeadingScale = 1.0);

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

    /// Drives to field point (`xIn`, `yIn`), reading pose from `odometry`.
    /// Doesn't control final heading — arrives facing whatever direction the
    /// approach left it (for that, see moveToPose()). Since a differential
    /// chassis can't strafe, it turns to face the point (or, if the point is
    /// behind it by more than 90 degrees, reverses instead of spinning all
    /// the way around) while driving, rather than turning first and then
    /// driving. Blocks until settled or timed out, then stops.
    void moveToPoint(double xIn, double yIn, const odom::Odometry& odometry, ExitConditions exit);

    /// Drives to field pose (`xIn`, `yIn`, `headingDeg`), reading pose from
    /// `odometry`. Uses a boomerang controller: aims at a "carrot" point
    /// placed behind the target along its facing direction (see
    /// motion::PoseExitConditions::boomerangLeadPct) so the chassis curves
    /// smoothly into the final heading instead of driving straight at the
    /// point and point-turning at the end. Blocks until settled or timed
    /// out, then stops.
    void moveToPose(double xIn, double yIn, double headingDeg, const odom::Odometry& odometry,
                     motion::PoseExitConditions exit);

    /// Follows `path` using pure pursuit: repeatedly steers toward a point
    /// `config.lookaheadIn` ahead on the path, at constant cruise voltage,
    /// until within `config.finalApproachIn` of the path's last waypoint —
    /// then hands off to moveToPoint() for a controlled, settled stop
    /// there. Blocks until that final moveToPoint() settles or times out.
    void followPath(const motion::Path& path, const odom::Odometry& odometry, motion::PursuitConfig config);

    void stop(BrakeMode mode = BrakeMode::brake);

    /// The drivetrain's own calibrated IMU — exposed so you can share it
    /// with an externally-constructed odom::Odometry (via
    /// Odometry::Sensors::imu) instead of opening a second sensor object on
    /// the same physical port.
    sensors::Imu& imu();

    /// Exposes the internal drive/turn PID controllers for live tuning
    /// (see gui::PidTunerPage) — adjusting gains through these takes effect
    /// immediately on the next driveDistance()/turnToHeading()/moveTo*()
    /// call, since they read gains fresh each update() rather than caching
    /// them at construction.
    PID& drivePID();
    PID& turnPID();

private:
    MotorGroup left_;
    MotorGroup right_;
    sensors::Imu imu_;
    DrivetrainConfig config_;
    PID drivePID_;
    PID turnPID_;

    double degreesToInches(double degrees) const;
};

} // namespace sapphirelib::chassis
