/**
 * \file sapphirelib/chassis/holonomic_drivetrain.hpp
 *
 * Closed-loop holonomic (mecanum / X-drive) drivetrain: four independently
 * driven corner wheels mixed for forward/strafe/turn. Built on IMU heading +
 * drive motor encoders — no tracking wheels required. For a 2-side
 * differential chassis, see TankDrivetrain instead.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

#include "pros/imu.hpp"
#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/control/pid.hpp"

namespace sapphirelib::chassis {

/// Closed-loop holonomic drivetrain: one motor per corner (front-left,
/// front-right, back-left, back-right), mixed using the standard
/// mecanum/X-drive equations so the chassis can drive, strafe, and turn
/// independently. driveDistance()/turnToHeading() only use the
/// forward/turn axes — see the class comment on driveDistance() for why
/// strafing isn't (yet) part of the closed-loop API.
class HolonomicDrivetrain {
public:
    /// Constructs the drivetrain's motor groups and IMU directly from ports
    /// (rather than accepting already-built MotorGroup/pros::Imu objects) —
    /// pros::MotorGroup holds a non-copyable, non-movable mutex, so it can
    /// only be constructed in place, never passed by value. Unlike
    /// TankDrivetrain's per-side ports, corners take a single port each:
    /// multi-motor mecanum/X-drive corners are rare in VRC, so a single
    /// port keeps the common case simple.
    HolonomicDrivetrain(std::int8_t frontLeftPort, std::int8_t frontRightPort,
                         std::int8_t backLeftPort, std::int8_t backRightPort, Gearset gearset,
                         std::uint8_t imuPort, DrivetrainConfig config, PID::Config drivePIDConfig,
                         PID::Config turnPIDConfig);

    /// Driver control: `throttle` (forward/back), `strafe` (left/right), and
    /// `turn` are each normalized [-1, 1], already curved by the caller if
    /// desired (see sapphirelib::curveJoystick()). The mix is normalized so
    /// no wheel exceeds full voltage without distorting the requested
    /// direction. Robot-centric: `throttle`/`strafe` are relative to the
    /// chassis's own nose. For field-centric ("headless") control, see
    /// holonomicFieldCentric() instead.
    void holonomic(double throttle, double strafe, double turn);

    /// Field-centric ("headless") driver control: `throttle`/`strafe` are
    /// relative to the field, not the chassis — "throttle" always drives
    /// toward the heading captured at construction time (or by the last
    /// resetFieldHeading() call), regardless of which way the chassis is
    /// currently facing. Internally rotates (throttle, strafe) into the
    /// chassis's current frame by the heading delta, then mixes exactly like
    /// holonomic(). `turn` is unaffected (rotation rate is frame-independent).
    void holonomicFieldCentric(double throttle, double strafe, double turn);

    /// Re-zeros the field-centric reference heading to the chassis's current
    /// IMU heading — whichever way the chassis is facing now becomes the new
    /// "throttle" direction for holonomicFieldCentric(). Typically bound to a
    /// driver button so they can redefine "forward" mid-match.
    void resetFieldHeading();

    /// Drives straight for `inches` (signed: negative reverses) using
    /// drive-encoder position PID with IMU-based heading correction. Only
    /// drives along the forward axis (no strafe component) — strafing
    /// distance control needs per-wheel-vector odometry math that isn't
    /// implemented yet; use holonomic() directly for open-loop strafing in
    /// the meantime. Blocks until settled or timed out, then stops.
    void driveDistance(double inches, ExitConditions exit = ExitConditions{1.0});

    /// Turns in place to `headingDeg` (absolute heading, matching
    /// pros::Imu::get_heading()'s 0-360 range) using IMU heading PID.
    /// Blocks until settled or timed out, then stops.
    void turnToHeading(double headingDeg, ExitConditions exit = ExitConditions{2.0});

    void stop(BrakeMode mode = BrakeMode::brake);

    /// Current IMU heading in degrees (0-360, clockwise-positive), for
    /// callers implementing field-centric ("headless") driver control.
    double headingDeg() const;

private:
    MotorGroup frontLeft_;
    MotorGroup frontRight_;
    MotorGroup backLeft_;
    MotorGroup backRight_;
    pros::Imu imu_;
    DrivetrainConfig config_;
    PID drivePID_;
    PID turnPID_;

    /// Field-centric reference heading — see resetFieldHeading(). Set to the
    /// IMU heading at construction time, so holonomicFieldCentric() works out
    /// of the box without callers needing to call resetFieldHeading() first.
    double fieldHeadingZeroDeg_;

    double degreesToInches(double degrees) const;
    void setWheelVoltages(double frontLeft, double frontRight, double backLeft, double backRight);
};

} // namespace sapphirelib::chassis
