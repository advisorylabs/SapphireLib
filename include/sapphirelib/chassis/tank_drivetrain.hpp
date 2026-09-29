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

#include <atomic>
#include <cstdint>
#include <initializer_list>

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/motion/motion_config.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::chassis {

/// Closed-loop differential (2-side) drivetrain: driveDistance()/
/// turnToHeading() using only an IMU and drive motor encoders. Also
/// provides tank()/arcade() for driver control. For mecanum/X-drive
/// chassis, see HolonomicDrivetrain instead.
///
/// Every blocking motion returns a motion::MotionResult (settled, timed
/// out, or aborted, plus its final error) and logs a `motion` start/end
/// event to the running telemetry::Logger, if there is one — see
/// docs/TELEMETRY_FORMAT.md.
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

    /// The forward and turn volts the two sides were last commanded:
    /// forward = (left + right) / 2, turn = (left - right) / 2, and strafe
    /// always 0. Covers every path to the motors — tank()/arcade() and every
    /// autonomous motion. Before each motor's own ±12V clamp, so an
    /// autonomous motion's raw PID output can read past 12. All zero after
    /// stop(), which commands no voltage.
    ///
    /// For telemetry and model fitting: the input side of the chassis's
    /// response. Safe to call from any task — the fields are separate
    /// lock-free atomics, so a read racing a command can mix fields from
    /// two consecutive ticks, but never tears a single value.
    AxisVolts appliedAxisVolts() const;

    /// Drives straight for `inches` (signed: negative reverses) using
    /// drive-encoder position PID with IMU-based heading correction.
    /// Blocks until settled or timed out, then stops.
    ///
    /// Measures from wherever the drive encoders read when it starts,
    /// without taring them — see MotorGroup::tarePosition() for why a tare
    /// would disturb odometry.
    motion::MotionResult driveDistance(double inches, ExitConditions exit = ExitConditions{1.0});

    /// Turns in place to `headingDeg` (absolute heading, matching
    /// pros::Imu::get_heading()'s 0-360 range) using IMU heading PID.
    /// `exit.errorThreshold` is in degrees here. Blocks until settled or
    /// timed out, then stops.
    motion::MotionResult turnToHeading(double headingDeg,
                                       ExitConditions exit = ExitConditions{2.0});

    /// The pose source for the moveToPoint()/moveToPose()/followPath()
    /// overloads that don't take one, so a routine can write
    /// `moveToPoint(24, 24)` instead of passing the odometry to every call.
    /// Set it once during setup (initialize()), before any motion runs —
    /// it isn't synchronized. `odometry` must outlive the drivetrain;
    /// nullptr unsets it. The overloads that take an Odometry ignore this.
    void setOdometry(const odom::Odometry* odometry);

    /// Drives to field point (`xIn`, `yIn`), reading pose from `odometry`.
    /// Doesn't control final heading — arrives facing whatever direction the
    /// approach left it (for that, see moveToPose()). Since a differential
    /// chassis can't strafe, it turns to face the point (or, if the point is
    /// behind it by more than 90 degrees, reverses instead of spinning all
    /// the way around) while driving, rather than turning first and then
    /// driving. `exit.errorThreshold` is the distance to the point, in
    /// inches. Blocks until settled or timed out, then stops.
    motion::MotionResult moveToPoint(double xIn, double yIn, const odom::Odometry& odometry,
                                     ExitConditions exit = ExitConditions{1.0});

    /// moveToPoint() reading pose from the setOdometry() odometry. If none
    /// was set, logs an error and returns ExitReason::aborted without
    /// moving.
    motion::MotionResult moveToPoint(double xIn, double yIn,
                                     ExitConditions exit = ExitConditions{1.0});

    /// Drives to field pose (`xIn`, `yIn`, `headingDeg`), reading pose from
    /// `odometry`. Uses a boomerang controller: aims at a "carrot" point
    /// placed behind the target along its facing direction (see
    /// motion::PoseExitConditions::boomerangLeadPct) so the chassis curves
    /// smoothly into the final heading instead of driving straight at the
    /// point and point-turning at the end. Blocks until settled or timed
    /// out, then stops. The result's finalError is the distance to the
    /// point.
    motion::MotionResult moveToPose(double xIn, double yIn, double headingDeg,
                                    const odom::Odometry& odometry,
                                    motion::PoseExitConditions exit = {});

    /// moveToPose() reading pose from the setOdometry() odometry. If none
    /// was set, logs an error and returns ExitReason::aborted without
    /// moving.
    motion::MotionResult moveToPose(double xIn, double yIn, double headingDeg,
                                    motion::PoseExitConditions exit = {});

    /// Follows `path` using pure pursuit: repeatedly steers toward a point
    /// `config.lookaheadIn` ahead on the path, at constant cruise voltage,
    /// until within `config.finalApproachIn` of the path's last waypoint —
    /// then hands off to moveToPoint() for a controlled, settled stop
    /// there. Blocks until that final moveToPoint() settles or times out.
    ///
    /// Returns the final approach's reason and error, with elapsedMs
    /// covering the whole path. If the pursuit phase runs past
    /// `config.timeoutMs` it stops there and returns ExitReason::timedOut
    /// (finalError is the distance left to the last waypoint); an empty
    /// path logs an error and returns ExitReason::aborted without moving.
    motion::MotionResult followPath(const motion::Path& path, const odom::Odometry& odometry,
                                    motion::PursuitConfig config);

    /// followPath() reading pose from the setOdometry() odometry. If none
    /// was set, logs an error and returns ExitReason::aborted without
    /// moving.
    motion::MotionResult followPath(const motion::Path& path, motion::PursuitConfig config);

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

    /// See setOdometry().
    const odom::Odometry* odometry_ = nullptr;

    /// See appliedAxisVolts(). Written by whichever task is commanding the
    /// motors, read by telemetry's sampler — atomics rather than a mutex,
    /// since PROS deletes competition tasks on every mode change and a
    /// mutex held at that moment would stay locked forever.
    std::atomic<double> appliedForwardVolts_{0.0};
    std::atomic<double> appliedTurnVolts_{0.0};

    double degreesToInches(double degrees) const;

    /// The IMU's scaled rotation since construction, wrapped to 0-360 — a
    /// heading Odometry::setPose()'s re-framing never shifts, for
    /// driveDistance()'s heading correction, which only cares about drift
    /// from where it started. See HolonomicDrivetrain::rotationHeadingDeg().
    double rotationHeadingDeg();

    /// The one place the drive motors get a voltage — every call site goes
    /// through here, so appliedAxisVolts() sees every command.
    void setSideVoltages(double leftVolts, double rightVolts);

    /// Mean of the two sides' encoders, in motor degrees —
    /// driveDistance()'s position reading.
    double sideAverageDegrees() const;

    /// moveToPoint()'s control loop. Logs no telemetry events of its own,
    /// so followPath()'s final approach shows up as part of the path rather
    /// than as a separate motion.
    motion::MotionResult approachPoint(double xIn, double yIn, const odom::Odometry& odometry,
                                       ExitConditions exit);
};

} // namespace sapphirelib::chassis
