/**
 * \file sapphirelib/chassis/holonomic_drivetrain.hpp
 *
 * Closed-loop holonomic (mecanum / X-drive) drivetrain: four independently
 * driven corner wheels mixed for forward/strafe/turn. Built on IMU heading +
 * drive motor encoders — no tracking wheels required. Optionally adds a
 * 5th/6th "Asterisk" pair of center wheels — see AsteriskConfig. For a
 * 2-side differential chassis, see TankDrivetrain instead.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

#include "pros/rtos.hpp"
#include "sapphirelib/chassis/drift_math.hpp"
#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/chassis/thermal_math.hpp"
#include "sapphirelib/control/heading_hold.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/motion_config.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"
#include "sapphirelib/sensors/imu.hpp"

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
    /// (rather than accepting already-built MotorGroup/sensors::Imu
    /// objects) — pros::MotorGroup holds a non-copyable, non-movable mutex,
    /// so it can only be constructed in place, never passed by value.
    /// Unlike TankDrivetrain's per-side ports, corners take a single port
    /// each: multi-motor mecanum/X-drive corners are rare in VRC, so a
    /// single port keeps the common case simple. `imuHeadingScale` corrects
    /// for the V5 IMU's multi-turn drift — see sensors::Imu's class
    /// comment; leave at the default 1.0 until you've run
    /// sensors::calibrateHeadingScale() for this robot.
    /// `asterisk` adds the 5th/6th center wheels described on
    /// AsteriskConfig — leave at std::nullopt for a standard 4-motor
    /// holonomic chassis.
    HolonomicDrivetrain(std::int8_t frontLeftPort, std::int8_t frontRightPort,
                         std::int8_t backLeftPort, std::int8_t backRightPort, Gearset gearset,
                         std::uint8_t imuPort, DrivetrainConfig config, PID::Config drivePIDConfig,
                         PID::Config turnPIDConfig, double imuHeadingScale = 1.0,
                         std::optional<AsteriskConfig> asterisk = std::nullopt);

    /// Driver control: `throttle` (forward/back), `strafe` (left/right), and
    /// `turn` are each normalized [-1, 1], already curved by the caller if
    /// desired (see sapphirelib::curveJoystick()). The mix is normalized so
    /// no wheel exceeds full voltage without distorting the requested
    /// direction. Robot-centric: `throttle`/`strafe` are relative to the
    /// chassis's own nose. For field-centric ("headless") control, see
    /// holonomicFieldCentric() instead.
    ///
    /// How each stick becomes volts depends on setDriverInputMode().
    void holonomic(double throttle, double strafe, double turn);

    /// Raw per-axis volts, before wheel mixing — never affected by
    /// setDriverInputMode(). For anything that isn't a driver's sticks: a
    /// characterization run, a calibration spin, a custom autonomous
    /// controller. Normalized the same way as holonomic() so no wheel exceeds
    /// 12V without distorting the direction.
    void holonomicVolts(double forwardVolts, double strafeVolts, double turnVolts);

    /// Chooses how holonomic() and the heading-hold/field-centric variants
    /// turn stick input into motor output — see DriverInputMode. Heading
    /// hold's turn output is always volts from its PID; this only changes
    /// the translation sticks there. Safe to call from any task.
    void setDriverInputMode(DriverInputMode mode);
    DriverInputMode driverInputMode() const;

    /// Installs measured per-axis models — used by DriverInputMode::velocity.
    /// Safe to call from any task (e.g. a tuning run finishing while driver
    /// control is live).
    void setAxisModels(HolonomicAxisModels models);
    HolonomicAxisModels axisModels() const;

    /// Field-centric ("headless") driver control: `throttle`/`strafe` are
    /// relative to the field, not the chassis — "throttle" always drives
    /// toward the heading captured at construction time (or by the last
    /// resetFieldHeading() call), regardless of which way the chassis is
    /// currently facing. Internally rotates (throttle, strafe) into the
    /// chassis's current frame by the heading delta, then mixes exactly like
    /// holonomic(). `turn` is unaffected (rotation rate is frame-independent).
    void holonomicFieldCentric(double throttle, double strafe, double turn);

    /// Driver control with the turn stick steering a *heading* rather than a
    /// turn rate: `turnInput` sweeps a held heading (see HeadingHoldConfig),
    /// and headingHoldPID() drives the chassis onto it every tick — so releasing
    /// the stick leaves the chassis pointed somewhere definite instead of
    /// wherever momentum carried it, and anything that knocks it off that
    /// heading is corrected without the driver reacting.
    ///
    /// Additive to driving, not instead of it: the PID's turn output is
    /// mixed with `throttle`/`strafe` exactly the way a stick turn was, so
    /// the chassis translates and holds its heading at the same time. That's
    /// the real payoff on a holonomic chassis — a strafe that used to wander
    /// off-heading now gets straightened continuously.
    ///
    /// Call one of these every loop iteration while driving: the held
    /// heading advances per call, and a gap longer than a control loop's
    /// period (an autonomous routine, a PID tuner run, a calibration spin)
    /// re-adopts the chassis's current heading rather than snapping back to
    /// a target from before the interruption.
    ///
    /// Closes the loop with its own PID (headingHoldPID()), not the turn PID
    /// autonomous uses: holding a heading under a driver wants a softer
    /// response than snapping onto one in autonomous, and sharing one set of
    /// gains forces one of them to be wrong.
    void holonomicHeadingHold(double throttle, double strafe, double turnInput);

    /// holonomicHeadingHold() with field-centric translation — the
    /// combination most drivers want. `throttle`/`strafe` are relative to
    /// the field (see holonomicFieldCentric()) while `turnInput` steers the
    /// held heading (see holonomicHeadingHold()).
    void holonomicFieldCentricHeadingHold(double throttle, double strafe, double turnInput);

    /// Tunes how the turn stick steers the held heading. Optional — the
    /// defaults on HeadingHoldConfig are a sane starting point. Takes effect
    /// on the next holonomicHeadingHold() call.
    void setHeadingHold(HeadingHoldConfig config);

    /// The heading holonomicHeadingHold() is currently holding, in degrees.
    /// Only meaningful once one of those has been called; it tracks the
    /// chassis within HeadingHoldConfig::maxLeadDeg, so it's a useful thing
    /// to put on a GUI page next to the live heading.
    double heldHeadingDeg() const;

    /// Re-zeros the field-centric reference heading to the chassis's current
    /// IMU heading — whichever way the chassis is facing now becomes the new
    /// "throttle" direction for holonomicFieldCentric(). Typically bound to a
    /// driver button so they can redefine "forward" mid-match.
    ///
    /// Doesn't disturb the held heading: redefining which way "forward"
    /// translates has nothing to do with which way the chassis is pointed.
    void resetFieldHeading();

    /// Drives to field point (`xIn`, `yIn`), reading pose from `odometry`.
    /// Holds the heading the chassis had when the motion started, with the
    /// turn PID — a holonomic chassis can translate and rotate
    /// independently, so if you need a different final heading, use
    /// moveToPose() instead rather than chaining a turnToHeading() after
    /// this. Blocks until settled or timed out, then stops.
    void moveToPoint(double xIn, double yIn, const odom::Odometry& odometry, ExitConditions exit);

    /// Drives to field pose (`xIn`, `yIn`, `headingDeg`), reading pose from
    /// `odometry`. Unlike TankDrivetrain's boomerang controller, this just
    /// runs moveToPoint()'s translation control and turnToHeading()'s
    /// heading control at the same time — a holonomic chassis doesn't need
    /// the carrot-point trick since translation and rotation don't
    /// interfere with each other. Blocks until settled or timed out, then
    /// stops.
    void moveToPose(double xIn, double yIn, double headingDeg, const odom::Odometry& odometry,
                     motion::PoseExitConditions exit);

    /// Follows `path` using pure pursuit: repeatedly drives toward a point
    /// `config.lookaheadIn` ahead on the path, at constant cruise voltage,
    /// until within `config.finalApproachIn` of the path's last waypoint —
    /// then hands off to moveToPoint() for a controlled, settled stop
    /// there. Holds the heading the chassis had when the path started, all
    /// the way through that final approach. Blocks until the final
    /// moveToPoint() settles or times out.
    void followPath(const motion::Path& path, const odom::Odometry& odometry, motion::PursuitConfig config);

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
    /// callers implementing field-centric ("headless") driver control. Not
    /// const — see sensors::Imu::getHeadingDeg()'s call-frequency caveat.
    double headingDeg();

    /// The drivetrain's own calibrated IMU — exposed so you can share it
    /// with an externally-constructed odom::Odometry (via
    /// Odometry::Sensors::imu) instead of opening a second sensor object on
    /// the same physical port.
    sensors::Imu& imu();

    /// Exposes the internal PID controllers for live tuning (see
    /// gui::PidTunerPage) — adjusting gains through these takes effect
    /// immediately on the next update, since they read gains fresh each
    /// update() rather than caching them at construction. drivePID() and
    /// turnPID() drive autonomous motions; headingHoldPID() drives
    /// holonomicHeadingHold() and starts out with turnPID()'s constructor
    /// config until tuned separately.
    PID& drivePID();
    PID& turnPID();
    PID& headingHoldPID();

    /// Attaches the forward/back TrackingWheel (typically the same
    /// RotationTrackingWheel passed to Odometry::Sensors::vertical) that the
    /// Asterisk center wheels use to detect drift while strafing — see
    /// AsteriskConfig::driftCorrectionKP. No-op if the drivetrain wasn't
    /// constructed with an `asterisk` config. Reading a TrackingWheel from
    /// more than one object is safe (only *commanding* a device from more
    /// than one place would conflict). `odometry` supplies the wheel's
    /// OdometryConfig::verticalOffsetIn, read fresh every tick so a
    /// recalibration (e.g. OdometryPage's "Calibrate Offsets") applies
    /// immediately — without it, turning while strafing rolls an off-center
    /// wheel and reads as drift. nullptr treats the wheel as centered.
    void setDriftSource(const odom::TrackingWheel* verticalWheel,
                        const odom::Odometry* odometry = nullptr);

private:
    MotorGroup frontLeft_;
    MotorGroup frontRight_;
    MotorGroup backLeft_;
    MotorGroup backRight_;
    sensors::Imu imu_;
    DrivetrainConfig config_;
    PID drivePID_;
    PID turnPID_;
    PID headingHoldPID_;

    std::atomic<DriverInputMode> driverInputMode_{DriverInputMode::voltage};
    mutable pros::MutexVar<HolonomicAxisModels> axisModels_;

    /// Field-centric reference heading — see resetFieldHeading(). Set to the
    /// IMU heading at construction time, so holonomicFieldCentric() works out
    /// of the box without callers needing to call resetFieldHeading() first.
    double fieldHeadingZeroDeg_;

    /// Asterisk center wheels — unset (std::nullopt/empty) for a standard
    /// 4-motor holonomic chassis. See AsteriskConfig.
    std::optional<AsteriskConfig> asterisk_;
    std::optional<MotorGroup> middleLeft_;
    std::optional<MotorGroup> middleRight_;

    /// Drift-correction state — see setDriftSource() and setWheelVoltages().
    const odom::TrackingWheel* driftSource_ = nullptr;
    const odom::Odometry* driftOffsetSource_ = nullptr;
    double lastDriftVerticalIn_ = 0.0;
    double lastDriftStrafeIn_ = 0.0;
    double lastDriftHeadingDeg_ = 0.0;
    std::uint32_t lastDriftTickMs_ = 0;

    /// Thermal-compensation state — see AsteriskConfig::thermalCompensation
    /// and refreshThermalFractions(). Only the *fractions* are cached, since
    /// motor temperature moves over tens of seconds while setWheelVoltages()
    /// runs at 100Hz. The correction itself is recomputed every call, from
    /// that tick's actual corner voltages — which is the whole point: the
    /// same front-right derating means a forward/back shortfall while
    /// driving and an unwanted drift while strafing, and only the live
    /// command distinguishes them.
    ///
    /// Per corner, not averaged: a single derated corner and four evenly
    /// derated ones need opposite responses, and an average can't tell them
    /// apart.
    CornerValues thermalFractions_;
    double centerThermalFraction_ = 1.0;
    std::uint32_t lastThermalPollMs_ = 0;

    /// Heading-hold state — see holonomicHeadingHold(). lastHeadingHoldMs_
    /// of 0 means "not holding", which is also what a long enough gap
    /// between calls decays back to.
    HeadingHoldConfig headingHold_;
    double heldHeadingDeg_ = 0.0;
    std::uint32_t lastHeadingHoldMs_ = 0;

    double degreesToInches(double degrees) const;
    void setWheelVoltages(double frontLeft, double frontRight, double backLeft, double backRight);

    /// Rotates a field-relative (throttle, strafe) into the chassis's frame
    /// — see holonomicFieldCentric().
    void fieldToRobot(double& throttle, double& strafe);

    /// One translation or turn stick, [-1, 1], to axis volts per the current
    /// DriverInputMode and `model`.
    double stickVolts(double input, const MotorFeedforward& model) const;

    /// Sideways travel implied by the four corner encoders — the strafe
    /// cross-combination from setWheelVoltages(), applied to positions.
    double encoderStrafeIn() const;

    /// moveToPoint() holding an explicit heading, so followPath() can keep
    /// its starting heading through the final approach.
    void moveToPointHolding(double xIn, double yIn, double holdHeadingDeg,
                            const odom::Odometry& odometry, ExitConditions exit);

    /// Advances the held heading by one tick of `turnInput` and returns the
    /// heading-hold PID's output, in volts, for getting onto it.
    double headingHoldTurnVolts(double turnInput);

    /// Re-reads corner and center motor temperatures (at most every
    /// kThermalPollIntervalMs) into thermalFractions_/centerThermalFraction_.
    void refreshThermalFractions();
};

} // namespace sapphirelib::chassis
