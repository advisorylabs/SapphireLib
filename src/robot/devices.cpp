/**
 * \file devices.cpp
 *
 * Construction of the drivetrain, tracking wheels, and odometry, in the order
 * initialize() needs it. Ports and geometry come from config.hpp.
 *
 * Team 96671H: Hitmen
 */

#include "robot/devices.hpp"

#include "robot/config.hpp"
#include "robot/macros.hpp"
#include "robot/tune.hpp"
#include "sapphirelib/util/log.hpp"

namespace robot {

using sapphirelib::PID;
using sapphirelib::chassis::AsteriskConfig;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::localization::DistanceSensorConfig;
using sapphirelib::localization::FieldMap;
using sapphirelib::localization::MonteCarloLocalizer;
using sapphirelib::odom::Odometry;
using sapphirelib::odom::OdometryConfig;
using sapphirelib::odom::Pose;
using sapphirelib::odom::RotationTrackingWheel;

HolonomicDrivetrain& drivetrain() {
    // Asterisk drivetrain: the standard 4 mecanum/X-drive corners plus a
    // straight-facing 5th/6th center wheel pair (middle-left/middle-right)
    // that add power during forward/backward driving and correct forward/
    // back drift while strafing, see AsteriskConfig and setDriftSource()
    // in initDevices(). All right-side motors are physically mounted
    // reversed.
    static HolonomicDrivetrain instance(
        ports::kFrontLeft, ports::kFrontRight, ports::kBackLeft, ports::kBackRight, Gearset::green,
        ports::kImu,
        DrivetrainConfig{.wheelDiameterIn = kDriveWheelDiameterIn,
                         .externalGearRatio = 1.0,
                         .headingCorrectionKP = 0.4},
        // kI/kD are continuous-time gains (per second), not per control
        // tick, see PIDGains' comment. The kD values below are the
        // previous per-tick numbers (0.1 / 0.02) converted at the
        // drivetrain's 10ms loop period, so they behave identically; run
        // Auto-Tune to replace them with measured ones.
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0},
        /*imuHeadingScale=*/1.0,
        /*asterisk=*/
        // TODO: driftCorrectionKP starts conservative (volts per in/sec of
        // sensed drift), tune it up on the real robot until sideways drift
        // is corrected without the center wheels fighting an intentional
        // strafe.
        //
        // turnContribution = 1.0 gives the center wheels full rotational
        // authority, so they drive every turn instead of coasting through
        // it. These are 5.5W motors against 11W corners; if they end up
        // saturating and dragging on fast turns, walk this down rather than
        // to 0.
        //
        // thermalCompensation = 1.0 has the center wheels make up whatever the
        // corner motors stop delivering as they heat up, both the plain loss
        // of speed when the corners derate together, and the forward/back
        // drift and twist that one derated corner throws into a strafe.
        // maxThermalCorrectionVolts caps how much they'll be asked for on top
        // of their normal share; 6V is half a motor's range, which leaves the
        // smaller 5.5W center motors margin before they're the ones
        // overheating. Works alongside driftCorrectionKP above, not instead of
        // it: this corrects ahead of the drift, that one cleans up what's left.
        AsteriskConfig{.middleLeftPort = ports::kMiddleLeft,
                       .middleRightPort = ports::kMiddleRight,
                       .driftCorrectionKP = 0.5,
                       .turnContribution = 1.0,
                       .thermalCompensation = 1.0,
                       .maxThermalCorrectionVolts = 6.0});
    return instance;
}

RotationTrackingWheel& verticalWheel() {
    // Dedicated tracking wheels on their own rotation sensors, the "IMU +
    // both wheels" odometry config (see Odometry's class comment).
    //
    // Vertical is reversed (negative port): on this chassis, the sensor's
    // raw positive direction is backward, confirmed by driving forward and
    // watching Y decrease instead of increase on the Odom page, so this
    // negation isn't a stylistic choice, it's required for Odometry's
    // forward/backward sign to actually match the chassis's forward
    // direction. If you re-mount or swap this sensor, re-check this.
    static RotationTrackingWheel instance(ports::kVerticalTracking, kTrackingWheelDiameterIn);
    return instance;
}

RotationTrackingWheel& horizontalWheel() {
    static RotationTrackingWheel instance(ports::kHorizontalTracking, kTrackingWheelDiameterIn);
    return instance;
}

Odometry& odometry() {
    // Calls drivetrain() and the wheel accessors itself, so the devices it
    // reads always exist first, even if something asks for odometry() before
    // initDevices() has run.
    static Odometry instance(
        Odometry::Sensors{
            .imu = &drivetrain().imu(), .vertical = &verticalWheel(), .horizontal = &horizontalWheel()},
        OdometryConfig{.verticalOffsetIn = kVerticalWheelOffsetIn,
                       .horizontalOffsetIn = kHorizontalWheelOffsetIn},
        // Also sets the shared IMU's field heading, so whichever way the
        // chassis faces at startup is heading 0 for turns too. An autonomous
        // routine that starts elsewhere calls odometry().setPose() first.
        Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0});
    return instance;
}

MonteCarloLocalizer& localizer() {
    // Four distance sensors, one per side, watching the field walls. The
    // localizer moves its particles by odometry's own measured travel, weighs
    // them by how well they explain the four readings, and eases odometry's
    // pose toward the answer (Odometry::setPositionCorrection()), so every
    // moveToPoint()/moveToPose()/followPath() drives by the corrected pose
    // with the same Auto-Tuned PIDs. Heading stays the IMU's.
    //
    // The map is just the perimeter, origin in the middle of the field.
    // Game elements get pushed around, and one in the map where it no longer
    // is does more harm than one missing from it: a reading off something
    // that isn't in the map is treated as an outlier, and the other sensors
    // carry on. Try a change in the simulator (tools/sim) before the robot.
    //
    // The settings are LocalizerConfig's defaults with TUNE.CFG's mcl.* lines
    // on top (see tune.hpp): what the simulator's tuner found, and what the
    // telemetry analyzer refined from real matches, without a rebuild.
    static MonteCarloLocalizer instance(
        odometry(),
        {
            DistanceSensorConfig{.port = ports::kDistanceFront,
                                 .mount = {.forwardIn = kFrontSensorForwardIn,
                                           .rightIn = kFrontSensorRightIn,
                                           .facingDeg = 0.0}},
            DistanceSensorConfig{.port = ports::kDistanceRight,
                                 .mount = {.forwardIn = kRightSensorForwardIn,
                                           .rightIn = kRightSensorRightIn,
                                           .facingDeg = 90.0}},
            DistanceSensorConfig{.port = ports::kDistanceBack,
                                 .mount = {.forwardIn = kBackSensorForwardIn,
                                           .rightIn = kBackSensorRightIn,
                                           .facingDeg = 180.0}},
            DistanceSensorConfig{.port = ports::kDistanceLeft,
                                 .mount = {.forwardIn = kLeftSensorForwardIn,
                                           .rightIn = kLeftSensorRightIn,
                                           .facingDeg = 270.0}},
        },
        FieldMap::centered(), localizerSettings());
    return instance;
}

void initDevices() {
    // Blocks until the IMU finishes calibrating (~2-3s).
    HolonomicDrivetrain& chassis = drivetrain();
    if (chassis.imu().calibrated()) {
        SAPPHIRELIB_LOG_INFO("init", "chassis ready (IMU calibrated)");
    } else {
        // Headings read 0 until the IMU answers, so turns, heading hold and
        // odometry would all be quietly wrong. Say so here, in the one place
        // anyone reading the terminal at startup will look.
        SAPPHIRELIB_LOG_ERROR("init", "chassis ready, but the IMU on port %u didn't calibrate",
                              static_cast<unsigned>(ports::kImu));
    }
    // HolonomicDrivetrain zeros its field heading at construction time, so
    // the field-centric driver control in driver.cpp is already field-centric
    // from here on.

    odometry().startTask();
    SAPPHIRELIB_LOG_INFO("init", "odometry task started");

    // After odometry, which it reads. It corrects nothing until an auton's
    // setPose() puts the pose in field coordinates (LocalizerConfig::
    // waitForSetPose): before that, the pose is relative to wherever the robot
    // sat at startup, and the particles can only hunt for the robot, which in
    // the simulator briefly latched onto the wrong spot on the way.
    localizer().startTask(localizerPeriodMs());
    SAPPHIRELIB_LOG_INFO("init", "localizer task started");

    // Center wheels read the same vertical tracking wheel Odometry uses to
    // detect forward/back drift while strafing, reading a sensor from two
    // places is safe, only commanding a motor from two places would
    // conflict. Passing odometry lets drift correction read the wheel's
    // (live-recalibratable) verticalOffsetIn, so turning while strafing
    // doesn't read as drift.
    chassis.setDriftSource(&verticalWheel(), &odometry());

    // The pose source for moveToPoint(x, y) and the other motions that don't
    // take one (see autons.cpp). Set here, before anything can run a motion,
    // because it isn't synchronized.
    chassis.setOdometry(&odometry());

    // Intake/claw/lift driver macros, see macros.hpp. Zeroes the lift here,
    // so start the program with the lift all the way down.
    macros::initialize();

    // TUNE.CFG's gains, axis models and lift gravity, over the values above
    // and in macros.cpp, now that everything it sets exists and before any
    // task runs a PID. A missing or rejected file changes nothing.
    applyTuneProfile();
}

} // namespace robot
