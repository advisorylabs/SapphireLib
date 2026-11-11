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
#include "robot/vision.hpp"
#include "sapphirelib/auton/auton.hpp"
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
    // in initDevices(). The two sides' motors are mounted mirrored, so one
    // side's ports are reversed in config.hpp.
    //
    // All six wheels are 2in and spin at 600rpm: the corners on blue
    // cartridges, direct, and the center pair on 5.5W motors (fixed 200rpm)
    // geared up 3:1, so the center wheels keep pace with the corners. The
    // gearset must match the corners' cartridges, since driveDistance() and
    // drift correction turn their encoder degrees into inches. The center
    // motors take it too, but they're only ever driven by voltage and read
    // for temperature, so it doesn't matter that theirs is really 200rpm.
    static HolonomicDrivetrain instance(
        ports::kFrontLeft, ports::kFrontRight, ports::kBackLeft, ports::kBackRight, Gearset::blue,
        ports::kImu,
        // The min*Volts push a motion on until it's first within its exit
        // threshold. Each is over what starts the robot moving that way from
        // rest, more than the axis model's kS (measured moving, see
        // initDevices()): a turn sat 5.5 degrees short at 1.5V until it timed
        // out (SL000028), and 1.6V took 0.9s over the last 6 (SL000022); 2.5V
        // finishes 90 degree turns 1 degree short. The center wheels drag
        // sideways: a move sat 1in off sideways at 2.2V (SL000028), 2.5V didn't
        // strafe either (SL000024), and 3V only crept, about 1in/s, so two
        // moves spent 2-3s on the last 1.5in (SL000029). Lower one if its
        // motions overshoot the threshold, raise it if they still stall.
        DrivetrainConfig{.wheelDiameterIn = kDriveWheelDiameterIn,
                         .externalGearRatio = 1.0,
                         .headingCorrectionKP = 0.4,
                         .minForwardVolts = 1.0,
                         .minStrafeVolts = 4.0,
                         .minTurnVolts = 2.5},
        // Auto-Tune's gains for the PID page's Drive and Turn entries,
        // designed from the axis models in initDevices(). kI/kD are
        // continuous-time gains (per second), not per control tick, see
        // PIDGains' comment. The heading hold and tracking PIDs start from
        // these too, and initDevices() gives them their own.
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 2.086, .kI = 0.0, .kD = 0.396}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.2587, .kI = 0.0, .kD = 0.0125}, .outputLimit = 12.0},
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
    // Vertical is reversed (negative port): on the test chassis, the
    // sensor's raw positive direction was backward, confirmed by driving
    // forward and watching Y decrease instead of increase on the Odom page,
    // so this negation isn't a stylistic choice, it's required for
    // Odometry's forward/backward sign to actually match the chassis's
    // forward direction. The competition robot keeps it until checked the
    // same way (see config.hpp); re-check it whenever this sensor is
    // re-mounted or swapped.
    static RotationTrackingWheel instance(ports::kVerticalTracking, kVerticalWheelDiameterIn);
    return instance;
}

RotationTrackingWheel& horizontalWheel() {
    static RotationTrackingWheel instance(ports::kHorizontalTracking, kHorizontalWheelDiameterIn);
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

namespace {

// What the localizer's sensors can see: the perimeter, and Override's four
// loaders, fixed to red's and blue's walls. Each reaches 3.74in in from its
// wall, is 4.02in wide, and is centered 11.44in from an end wall (the
// manual's Appendix A). With only the walls to explain the front sensor
// reading a loader's face, the localizer put the robot 3.7in nearer the wall
// than it was (SL000028). A sensor sees a loader well off its beam's center,
// which the localizer models as the beam's cone (BeamModel::coneHalfAngleDeg).
// Only the faces a sensor out on the field can see: the back one is the
// wall's.
//
// And what else the sensors see that stays put through autonomous, from the
// field as measured: the center goal (8.77in tall, so the sensors see the
// goal itself), the Pin standing in each medium (neutral) goal and in each
// of the Midfield's four Cups, and the Pins stacked on the alliance goal the
// Triple Pin routine scores on, which hold its preload from its first score
// on. The sensors are mounted above an empty alliance or medium goal, so a
// goal is seen only by the Pin in it, and a goal without one is left out.
// Each of them is a believable wall in the wrong place to a localizer that
// doesn't know it's there. The Cups and Pins standing on their own, and the
// wall groups, get picked up and pushed around, and stay out.
//
// The Toggles, on top of the middle of every wall, read the wall 3 to 4in
// long, not short (SL000040 head on; SL000070's back sensor 4in long at a
// slant, which pulled the pose 1.2in east and 3.8in south on the drive to
// the second Pin): blind stretches, so the localizer leaves out a reading
// whose beam lands on one (FieldMap::addBlindStretch()).
FieldMap fieldMap() {
    constexpr double kHalfIn = sapphirelib::localization::kVrcFieldSizeIn / 2.0;
    constexpr double kLoaderDepthIn = 3.74;
    constexpr double kLoaderFromEndWallIn = 11.44;
    constexpr double kLoaderHalfWidthIn = 4.02 / 2.0;
    FieldMap map = FieldMap::centered();
    for (const double side : {-1.0, 1.0}) {    // red's wall, then blue's
        for (const double end : {-1.0, 1.0}) { // the near end, then the far one
            const double wallX = side * kHalfIn;
            const double faceX = side * (kHalfIn - kLoaderDepthIn);
            const double nearY = end * (kHalfIn - kLoaderFromEndWallIn) - kLoaderHalfWidthIn;
            const double farY = nearY + 2.0 * kLoaderHalfWidthIn;
            map.addSegment({faceX, nearY, faceX, farY});
            map.addSegment({wallX, nearY, faceX, nearY});
            map.addSegment({wallX, farY, faceX, farY});
        }
    }
    // Goals and Cups sit on tile corners, 23.545in apart (the manual's Appendix A)
    constexpr double kTileIn = 23.545;
    const auto square = [&map](double xIn, double yIn, double acrossIn) {
        const double half = acrossIn / 2.0;
        map.addBox(xIn - half, yIn - half, xIn + half, yIn + half);
    };
    constexpr double kGoalAcrossIn = 5.61; // across the flats
    constexpr double kCupAcrossIn = 3.15;
    constexpr double kPinAcrossIn = 1.6;
    square(0.0, 0.0, kGoalAcrossIn); // the center goal
    constexpr double kMediumGoals[][2] = {{-1.0, 2.0}, {-2.0, 1.0}, {2.0, -1.0}, {1.0, -2.0}};
    for (const auto& goal : kMediumGoals) {
        square(goal[0] * kTileIn, goal[1] * kTileIn, kPinAcrossIn); // a medium goal's Pin
    }
    constexpr double kMidfieldCups[][2] = {{-1.0, 0.0}, {0.0, -1.0}, {0.0, 1.0}, {1.0, 0.0}};
    for (const auto& cup : kMidfieldCups) {
        square(cup[0] * kTileIn, cup[1] * kTileIn, kCupAcrossIn); // a Midfield Cup and its Pin
    }
    square(-2.0 * kTileIn, -kTileIn, kPinAcrossIn); // the Triple Pin's stack, on red's goal
    constexpr double kToggleHalfIn = 25.99 / 2.0;
    map.addBlindStretch({-kHalfIn, -kToggleHalfIn, -kHalfIn, kToggleHalfIn}); // red's wall
    map.addBlindStretch({kHalfIn, -kToggleHalfIn, kHalfIn, kToggleHalfIn});   // blue's
    map.addBlindStretch({-kToggleHalfIn, -kHalfIn, kToggleHalfIn, -kHalfIn}); // the near wall
    map.addBlindStretch({-kToggleHalfIn, kHalfIn, kToggleHalfIn, kHalfIn});   // the far wall
    return map;
}

} // namespace

MonteCarloLocalizer& localizer() {
    // Four distance sensors, one per side, watching the field walls. The
    // localizer moves its particles by odometry's own measured travel, weighs
    // them by how well they explain the four readings, and eases odometry's
    // pose toward the answer (Odometry::setPositionCorrection()), so every
    // moveToPoint()/moveToPose()/followPath() drives by the corrected pose
    // with the same Auto-Tuned PIDs. Heading stays the IMU's.
    //
    // The map is the perimeter and the four loaders (fieldMap() above), origin
    // in the middle of the field. Game elements get pushed around, and one in
    // the map where it no longer is does more harm than one missing from it:
    // a reading off something that isn't in the map is treated as an outlier,
    // and the other sensors carry on. Try a change in the simulator
    // (tools/sim) before the robot.
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
        fieldMap(), localizerSettings());
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
    // take one. Set here, before anything can run a motion, because it isn't
    // synchronized.
    chassis.setOdometry(&odometry());

    // The auton functions autons.cpp writes routines with (drive(),
    // moveToPoint(), resetBack(), ..., sapphirelib/auton/auton.hpp) drive
    // this drivetrain by this odometry, and reset off the walls with the
    // localizer's sensors. Override's loaders sit near the corners, on the
    // side walls, 3.74in deep, so a wall reset stays 16in clear of a corner:
    // the Triple Pin's back sensor, below the goal, landed its beam 18.7in from
    // the bottom-left corner when that routine reset there, and the 18in this
    // was dropped it a degree or an inch off (the simulator finds that cone clean, loader and all, an
    // inch and 3 degrees either way, at 16). A reset may move the robot up
    // to 60in, near enough no cap: odometry can drift past 8in over a
    // routine (SL000066, about 13in by the third Pin), and the old 8in cap
    // then threw out the very reset meant to fix it. Only a reading no drift
    // could explain is skipped, so keep a game element out of the beam.
    sapphirelib::auton::use(chassis, odometry(), &localizer(),
                            {.wallReset = {.cornerMarginIn = 16.0, .maxJumpIn = 60.0}});

    // Lift/claw/side piston driver macros, see macros.hpp. Zeroes the lift's
    // encoders here, and driver control zeroes them again on every press of
    // the bottom switch.
    macros::initialize();

    // The AI Vision sensor, set up to see the yellow Pins (vision.hpp). Only
    // a routine steering by it would read it, and none does now.
    vision::initialize();

    // Auto-Tune's measured axis models (its readout and the SD log print
    // them), for velocity sticks and trajectories. TUNE.CFG's model.* lines
    // override them below.
    chassis.setAxisModels({
        .forward = {.kS = 0.30, .kV = 0.1698, .kA = 0.0383},
        .strafe = {.kS = 2.21, .kV = 0.3234, .kA = 0.0838},
        .turn = {.kS = 1.10, .kV = 0.0318, .kA = 0.0019},
    });

    // Auto-Tune's gains for the PID page's other entries (Drive and Turn are
    // in drivetrain()). Hold's kD is 0 by design: at its slow 0.9s settle,
    // the turn axis's own drag already damps it. TUNE.CFG's pid.* lines
    // override these below.
    chassis.headingHoldPID().setGains({.kP = 0.0798, .kI = 0.0, .kD = 0.0});
    chassis.trackingForwardPID().setGains({.kP = 2.086, .kI = 0.0, .kD = 0.396});
    // Strafe by hand, not Auto-Tune's 17.817/2.120: that designs each Track axis for the same
    // 0.4s response from its linear model, and this axis has 2.2V of static friction the design
    // doesn't see. Saturating at 0.7in of error, it banged between +12V and -12V every 0.1-0.2s
    // with under 1.2in of error (SL000069: the zig-zag into the second Pin, and the scatter left
    // and right of it from run to run). The simulator, on the robot's models, reverses half as
    // often at 6.0, and the Triple Pin on trajectories still landed all three Pins within 0.3in.
    // An Auto-Tune saved to TUNE.CFG overrides this, so leave pid.trackStrafe out of it.
    chassis.trackingStrafePID().setGains({.kP = 6.0, .kI = 0.0, .kD = 1.1});
    chassis.trackingTurnPID().setGains({.kP = 0.405, .kI = 0.0, .kD = 0.024});

    // TUNE.CFG's gains, axis models and lift gravity, over the values above
    // and in macros.cpp, now that everything it sets exists and before any
    // task runs a PID. A missing or rejected file changes nothing.
    applyTuneProfile();
}

} // namespace robot
