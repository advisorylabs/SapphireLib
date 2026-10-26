/**
 * \file examples/autonomous.cpp
 *
 * Writing autonomous routines with sapphirelib/auton/auton.hpp on a holonomic
 * chassis with two tracking wheels and four distance sensors: use() once in
 * initialize(), routines as plain lists of calls, the brain screen's selector
 * remembering the pick, and a background motion timing an intake. See
 * docs/QUICKSTART.md for every function.
 *
 * Not built into the program (the Makefile only compiles src/), but
 * `make check-examples` compiles it against the current headers so it can't
 * silently fall out of date.
 *
 * Team 96671H: Hitmen
 */

#include "main.h"
#include "sapphirelib/api.hpp"

using namespace sapphirelib::auton;
using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::gui::AutonSelectorPage;
using sapphirelib::gui::Gui;
using sapphirelib::localization::DistanceSensorConfig;
using sapphirelib::localization::FieldMap;
using sapphirelib::localization::MonteCarloLocalizer;
using sapphirelib::odom::Odometry;
using sapphirelib::odom::OdometryConfig;
using sapphirelib::odom::RotationTrackingWheel;

namespace {

/// Built on the first call, from initialize(): the constructor blocks while
/// the IMU calibrates.
HolonomicDrivetrain& drivetrain() {
    static HolonomicDrivetrain instance(
        /*frontLeftPort=*/1, /*frontRightPort=*/-2, /*backLeftPort=*/3, /*backRightPort=*/-4,
        Gearset::green, /*imuPort=*/10,
        DrivetrainConfig{.wheelDiameterIn = 4.0, .externalGearRatio = 1.0},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0});
    return instance;
}

Odometry& odometry() {
    static RotationTrackingWheel vertical(11, 2.75);
    static RotationTrackingWheel horizontal(12, 2.75);
    static Odometry instance(Odometry::Sensors{.imu = &drivetrain().imu(),
                                               .vertical = &vertical,
                                               .horizontal = &horizontal},
                             OdometryConfig{.verticalOffsetIn = 0.5, .horizontalOffsetIn = -3.0});
    return instance;
}

/// One distance sensor per side, for the localizer and the wall resets.
MonteCarloLocalizer& localizer() {
    static MonteCarloLocalizer instance(
        odometry(),
        {
            DistanceSensorConfig{.port = 13, .mount = {.forwardIn = 6.5, .facingDeg = 0}},
            DistanceSensorConfig{.port = 14, .mount = {.rightIn = 7.0, .facingDeg = 90}},
            DistanceSensorConfig{.port = 15, .mount = {.forwardIn = -6.5, .facingDeg = 180}},
            DistanceSensorConfig{.port = 16, .mount = {.rightIn = -7.0, .facingDeg = 270}},
        },
        FieldMap::centered());
    return instance;
}

pros::Motor intake(5);
pros::Distance intakeSensor(6);

// Owned by the Gui; kept here only so autonomous() can run the selection.
AutonSelectorPage* autons = nullptr;

// --- The routines ---

// Two goals on the right: drive up the side, score, then swing over to the
// second. Every routine starts with setPose() at the robot's real spot.
void rightSide() {
    setPose(48, -60, 0);
    // run the intake from 12in into the drive until something's in it, then
    // stop driving there: a background motion, cancelled from the sensor
    runAsync([] { moveToPoint(48, -12); });
    waitUntilTraveled(12);
    intake.move_voltage(12000);
    sapphirelib::waitUntil(
        [] { return !isMotionRunning() || intakeSensor.get_distance() < 50; }, 4000);
    cancelMotion();
    waitForMotion();
    intake.move_voltage(0);

    // a routine can stop instead of driving on from a spot it never reached
    if (!moveToPose(24, -24, 270).settled()) return;
    resetRight(); // x from the wall the right sensor sees
    turnTo(90, clockwise);
    moveToPoint(48, -24, {.maxVolts = 8});
}

// Skills: a few motions chained without stopping between them
void skills() {
    setPose(-48, -60, 0);
    moveToPoint(-48, -24, {.chain = true});
    moveToPoint(-24, 0, {.chain = true});
    moveToPoint(0, 24);
    turnToPoint(48, 48);
    followPath({{0, 24}, {24, 36}, {48, 48}});
}

} // namespace

void initialize() {
    sapphirelib::initialize();
    drivetrain(); // blocks while the IMU calibrates
    odometry().startTask();
    localizer().startTask();
    // every auton function now drives this chassis
    use(drivetrain(), odometry(), &localizer(), {.wallReset = {.cornerMarginIn = 18.0}});

    // static, built here rather than at namespace scope: it draws on the screen
    static Gui gui("SapphireLib");
    auto selector = std::make_unique<AutonSelectorPage>();
    autons = selector.get();
    autons->addRoutine("Right Side", rightSide);
    autons->addRoutine("Skills", skills);
    autons->rememberSelection(); // the pick survives a restart
    gui.addPage(std::move(selector));
    gui.start();
}

void autonomous() {
    if (autons != nullptr) autons->run();
    waitForMotion();
    drivetrain().stop();
}
