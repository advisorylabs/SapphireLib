/**
 * \file examples/localization.cpp
 *
 * Monte Carlo localization example: a holonomic chassis with two tracking
 * wheels (the "IMU + both wheels" odometry config) and four distance sensors,
 * one per side, watching the field walls. The localizer keeps odometry's pose
 * honest, so a long autonomous lands its last motion as well as its first.
 * Nothing about the motions changes: they read odometry's pose as always, and
 * the localizer corrects that pose underneath them.
 *
 * See docs/LOCALIZATION.md for how it works and how to tune it, and
 * tools/sim for a simulator running the same math, where every setting can be
 * tried without a robot.
 *
 * Not built into the program (the Makefile only compiles src/), but
 * `make check-examples` compiles it against the current headers so it can't
 * silently fall out of date. Copy the relevant pieces into your own
 * src/main.cpp.
 *
 * Team 96671H: Hitmen
 */

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::localization::DistanceSensorConfig;
using sapphirelib::localization::FieldMap;
using sapphirelib::localization::LocalizationStatus;
using sapphirelib::localization::LocalizerConfig;
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
        DrivetrainConfig{.wheelDiameterIn = 4.0, .externalGearRatio = 1.0,
                         .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0});
    return instance;
}

Odometry& odometry() {
    static RotationTrackingWheel vertical(11, 2.75);
    static RotationTrackingWheel horizontal(12, 2.75);
    static Odometry instance(
        Odometry::Sensors{
            .imu = &drivetrain().imu(), .vertical = &vertical, .horizontal = &horizontal},
        OdometryConfig{.verticalOffsetIn = 0.5, .horizontalOffsetIn = -3.0});
    return instance;
}

/// Four distance sensors, their faces measured from the tracking center:
/// inches forward (negative behind) and right (negative left), and which way
/// each faces relative to the robot's front. The map is the field's perimeter
/// with the origin in the middle; add fixed field structures with addBox() if
/// the sensors will see them, but leave out anything that gets pushed around.
MonteCarloLocalizer& localizer() {
    static MonteCarloLocalizer instance(
        odometry(),
        {
            DistanceSensorConfig{.port = 13, .mount = {.forwardIn = 6.5, .facingDeg = 0}},
            DistanceSensorConfig{.port = 14, .mount = {.rightIn = 7.0, .facingDeg = 90}},
            DistanceSensorConfig{.port = 15, .mount = {.forwardIn = -6.5, .facingDeg = 180}},
            DistanceSensorConfig{.port = 16, .mount = {.rightIn = -7.0, .facingDeg = 270}},
        },
        FieldMap::centered(),
        // the defaults suit most robots; this one trusts its sensors a bit less
        LocalizerConfig{.filter = {.beam = {.outlierProbability = 0.15}}});
    return instance;
}

} // namespace

void initialize() {
    sapphirelib::initialize();
    drivetrain(); // blocks while the IMU calibrates
    odometry().startTask();
    localizer().startTask(); // after odometry, which it reads
    drivetrain().setOdometry(&odometry());
}

void autonomous() {
    // where the robot really starts, in field coordinates. The localizer
    // notices the reset and starts its particles here
    odometry().setPose({.xIn = -36, .yIn = -60, .headingDeg = 0});

    // every motion drives by the corrected pose
    if (!drivetrain().moveToPoint(-36, -24).settled()) return;
    drivetrain().moveToPose(0, 0, 90);
    drivetrain().moveToPose(36, 24, 0);
    drivetrain().moveToPoint(-36, -60);
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    for (int tick = 0;; ++tick) {
        // what the localizer thinks, on the controller screen. Every 10th tick:
        // controllers drop text sent faster than about every 50ms
        if (tick % 10 == 0) {
            const LocalizationStatus status = localizer().status();
            master.print(0, 0, "MCL %.1fin %s   ", status.spreadIn,
                         status.correcting ? "ok" : "--");
        }

        drivetrain().holonomic(master.get_analog(ANALOG_LEFT_Y) / 127.0,
                               master.get_analog(ANALOG_LEFT_X) / 127.0,
                               master.get_analog(ANALOG_RIGHT_X) / 127.0);

        // A turns correcting off, B back on: drive the same route both ways
        // and compare where the robot ends up
        if (master.get_digital_new_press(DIGITAL_A)) localizer().setCorrectionEnabled(false);
        if (master.get_digital_new_press(DIGITAL_B)) localizer().setCorrectionEnabled(true);
        pros::delay(10);
    }
}
