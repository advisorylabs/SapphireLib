/**
 * \file examples/gui.cpp
 *
 * Phase 4 example: wiring up SapphireLib's default brain-screen GUI —
 * HomePage (status/telemetry), AutonSelectorPage (pick a routine before a
 * match), and OdometryPage (live pose visualization) — on top of a
 * holonomic chassis. Also shows the minimal odom::Odometry wiring needed to
 * feed OdometryPage, using drive encoders as the forward-distance fallback
 * (no dedicated tracking wheels) — see docs/ROADMAP.md Phase 2 for the
 * other three supported sensor configs.
 *
 * This file is illustrative, not built by the project (the Makefile only
 * compiles src/). Copy the relevant pieces into your own src/main.cpp.
 *
 * Team 96671H — Hitmen
 */

#include <memory>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::ExitConditions;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::chassis::MotorGroup;
using sapphirelib::gui::AutonSelectorPage;
using sapphirelib::gui::Gui;
using sapphirelib::gui::HomePage;
using sapphirelib::gui::OdometryPage;
using sapphirelib::odom::MotorGroupTrackingWheel;
using sapphirelib::odom::Odometry;
using sapphirelib::odom::OdometryConfig;
using sapphirelib::odom::Pose;

namespace {

constexpr std::uint8_t kImuPort = 10;
constexpr double kWheelDiameterIn = 4.0;

HolonomicDrivetrain* drivetrain = nullptr;
Odometry* odometry = nullptr;
AutonSelectorPage* autonSelector = nullptr;

void doNothingAuton() {}

void driveSquareAuton() {
    drivetrain->driveDistance(24.0, ExitConditions{.errorThreshold = 1.0});
    drivetrain->turnToHeading(90.0, ExitConditions{.errorThreshold = 2.0});
}

} // namespace

void initialize() {
    sapphirelib::initialize();

    static HolonomicDrivetrain chassis(
        /*frontLeftPort=*/1, /*frontRightPort=*/-2, /*backLeftPort=*/3, /*backRightPort=*/-4,
        Gearset::green, kImuPort,
        DrivetrainConfig{.wheelDiameterIn = kWheelDiameterIn, .externalGearRatio = 1.0,
                          .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0});
    drivetrain = &chassis;

    // Odometry needs a forward-distance source of its own — since
    // HolonomicDrivetrain doesn't expose its internal motor groups, this
    // reads the front-left drive motor a second time (safe: reading a
    // motor's position from more than one object is fine, only *commanding*
    // it from more than one would conflict) as the "IMU + drive encoders
    // only" fallback config. Swap in a RotationTrackingWheel instead if you
    // have a dedicated tracking wheel — see docs/ROADMAP.md Phase 2.
    static MotorGroup forwardEncoder({1}, Gearset::green);
    static MotorGroupTrackingWheel forwardWheel(forwardEncoder, kWheelDiameterIn);
    static Odometry odom(
        Odometry::Sensors{.imu = &chassis.imu(), .vertical = &forwardWheel, .horizontal = nullptr},
        OdometryConfig{.verticalOffsetIn = 0.0}, Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0});
    odometry = &odom;
    odometry->startTask();

    // SapphireLib's default brain-screen GUI. Entirely opt-in — skip this
    // block for your own UI, or to keep the screen blank.
    static Gui gui("SapphireLib - 96671H");

    gui.addPage(std::make_unique<HomePage>(&chassis.imu()));

    auto autonSelectorPage = std::make_unique<AutonSelectorPage>();
    autonSelector = autonSelectorPage.get();
    autonSelector->addRoutine("Do Nothing", &doNothingAuton);
    autonSelector->addRoutine("Drive Square", &driveSquareAuton);
    gui.addPage(std::move(autonSelectorPage));

    // fieldWidthIn/fieldHeightIn default to a 144x144in (12x12ft) VRC field
    // — pass your own for a different game/field size.
    gui.addPage(std::make_unique<OdometryPage>(*odometry));

    gui.start();
}

void opcontrol() {}

void autonomous() {
    if (autonSelector) autonSelector->run();
}
