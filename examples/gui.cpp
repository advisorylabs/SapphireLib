/**
 * \file examples/gui.cpp
 *
 * Phase 4 example: wiring up SapphireLib's default brain-screen GUI —
 * HomePage (status/telemetry), AutonSelectorPage (pick a routine before a
 * match), and OdometryPage (live pose visualization) — on top of a
 * holonomic chassis. Also shows the minimal odom::Odometry wiring needed to
 * feed OdometryPage and the pose motions, using drive encoders as the
 * forward-distance fallback (no dedicated tracking wheels) — see
 * docs/ROADMAP.md Phase 2 for the other three supported sensor configs.
 *
 * Not built into the program (the Makefile only compiles src/), but
 * `make check-examples` compiles it against the current headers so it can't
 * silently fall out of date. Copy the relevant pieces into your own
 * src/main.cpp.
 *
 * Team 96671H — Hitmen
 */

#include <memory>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
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

/// Built on the first call, which initialize() makes, rather than at namespace
/// scope: constructing a drivetrain blocks for ~2-3s while its IMU calibrates,
/// and static initialization runs before initialize() — before the program has
/// even started.
HolonomicDrivetrain& drivetrain() {
    static HolonomicDrivetrain instance(
        /*frontLeftPort=*/1, /*frontRightPort=*/-2, /*backLeftPort=*/3, /*backRightPort=*/-4,
        Gearset::green, kImuPort,
        DrivetrainConfig{.wheelDiameterIn = kWheelDiameterIn, .externalGearRatio = 1.0,
                         .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0});
    return instance;
}

/// Odometry needs a forward-distance source of its own — since
/// HolonomicDrivetrain doesn't expose its internal motor groups, this reads
/// the front-left drive motor a second time (safe: reading a motor's position
/// from more than one object is fine, only *commanding* it from more than one
/// would conflict) as the "IMU + drive encoders only" fallback config. Swap in
/// a RotationTrackingWheel instead if you have a dedicated tracking wheel —
/// see docs/ROADMAP.md Phase 2. It shares the drivetrain's IMU rather than
/// opening a second sensor object on the same port, so calling this also
/// builds the drivetrain first if nothing has yet.
Odometry& odometry() {
    static MotorGroup forwardEncoder({1}, Gearset::green);
    static MotorGroupTrackingWheel forwardWheel(forwardEncoder, kWheelDiameterIn);
    static Odometry instance(
        Odometry::Sensors{
            .imu = &drivetrain().imu(), .vertical = &forwardWheel, .horizontal = nullptr},
        OdometryConfig{.verticalOffsetIn = 0.0}, Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0});
    return instance;
}

// Owned by the Gui; kept here only so autonomous() can run the selection.
AutonSelectorPage* autonSelector = nullptr;

void doNothingAuton() {}

void driveSquareAuton() {
    drivetrain().driveDistance(24.0);
    drivetrain().turnToHeading(90.0);
}

void driveToPointsAuton() {
    // Field inches from where odometry started, heading 0 facing +y. These read
    // the pose from the odometry given to setOdometry() in initialize().
    if (!drivetrain().moveToPoint(0.0, 24.0).settled()) return;
    drivetrain().moveToPoint(24.0, 24.0);
}

} // namespace

void initialize() {
    sapphirelib::initialize();

    // SapphireLib's default brain-screen GUI. Entirely opt-in — leave out the
    // Gui lines for your own UI, or to keep the screen blank. Built before the
    // devices so its header is already showing while the IMU calibrates: a
    // hang or fault in device setup then reads as "initialize() stopped here"
    // rather than as a broken screen.
    static Gui gui("SapphireLib - 96671H");

    drivetrain(); // blocks while the IMU calibrates
    odometry().startTask();
    // The pose source for the moveToPoint()/moveToPose()/followPath() calls
    // that don't pass one. Set it here, before any motion can run.
    drivetrain().setOdometry(&odometry());

    gui.addPage(std::make_unique<HomePage>(&drivetrain().imu()));

    auto autonSelectorPage = std::make_unique<AutonSelectorPage>();
    autonSelector = autonSelectorPage.get();
    // The first routine runs if nobody picks one, so keep it harmless.
    autonSelector->addRoutine("Do Nothing", &doNothingAuton);
    autonSelector->addRoutine("Drive Square", &driveSquareAuton);
    autonSelector->addRoutine("Drive To Points", &driveToPointsAuton);
    gui.addPage(std::move(autonSelectorPage));

    // fieldWidthIn/fieldHeightIn default to a 144x144in (12x12ft) VRC field
    // — pass your own for a different game/field size.
    gui.addPage(std::make_unique<OdometryPage>(odometry()));

    gui.start();
}

void opcontrol() {}

void autonomous() {
    if (autonSelector) autonSelector->run();
}
