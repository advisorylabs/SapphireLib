/**
 * \file examples/trajectory.cpp
 *
 * Planned trajectories on a holonomic chassis with two tracking wheels: a
 * chain of sections drawn in Sapphire Sim's Paths tab and pasted in with
 * Copy C++, planned at the start of autonomous from the axis models Auto-Tune
 * measured, and driven with followTrajectory(). A MotionQueue runs it in the
 * background so the routine can start a mechanism as the robot enters the
 * second section, without slowing down. See docs/TRAJECTORIES.md.
 *
 * Not built into the program (the Makefile only compiles src/), but
 * `make check-examples` compiles it against the current headers so it can't
 * silently fall out of date.
 *
 * Team 96671H: Hitmen
 */

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::motion::TrajectoryResult;
using sapphirelib::motion::TrajectorySection;
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

pros::Motor intake(5);

// Sapphire Sim path "rush": full speed up the left side turning to face the goal, then slower
// through the intake, coming to rest facing it
const std::vector<TrajectorySection> kRush{
    {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},
                   {.xIn = -50, .yIn = -24},
                   {.xIn = -30, .yIn = 0, .headingDeg = 90}}},
    {.waypoints = {{.xIn = 0, .yIn = 12}, {.xIn = 24, .yIn = 20}},
     .constraints = {.voltageBudget = 7, .strafe = {.maxVelocity = 30}}},
};

} // namespace

void initialize() {
    sapphirelib::initialize();
    drivetrain(); // blocks while the IMU calibrates
    odometry().startTask();
    drivetrain().setOdometry(&odometry());
    // The models the planner and the feedforward use: what Auto-Tune measured (its readout and the
    // SD log print them), or load them from TUNE.CFG. Without them followTrajectory() aborts
    drivetrain().setAxisModels({
        .forward = {.kS = 0.97, .kV = 0.20, .kA = 0.045},
        .strafe = {.kS = 1.28, .kV = 0.24, .kA = 0.055},
        .turn = {.kS = 0.55, .kV = 0.045, .kA = 0.0067},
    });
}

void autonomous() {
    // static: the queue's task, and the trajectory it reads, outlive this call
    static sapphirelib::motion::MotionQueue queue;
    static TrajectoryResult rush;

    // a few milliseconds, once per routine: plan before driving, never inside a loop
    rush = sapphirelib::motion::generateTrajectory(kRush, drivetrain().axisModels());
    if (!rush.ok) {
        printf("rush: %s\n", rush.error.c_str());
        return;
    }

    // the trajectory's first point, where the robot really starts
    odometry().setPose({.xIn = -48, .yIn = -60, .headingDeg = 0});
    // the number rush's motion will have: until it starts, trajectoryProgress() is still whatever
    // trajectory ran last, which may well be past section 1
    const std::uint32_t rushMotion = drivetrain().trajectoryProgress().motion + 1;
    queue.enqueue([] {
        // waits up to 4in behind the plan before the plan waits for it; times out 1s past the plan
        drivetrain().followTrajectory(rush.trajectory, {.overtimeMs = 1000, .maxLagIn = 4.0});
    });
    queue.run();

    // as the robot enters the slower second section, start the intake, without the robot slowing
    if (sapphirelib::waitUntil(
            [rushMotion] {
                const auto progress = drivetrain().trajectoryProgress();
                return progress.motion == rushMotion && progress.section >= 1;
            },
            4000)) {
        intake.move_voltage(12000);
    }
    queue.waitUntilDone();
    intake.move_voltage(0);
}
