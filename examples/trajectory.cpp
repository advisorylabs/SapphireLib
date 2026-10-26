/**
 * \file examples/trajectory.cpp
 *
 * Planned trajectories on a holonomic chassis with two tracking wheels: a
 * chain of sections drawn in Sapphire Sim's Paths tab and pasted in with
 * Copy C++, driven with the auton functions' followTrajectory() while the
 * routine starts a mechanism at one of its points, without the robot slowing
 * down. Then branching, which needs the drivetrain's own followTrajectory():
 * picking between plans made up front, with a stop at the decision point or
 * at speed. See docs/TRAJECTORIES.md.
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
using sapphirelib::motion::generateTrajectory;
using sapphirelib::motion::Trajectory;
using sapphirelib::motion::TrajectoryOptions;
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
pros::Distance front(6);

// --- A trajectory, with a mechanism timed to one of its points ---

// Sapphire Sim path "rush": full speed up the left side turning to face the goal, then slower
// through the intake, coming to rest facing it. Its points are numbered in order across the
// sections: 0 to 2 in the first, 3 and 4 in the second
const std::vector<TrajectorySection> kRush{
    {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},
                   {.xIn = -50, .yIn = -24},
                   {.xIn = -30, .yIn = 0, .headingDeg = 90}}},
    {.waypoints = {{.xIn = 0, .yIn = 12}, {.xIn = 24, .yIn = 20}},
     .constraints = {.voltageBudget = 7, .strafe = {.maxVelocity = 30}}},
};

void rush() {
    // the trajectory's first point, where the robot really starts
    setPose(-48, -60, 0);
    // a few milliseconds, once: plan before driving rather than as it starts
    if (!planTrajectory(kRush)) return; // the terminal says why
    runAsync([] { followTrajectory(kRush, {.overtimeMs = 1000}); });
    // as the robot enters the slower second section (point 2, where the first ends), start the
    // intake, without the robot slowing down
    if (waitUntilWaypoint(2)) intake.move_voltage(12000);
    waitForMotion();
    intake.move_voltage(0);
}

// --- Branching (docs/TRAJECTORIES.md#branching) ---
//
// Two ways to pick between paths on a sensor, both robot-relative: the routine sets where the
// robot sits as (0, 0) facing 0. The front sensor picks the branch: something within 24in, swerve
// right; clear, go left. Each branch, and the way home from it, is planned at the start with
// TrajectoryOptions::from, so every piece starts exactly where the one before it ends. These use
// the drivetrain's followTrajectory() directly: plans it must be able to read to the end, so
// static

// Plans one piece, from whatever axis models are installed, and says why if it can't
bool plan(const char* name, TrajectoryResult& out, const std::vector<TrajectorySection>& sections,
          const TrajectoryOptions& options = {}) {
    out = generateTrajectory(sections, drivetrain().axisModels(), options);
    if (!out.ok) printf("%s: %s\n", name, out.error.c_str());
    return out.ok;
}

bool blocked() { return front.get_distance() < 610; } // 24in

const std::vector<TrajectorySection> kHome{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}}}};

// With a stop: drive to the decision point and stop there, read the sensor, then take the
// branch. Works with any trajectory; costs the stop
const std::vector<TrajectorySection> kTrunk{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 24}}}};
const std::vector<TrajectorySection> kRight{
    {.waypoints = {{.xIn = 12, .yIn = 38, .headingDeg = 45}}}};
const std::vector<TrajectorySection> kLeft{
    {.waypoints = {{.xIn = -12, .yIn = 38, .headingDeg = -45}}}};

void branchWithStop() {
    static TrajectoryResult trunk, right, left, homeFromRight, homeFromLeft;
    if (!plan("trunk", trunk, kTrunk) || !plan("right", right, kRight, {.from = &trunk.trajectory}) ||
        !plan("left", left, kLeft, {.from = &trunk.trajectory}) ||
        !plan("home", homeFromRight, kHome, {.from = &right.trajectory}) ||
        !plan("home", homeFromLeft, kHome, {.from = &left.trajectory})) {
        return;
    }
    setPose(0, 0, 0);
    if (!drivetrain().followTrajectory(trunk.trajectory).settled()) return;
    const bool swerve = blocked();
    if (!drivetrain().followTrajectory(swerve ? right.trajectory : left.trajectory).settled()) {
        return;
    }
    drivetrain().followTrajectory(swerve ? homeFromRight.trajectory : homeFromLeft.trajectory);
}

// At speed: the trunk ends at 24in/s instead of at rest, each branch carries on from it at that
// speed, and next reads the sensor as the robot reaches the junction, on the motion's task: one
// quick read and a pick between plans already made
const std::vector<TrajectorySection> kSpeedTrunk{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 30}}}};
const std::vector<TrajectorySection> kSpeedRight{
    {.waypoints = {{.xIn = 14, .yIn = 50, .headingDeg = 45}}}};
const std::vector<TrajectorySection> kSpeedLeft{
    {.waypoints = {{.xIn = -14, .yIn = 50, .headingDeg = -45}}}};

void branchAtSpeed() {
    static TrajectoryResult trunk, right, left, homeFromRight, homeFromLeft;
    if (!plan("trunk", trunk, kSpeedTrunk, {.endSpeedInPerS = 24}) ||
        !plan("right", right, kSpeedRight, {.from = &trunk.trajectory}) ||
        !plan("left", left, kSpeedLeft, {.from = &trunk.trajectory}) ||
        !plan("home", homeFromRight, kHome, {.from = &right.trajectory}) ||
        !plan("home", homeFromLeft, kHome, {.from = &left.trajectory})) {
        return;
    }
    setPose(0, 0, 0);
    // which branch next picked, for the way home; null if the trunk never got to the junction
    static std::atomic<const Trajectory*> took{nullptr};
    took.store(nullptr);
    const auto result = drivetrain().followTrajectory(
        trunk.trajectory, {.next = [](const Trajectory&) -> const Trajectory* {
            const Trajectory* branch = blocked() ? &right.trajectory : &left.trajectory;
            took.store(branch);
            return branch;
        }});
    const Trajectory* branch = took.load();
    if (!result.settled() || branch == nullptr) return;
    drivetrain().followTrajectory(branch == &right.trajectory ? homeFromRight.trajectory
                                                              : homeFromLeft.trajectory);
}

} // namespace

void initialize() {
    sapphirelib::initialize();
    drivetrain(); // blocks while the IMU calibrates
    odometry().startTask();
    use(drivetrain(), odometry());
    // The models the planner and the feedforward use: what Auto-Tune measured (its readout and the
    // SD log print them), or load them from TUNE.CFG. Without them followTrajectory() aborts
    drivetrain().setAxisModels({
        .forward = {.kS = 0.97, .kV = 0.20, .kA = 0.045},
        .strafe = {.kS = 1.28, .kV = 0.24, .kA = 0.055},
        .turn = {.kS = 0.55, .kV = 0.045, .kA = 0.0067},
    });
}

void autonomous() {
    rush();
    // or: branchWithStop(); branchAtSpeed();
}
