/**
 * \file autons.cpp
 *
 * Every autonomous routine, and registerAutons(), which lists them on the
 * brain screen. autonomous() runs whichever one is picked there (see
 * runSelectedAuton() in screen.cpp); so does X+UP off the field.
 *
 * Team 96671H: Hitmen
 */

#include "robot/autons.hpp"

#include <atomic>
#include <cstdint>
#include <vector>

#include "pros/rtos.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"
#include "robot/teach.hpp"
#include "sapphirelib/motion/motion_queue.hpp"
#include "sapphirelib/motion/trajectory_generator.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"
#include "sapphirelib/util/wait.hpp"

namespace robot {

using sapphirelib::gui::AutonSelectorPage;
using sapphirelib::motion::generateTrajectory;
using sapphirelib::motion::Trajectory;
using sapphirelib::motion::TrajectoryOptions;
using sapphirelib::motion::TrajectoryResult;
using sapphirelib::motion::TrajectorySection;

namespace {

void doNothingAuton() {}

// The lift's own motors, every one in config.hpp's kLiftMotors, driven
// directly (the driver macros' lift loop only runs from opcontrol, so the two
// never command them at once). Its position is the macros' too: motor
// degrees, zeroed on the bottom switch, so start it with the lift down.
// TODO: 1000 motor degrees of travel was the test chassis's lift; check this
// one's before running it, or the up stroke drives into the top hard stop
// and never ends.
void liftTesting() {
    sapphirelib::mechanism::PositionMechanism& lift = macros::liftMechanism();
    for (int i = 0; i < 18; i++) {
        // + is up for every lift motor (see config.hpp). With no reading
        // (every motor unplugged), both waits end at once: NaN compares
        // false.
        lift.motors().move(127);
        while (lift.position() < 1000) {
            pros::delay(2);
        }
        // Down no harder than the macros ever drive it (kLiftMaxDownVolts).
        lift.motors().move_voltage(
            -static_cast<std::int32_t>(lift.config().downVoltsLimit() * 1000.0));
        while (lift.position() > 15 && !macros::liftAtBottom()) {
            pros::delay(2);
        }
        lift.motors().move(0);
        pros::delay(6000);
    }
}

void driveForwardAuton() { drivetrain().driveDistance(24); }

void turnTestingAuton() {
    drivetrain().turnToHeading(90);
    pros::delay(1000);
    drivetrain().turnToHeading(0);
}

// For checking followTrajectory() on the robot (docs/TRAJECTORIES.md): a
// chain the Paths tab of the simulator could have written. Up and to the
// right turning to face 45 degrees, a slower section that stops, then
// straight back to the start. Inches from wherever the robot sits (the
// routine sets that as (0, 0) facing 0), so it needs about 40in clear ahead
// and 30in to the right.
const std::vector<TrajectorySection> kTrajectoryTest{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0},
                   {.xIn = 0, .yIn = 24},
                   {.xIn = 12, .yIn = 36, .headingDeg = 45}}},
    {.waypoints = {{.xIn = 24, .yIn = 36}},
     .constraints = {.voltageBudget = 7.0},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}}},
};

void trajectoryTestAuton() {
    // both static: the queue's task outlives this call, and the motion it
    // runs reads the trajectory to the end even if a competition switch ends
    // autonomous first
    static sapphirelib::motion::MotionQueue queue;
    static TrajectoryResult plan;

    // planned now, from whatever axis models Auto-Tune or TUNE.CFG installed
    const std::uint64_t startUs = sapphirelib::micros();
    plan = generateTrajectory(kTrajectoryTest, drivetrain().axisModels());
    const double planMs = (sapphirelib::micros() - startUs) / 1000.0;
    if (!plan.ok) {
        SAPPHIRELIB_LOG_ERROR("auton", "trajectory test: %s", plan.error.c_str());
        return;
    }
    SAPPHIRELIB_LOG_INFO("auton", "trajectory test planned in %.1fms: %.2fs over %.0fin", planMs,
                         plan.trajectory.durationS(), plan.trajectory.lengthIn());

    odometry().setPose({.xIn = 0, .yIn = 0, .headingDeg = 0});
    // the number this trajectory's motion will have: until it starts, the
    // progress is still whichever trajectory ran last
    const std::uint32_t motion = drivetrain().trajectoryProgress().motion + 1;
    queue.enqueue([] { drivetrain().followTrajectory(plan.trajectory); });
    queue.run();
    // where a mechanism would start as the robot enters the slower section,
    // without the robot slowing down for it: here, a mark in the log
    if (sapphirelib::waitUntil(
            [motion] {
                const auto progress = drivetrain().trajectoryProgress();
                return progress.motion == motion && progress.section >= 1;
            },
            5000)) {
        sapphirelib::telemetry::event("mark", "trajectory_test,section=1");
    }
    queue.waitUntilDone();
}

// Plans one piece of a routine, from whatever axis models are installed, and
// says why in the terminal and SD log if it can't.
bool planPiece(const char* name, TrajectoryResult& out,
               const std::vector<TrajectorySection>& sections,
               const TrajectoryOptions& options = {}) {
    out = generateTrajectory(sections, drivetrain().axisModels(), options);
    if (!out.ok) SAPPHIRELIB_LOG_ERROR("auton", "%s: %s", name, out.error.c_str());
    return out.ok;
}

// Branching (docs/TRAJECTORIES.md#branching), two ways. Both are robot-
// relative like the Trajectory Test: the routine sets where the robot sits as
// (0, 0) facing 0, and needs about 50in clear ahead and 15in to each side. The
// claw's pin sensor picks the branch: hold a pin in the claw for the right
// one, leave it empty for the left. Each branch, and the way home from it, is
// planned at the start with TrajectoryOptions::from, so every piece starts
// exactly where the one before it ends.
//
// Branch Test, with a stop: drive to the decision point and stop there, read
// the sensor, then take the branch. Works with any trajectory; costs the stop.
const std::vector<TrajectorySection> kBranchTrunk{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 24}}}};
const std::vector<TrajectorySection> kBranchRight{
    {.waypoints = {{.xIn = 12, .yIn = 38, .headingDeg = 45}}}};
const std::vector<TrajectorySection> kBranchLeft{
    {.waypoints = {{.xIn = -12, .yIn = 38, .headingDeg = -45}}}};
// from either branch's end, straight back to the start
const std::vector<TrajectorySection> kBranchHome{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}}}};

void branchTestAuton() {
    // static, as in the Trajectory Test: a motion reads its trajectory to the end
    static TrajectoryResult trunk, right, left, homeFromRight, homeFromLeft;
    if (!planPiece("branch test", trunk, kBranchTrunk) ||
        !planPiece("branch test: right", right, kBranchRight, {.from = &trunk.trajectory}) ||
        !planPiece("branch test: left", left, kBranchLeft, {.from = &trunk.trajectory}) ||
        !planPiece("branch test: home", homeFromRight, kBranchHome, {.from = &right.trajectory}) ||
        !planPiece("branch test: home", homeFromLeft, kBranchHome, {.from = &left.trajectory})) {
        return;
    }

    odometry().setPose({.xIn = 0, .yIn = 0, .headingDeg = 0});
    if (!drivetrain().followTrajectory(trunk.trajectory).settled()) return;
    const bool pin = macros::pinInClaw();
    sapphirelib::telemetry::event("mark", "branch_test,pin=%d", pin ? 1 : 0);
    if (!drivetrain().followTrajectory(pin ? right.trajectory : left.trajectory).settled()) return;
    drivetrain().followTrajectory(pin ? homeFromRight.trajectory : homeFromLeft.trajectory);
}

// Branch At Speed: the same decision without the stop. The trunk ends at
// 24in/s instead of at rest, each branch carries on from it at that speed,
// and next reads the sensor as the robot reaches the junction, on the
// motion's task: one quick read and a pick between plans already made.
const std::vector<TrajectorySection> kSpeedTrunk{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 30}}}};
constexpr double kJunctionInPerS = 24.0;
const std::vector<TrajectorySection> kSpeedRight{
    {.waypoints = {{.xIn = 14, .yIn = 50, .headingDeg = 45}}}};
const std::vector<TrajectorySection> kSpeedLeft{
    {.waypoints = {{.xIn = -14, .yIn = 50, .headingDeg = -45}}}};

void branchAtSpeedAuton() {
    static TrajectoryResult trunk, right, left, homeFromRight, homeFromLeft;
    if (!planPiece("branch at speed", trunk, kSpeedTrunk, {.endSpeedInPerS = kJunctionInPerS}) ||
        !planPiece("branch at speed: right", right, kSpeedRight, {.from = &trunk.trajectory}) ||
        !planPiece("branch at speed: left", left, kSpeedLeft, {.from = &trunk.trajectory}) ||
        !planPiece("branch at speed: home", homeFromRight, kBranchHome,
                   {.from = &right.trajectory}) ||
        !planPiece("branch at speed: home", homeFromLeft, kBranchHome,
                   {.from = &left.trajectory})) {
        return;
    }

    odometry().setPose({.xIn = 0, .yIn = 0, .headingDeg = 0});
    // which branch next picked, for the way home; null if the trunk never got
    // to the junction (blocked until it timed out)
    static std::atomic<const Trajectory*> took{nullptr};
    took.store(nullptr);
    const auto result = drivetrain().followTrajectory(
        trunk.trajectory, {.next = [](const Trajectory&) -> const Trajectory* {
            const Trajectory* branch = macros::pinInClaw() ? &right.trajectory : &left.trajectory;
            took.store(branch);
            return branch;
        }});
    const Trajectory* branch = took.load();
    if (!result.settled() || branch == nullptr) return;
    sapphirelib::telemetry::event("mark", "branch_at_speed,pin=%d",
                                  branch == &right.trajectory ? 1 : 0);
    drivetrain().followTrajectory(branch == &right.trajectory ? homeFromRight.trajectory
                                                              : homeFromLeft.trajectory);
}

// An odometry-based routine, for when this robot has one. Start it with
// setPose() to say where on the field the robot sits and which way it faces;
// that one call puts the pose, turnToHeading() and moveToPose() in the same
// field frame (see odom::Pose), and tells the localizer where to start its
// particles. Use field coordinates with the origin in the middle of the field
// (walls at +-70.25in), the frame the localizer's map is in: from then on it
// keeps the pose honest against the walls, so the tenth motion lands as well
// as the first. Without a setPose(), points are inches from where the robot
// sat at initialize(): +y the way it faced, +x to its right, and the
// localizer can't help.
// moveToPoint(x, y) reads the pose from the odometry initDevices() gave the
// drivetrain, and every motion returns how it ended, so a routine can stop
// instead of driving on from a spot it never reached:
//
//   void twoGoalAuton() {
//       odometry().setPose({.xIn = -48, .yIn = -60, .headingDeg = 270}); // facing -x
//       if (!drivetrain().moveToPoint(-60, -60).settled()) return; // blocked, or timed out
//       drivetrain().turnToHeading(0); // field heading: face +y (downfield)
//       drivetrain().moveToPose(-60, -24, 0);
//   }
//
// Try a routine in the simulator first (tools/sim), which runs the same
// motions, odometry and localizer against a simulated field.
//
// and in registerAutons(): selector.addRoutine("Two Goal", &twoGoalAuton);

} // namespace

void registerAutons(AutonSelectorPage& selector) {
    selector.addRoutine("Do Nothing", &doNothingAuton);
    selector.addRoutine("Lift Testing", &liftTesting);
    selector.addRoutine("Drive Forward", &driveForwardAuton);
    selector.addRoutine("Turn Testing", &turnTestingAuton);
    selector.addRoutine("Trajectory Test", &trajectoryTestAuton);
    selector.addRoutine("Branch Test", &branchTestAuton);
    selector.addRoutine("Branch At Speed", &branchAtSpeedAuton);
    // replays the drive taught last (teach.cpp): X+A in driver control
    selector.addRoutine("Taught Route", &teach::replayAuton);
    // bench only: teach mode's SD writes against the logger's (teach.hpp)
    selector.addRoutine("SD Stress Test", &teach::sdStressAuton);
}

} // namespace robot
