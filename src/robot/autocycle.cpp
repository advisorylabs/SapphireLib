/**
 * \file autocycle.cpp
 *
 * The auto cycle (see autocycle.hpp): a step at a time, one opcontrol tick
 * at a time, so A or a stick can end it on any tick. Blocking motions
 * (moveToPose() and the like) can't be stopped part way, so each tick drives
 * toward the step's target itself, with the drivetrain's own drive and turn
 * PIDs, as moveToPose() does.
 *
 * Team 96671H: Hitmen
 */

#include "robot/autocycle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include "pros/distance.hpp"
#include "robot/config.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/timing.hpp"

namespace robot::autocycle {

namespace {

using sapphirelib::input::Axis;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
using sapphirelib::odom::Pose;

constexpr double kPi = 3.14159265358979323846;

// --- The route, in field coordinates (tools/sim/js/field.js, the manual's
// Appendix A), red's side ---
// Red's right-hand loader and the alliance goal on its left. Blue's are the
// same turned half way round, and so is the field's perimeter, the only
// thing the localizer sees, so the cycle runs on red's numbers from either
// station and the localizer can't tell the difference.
constexpr double kTileIn = 23.545;
constexpr double kFieldHalfIn = 70.25;
// The loader reaches this far in from the side wall, its middle this far from
// the end wall.
constexpr double kLoaderDepthIn = 3.74;
constexpr double kLoaderFromEndWallIn = 11.44;
// Where the cycle starts: the tracking center this far out from the loader's
// face (autocycle.hpp), in line with its middle, facing it (-x).
constexpr double kLoaderStandoffIn = 9.5;
constexpr double kStartXIn = -kFieldHalfIn + kLoaderDepthIn + kLoaderStandoffIn; // -57.01
constexpr double kLoaderLineYIn = -kFieldHalfIn + kLoaderFromEndWallIn;          // -58.81
constexpr double kFacingLoaderDeg = 270.0;
// The alliance goal on the left, two tiles toward red's wall and one toward
// the loader's end from the middle.
constexpr double kGoalXIn = -2.0 * kTileIn;
constexpr double kGoalYIn = -kTileIn;
constexpr double kFacingGoalDeg = 0.0;
// The robot turns for the goal on its line.
constexpr double kGoalLineXIn = kGoalXIn; // -47.09
// Where the robot stops at the goal: the tracking center this far from the
// goal's middle. TODO: check on the robot that the pin ends up over the goal.
constexpr double kGoalStandoffIn = 6.0;

// Driving up to the goal, the cycle measures how far along it is with the
// back distance sensor (the localizer's, ports::kDistanceBack) against the
// end wall behind it, not odometry: driving forward from a stop, the
// vertical tracking wheel loses travel, 10in of the 26in up to the goal in
// SL000021 and SL000023, and the localizer doesn't trust itself enough to
// correct it until the robot is already at the goal. Backing up, the wheel
// and the localizer agree. Readings past this are ignored (the sensor's
// range, or something in the way of it), and odometry used instead.
constexpr double kBackSensorMaxIn = 78.0;
constexpr double kEndWallYIn = -kFieldHalfIn;
// The sensor doesn't replace odometry outright: the robot's movement still
// comes from the raw tracking wheels, which are smooth, and the sensor only
// says how far off they are, which changes slowly (by how much the wheel has
// slipped). Each reading moves that offset this much of the way, and between
// readings, or with none for a while, the last offset holds. Driving by the
// sensor itself, and by odometry whenever it missed a reading (30% of them in
// SL000026), jumped inches between the two, and the drive PID's derivative
// turned that into wobble on the way to the goal: +11V to -10.8V and back.
constexpr double kBackOffsetSmoothing = 0.2;

// --- Picking up a pin ---
// Coming back, the robot drives at full speed to this far short of where it
// started, waits there for the lift to be down, then creeps in at
// kIntoLoaderVolts, closing the claw once the sensor reads a pin within
// kGrabMm. It creeps toward a point this far past where it started, so
// that's never where it stops: in SL000026 the localizer put the start spot
// 4-5in nearer the wall than kStartXIn, and creeping to 2in past it stopped
// short of the pin. Pressed up against the loader with no pin, it times out.
constexpr double kLoaderStagingIn = 2.0;
constexpr double kIntoLoaderIn = 10.0;
constexpr double kIntoLoaderVolts = 4.0;
constexpr std::int32_t kGrabMm = 45;
// Once the claw closes, the robot carries on this far into the loader, still
// at kIntoLoaderVolts, before backing out with the pin: by the front distance
// sensor's reading of whatever it sees ahead (the loader, or the wall behind
// it) falling this much, or odometry if it sees nothing within
// kFrontSensorMaxIn. Pushed up against the loader, or after kSeatTimeoutMs,
// it backs out anyway.
constexpr double kSeatIn = 2.0;
constexpr double kFrontSensorMaxIn = 30.0;
constexpr std::uint32_t kSeatTimeoutMs = 1000;

// --- Driving each step ---
// The fastest any step drives, in volts. TODO: tune on the robot; lower it if
// the robot tips coming off the goal with the lift still up.
constexpr double kMaxVolts = 12.0;
// Pushing at least kStallVolts for kStallMs while moving less than
// kStallMoveIn ends the cycle: driven into the goal or another robot, rather
// than shoving it for the whole kStepTimeoutMs. Measured as distance over the
// whole window, not odometry's velocity: the tracking wheels' pose only
// changes every other 10ms update, so the velocity reads 0 half the time, and
// the cycle's 20ms tick can land on those every time (SL000022: "stalled"
// 320ms into the drive to the goal, already doing 12in/s). 0.6in in 300ms is
// 2in/s.
constexpr double kStallVolts = 4.0;
constexpr double kStallMoveIn = 0.6;
constexpr std::uint32_t kStallMs = 300;
// Driving up to the goal, moving less than this in kStillMs, however hard
// it's pushing, counts as there: it's come to rest.
constexpr double kStillMoveIn = 0.3;
constexpr std::uint32_t kStillMs = 300;
// Until a step first gets within its tolerance, it pushes at least this hard:
// near the target the PIDs alone ask for less than it takes to move the
// robot at all (the axis models' kS: forward 0.3V, turn 1.1V). In SL000020
// the turn PID gave 1.1V at 4.4 degrees short and the robot sat there until
// the step timed out; in SL000022, 1.6V crept the last 6 degrees in 0.9s.
// Only until then, position and heading each on its own: kicking at these
// volts every time a hold drifted just past the tolerance made it jitter.
constexpr double kMinDriveVolts = 1.0;
constexpr double kMinTurnVolts = 2.5;
// A step's target counts as reached within these, held for kSettleMs.
constexpr double kPositionToleranceIn = 1.0;
constexpr double kHeadingToleranceDeg = 3.0;
constexpr std::uint32_t kSettleMs = 60;
// A step that hasn't reached its target by then ends the cycle, with a
// rumble: something's in the way, and the driver should take over. Not the
// wait at the goal, which is the driver's.
constexpr std::uint32_t kStepTimeoutMs = 3000;
// The wait for the lift to come down before driving into the loader.
constexpr std::uint32_t kLiftDownTimeoutMs = 4000;

// Any drive stick past this ends the cycle.
constexpr double kCancelStick = 0.15;
// A longer gap than this between update() calls means opcontrol stopped or
// something else had the chassis, and the cycle ends.
constexpr std::uint32_t kResumeGapMs = 100;

enum class Step {
    backOffLoader, // back away from the loader, past the goal's line
    turnToGoal,
    toGoal,
    atGoal,        // the driver raises the lift and scores
    backOffGoal,   // the way back, in line with the loader
    turnToLoader,
    toLoader,      // short of it, waiting for the lift to be down
    intoLoader,    // creeping in until the pin is in the claw
    seatInLoader,  // the claw closed, kSeatIn further in
};

const char* stepName(Step step) {
    switch (step) {
    case Step::backOffLoader: return "back_off_loader";
    case Step::turnToGoal: return "turn_to_goal";
    case Step::toGoal: return "to_goal";
    case Step::atGoal: return "at_goal";
    case Step::backOffGoal: return "back_off_goal";
    case Step::turnToLoader: return "turn_to_loader";
    case Step::toLoader: return "to_loader";
    case Step::intoLoader: return "into_loader";
    case Step::seatInLoader: return "seat_in_loader";
    }
    return "unknown";
}

// The controller's line 1 for each step, 15 columns at most.
const char* stepLabel(Step step) {
    switch (step) {
    case Step::backOffLoader:
    case Step::turnToGoal:
    case Step::toGoal: return "AUTO: TO GOAL";
    case Step::atGoal: return "AUTO: SCORE R2";
    case Step::backOffGoal:
    case Step::turnToLoader:
    case Step::toLoader: return "AUTO: TO LOADER";
    case Step::intoLoader:
    case Step::seatInLoader: return "AUTO: GRAB";
    }
    return "AUTO";
}

// Each step's target on the field.
Pose targetOf(Step step) {
    switch (step) {
    case Step::backOffLoader: return {kGoalLineXIn, kLoaderLineYIn, kFacingLoaderDeg};
    case Step::turnToGoal: return {kGoalLineXIn, kLoaderLineYIn, kFacingGoalDeg};
    case Step::toGoal:
    case Step::atGoal: return {kGoalLineXIn, kGoalYIn - kGoalStandoffIn, kFacingGoalDeg};
    case Step::backOffGoal: return {kGoalLineXIn, kLoaderLineYIn, kFacingGoalDeg};
    case Step::turnToLoader: return {kGoalLineXIn, kLoaderLineYIn, kFacingLoaderDeg};
    case Step::toLoader: return {kStartXIn + kLoaderStagingIn, kLoaderLineYIn, kFacingLoaderDeg};
    case Step::intoLoader: return {kStartXIn - kIntoLoaderIn, kLoaderLineYIn, kFacingLoaderDeg};
    case Step::seatInLoader: break; // from where the grab happened, see Cycle::seatTarget
    }
    return {kStartXIn, kLoaderLineYIn, kFacingLoaderDeg};
}

Step nextStep(Step step) {
    switch (step) {
    case Step::backOffLoader: return Step::turnToGoal;
    case Step::turnToGoal: return Step::toGoal;
    case Step::toGoal: return Step::atGoal;
    case Step::atGoal: return Step::backOffGoal;
    case Step::backOffGoal: return Step::turnToLoader;
    case Step::turnToLoader: return Step::toLoader;
    case Step::toLoader: return Step::intoLoader;
    case Step::intoLoader: return Step::seatInLoader;
    case Step::seatInLoader: return Step::backOffLoader;
    }
    return Step::backOffLoader;
}

struct Cycle {
    bool active = false;
    Step step = Step::backOffLoader;
    std::uint32_t stepStartMs = 0;
    // Since when the step's target has been within tolerance, for kSettleMs.
    std::optional<std::uint32_t> withinSinceMs;
    // When the robot started pushing without having moved kStallMoveIn
    // since, and where it was then, for stalled().
    std::optional<std::uint32_t> stallStartMs;
    Pose stallStartPose;
    // The same for still(), whatever the volts.
    std::optional<std::uint32_t> stillStartMs;
    Pose stillStartPose;
    // The drive's volts last tick, for stalled().
    double driveVolts = 0.0;
    // Whether this step has been within its position, and its heading,
    // tolerance yet, for kMinDriveVolts and kMinTurnVolts.
    bool positionReached = false;
    bool headingReached = false;
    // How far the back sensor says the raw pose's y is off, driving up to the
    // goal and back (see kBackOffsetSmoothing). Kept from one to the other:
    // backing up reads true, so what the drive up lost is still lost.
    std::optional<double> backOffsetIn;
    // seatInLoader: where it heads (past kSeatIn, so it doesn't slow down
    // short of it), and where it started, by the front sensor if it saw
    // anything and by the raw pose.
    Pose seatTarget;
    std::optional<double> seatFrontStartIn;
    Pose seatRawStart;
    // atGoal: wherever the robot came to rest at the goal, by the raw pose,
    // which it holds rather than driving on toward the goal's target. Short
    // of it by odometry (the forward drive reads short), it shoved the goal
    // at up to 12V the whole time the driver scored (SL000025).
    Pose goalHold;
    std::uint32_t lastMs = 0;
    std::uint32_t cycles = 0; // pins grabbed, for the log
};

Cycle cycle;
sapphirelib::GapDetector gap{kResumeGapMs};

// The localizer's back sensor, read here too (reading a sensor from two
// tasks is safe), and where it's mounted, as start() last found it.
pros::Distance backSensor(ports::kDistanceBack);
pros::Distance frontSensor(ports::kDistanceFront);
sapphirelib::localization::DistanceSensorMount backMount{
    .forwardIn = kBackSensorForwardIn, .rightIn = kBackSensorRightIn, .facingDeg = 180.0};

// Where the cycle drives by: getPose(), but driving up to the goal, how far
// along it is from the back sensor (see kBackSensorMaxIn). Its beam points
// straight back at the end wall when facing the goal, and the tracking
// center is the reading (along the beam's slant, if the robot's turned a
// little) ahead of the wall, less how far the sensor sits behind and to the
// side of it.
//
// Backing off the goal too: odometry reads backing up right, but it starts
// from a forward drive that read short, so it came back 3-4in short of the
// loader's line (SL000025, with the back sensor dead all session).
//
// Holding at the goal, it's the raw pose instead (see Cycle::goalHold).
Pose cyclePose() {
    if (cycle.step == Step::atGoal) return odometry().snapshot().rawPose;
    const sapphirelib::odom::Odometry::Snapshot snapshot = odometry().snapshot();
    Pose pose = snapshot.pose;
    if (cycle.step != Step::toGoal && cycle.step != Step::backOffGoal) return pose;
    const std::int32_t mm = backSensor.get_distance(); // 9999 seeing nothing, PROS_ERR unplugged
    const double readingIn = mm / 25.4;
    if (mm > 0 && readingIn <= kBackSensorMaxIn) {
        const double h = pose.headingDeg * (kPi / 180.0);
        const double sensedYIn = kEndWallYIn + (readingIn - backMount.forwardIn) * std::cos(h) +
                                 backMount.rightIn * std::sin(h);
        const double offsetIn = sensedYIn - snapshot.rawPose.yIn;
        cycle.backOffsetIn =
            cycle.backOffsetIn
                ? *cycle.backOffsetIn + kBackOffsetSmoothing * (offsetIn - *cycle.backOffsetIn)
                : offsetIn;
    }
    // No reading yet this drive: odometry, localizer and all.
    if (!cycle.backOffsetIn) return pose;
    pose.yIn = snapshot.rawPose.yIn + *cycle.backOffsetIn;
    return pose;
}

void enter(Step step, std::uint32_t now) {
    cycle.step = step;
    cycle.stepStartMs = now;
    // A new drive up to the goal slips its own way.
    if (step == Step::toGoal) cycle.backOffsetIn.reset();
    if (step == Step::atGoal) cycle.goalHold = odometry().snapshot().rawPose;
    cycle.withinSinceMs.reset();
    cycle.stallStartMs.reset();
    cycle.stillStartMs.reset();
    cycle.positionReached = false;
    cycle.headingReached = false;
    // Each step starts its loops fresh, or the derivative kicks on the new
    // target.
    drivetrain().drivePID().reset();
    drivetrain().turnPID().reset();
    sapphirelib::telemetry::event("autocycle", "step,%s,cycle=%u", stepName(step),
                                  static_cast<unsigned>(cycle.cycles));
}

void start(Controller& controller, std::uint32_t now) {
    cycle = Cycle{.active = true, .lastMs = now};
    // A Calibrate Sensors run may have moved it since the last cycle.
    backMount = localizer().sensorMounts()[2];
    // The robot is on the start spot (autocycle.hpp), so that's where
    // odometry says it is from here on, in field coordinates, and the
    // localizer, which waits for a setPose() (LocalizerConfig::
    // waitForSetPose), starts correcting the drift from it.
    const Pose before = odometry().getPose();
    odometry().setPose({.xIn = kStartXIn, .yIn = kLoaderLineYIn, .headingDeg = kFacingLoaderDeg});
    // The cycle closes the claw itself, at kGrabMm.
    macros::setAutoClawPaused(true);
    sapphirelib::telemetry::event("autocycle", "start,was_x=%.2f,was_y=%.2f,was_heading_deg=%.2f",
                                  before.xIn, before.yIn, before.headingDeg);
    // No pin to score yet: pick one up first.
    enter(macros::clawClosed() ? Step::backOffLoader : Step::intoLoader, now);
    controller.rumble(".");
}

void end(const char* why) {
    if (!cycle.active) return;
    cycle.active = false;
    macros::setAutoClawPaused(false);
    sapphirelib::telemetry::event("autocycle", "end,%s,step=%s,cycles=%u", why,
                                  stepName(cycle.step), static_cast<unsigned>(cycle.cycles));
}

bool sticksMoved(const Controller& controller) {
    for (const Axis axis : {Axis::leftY, Axis::leftX, Axis::rightX}) {
        if (std::fabs(controller.axis(axis)) > kCancelStick) return true;
    }
    return false;
}

// Volts limited to maxVolts, and, until the step first got within its
// tolerance, at least minVolts.
double shape(double volts, double minVolts, double maxVolts, bool reached) {
    if (!reached && std::fabs(volts) < minVolts) volts = std::copysign(minVolts, volts);
    return std::clamp(volts, -maxVolts, maxVolts);
}

// One tick toward `target`, at most `maxVolts`, as moveToPose() does: the
// drive PID on the distance, pointed at the target in the robot's frame, and
// the turn PID on the heading. By getPose(), so the localizer's corrections
// keep the cycle on the field's goal and loader. Returns whether it's within
// tolerance.
bool driveToward(const Pose& target, double maxVolts, double dtS) {
    const Pose pose = cyclePose();
    const double dxIn = target.xIn - pose.xIn;
    const double dyIn = target.yIn - pose.yIn;
    const double distanceIn = std::hypot(dxIn, dyIn);
    const double headingErrorDeg =
        sapphirelib::wrapDegrees180(target.headingDeg - pose.headingDeg);
    const bool positionOut = distanceIn > kPositionToleranceIn;
    const bool headingOut = std::fabs(headingErrorDeg) > kHeadingToleranceDeg;
    if (!positionOut) cycle.positionReached = true;
    if (!headingOut) cycle.headingReached = true;

    const double outputVolts = shape(drivetrain().drivePID().update(distanceIn, 0.0, dtS),
                                     kMinDriveVolts, maxVolts, cycle.positionReached);
    cycle.driveVolts = outputVolts;
    const sapphirelib::motion::LocalOffset local =
        sapphirelib::motion::toLocalFrame(dxIn, dyIn, pose.headingDeg);
    const double scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;
    const double turnVolts =
        shape(drivetrain().turnPID().update(headingErrorDeg, 0.0, dtS), kMinTurnVolts, maxVolts,
              cycle.headingReached);
    drivetrain().holonomicVolts(local.forwardIn * scale, local.lateralIn * scale, turnVolts);

    return !positionOut && !headingOut;
}

// Whether the step's target has been within tolerance for kSettleMs.
bool settled(bool within, std::uint32_t now) {
    if (!within) {
        cycle.withinSinceMs.reset();
        return false;
    }
    if (!cycle.withinSinceMs) cycle.withinSinceMs = now;
    return sapphirelib::elapsedMs(*cycle.withinSinceMs, now) >= kSettleMs;
}

// Whether the robot has been pushing at least kStallVolts for kStallMs and
// moved less than kStallMoveIn in that time. By the raw pose, which a
// correction easing in never moves.
bool stalled(std::uint32_t now) {
    if (std::fabs(cycle.driveVolts) < kStallVolts) {
        cycle.stallStartMs.reset();
        return false;
    }
    const Pose pose = odometry().snapshot().rawPose;
    if (!cycle.stallStartMs || std::hypot(pose.xIn - cycle.stallStartPose.xIn,
                                          pose.yIn - cycle.stallStartPose.yIn) >= kStallMoveIn) {
        // Pushing from here on, or moving: the window starts again.
        cycle.stallStartMs = now;
        cycle.stallStartPose = pose;
        return false;
    }
    return sapphirelib::elapsedMs(*cycle.stallStartMs, now) >= kStallMs;
}

// Whether the robot has moved less than kStillMoveIn in the last kStillMs,
// whatever it's being told. By the raw pose, as stalled().
bool still(std::uint32_t now) {
    const Pose pose = odometry().snapshot().rawPose;
    if (!cycle.stillStartMs || std::hypot(pose.xIn - cycle.stillStartPose.xIn,
                                          pose.yIn - cycle.stillStartPose.yIn) >= kStillMoveIn) {
        cycle.stillStartMs = now;
        cycle.stillStartPose = pose;
        return false;
    }
    return sapphirelib::elapsedMs(*cycle.stillStartMs, now) >= kStillMs;
}

// One tick of the running step: drive it, and move on once it's done.
// Returns why the cycle should end ("timeout", "stalled"), or nullptr to
// carry on.
const char* runStep(std::uint32_t now, double dtS) {
    const std::uint32_t inStepMs = sapphirelib::elapsedMs(cycle.stepStartMs, now);
    const Step step = cycle.step;

    if (step == Step::intoLoader) {
        // Slowly, so the claw closes at kGrabMm rather than past it.
        driveToward(targetOf(step), kIntoLoaderVolts, dtS);
        if (macros::pinWithinMm(kGrabMm)) {
            macros::setClawClosed(true);
            ++cycle.cycles;
            sapphirelib::telemetry::event("autocycle", "grab,cycle=%u",
                                          static_cast<unsigned>(cycle.cycles));
        }
        // Closed by the grab above, or by the driver's R1: on in, without
        // stopping.
        if (macros::clawClosed()) {
            const Pose pose = odometry().getPose();
            const double h = pose.headingDeg * (kPi / 180.0);
            const double aheadIn = kSeatIn + kPositionToleranceIn;
            cycle.seatTarget = {.xIn = pose.xIn + aheadIn * std::sin(h),
                                .yIn = pose.yIn + aheadIn * std::cos(h),
                                .headingDeg = kFacingLoaderDeg};
            const std::int32_t frontMm = frontSensor.get_distance();
            cycle.seatFrontStartIn.reset();
            if (frontMm > 0 && frontMm / 25.4 <= kFrontSensorMaxIn) {
                cycle.seatFrontStartIn = frontMm / 25.4;
            }
            cycle.seatRawStart = odometry().snapshot().rawPose;
            enter(nextStep(step), now);
            return nullptr;
        }
        return inStepMs < kStepTimeoutMs ? nullptr : "timeout";
    }

    if (step == Step::seatInLoader) {
        driveToward(cycle.seatTarget, kIntoLoaderVolts, dtS);
        double inIn = 0.0;
        const std::int32_t frontMm = frontSensor.get_distance();
        if (cycle.seatFrontStartIn && frontMm > 0 && frontMm / 25.4 <= kFrontSensorMaxIn) {
            inIn = *cycle.seatFrontStartIn - frontMm / 25.4;
        } else {
            const Pose raw = odometry().snapshot().rawPose;
            inIn = std::hypot(raw.xIn - cycle.seatRawStart.xIn, raw.yIn - cycle.seatRawStart.yIn);
        }
        // Pushed up against the loader is as far in as it goes.
        const bool pushed = stalled(now);
        if (inIn >= kSeatIn || pushed || inStepMs >= kSeatTimeoutMs) {
            sapphirelib::telemetry::event("autocycle", "seated,in=%.2f,pushed=%d,front=%d", inIn,
                                          pushed ? 1 : 0, cycle.seatFrontStartIn ? 1 : 0);
            enter(nextStep(step), now);
        }
        return nullptr;
    }

    const bool within =
        driveToward(step == Step::atGoal ? cycle.goalHold : targetOf(step), kMaxVolts, dtS);

    if (step == Step::atGoal) {
        // Holds the spot for as long as the driver takes, until the score
        // opens the claw (or R1 does).
        // No stall check: pressed against the goal is fine here.
        if (!macros::clawClosed()) enter(nextStep(step), now);
        return nullptr;
    }

    // The loader only once the lift is down, which the score does once the
    // robot is 3in off the goal.
    const bool ready = step != Step::toLoader || macros::liftDown();
    if (settled(within, now) && ready) {
        enter(nextStep(step), now);
        return nullptr;
    }
    if (step == Step::toGoal) {
        // At the goal however it got there: up against it, or come to rest
        // short of the tolerance (SL000024: still for 2s, 1.2in off sideways
        // by a localizer correction easing in, with 2.5V too little to
        // strafe), or out of time. The driver's lining up the score from
        // here anyway.
        const char* how = stalled(now)                   ? "stalled"
                          : still(now)                   ? "still"
                          : inStepMs >= kStepTimeoutMs ? "timeout"
                                                         : nullptr;
        if (how != nullptr) {
            const Pose pose = cyclePose();
            const Pose target = targetOf(step);
            sapphirelib::telemetry::event("autocycle", "at_goal,%s,off_x=%.2f,off_y=%.2f", how,
                                          pose.xIn - target.xIn, pose.yIn - target.yIn);
            enter(nextStep(step), now);
        }
        return nullptr;
    }
    if (stalled(now)) {
        // Blocked on the way back (SL000025: twice with the horizontal
        // tracking wheel cut out, which reads every strafe as standing
        // still): on to the next step, rather than ending the cycle. The
        // sticks still end it.
        sapphirelib::telemetry::event("autocycle", "stalled,%s", stepName(step));
        enter(nextStep(step), now);
        return nullptr;
    }
    const std::uint32_t timeoutMs = step == Step::toLoader ? kLiftDownTimeoutMs : kStepTimeoutMs;
    return inStepMs < timeoutMs ? nullptr : "timeout";
}

} // namespace

bool update(Controller& controller) {
    const std::uint32_t now = controller.now();
    // A stop since the last tick (autonomous, a disable, a GUI routine with
    // the chassis): whatever the cycle was doing is stale.
    if (gap.update(now) || controller.resumed()) end("resumed");

    const bool aPressed = controller.pressed(Button::a) && !controller.held(Button::x); // X+A is teach mode's
    if (!cycle.active) {
        if (!aPressed) return false;
        start(controller, now);
    } else if (aPressed) {
        end("button");
        return false;
    } else if (sticksMoved(controller)) {
        end("sticks");
        return false;
    }

    const double dtS = sapphirelib::elapsedMs(cycle.lastMs, now) / 1000.0;
    cycle.lastMs = now;
    if (const char* why = runStep(now, dtS)) {
        end(why);
        drivetrain().holonomicVolts(0.0, 0.0, 0.0);
        controller.rumble("--");
        return true;
    }
    controller.screen().setLine(1, "%s", stepLabel(cycle.step));
    return true;
}

void cancel() { end("cancelled"); }

bool active() { return cycle.active; }

} // namespace robot::autocycle
