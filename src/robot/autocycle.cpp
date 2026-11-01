/**
 * \file autocycle.cpp
 *
 * The auto cycle (see autocycle.hpp), written as an autonomous routine is
 * (autons.cpp, docs/QUICKSTART.md), and run in the background with
 * runAsync() so opcontrol's loop keeps going around it: the macros keep the
 * lift and claw on the driver's buttons, and update() watches A and the
 * sticks, ending it with cancelMotion().
 *
 * Team 96671H: Hitmen
 */

#include "robot/autocycle.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>

#include "robot/macros.hpp"
#include "robot/screen.hpp"
#include "sapphirelib/auton/auton.hpp"
#include "sapphirelib/chassis/competition_watch.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/timing.hpp"

// The routine sits outside namespace robot, as autons.cpp's do, so the auton
// functions are never hidden by something of the robot's with the same name.
namespace {

using namespace sapphirelib::auton;
using sapphirelib::chassis::CompetitionWatch;
using sapphirelib::input::Axis;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
namespace macros = robot::macros;

// Any drive stick past this ends the cycle. Raise it if a controller's drift
// ends the cycle by itself (the cancel event logs the sticks).
constexpr double kCancelStick = 0.1;

// At the loader, how long the claw must have been shut before the robot backs
// the pin out, so the claw's tight on it first (and the lift has started
// lifting it, 65ms in: kGrabLiftDelayMs in macros.cpp). At the goal, how long
// it must have been open before the robot backs off, so it's let go of the
// pin rather than dragging it off.
constexpr std::uint32_t kGrabMs = 250;
constexpr std::uint32_t kReleaseMs = 150;

// How often the routine's waits look at the claw.
constexpr std::uint32_t kPollMs = 10;

// Which part of the trip the routine is on, for the controller and the log.
enum class Leg { grab, toGoal, score, toLoader };

const char* legName(Leg leg) {
    switch (leg) {
    case Leg::grab: return "grab";
    case Leg::toGoal: return "to_goal";
    case Leg::score: return "score";
    case Leg::toLoader: return "to_loader";
    }
    return "unknown";
}

// The controller's line 1 for each, 15 columns at most.
const char* legLabel(Leg leg) {
    switch (leg) {
    case Leg::grab: return "AUTO: GRAB R1";
    case Leg::toGoal: return "AUTO: TO GOAL";
    case Leg::score: return "AUTO: SCORE R2";
    case Leg::toLoader: return "AUTO: TO LOADER";
    }
    return "AUTO";
}

// Whether the cycle's runAsync() block is going: from A until it returns.
std::atomic<bool> running{false};
// Set by the routine on its task, read by update() on opcontrol's.
std::atomic<Leg> leg{Leg::grab};
std::atomic<std::uint32_t> trips{0}; // pins it's taken to the goal

// macros::clawClosed() as of opcontrol's latest update(), and when it last
// changed, for the routine's waits: the macros' own state is opcontrol's
// alone. update() stores the time first, so a wait that sees the new state
// sees the new time with it.
std::atomic<bool> clawShut{false};
std::atomic<std::uint32_t> clawChangedMs{0};

// Waits until the claw has been shut (or open) for `ms`. False if the cycle
// ended first: cancelMotion(), or the competition mode changing.
bool waitForClaw(bool shut, std::uint32_t ms, const CompetitionWatch& watch) {
    while (!watch.changed()) {
        if (clawShut.load() == shut &&
            sapphirelib::elapsedMs(clawChangedMs.load(), sapphirelib::millis()) >= ms) {
            return true;
        }
        sapphirelib::delayMs(kPollMs);
    }
    return false;
}

// Where the robot is, with the localizer's correction, for tuning the route's
// points against where it really stopped.
void logWhere(const char* what) {
    const sapphirelib::odom::Pose at = pose();
    sapphirelib::telemetry::event("autocycle", "%s,x=%.2f,y=%.2f,heading_deg=%.2f,trip=%u", what,
                                  at.xIn, at.yIn, at.headingDeg,
                                  static_cast<unsigned>(trips.load()));
}

// The route, red's numbers: blue's loader and goal are red's turned half way
// round, and so are the walls and loaders the localizer sees, so these work
// from either station. Field inches, (0, 0) the middle of the field, red's
// wall at x = -70.25 and the end wall by the loader at y = -70.25. The goal
// is red's alliance goal on that side, at (-47.09, -23.55).
//
// Each way, every motion but the last is chained: it ends a few inches or
// degrees short, still moving, and the next one carries on from there, so the
// robot only stops where it waits for the driver. The drive after each turn is
// a moveToPose() rather than a moveToPoint(), which would hold whatever heading
// it started on: a chained turn ends up to 5 degrees short, still turning, and
// moveToPose() finishes it on the way.
//
// Once the cycle's cancelled, every motion left returns at once and drives
// nothing, so only the waits and the logging need to check.
void cycle(const CompetitionWatch& watch) {
    // On the loader, facing it: its face is 3.74in off the wall, so 9.5in out
    // from it is x = -70.25 + 3.74 + 9.5 = -57, and 11.5in off the end wall is
    // y = -58.75.
    setPose(-57, -58.75, 270);

    while (true) {
        // The pin: in the claw already the first time, then the driver's R1.
        leg = Leg::grab;
        if (!waitForClaw(true, kGrabMs, watch)) return;
        sapphirelib::telemetry::event("autocycle", "grabbed,trip=%u",
                                      static_cast<unsigned>(trips.load()));

        // Out 5in past the goal's line, tuned on the robot (SL000032): the
        // chained backout hands off 3in short, and on the way up the localizer
        // moves the robot's x toward the wall and the robot strafes back out
        // to follow it (SL000030), so out here it needs the least strafing.
        leg = Leg::toGoal;
        moveToPoint(-42, -58.75, {.chain = true}); // straight back out
        turnTo(0, {.chain = true});                // 90 degrees right, to face the goal
        moveToPose(-47, -30.65, 0, {.chain = true, .exitError = 2}); // up to 2in short
        moveToPose(-47, -28.65, 0, {.timeoutMs = 1500, .maxVolts = 6}); // creep in, pressing on it
        if (watch.changed()) return;
        logWhere("at_goal");

        // The driver raises the lift and scores: on once the score has opened
        // the claw.
        leg = Leg::score;
        if (!waitForClaw(false, kReleaseMs, watch)) return;
        trips.fetch_add(1);
        sapphirelib::telemetry::event("autocycle", "scored,trip=%u",
                                      static_cast<unsigned>(trips.load()));

        leg = Leg::toLoader;
        // Back off to 1in shy of the loader's line: the robot slides about 3in
        // on into the chained turn, and turning any nearer the end wall, its
        // corner hit it (SL000034, from y = -58.9 on). The drive up to the
        // loader brings it the rest of the way onto the line. Pressing on the
        // loader stops the robot short of that point, so it gives up pushing
        // after 2s (it gets there in 1-1.5s) rather than the 4s default.
        moveToPoint(-47, -57.75, {.chain = true});         // back off the goal the way it came
        turnTo(270, {.chain = true});                      // 90 degrees left, to face the loader
        moveToPose(-58, -58.75, 270, {.timeoutMs = 2000}); // up to the loader, pressing on it
        if (watch.changed()) return;
        logWhere("at_loader");
    }
}

bool sticksMoved(const Controller& controller) {
    for (const Axis axis : {Axis::leftY, Axis::leftX, Axis::rightX}) {
        if (std::fabs(controller.axis(axis)) > kCancelStick) return true;
    }
    return false;
}

void startCycle(Controller& controller) {
    // The routine's watch, taken here on opcontrol's task, as runAsync()
    // takes the block's, so a cancel before the block starts still ends it.
    const CompetitionWatch watch;
    const sapphirelib::odom::Pose before = pose();
    sapphirelib::telemetry::event(
        "autocycle", "start,was_x=%.2f,was_y=%.2f,was_heading_deg=%.2f,claw=%d", before.xIn,
        before.yIn, before.headingDeg, macros::clawClosed() ? 1 : 0);
    running = true;
    leg = Leg::grab;
    trips = 0;
    runAsync([watch] {
        cycle(watch);
        sapphirelib::telemetry::event("autocycle", "end,leg=%s,trips=%u", legName(leg.load()),
                                      static_cast<unsigned>(trips.load()));
        running = false;
    });
    controller.rumble(".");
}

// Ends the cycle, and waits for its block to return: so the motion's own stop
// (a brake) lands before the sticks take over, not after. The motion and the
// waits each check every 10ms.
void stopCycle() {
    cancelMotion();
    waitForMotion();
    // Already, unless the block was dropped before it started.
    running = false;
}

} // namespace

bool robot::autocycle::update(Controller& controller) {
    const bool shut = macros::clawClosed();
    if (shut != clawShut.load()) {
        clawChangedMs = controller.now();
        clawShut = shut;
    }

    if (!running.load()) {
        // A with the sticks still, and nothing else driving the chassis: a GUI
        // routine, or a block still finishing in the background.
        if (!controller.pressed(Button::a) || sticksMoved(controller) || robot::screenBusy() ||
            isMotionRunning()) {
            return false;
        }
        startCycle(controller);
        controller.screen().setLine(1, "%s", legLabel(leg.load()));
        return true;
    }

    // Driver control stopped and started again since the last tick, a GUI
    // routine took the chassis, A again, or a drive stick.
    const char* why = controller.resumed()           ? "resumed"
                      : robot::screenBusy()          ? "screen"
                      : controller.pressed(Button::a) ? "button"
                      : sticksMoved(controller)      ? "sticks"
                                                     : nullptr;
    if (why != nullptr) {
        sapphirelib::telemetry::event("autocycle", "cancel,%s,leg=%s,lx=%.2f,ly=%.2f,rx=%.2f", why,
                                      legName(leg.load()), controller.axis(Axis::leftX),
                                      controller.axis(Axis::leftY), controller.axis(Axis::rightX));
        stopCycle();
        return false;
    }

    controller.screen().setLine(1, "%s", legLabel(leg.load()));
    return true;
}

void robot::autocycle::cancel() {
    if (!running.load()) return;
    sapphirelib::telemetry::event("autocycle", "cancel,shortcut,leg=%s", legName(leg.load()));
    stopCycle();
}

bool robot::autocycle::active() { return running.load(); }
