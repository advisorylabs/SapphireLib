/**
 * \file autons.cpp
 *
 * Every autonomous routine, and registerAutons(), which lists them on the
 * brain screen. autonomous() runs whichever one is picked there (see
 * runSelectedAuton() in screen.cpp); so does X+UP off the field.
 *
 * Writing a routine (docs/QUICKSTART.md has every function):
 *   1. Write it below as a plain function, starting with setPose() at the
 *      spot the robot starts: field inches, the middle of the field is
 *      (0, 0), heading 0 faces +y and 90 faces +x. The test routines here
 *      start at (0, 0) facing 0 instead, so they run anywhere with room.
 *   2. Add it to registerAutons() at the bottom.
 *   3. Try it in the simulator (tools/sim) before the robot.
 *
 * Team 96671H: Hitmen
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "pros/rtos.hpp"
#include "robot/autons.hpp"
#include "robot/macros.hpp"
#include "sapphirelib/auton/auton.hpp"
#include "sapphirelib/motion/trajectory_config.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/wait.hpp"

// The routines sit outside namespace robot, so the auton functions (drive(),
// moveToPoint(), ...) are never hidden by something of the robot's with the
// same name.
namespace {

using namespace sapphirelib::auton;
using sapphirelib::motion::TrajectorySection;
namespace macros = robot::macros;

void doNothing() {}

// Tuning the drive and turn PIDs (docs/TUNING.md): a straight drive, a
// ladder of turns from big to small, and back. Each should stop on target
// without creeping or oscillating. Needs about 4ft clear ahead.
void tuningTest() {
    setPose(0, 0, 0);
    drive(48);
    turnTo(90);
    turnTo(135);
    turnTo(150);
    turnTo(160);
    turnTo(165);
    turnTo(0);
    drive(-48);
}

// Odometry and the point motions: a 24in square, holding the heading on
// every side (a holonomic drive strafes the sideways ones), the second side
// chained into the third without stopping, then home.
void odomTest() {
    setPose(0, 0, 0);
    moveToPoint(0, 24);
    moveToPoint(24, 24, {.chain = true});
    moveToPoint(24, 0);
    moveToPoint(0, 0);
    turnToPoint(0, 24); // face where it started out
}

// Background motions: the side piston goes out 12in into a drive and comes
// back in near the end, while the robot keeps driving. Then a turn around
// that has to go right, and home. Needs about 3ft clear ahead.
void asyncExample() {
    setPose(0, 0, 0);
    runAsync([] { moveToPoint(0, 36); });
    waitUntilTraveled(12);      // 12in into the move...
    macros::setExtended(true);  // ...push the side piston out
    waitUntilNear(0, 36, 6);    // 6in from the end...
    macros::setExtended(false); // ...pull it back in
    waitForMotion();
    turnTo(180, clockwise);
    moveToPoint(0, 0);
}

// Pure pursuit through points: an S-curve out to the right, then straight
// back to the start. Needs about 4ft ahead and 2ft to the right.
void pathExample() {
    setPose(0, -48, 0);
    followPath({{0, -48}, {0, -16}, {16, -28}, {16, -44}});
    moveToPoint(0, -48);
    turnTo(0);
}

// A trajectory as the simulator's Paths tab writes one (Copy C++), with
// things done at its points without slowing down: out 24in with the
// side piston going out as it passes point 1, a stop at point 2 where the claw
// opens and the side piston comes back in, then straight back to the start
// (point 3), where the claw closes again. A point the plan never reaches
// (held up until the motion timed out) ends the routine there, so the claw
// never opens short of where it should. Needs about 30in clear ahead.
const std::vector<TrajectorySection> kTrajectoryExample{
    {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 12}, {.xIn = 0, .yIn = 24}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = 0, .yIn = 0}}},
};

void trajectoryExample() {
    setPose(0, 0, 0);
    planTrajectory(kTrajectoryExample); // a few ms now, rather than once moving
    runAsync([] { followTrajectory(kTrajectoryExample); });
    if (!waitUntilWaypoint(1)) return;
    macros::setExtended(true);
    if (!waitUntilWaypoint(2)) return;
    macros::setClawClosed(false);
    macros::setExtended(false);
    if (!waitUntilWaypoint(3)) return;
    macros::setClawClosed(true);
    waitForMotion();
}

// Distance resets off one wall: start with the back of the robot about a
// foot from a wall and square to it (any wall: it says it's the near one).
// The back sensor sets y, then the robot drives out and back and resets
// again. The terminal says how far each reset moved the robot: the second
// is how far odometry drifted on the way.
void wallResetTest() {
    setPose(0, -58, 0);
    resetBack();
    drive(24);
    drive(-24);
    pros::delay(250); // let the sensor settle on the wall
    resetBack();
}

// The lift's own motors, every one in config.hpp's kLiftMotors, on open loop
// volts rather than its levels: through the lift's task (setVolts()), not
// straight to the motors, which that task would fight. Its position is the
// macros' too: motor degrees, zeroed on the bottom switch, so start it with
// the lift down.
// TODO: 1000 motor degrees of travel was the test chassis's lift; check this
// one's before running it, or the up stroke drives into the top hard stop
// and never ends.
void liftTesting() {
    sapphirelib::mechanism::PositionMechanism& lift = macros::liftMechanism();
    for (int i = 0; i < 18; i++) {
        // + is up for every lift motor (see config.hpp). With no reading
        // (every motor unplugged), both waits end at once: NaN compares
        // false.
        lift.setVolts(12.0);
        while (lift.position() < 1000) {
            pros::delay(2);
        }
        // Down no harder than the macros ever drive it (kLiftMaxDownVolts).
        lift.setVolts(-lift.config().downVoltsLimit());
        while (lift.position() > 15 && !macros::liftAtBottom()) {
            pros::delay(2);
        }
        lift.stop();
        pros::delay(6000);
    }
}

// The lift from autonomous: up to ALLIANCE's L3 while driving 24in out, a
// score the way R2 does it (dip, open the claw, back off), then down to the
// bottom on the way home. Start with a pin in the claw and the lift down.
// Needs about 30in clear ahead; no goal needed, it scores on thin air.
void liftExample() {
    setPose(0, 0, 0);
    macros::setLiftLevel(macros::Goal::alliance, 3); // start it up, without waiting...
    drive(24);                                       // ...while driving out
    macros::waitForLift(1500);                       // up before the dip
    macros::setLiftDeg(macros::liftScoreDeg());      // dip, as R2 does...
    macros::waitForLift(1000);
    macros::setClawClosed(false); // ...and let go
    drive(-6);                    // off the goal before lowering, as R2 waits to
    macros::setLiftLevel(macros::Goal::alliance, 0); // down...
    drive(-18);                                      // ...on the way home
}

// Blue side: starts with its back against the wall, facing +x. Rams the wall
// toggles twice, scores the payload on the goal to its right, then backs off,
// turns, and picks up the second pin.
//
// The localizer stays off until the end. From this spot it reads the wall
// behind about 3in long, and expects the side sensors to see the loaders they
// read past, so it moved the pose 3in off the wall and 5in down the field
// before the first ram (SL000040). The rams' point was then behind the wall:
// each pushed it for its whole 3s timeout, sliding along it, and every point
// after was off too. Turned back on at the goal, it moved the pose 7in down
// the field at once (SL000042): the back-off turned around and drove into
// the goal, and the drive to the second pin ran into it too.
//
// Each run onto a pin goes until the claw shuts (untilGrabbed(), below), then
// sends the lift up at once. Its end point is past where the pin seats, so with
// the pin in the claw the robot can't reach it: waiting out the motion left the
// lift down, the stack on the floor, for the whole timeout.
void untilGrabbed();

void triplePin() {
    macros::setClawClosed(true);
    macros::setExtended(true);
    setPose(-60.6, -5, 90);
    setMcl(false);
    macros::setLiftLevel(macros::Goal::alliance, 1);
    // Out and back on the drive encoders, which the wall can't confuse: back
    // 4in further than it went out, so it hits hard, and the timeout ends the
    // push once it's pinned (it reaches the wall in about 0.5s).
    for (int i = 0; i < 2; i++) {
        drive(10, {.chain = true, .exitError = 5});
        drive(-14, {.timeoutMs = 600});
    }
    // pinned where it started: x is the wall's again, whatever the hits did
    const sapphirelib::odom::Pose here = pose();
    setPose(-60.6, here.yIn, here.headingDeg);
    moveToPose(-54.5, -11, 135, {.chain = true});
    moveToPoint(-50, -21.5, {.timeoutMs = 1000}); // up onto the goal, pressing on it
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    pros::delay(250);
    macros::setClawClosed(false);
    pros::delay(250);
    moveToPoint(-50, -5, {.timeoutMs = 1500});
    macros::setLiftLevel(macros::Goal::alliance, 0);
    turnToPoint(-37, -10);
    moveToPoint(-37, -10, {.timeoutMs = 1500});
    macros::setAutoClaw(true);
    runAsync([] { moveToPose(-25.5, -25.5, 133, {.timeoutMs = 1750}); });
    untilGrabbed(); // ends the run as the claw shuts, rather than pushing to the timeout
    macros::setAutoClaw(false); // full speed for the turn
    macros::setLiftLevel(macros::Goal::alliance, 2);
    turnToPoint(-56, -18);
    moveToPoint(-56, -18, {.timeoutMs = 1250});
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    macros::setClawClosed(false);
    drive(-12, {.timeoutMs = 1500});
    macros::setLiftLevel(macros::Goal::alliance, 0);
    turnToPoint(-48, -47.5);
    drive(15, {.chain = true});
    macros::setAutoClaw(true);
    runAsync([] { moveToPoint(-48, -47.5, {.timeoutMs = 2000}); });
    untilGrabbed();
    macros::setAutoClaw(false);
    macros::setLiftLevel(macros::Goal::alliance, 3);
    turnToPoint(-44, -27);
    moveToPoint(-44, -27, {.timeoutMs = 1500});
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    macros::setClawClosed(false);


}

/// Triple Pin on points: three Pins stacked on the alliance Goal by the left wall, every drive a
// turn to face a point and a straight drive to it, as triplePin() above drives, so every stop is a
// number to change in triplePinPoints() below. The simulator runs the same (Drive tab, "Triple
// Pin (points)"; tools/sim/js/routines.js's TRIPLE_PIN has these numbers: change both).
//
// The goal, at (-47.09, -23.55) (kGoalXIn, kGoalYIn), takes the preload at L1 from the north, the
// Pin standing at (-23.55, -23.55) at L2 from the east, and the one at (-47.09, -47.09) at L3 from
// the south: where the manual's Appendix A has them. ontoGoal() and ontoPin() take the goal's or
// the Pin's spot and work out where the robot stops for it, facing it, so the routine's other
// numbers are the stops between, where it turns: roughly the old trajectories' route. Each stop is
// on the line the robot gets to it along (backTo() and offGoalTo() back straight along the robot's
// heading), so moving one means moving it along that line, or the ones after it with it.
//
// No wall resets, and the localizer's corrections off (setMcl(false)): odometry alone carries the
// routine from the rams at the start.
//
// In the simulator, with the robot's Auto-Tune models, these gains and push floors (devices.cpp),
// and its 13.75 x 17in size, it stacks all three within 0.25in of the goal's middle, the last at
// 13.3s, done at 13.9s; it stops within 0.5in of every stop, and touches nothing but the goal
// coming into the aligner (1.5in clear of it otherwise).

// Where a Goal and a Pin stop in the front of the robot. The front edge is kFrontIn ahead of the
// tracking center, and the goal aligner's back kAlignerDepthIn behind that; a Goal (5.61in across
// its flats) or a Pin (1.6in across) stops against it with its middle half its width further on.
constexpr double kFrontIn = 8.35;
constexpr double kAlignerDepthIn = 3.5;
constexpr double kGoalSeatIn = kFrontIn - kAlignerDepthIn + 5.61 / 2.0; // 7.65
constexpr double kPinSeatIn = kFrontIn - kAlignerDepthIn + 1.6 / 2.0;   // 5.65

// The goal the Triple Pin stacks on, where the manual has it.
constexpr double kGoalXIn = -47.09;
constexpr double kGoalYIn = -23.55;

// Every turn chains: it ends within 5 degrees, still turning, without a settle. Settling each one
// to 2 degrees cost 1.4s over the routine in the simulator (with devices.cpp's push floors, so the
// turns get there), and put the last score past 14.5s. What a turn leaves, the drive after it takes
// out: ontoGoal()'s and ontoPin()'s approaches turn to face their target as they go.
const Options kTurn{.chain = true};

// Onto the goal (ontoGoal()): an approach (auton::approach()) facing the goal's middle, at
// kGoalApproach.cruiseInPerS, ending kGoalShortIn short of where the goal seats, then the press:
// on at no more than 6V until the goal stops the robot and the timeout ends it, so the goal, not
// odometry, is the stop. Not a moveToPoint(): that holds the heading the turn left it on, a few
// degrees off, and in the simulator the goal landed 1 to 3in off the aligner's middle, past the
// 1.25in it catches. The approach ending further than kGoalPressReachIn from its end means
// something stopped the robot well short of the goal: no press and no score, and the Pin stays in
// the claw for driver control.
constexpr double kGoalShortIn = 1.0;
const sapphirelib::motion::ApproachConfig kGoalApproach{.cruiseInPerS = 30.0,
                                                        .accelInPerS2 = 120.0,
                                                        .decelInPerS2 = 120.0,
                                                        .endShortIn = kGoalSeatIn + kGoalShortIn,
                                                        .steerMinIn = kGoalSeatIn + 3.0,
                                                        .maxLagIn = 2.0,
                                                        .timeoutMs = 1500};
constexpr double kGoalPressReachIn = 8.0;
constexpr double kGoalPressIn = 3.0;

// How long a score's dip has before the claw lets go, as R2's (macros.cpp's kScoreReleaseMs): the
// claw opens once the lift reaches the dip or this long into it, whichever comes first, and the
// lift carries on down meanwhile.
constexpr std::uint32_t kScoreReleaseMs = 100;

// How far off the goal the robot backs before the lift comes down: the claw clear of the stack.
constexpr double kLiftClearIn = 3.5;

// The drives to the stops, where the robot turns for its next run (backTo(), offGoalTo(),
// driveTo()): chained, so each ends the first tick it's within 1in, still rolling, rather than
// settling 200ms, which saved 1.1s over the routine in the simulator with the same scores. With
// the robot's models and gains it stopped within 0.5in of every stop.
const Options kStop{.chain = true, .exitError = 1.0};

// --- The runs onto the Pins: the last third at autoClaw speed ---
//
// autoClaw's mode slows the robot to its speed limit (setAutoClaw() sets setSpeedLimit() to
// macros.cpp's kClawSpeedVolts), but only for motions that start after it. So each run (ontoPin())
// is a moveToPoint() for the first kAutoClawFromRun of the way (chained: it hands off within 3in
// of there, slowing as it comes), then autoClaw on, then an approach for the last third, which
// starts under autoClaw's limit. "The way" is the run: from where the robot turned to face the Pin
// to where the Pin seats in the aligner.
//
// The approach turns the robot to face the Pin and drives at it at a steady
// kPinApproach.cruiseInPerS, never strafing, through to kPinPastIn past where the Pin seats, so
// the robot meets the Pin at that speed rather than slowing onto it: autoClaw's lead (macros.cpp's
// kClawLeadMs) times the grab off the robot's speed as if it held, and here it does. It slows only
// past the Pin (18^2 / (2 x 80) = 2.0in to stop, inside the 3in), and ontoPin() stops it
// kPinCreepAfterGrabMs after the grab anyway. 18 in/s is under what autoClaw's 5V allows (about
// 27 in/s); SL000066 knocked a Pin over at 36. autoClaw goes off again once the claw has the Pin:
// its speed limit would hold the turn back to the goal to 5V.
constexpr double kAutoClawFromRun = 2.0 / 3.0;
constexpr double kPinPastIn = 3.0;
const sapphirelib::motion::ApproachConfig kPinApproach{.cruiseInPerS = 18.0,
                                                       .accelInPerS2 = 60.0,
                                                       .decelInPerS2 = 80.0,
                                                       .endShortIn = kPinSeatIn - kPinPastIn,
                                                       // the Pin nearly in the aligner: hold
                                                       .steerMinIn = kPinSeatIn + 2.0,
                                                       .maxLagIn = 2.0,
                                                       .timeoutMs = 2500};

// How far ontoPin() creeps on for a Pin the approach stopped short of: one further than odometry
// had it is still ahead of the jaws (SL000060 stopped 1.2in short and the claw never shut), and
// its timeout if the claw never shuts. autoClaw caps it at 5V (macros.cpp's kClawSpeedVolts).
// Once the claw shuts it stops kPinCreepAfterGrabMs later, the time autoClaw's lead allows for the
// jaws to close on a Pin still coming in.
constexpr double kPinCreepIn = 2.5;
constexpr std::uint32_t kPinCreepMs = 600;
constexpr std::uint32_t kPinCreepAfterGrabMs = 125;

// `inches` short of (xIn, yIn) on the line from `from` to it.
sapphirelib::motion::FieldPoint shortOf(const sapphirelib::odom::Pose& from, double xIn, double yIn,
                                        double inches) {
    const double d = std::hypot(xIn - from.xIn, yIn - from.yIn);
    const double k = d > 0.0 ? std::max(d - inches, 0.0) / d : 0.0;
    return {.xIn = from.xIn + (xIn - from.xIn) * k, .yIn = from.yIn + (yIn - from.yIn) * k};
}

// Onto the goal whose middle is (goalXIn, goalYIn): turn to face it, the approach on to just short
// of where it seats in the aligner, then the press (see kGoalApproach). False, without the press,
// if something stopped the robot well short of it, with an auton,no_score line in the log saying
// how far short.
bool ontoGoal(double goalXIn, double goalYIn) {
    turnToPoint(goalXIn, goalYIn, kTurn);
    const double shortIn = approach(goalXIn, goalYIn, kGoalApproach).finalError;
    if (shortIn > kGoalPressReachIn) {
        sapphirelib::telemetry::event("auton", "no_score,short_in=%.1f", shortIn);
        return false;
    }
    drive(kGoalPressIn, {.timeoutMs = 300, .maxVolts = 6});
    return true;
}

// R2's score from a routine: up at the level, the dip, and let go as R2 does (kScoreReleaseMs).
void scoreOnGoal() {
    macros::waitForLift(1500);
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(kScoreReleaseMs);
    macros::setClawClosed(false);
    pros::delay(100); // the claw open before backing off
}

// How far behind the robot (xIn, yIn) is, along its heading.
double behindIn(double xIn, double yIn) {
    const sapphirelib::odom::Pose here = pose();
    const double a = here.headingDeg * 3.14159265358979323846 / 180.0;
    return -((xIn - here.xIn) * std::sin(a) + (yIn - here.yIn) * std::cos(a));
}

// Straight back off the goal until the robot is level with (xIn, yIn), holding the heading, the
// lift down to the bottom once the claw has cleared the stack (kLiftClearIn). It backs along its
// heading, not to the point: put the point on the line straight back from the goal through where
// the robot came onto it, or it stops beside it. Straight back, so the claw lifts clear of the
// stack square and a corner doesn't swing into the goal. kStop by default: a routine's last motion
// passes {} instead, to settle.
void offGoalTo(double xIn, double yIn, Options options = kStop) {
    const double backIn = behindIn(xIn, yIn);
    runAsync([backIn, options] { drive(-backIn, options); });
    waitUntilTraveled(kLiftClearIn);
    macros::setLiftLevel(macros::Goal::alliance, 0);
    waitForMotion();
}

// Straight back until level with (xIn, yIn), holding the heading, as offGoalTo() but with the lift
// left alone: put the point on the line the robot came in along.
void backTo(double xIn, double yIn) { drive(-behindIn(xIn, yIn), kStop); }

// To (xIn, yIn), facing the way it goes: turn to face it, then straight there (kStop).
void driveTo(double xIn, double yIn) {
    turnToPoint(xIn, yIn, kTurn);
    moveToPoint(xIn, yIn, kStop);
}

// The running motion until the claw shuts (and kPinCreepAfterGrabMs on, then stopped) or it ends.
void untilGrabbed() {
    sapphirelib::waitUntil([] { return macros::clawClosed() || !isMotionRunning(); }, 0);
    if (macros::clawClosed()) {
        pros::delay(kPinCreepAfterGrabMs); // the jaws shut on it before the robot stops
        cancelMotion();
    }
    waitForMotion();
}

// Onto the Pin standing at (pinXIn, pinYIn): turn to face it, a moveToPoint() for two thirds of
// the way to where it seats, then autoClaw on and the approach at its speed for the last third
// (see kPinApproach). The robot stops kPinCreepAfterGrabMs after the claw shuts. If the approach
// ends with the claw still open, the rest of the way straight ahead, at autoClaw's 5V, until it
// shuts: the Pin was further than odometry had it, and is still ahead of the jaws. autoClaw off
// at the end. False if the claw didn't shut on it. An auton,pin_run line in the log says where the
// run left the robot.
bool ontoPin(double pinXIn, double pinYIn) {
    turnToPoint(pinXIn, pinYIn, kTurn);
    const sapphirelib::odom::Pose here = pose();
    const double runIn = std::max(std::hypot(pinXIn - here.xIn, pinYIn - here.yIn) - kPinSeatIn, 0.0);
    const sapphirelib::motion::FieldPoint handoff =
        shortOf(here, pinXIn, pinYIn, kPinSeatIn + runIn * (1.0 - kAutoClawFromRun));
    moveToPoint(handoff.xIn, handoff.yIn, {.chain = true});
    macros::setAutoClaw(true); // before the approach starts, so its speed limit holds it
    runAsync([pinXIn, pinYIn] { approach(pinXIn, pinYIn, kPinApproach); });
    untilGrabbed();
    if (!macros::clawClosed()) {
        runAsync([] { drive(kPinCreepIn, {.timeoutMs = kPinCreepMs}); });
        untilGrabbed();
    }
    const sapphirelib::odom::Pose end = pose();
    sapphirelib::telemetry::event(
        "auton", "pin_run,grabbed=%d,x=%.2f,y=%.2f,to_seat_in=%.2f", macros::clawClosed() ? 1 : 0,
        end.xIn, end.yIn, std::hypot(pinXIn - end.xIn, pinYIn - end.yIn) - kPinSeatIn);
    macros::setAutoClaw(false); // full speed for what comes next
    return macros::clawClosed();
}

void triplePinPoints() {
    // odometry alone: the localizer's corrections off for the whole routine (SL000040, 42 and 70)
    setMcl(false);
    macros::setClawClosed(true);
    macros::setExtended(macros::Side::left, false);
    setPose(-60.6, -5, 90);
    macros::setLiftLevel(macros::Goal::alliance, 1);
    // the Toggle, twice, as triplePin() rams it, and x from the wall it ends pinned against. Out
    // until within 5in of the 10 (not 1in, which crawled the last inches at 1V), and back until
    // the 600ms runs out, about when it reaches the wall (SL000053)
    for (int i = 0; i < 2; i++) {
        drive(10, {.chain = true, .exitError = 5});
        drive(-14, {.timeoutMs = 600});
    }
    const sapphirelib::odom::Pose here = pose();
    setPose(-60.6, here.yIn, here.headingDeg);

    // Each stop below is on the line the robot gets to it along, so it stops on it, not beside it.
    // 1: straight out from the wall to above the goal (no turn pinned against the wall: a back
    // corner would swing into it), then the preload onto it from the north (171) at L1
    moveToPoint(-50, -5);
    if (!ontoGoal(kGoalXIn, kGoalYIn)) return;
    scoreOnGoal();

    // 2: straight back off the goal to y = 0 (on the line from the goal's middle out through
    // (-50, -5)), the second Pin from there (131), straight back off it 8.3in up the line it came
    // in along, then onto the goal from the east (253) at L2
    offGoalTo(-50.8, 0);
    if (!ontoPin(-23.55, -23.55)) return; // missed it: stop here
    macros::setLiftLevel(macros::Goal::alliance, 2);
    backTo(-29.8, -18.1);
    if (!ontoGoal(kGoalXIn, kGoalYIn)) return;
    scoreOnGoal();

    // 3: straight back off the goal (on the line it came in along) to where the third Pin's run
    // (214), down the gap between the two Goals, passes this one 3in clear; then to 16in below the
    // goal and onto it from the south at L3
    offGoalTo(-27.1, -17.2);
    if (!ontoPin(-47.09, -47.09)) return;
    macros::setLiftLevel(macros::Goal::alliance, 3);
    driveTo(-47.09, -39.55);
    if (!ontoGoal(kGoalXIn, kGoalYIn)) return;
    scoreOnGoal();

    // straight back off the goal to (-47.09, -35.5), so driver control starts clear of it. Settled,
    // not chained ({}): the routine's last motion
    offGoalTo(-47.09, -35.5, {});
}

} // namespace

void robot::registerAutons(sapphirelib::gui::AutonSelectorPage& selector) {
    selector.addRoutine("Do Nothing", &doNothing);
    selector.addRoutine("Triple Pin", &triplePin);
    selector.addRoutine("Triple Pin (points)", &triplePinPoints);
    selector.addRoutine("Tuning Test", &tuningTest);
    selector.addRoutine("Odom Test", &odomTest);
    selector.addRoutine("Async Example", &asyncExample);
    selector.addRoutine("Path Example", &pathExample);
    selector.addRoutine("Trajectory Example", &trajectoryExample);
    selector.addRoutine("Wall Reset Test", &wallResetTest);
    selector.addRoutine("Lift Testing", &liftTesting);
    selector.addRoutine("Lift Example", &liftExample);
}
