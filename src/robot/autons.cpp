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
#include <cstdlib>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "robot/autons.hpp"
#include "robot/config.hpp"
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

void untilGrabbed();


// The Skills' distance resets, each where the simulator's wallResetCheck() found the beam's cone
// clean (with the Triple Pin's two Pins off the field). Beside the loader, the left sensor's beam
// lands on the end wall 14in from the corner, inside use()'s 16in margin, so 10in: clean anywhere
// within 2in and 6 degrees of the loader spot. Below the goal by the left wall, the back sensor's
// lands 14 to 23in from it, so 12in: at 10, the cone sees the loader from the edges of where the
// robot stops there.
const sapphirelib::localization::WallResetConfig kLoaderReset{.cornerMarginIn = 10.0,
                                                              .maxJumpIn = 60.0};
const sapphirelib::localization::WallResetConfig kGoalReset{.cornerMarginIn = 2.0,
                                                            .maxJumpIn = 60.0};

// The preload onto the goal by the left wall at L1: red's Toggle rammed twice from the start, x
// from the wall it ends pinned against, then onto the goal from the north and the score. The
// Triple Pin's first score, and both Skills' (skills() goes on from here through the gap beside
// the goal). Ends with the claw open on the goal.
void preloadOntoGoal() {
    macros::setClawClosed(true);
    macros::setExtended(true);
    setPose(-60.6, -5, 90);
    setMcl(false);
    macros::setLiftLevel(macros::Goal::alliance, 1);
    for (int i = 0; i < 2; i++) {
        drive(10, {.chain = true, .exitError = 5});
        drive(-14, {.timeoutMs = 600});
    }
    const sapphirelib::odom::Pose here = pose();
    setPose(-60.6, here.yIn, here.headingDeg);
    moveToPose(-54.5, -11, 135, {.chain = true});
    moveToPoint(-50, -21.5, {.timeoutMs = 1000});
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    pros::delay(125);
    macros::setClawClosed(false);
    pros::delay(125);
}

void triplePin() {
    preloadOntoGoal();
    moveToPoint(-50, -5, {.timeoutMs = 1250, .chain = true, .minVolts = 8});
    macros::setLiftLevel(macros::Goal::alliance, 0);
    turnToPoint(-32, -12.5, {.chain = true});
    moveToPoint(-32, -12.5, {.timeoutMs = 1500, .chain = true});
    turnToPoint(-24.5, -21, {.chain = true});
    macros::setAutoClaw(true);
    runAsync([] { moveToPoint(-24.5, -21, {.timeoutMs = 1750, .maxVolts = 3}); });
    untilGrabbed();
    macros::setAutoClaw(false);
    macros::setLiftLevel(macros::Goal::alliance, 2);
    turnToPoint(-40, -20, {.chain = true});
    moveToPoint(-40, -20, {.timeoutMs = 1000});
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    macros::setClawClosed(false);
    drive(-15, {.timeoutMs = 1500, .chain = true});
    macros::setLiftLevel(macros::Goal::alliance, 0);
    turnToPoint(-46.25, -47.5);
    resetRight(kGoalReset);
    pros::delay(100);
    drive(15, {.chain = true});
    macros::setAutoClaw(true);
    runAsync([] { moveToPoint(-46.25, -47.5, {.timeoutMs = 2000, .maxVolts = 3}); });
    untilGrabbed();
    macros::setAutoClaw(false);
    macros::setLiftLevel(macros::Goal::alliance, 3);
    turnToPoint(-40, -23, {.chain = true});
    moveToPoint(-40, -23, {.timeoutMs = 1000});
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    macros::setClawClosed(false);
}

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

// --- Skills ---

// Where the robot is when the claw shuts on a Pin in the loader. The Pin sits in the loader where
// it always does, and the claw holds it where it always does, so x is known there, as the start's
// rams know it against the Toggle: the auto cycle's spot on the loader (autocycle.cpp), the
// tracking center 9.5in out from the loader's face. Not the front sensor: its beam passes 2in
// beside the loader, and its cone (15 degrees) reads the loader's corner 3.5in short of the wall
// (the simulator's wallResetCheck()).
constexpr double kLoaderXIn = -57.0;


// Once the claw has a Pin at the loader, the lift goes up kLoaderGrabLiftDeg at once, as driver
// control's grab does (macros.cpp's kBottomClosedLiftDeg): the Pin off the floor so it doesn't
// scrape, but no higher, since the claw is still inside the loader. kLoaderLiftHoldMs later, with
// the robot backing out, it goes on up to where the Pin is going (waitOutOfLoader()).
constexpr double kLoaderGrabLiftDeg = 75.0;
constexpr std::uint32_t kLoaderLiftHoldMs = 500;

// When fromLoader() last sent the lift to kLoaderGrabLiftDeg.
std::uint32_t loaderLiftMs = 0;

// Waits out what's left of kLoaderLiftHoldMs since fromLoader() lifted its Pin off the floor, so
// the lift goes no higher until the claw is out of the loader. For the caller backing out, before
// it sends the lift to the Pin's level.
void waitOutOfLoader() {
    const std::uint32_t heldMs = pros::millis() - loaderLiftMs;
    if (heldMs < kLoaderLiftHoldMs) pros::delay(kLoaderLiftHoldMs - heldMs);
}

// How far on from where it starts fromLoader()'s run aims: far past the loader from wherever it
// starts, so only the claw shutting (or the timeout) ends it.
constexpr double kLoaderRunIn = 36.0;

// Where fromLoader()'s runs start, on the loader's line: 12in out from where the claw shuts on the
// Pin, the claw sensor reading far past kPieceDetectMm (macros.cpp) from there.
constexpr double kLoaderRunStartXIn = -45.0;
constexpr double kLoaderLineYIn = -58.75;

// After a grab at the loader, a Pin in the claw reads this close on the claw sensor. Nothing this
// close within kPinHeldCheckMs of the robot stopping, and the claw shut on air: it grabbed at the
// wrong time. The sensor measures about every 33ms, so the wait sees two of its readings.
constexpr std::int32_t kPinHeldMm = 50;
constexpr std::uint32_t kPinHeldCheckMs = 60;

// How many runs at the loader fromLoader() makes for one Pin before it gives up and the routine
// stops, as it does for a run the claw never shuts on.
constexpr int kLoaderTries = 3;

// How long after the claw opens setAutoClaw() arms at once: macros.cpp's kAutoClawClearMs. A run
// that starts sooner might reach the Pin before autoClaw's armed, and never grab.
constexpr std::uint32_t kAutoClawRearmMs = 400;

// What the left distance sensor reads, in mm, with the robot lined up on the loader facing it
// (270): the end wall, off its left side. Each run reads the sensor as it starts and aims its y to
// make it read this, whatever odometry says the y is.
constexpr std::int32_t kLoaderLeftMm = 185;

// A reading further than this off kLoaderLeftMm (about 3in) isn't the end wall where it should be:
// a game element beside the robot, or nothing seen (9999). The run then keeps to odometry's line,
// kLoaderLineYIn, as it did before it went by the sensor.
constexpr std::int32_t kLoaderLeftTrustMm = 100000;

// The left distance sensor (the localizer's west one, config.hpp's kDistanceLeft), in mm. Built
// on first use, at runtime.
std::int32_t leftSensorMm() {
    static pros::Distance sensor(robot::ports::kDistanceLeft);
    return sensor.get_distance();
}

// The y a run at the loader aims for, from `here` facing the loader (270): where the left sensor
// would read kLoaderLeftMm, by how far it reads off it now. It looks along -y facing 270, so a
// longer reading means the robot sits too far +y. kLoaderLineYIn if the reading can't be trusted
// (kLoaderLeftTrustMm). An auton,loader_line line in the log says which.
double loaderLineYIn(const sapphirelib::odom::Pose& here) {
    const std::int32_t mm = leftSensorMm();
    const std::int32_t offMm = mm - kLoaderLeftMm;
    const bool trusted = std::abs(offMm) <= kLoaderLeftTrustMm;
    const double yIn = trusted ? here.yIn - offMm / 25.4 : kLoaderLineYIn;
    sapphirelib::telemetry::event("auton", "loader_line,left_mm=%d,used=%d,odom_y=%.2f,y=%.2f",
                                  static_cast<int>(mm), trusted ? 1 : 0, here.yIn, yIn);
    return yIn;
}

// Where autoClaw grabs at the loader (setAutoClaw()'s grabMm): the claw shuts the first time its
// sensor reads this close. kLoaderGrabMm for every trip but the ones for red's other goal, by the
// near wall, which grab at kSecondGoalGrabMm.
constexpr std::int32_t kLoaderGrabMm = 100;
constexpr std::int32_t kSecondGoalGrabMm = 120;

// One run at the loader by the near end wall, from facing it (270) on or near its line: to
// kLoaderRunStartXIn at full speed, then autoClaw on, grabbing at `grabMm`, and straight ahead at
// 3.5V until the claw shuts (untilGrabbed()). Straight ahead along the y where the left
// sensor reads kLoaderLeftMm (loaderLineYIn()): the run strafes onto it on the way in. Whether
// the claw shut. autoClaw grabs only with the lift all the way down: one still coming down from a
// high level grabs once it gets there, inside the run's 2.5s.
bool loaderRun(std::int32_t grabMm) {
    moveToPose(kLoaderRunStartXIn, kLoaderLineYIn, 270, {.chain = true});
    macros::setAutoClaw(true, grabMm);
    const sapphirelib::odom::Pose start = pose();
    const double lineYIn = loaderLineYIn(start);
    runAsync([start, lineYIn] {
        moveToPose(start.xIn - kLoaderRunIn, lineYIn, 270, {.timeoutMs = 2500, .maxVolts = 3.5});
    });
    untilGrabbed();
    macros::setAutoClaw(false); // full speed for what comes next
    return macros::clawClosed();
}

// Whether the claw has a Pin after a grab: the claw sensor reads kPinHeldMm or closer within
// kPinHeldCheckMs.
bool pinHeld() {
    return sapphirelib::waitUntil([] { return macros::clawSensorMm() <= kPinHeldMm; },
                                  kPinHeldCheckMs);
}

// A Pin from the loader by the near end wall (loaderRun(), grabbing at `grabMm`). Once the claw
// has one (pinHeld()), stopped there, both axes: x from the loader (kLoaderXIn), y from the left
// sensor off the end wall (kLoaderReset). A grab with nothing within kPinHeldMm shut on air: the claw opens, the
// robot backs out to the run's start, and runs in again once autoClaw can arm (kAutoClawRearmMs),
// up to kLoaderTries runs in all, with an auton,loader_miss line in the log for each. So the
// caller never scores an empty claw, and its next level waits for a real Pin. False, with no
// resets, if a run never shut the claw, or every run missed: no Pin.
bool fromLoader(std::int32_t grabMm = kLoaderGrabMm) {
    for (int run = 1; run <= kLoaderTries; run++) {
        if (!loaderRun(grabMm)) return false;
        if (pinHeld()) {
            macros::setLiftDeg(kLoaderGrabLiftDeg); // off the floor, still inside the loader
            loaderLiftMs = pros::millis();
            const sapphirelib::odom::Pose here = pose();
            setPose(kLoaderXIn, here.yIn, here.headingDeg);
            resetLeft(kLoaderReset);
            return true;
        }
        sapphirelib::telemetry::event("auton", "loader_miss,run=%d,claw_mm=%d", run,
                                      static_cast<int>(macros::clawSensorMm()));
        macros::setClawClosed(false);
        if (run == kLoaderTries) break;
        const std::uint32_t openedMs = pros::millis();
        moveToPose(kLoaderRunStartXIn, kLoaderLineYIn, 270, kStop);
        const std::uint32_t openMs = pros::millis() - openedMs;
        if (openMs < kAutoClawRearmMs) pros::delay(kAutoClawRearmMs - openMs);
    }
    return false;
}

// Onto the goal by the left wall from below it, facing +y (0), as the auto cycle drives: to just
// short of it without stopping, then on at 6V, pressing on it until the 750ms runs out. y off the
// end wall, then the score at whatever level the lift was sent to.
void scoreOnFirstGoalFromBelow() {
    turnTo(0, kTurn);
    moveToPose(-43, -35.0, 0, {.chain = true, .exitError = 2});
    moveToPose(-43, -25.0, 0, {.timeoutMs = 750, .maxVolts = 6, .exitError = 12});
    resetBack(kGoalReset);
    scoreOnGoal();
}

// Pins from the loader onto the goal by the left wall, from `fromLevel` up to its top (ALLIANCE's
// L6), starting on that goal from the score before. Each trip: straight back off the goal to the
// loader's line (the lift down once the claw's clear of the stack), a turn left to face the loader,
// the Pin (fromLoader()), straight back out (the lift up once out of the loader), a turn right to
// face the goal, and onto it (scoreOnFirstGoalFromBelow()). False if the claw came away from the
// loader empty: the routine stops there.
bool loaderOntoFirstGoal(int fromLevel) {
    for (int level = fromLevel; level <= 6; level++) {
        runAsync([] { moveToPoint(-35, -57.75, {.chain = true}); });
        waitUntilTraveled(kLiftClearIn);
        macros::setLiftLevel(macros::Goal::alliance, 0);
        waitForMotion();
        turnTo(270, kTurn);
        if (!fromLoader()) return false;
        runAsync([] { moveToPoint(-42, -57.75, {.chain = true}); });
        waitOutOfLoader();
        macros::setLiftLevel(macros::Goal::alliance, level);
        waitForMotion();
        scoreOnFirstGoalFromBelow();
    }
    return true;
}

// How much higher than its level Skills 2 lifts each Pin onto red's other goal, above L1 (L1 at
// its level): on up to the goal with the Pin clear of the stack's top. The dip is still from the
// level (liftScoreDeg()), so the Pin goes down onto the stack as far as it would from the level.
constexpr double kSecondGoalExtraLiftDeg = 40.0;

// The end of both Skills, from on the first goal after its top score: the near wall's Toggle,
// `secondGoalPins` onto red's other goal (L1 up), each above L1 lifted `extraLiftDeg` past its
// level, and one onto the center goal.
void skillsFinish(int secondGoalPins, double extraLiftDeg = 0.0) {
    // the near wall's Toggle, rammed backwards twice as the start rams red's: straight back off the
    // goal to the loader's line, a turn right, along the end wall under the second goal, and up off
    // the wall so the turn left, to put its back to the Toggle, clears it (settled, so the rams go
    // in square). Then out and back into it, twice, and y from the wall it ends pinned against, as
    // the start sets x
    runAsync([] { moveToPoint(-47, -57.75, {.chain = true}); });
    waitUntilTraveled(kLiftClearIn);
    macros::setLiftLevel(macros::Goal::alliance, 0);
    waitForMotion();
    turnTo(90, kTurn);
    moveToPoint(-5, -58.5, {.chain = true});
    moveToPoint(5, -56, kStop);
    turnTo(0);
    for (int i = 0; i < 2; i++) {
        drive(10, {.chain = true, .exitError = 5});
        drive(-14, {.timeoutMs = 600});
    }
    const sapphirelib::odom::Pose atToggle = pose();
    setPose(atToggle.xIn, -60.6, atToggle.headingDeg);

    // back toward the loader: out off the Toggle far enough to turn clear of it, a turn left, and
    // down to the end wall's line before passing under the second goal
    moveToPoint(5, -56, kStop);
    turnTo(270, kTurn);
    moveToPoint(-10, -58.5, {.chain = true});

    // onto red's other goal, by the near wall. Each trip: the Pin, straight back out (the lift up
    // once clear of the loader), a turn right to face the goal (60), and onto it, pressing on it;
    // the score, then straight back off it the way it came (the lift down once clear) and a turn
    // left to face the loader again. Each Pin grabbed at kSecondGoalGrabMm
    for (int level = 1; level <= secondGoalPins; level++) {
        if (!fromLoader(kSecondGoalGrabMm)) return;
        runAsync([] { moveToPoint(-42, -57.75, {.chain = true}); });
        waitOutOfLoader();
        macros::setLiftLevel(macros::Goal::alliance, level);
        if (level > 1 && extraLiftDeg != 0.0) {
            macros::setLiftDeg(macros::liftLevelDeg() + extraLiftDeg);
        }
        waitForMotion();
        turnToPoint(-25, -52, kTurn);
        moveToPoint(-25, -52, {.timeoutMs = 1000});
        scoreOnGoal();
        runAsync([] { moveToPoint(-42, -57.75, {.chain = true}); });
        waitUntilTraveled(kLiftClearIn);
        macros::setLiftLevel(macros::Goal::alliance, 0);
        waitForMotion();
        turnTo(270, kTurn);
    }

    // one onto the center goal: the Pin, straight back out, a turn right to face the goal (41),
    // and the long drive onto it, about 3in clear of the second goal. Then off it, settled: the
    // routine's last motion
    if (!fromLoader()) return;
    runAsync([] { moveToPoint(-50, -57.75, {.chain = true}); });
    waitOutOfLoader();
    macros::setLiftLevel(macros::Goal::center, 2);
    waitForMotion();
    turnToPoint(0, 0, kTurn);
    moveToPoint(-3.3, -3.8, {.timeoutMs = 2500});
    scoreOnGoal();
    drive(-6);
}

// Skills 2: the Triple Pin, then Pins from the loader by the near end wall, as the auto cycle runs
// them in driver control (autocycle.cpp) but on its own: three more onto the Triple Pin's goal, to
// six (ALLIANCE's L4 to L6), then the near wall's Toggle rammed backwards twice as the start rams
// red's, three onto red's other goal, the one by the near wall at (-23.55, -47.09) (L1 to L3, the
// L2 and L3 ones lifted kSecondGoalExtraLiftDeg past their levels), and one onto the center goal
// (CENTER's L2: it starts with a Pin in it).
//
// The resets: the back sensor off the end wall on every score on the first goal, the Triple Pin's
// third included; both axes every time the claw has a Pin at the loader (fromLoader()); and y against
// the Toggle after its rams, as the start sets x. The localizer's corrections stay off, as the
// Triple Pin leaves them.
//
// Every turn by the end wall is at y = -57.75, 1in shy of the loader's line: any nearer, a corner
// hits the wall (SL000034); the simulator's field has the corners 1in clear of it there. On the
// drives along the end wall, the second goal is 1.2in from the robot's side.
void skills2() {
    // the Triple Pin, as a match runs it: L1 to L3 on the goal by the left wall. Then y off the end
    // wall while it's still on the goal from the third score
    triplePin();
    resetBack(kGoalReset);
    if (!loaderOntoFirstGoal(4)) return;
    skillsFinish(3, kSecondGoalExtraLiftDeg);
}

// --- Skills: through the gap beside the first goal to the stack ---

// The gap between the goal by the left wall and the left wall's Cups: the Cups reach 3.2in out
// from the wall (x = -67.1) and the goal's west face is at x = -49.9, 17.2in apart for a robot
// 13.75in wide. The run down it is at kGapXIn, 1in west of the middle: a Cup knocked along the
// wall costs less than the robot's side catching the goal. The swing into it aims 0.5in further
// west again (kGapEntryXIn), so its chained stop, within 1in, leaves the robot about on the line.
// In the simulator, with the robot's models and gains: the goal 1.5in from the robot's side at the
// nearest, the Cups 1.2in, the loader 2in.
constexpr double kGapXIn = -54;
constexpr double kGapEntryXIn = kGapXIn - 0.5;

// How far straight back off the goal the robot backs from the preload's score before it swings into
// the gap: 5in left a front corner 0.2in from the goal as it turned, in the simulator; 7in, 1.5in.
constexpr double kOffPreloadIn = 15.0;

// Where the robot stops beside the stack for its resets, facing the near wall (180): y from the
// front sensor off the near wall, x from the right sensor off the left wall. Not on the gap's line:
// from there, west of x = -57, the front sensor's cone takes in the near loader's corner, 9 to 13in
// short of the wall (the simulator's wallResetCheck()). Here both cones see only their walls
// anywhere within 1.5in and 4 degrees of the stop. The front beam lands on the near wall 11in from
// the corner, so a 2in corner margin; and no reset that moves the robot more than 6in, so a reading
// off the loader (the robot further west than odometry has it) is skipped rather than taken.
constexpr double kStackResetXIn = -55.5;
constexpr double kStackResetYIn = -35.5;
const sapphirelib::localization::WallResetConfig kStackReset{.cornerMarginIn = 2.0,
                                                             .maxJumpIn = 6.0};

// Where the claw is aimed for the stack, as the Triple Pin aims for it: a little past where it
// stands, so only the claw shutting (or the timeout) ends the run.
constexpr double kStackAimXIn = -46.5;
constexpr double kStackAimYIn = -40.0;

// Skills: the preload onto the goal by the left wall (L1), then down the gap between that goal and
// the left wall to beside the stack near the loader, where the front and right sensors reset both
// axes; the stack onto the same goal from below (L2), then four from the loader onto it, to six (L3
// to L6). The rest as Skills 2: the near wall's Toggle, four onto red's other goal by the near wall
// (L1 to L4), and one onto the center goal.
void skills() {
    // 1: the preload onto the goal by the left wall at L1, as the Triple Pin starts
    preloadOntoGoal();

    // 2: the gap. Straight back off the goal (the lift down once the claw's clear of the stack),
    // then a swing into the gap facing the near wall (180), down it past the goal, and on to beside
    // the stack, settled for the resets: y off the near wall ahead, x off the left wall to the right
    runAsync([] { drive(-kOffPreloadIn, kStop); });
    waitUntilTraveled(kLiftClearIn);
    macros::setLiftLevel(macros::Goal::alliance, 0);
    waitForMotion();
    moveToPose(kGapEntryXIn, -14, 180, kStop);
    moveToPose(kGapXIn, -20, 180, kStop);
    moveToPose(kStackResetXIn, kStackResetYIn, 180);
    resetFront(kStackReset);
    resetRight(kStackReset);

    // 3: the stack, as the Triple Pin takes it: a turn left to face it, then on at 3V with autoClaw
    // until the claw shuts. Then on to below the goal and onto it at L2
    turnToPoint(kStackAimXIn, kStackAimYIn);
    macros::setAutoClaw(true);
    runAsync([] { moveToPoint(kStackAimXIn, kStackAimYIn, {.timeoutMs = 2000, .maxVolts = 3}); });
    untilGrabbed();
    macros::setAutoClaw(false);
    if (!macros::clawClosed()) return; // missed it: stop here
    macros::setLiftLevel(macros::Goal::alliance, 2);
    moveToPoint(-47, kStackAimYIn, kStop);
    scoreOnFirstGoalFromBelow();

    // 4: four from the loader onto the same goal, to six (L3 to L6), then the rest as Skills 2 but
    // with four onto the second goal
    if (!loaderOntoFirstGoal(3)) return;
    skillsFinish(4);
}

} // namespace

void robot::registerAutons(sapphirelib::gui::AutonSelectorPage& selector) {
    selector.addRoutine("Do Nothing", &doNothing);
    selector.addRoutine("Triple Pin", &triplePin);
    selector.addRoutine("Skills", &skills);
    selector.addRoutine("Skills 2", &skills2);
    selector.addRoutine("Tuning Test", &tuningTest);
    selector.addRoutine("Odom Test", &odomTest);
    selector.addRoutine("Async Example", &asyncExample);
    selector.addRoutine("Path Example", &pathExample);
    selector.addRoutine("Trajectory Example", &trajectoryExample);
    selector.addRoutine("Wall Reset Test", &wallResetTest);
    selector.addRoutine("Lift Testing", &liftTesting);
    selector.addRoutine("Lift Example", &liftExample);
}
