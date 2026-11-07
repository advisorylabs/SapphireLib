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

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

#include "pros/rtos.hpp"
#include "robot/autons.hpp"
#include "robot/macros.hpp"
#include "sapphirelib/auton/auton.hpp"
#include "sapphirelib/motion/trajectory_config.hpp"

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
    moveToPoint(-50, -5, {.timeoutMs = 1000});
    macros::setLiftLevel(macros::Goal::alliance, 0);
    turnTo(105);
    moveToPoint(-35.5, -10.8, {.timeoutMs = 1000});
    macros::setAutoClaw(true, 75);
    moveToPose(-22, -17.5, 133, {.timeoutMs = 1750});
    macros::setLiftLevel(macros::Goal::alliance, 2);
    turnTo(255);
    moveToPoint(-47, -18, {.timeoutMs = 750});
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    macros::setClawClosed(false);
    drive(-12, {.timeoutMs = 1500});
    macros::setLiftLevel(macros::Goal::alliance, 0);
    turnToPoint(-43, -45);
    drive(15, {.chain = true});
    macros::setAutoClaw(true, 65);
    moveToPoint(-43, -45, {.timeoutMs = 2000}); 
    macros::setLiftLevel(macros::Goal::alliance, 3);
    turnTo(0);
    moveToPoint(-43, -24, {.timeoutMs = 1500});
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    macros::setClawClosed(false);


}

/// Triple Pin on trajectories: an example of triplePin() above with every drive planned in the
// simulator's Paths tab, and a wall reset wherever a distance sensor sees clean wall. Not run on
// the robot yet; the simulator runs it (Drive tab, "Triple Pin (paths)"), with these paths and
// config.hpp's sensors, and tools/sim/js/routines.js explains each choice at length.
//
// Three Pins stacked on the alliance Goal by the left wall, at (-47.09, -23.55), "the goal": the
// preload at L1, the Pin standing at (-23.55, -23.55) at L2, the one at (-47.09, -47.09) at L3. The
// claw holds a Pin about 5.3in ahead of the tracking center (from where triplePin() grabs its
// second Pin), so the robot stops 5.3in short of the goal's middle, facing it, settling to half an
// inch (kTriplePinScoreFollow). At a Pin it aims an inch past that, 4.3in short, the back of the
// claw stopping the Pin: autoClaw shuts the claw at 125mm, the Pin about 5.7in ahead, and a robot
// aiming at 5.3in that settles short, or was bumped back, never gets there. Each approach ends in
// a short straight run at no more than 24 in/s.
//
// The resets: only where something that needs the pose follows, and only where the simulator's
// wallResetCheck() finds the beam's whole cone on clean wall, an inch and 3 degrees either way
// (the localizer's own checks can't tell a Cup, a Pin or a Goal in the beam from the wall, and a
// reading 2in short passes them):
//   - nowhere before the third Pin. After the rams the back sensor is on the Toggle (where
//     SL000040 read the wall 3in long) and the side ones are within 18in of the corners; above
//     and on the goal from the north the Cup-Pin-Cup by the left wall is in the cone; around the
//     second Pin the two Goals block the walls. On the goal from the east the near wall is clear
//     but 42in off, and a reset there can't move a robot that has already stopped: the back-off
//     after it would steer the correction out sideways with the claw still around the stack.
//   - y at the stop east of the third Pin, for the run onto it (not x: the front sensor's beam is
//     along that run).
//   - x and y on the third Pin, still, for the drive back to the goal.
// Each is averaged over 4 readings (averagedResets()): one reading at 28in is +-0.7in in the
// simulator's noise, enough to miss the goal by itself. The simulator says what they buy: a 1in
// bump on the way to the third Pin costs it the Pin without them, and nothing with them; with no
// bump they cost the last score about half an inch of the inch and a quarter it has. Measure the
// sensors' real noise from a log (the simulator's Tune tab) to know how many readings they need.
//
// The kTriplePin... paths are the Paths tab's: Paths > Paste C++ reads them all at once, a path
// each, to see them on the field and change them; Copy C++ writes one back. Planned for a robot up
// to 17in square: the gap between the two alliance Goals the third Pin's legs use is 18in.
const sapphirelib::motion::TrajectoryFollowConfig kTriplePinScoreFollow{
    .positionErrorThresholdIn = 0.5};

// Sapphire Sim path "triple pin to goal"
const std::vector<sapphirelib::motion::TrajectorySection> kTriplePinToGoal{
    {.waypoints = {{.xIn = -60.6, .yIn = -5, .headingDeg = 90},
                   {.xIn = -52, .yIn = -7},
                   {.xIn = -47.09, .yIn = -12.25, .headingDeg = 180}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -47.09, .yIn = -18.25}},
     .constraints = {.forward = {.maxVelocity = 24}}},
};

// Sapphire Sim path "triple pin to second pin"
const std::vector<sapphirelib::motion::TrajectorySection> kTriplePinToSecondPin{
    {.waypoints = {{.xIn = -47.09, .yIn = -18.25, .headingDeg = 180},
                   {.xIn = -47.09, .yIn = -12.25}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -35, .yIn = -11},
                   {.xIn = -23.55, .yIn = -12.25}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -23.55, .yIn = -19.25}},
     .constraints = {.forward = {.maxVelocity = 24}}},
};

// Sapphire Sim path "triple pin second to goal"
const std::vector<sapphirelib::motion::TrajectorySection> kTriplePinSecondToGoal{
    {.waypoints = {{.xIn = -23.55, .yIn = -19.25, .headingDeg = 180},
                   {.xIn = -30, .yIn = -21},
                   {.xIn = -35.3, .yIn = -23.55, .headingDeg = 270}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -41.79, .yIn = -23.55}},
     .constraints = {.forward = {.maxVelocity = 24}}},
};

// Sapphire Sim path "triple pin to third pin"
const std::vector<sapphirelib::motion::TrajectorySection> kTriplePinToThirdPin{
    {.waypoints = {{.xIn = -41.79, .yIn = -23.55, .headingDeg = 270},
                   {.xIn = -35.3, .yIn = -23.55}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -35.3, .yIn = -35},
                   {.xIn = -37.5, .yIn = -47.09}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -42.79, .yIn = -47.09}},
     .constraints = {.forward = {.maxVelocity = 24}}},
};

// Sapphire Sim path "triple pin third to goal"
const std::vector<sapphirelib::motion::TrajectorySection> kTriplePinThirdToGoal{
    {.waypoints = {{.xIn = -42.79, .yIn = -47.09, .headingDeg = 270},
                   {.xIn = -36, .yIn = -40},
                   {.xIn = -35.3, .yIn = -23.55}},
     .stopAtEnd = true},
    {.waypoints = {{.xIn = -41.79, .yIn = -23.55}},
     .constraints = {.forward = {.maxVelocity = 24}}},
};

// Sapphire Sim path "triple pin off goal"
const std::vector<sapphirelib::motion::TrajectorySection> kTriplePinOffGoal{
    {.waypoints = {{.xIn = -41.79, .yIn = -23.55, .headingDeg = 270},
                   {.xIn = -35.79, .yIn = -23.55}}},
};

// R2's score from a routine: up at the level, the dip, and let go.
void scoreOnGoal() {
    macros::waitForLift(1500);
    macros::setLiftDeg(macros::liftScoreDeg());
    macros::waitForLift(1000);
    macros::setClawClosed(false);
    pros::delay(100); // the claw open before backing off
}

// What a wall reset sets, for averagedResets().
enum class ResetAxis : std::uint8_t { x, y };

struct WallReading {
    bool (*reset)(); // resetFront and the rest
    ResetAxis axis;  // what it sets
};

double poseAlong(ResetAxis axis) {
    const sapphirelib::odom::Pose here = pose();
    return axis == ResetAxis::x ? here.xIn : here.yIn;
}

// Wall resets averaged over several readings, with the auton functions as they are: resetFront()
// and the rest each set x or y from one reading alone, so `readings` of them 40ms apart (the
// sensors read about every 33ms), averaged, average the readings, and four halve one reading's
// noise. Each is carried forward to now by what odometry has moved since, so the robot may move
// across the beams meanwhile, but not along them. Every wall is read every round; a reading a
// reset skips (its log line says why) is left out. Returns how many took.
int averagedResets(std::initializer_list<WallReading> walls, int readings = 4) {
    struct Sum {
        double total = 0.0; // each reading's x or y, carried forward to now
        int count = 0;
        double last = 0.0; // odometry's x or y when it last looked
    };
    std::vector<Sum> sums(walls.size());
    int took = 0;
    for (int i = 0; i < readings; i++) {
        if (i > 0) pros::delay(40);
        std::size_t k = 0;
        for (const WallReading& wall : walls) {
            Sum& sum = sums[k++];
            sum.total += sum.count * (poseAlong(wall.axis) - sum.last);
            if (wall.reset()) {
                sum.total += poseAlong(wall.axis);
                sum.count++;
                took++;
            }
            sum.last = poseAlong(wall.axis);
        }
    }
    sapphirelib::odom::Pose averaged = pose();
    std::size_t k = 0;
    for (const WallReading& wall : walls) {
        const Sum& sum = sums[k++];
        if (sum.count == 0) continue;
        const double value =
            (sum.total + sum.count * (poseAlong(wall.axis) - sum.last)) / sum.count;
        (wall.axis == ResetAxis::x ? averaged.xIn : averaged.yIn) = value;
    }
    setPose(averaged.xIn, averaged.yIn, averaged.headingDeg);
    return took;
}

void triplePinPaths() {
    setPose(-60.6, -5, 90);
    setMcl(false); // as in triplePin(): the resets do the correcting
    // planned now, a few ms each, rather than once moving
    for (const std::vector<TrajectorySection>* leg :
         {&kTriplePinToGoal, &kTriplePinToSecondPin, &kTriplePinSecondToGoal, &kTriplePinToThirdPin,
          &kTriplePinThirdToGoal, &kTriplePinOffGoal}) {
        if (!planTrajectory(*leg)) return;
    }
    macros::setLiftLevel(macros::Goal::alliance, 1);
    // the Toggle, twice, as triplePin() rams it, and x from the wall it ends pinned against
    for (int i = 0; i < 2; i++) {
        drive(10, {.chain = true, .exitError = 1});
        drive(-14, {.timeoutMs = 750});
    }
    const sapphirelib::odom::Pose here = pose();
    setPose(-60.6, here.yIn, here.headingDeg);

    // 1: the preload, onto the goal from the north at L1
    if (!followTrajectory(kTriplePinToGoal, kTriplePinScoreFollow).settled()) return;
    scoreOnGoal();

    // 2: straight back off the goal (point 1) before the lift comes down, then autoClaw grabs the
    // second Pin on the way in. Onto the goal from the east at L2
    runAsync([] { followTrajectory(kTriplePinToSecondPin); });
    if (!waitUntilWaypoint(1)) return;
    macros::setLiftLevel(macros::Goal::alliance, 0);
    macros::setAutoClaw(true);
    waitForMotion();
    if (!macros::clawClosed()) return; // missed it: stop here
    macros::setLiftLevel(macros::Goal::alliance, 2);
    followTrajectory(kTriplePinSecondToGoal, kTriplePinScoreFollow);
    scoreOnGoal();

    // 3: back off east, down the gap between the Goals to a stop east of the third Pin (point 3),
    // and y off the near wall there while it runs west onto the Pin. On it, x and y again, still,
    // and onto the goal at L3
    runAsync([] { followTrajectory(kTriplePinToThirdPin); });
    if (!waitUntilWaypoint(1)) return;
    macros::setLiftLevel(macros::Goal::alliance, 0);
    macros::setAutoClaw(true);
    if (!waitUntilWaypoint(3)) return;
    averagedResets({{&resetLeft, ResetAxis::y}});
    waitForMotion();
    if (!macros::clawClosed()) return;
    averagedResets({{&resetFront, ResetAxis::x}, {&resetLeft, ResetAxis::y}});
    macros::setLiftLevel(macros::Goal::alliance, 3);
    followTrajectory(kTriplePinThirdToGoal, kTriplePinScoreFollow);
    scoreOnGoal();

    // off the goal, so driver control starts clear of it, the lift coming down once it's out
    runAsync([] { followTrajectory(kTriplePinOffGoal); });
    waitUntilTraveled(3);
    macros::setLiftLevel(macros::Goal::alliance, 0);
    waitForMotion();
}

} // namespace

void robot::registerAutons(sapphirelib::gui::AutonSelectorPage& selector) {
    selector.addRoutine("Do Nothing", &doNothing);
    selector.addRoutine("Triple Pin", &triplePin);
    selector.addRoutine("Triple Pin (paths)", &triplePinPaths);
    selector.addRoutine("Tuning Test", &tuningTest);
    selector.addRoutine("Odom Test", &odomTest);
    selector.addRoutine("Async Example", &asyncExample);
    selector.addRoutine("Path Example", &pathExample);
    selector.addRoutine("Trajectory Example", &trajectoryExample);
    selector.addRoutine("Wall Reset Test", &wallResetTest);
    selector.addRoutine("Lift Testing", &liftTesting);
    selector.addRoutine("Lift Example", &liftExample);
}
