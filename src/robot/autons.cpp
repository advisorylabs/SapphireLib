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

#include <cstdint>
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

// Background motions: the extender goes out 12in into a drive and comes
// back in near the end, while the robot keeps driving. Then a turn around
// that has to go right, and home. Needs about 3ft clear ahead.
void asyncExample() {
    setPose(0, 0, 0);
    runAsync([] { moveToPoint(0, 36); });
    waitUntilTraveled(12);      // 12in into the move...
    macros::setExtended(true);  // ...push the extender out
    waitUntilNear(0, 36, 6);    // 6in from the end...
    macros::setExtended(false); // ...pull it back in
    waitForMotion();
    turnTo(180, clockwise);
    moveToPoint(0, 0);
}

// Pure pursuit through points: an S-curve out to the right, then straight
// back to the start. Needs about 4ft ahead and 2ft to the right.
void pathExample() {
    setPose(0, 0, 0);
    followPath({{0, 0}, {0, 16}, {16, 28}, {16, 44}});
    moveToPoint(0, 0);
    turnTo(0);
}

// A trajectory as the simulator's Paths tab writes one (Copy C++), with
// things done at its points without slowing down: out 24in with the
// extender going out as it passes point 1, a stop at point 2 where the claw
// opens and the extender comes back in, then straight back to the start
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

} // namespace

void robot::registerAutons(sapphirelib::gui::AutonSelectorPage& selector) {
    selector.addRoutine("Do Nothing", &doNothing);
    selector.addRoutine("Tuning Test", &tuningTest);
    selector.addRoutine("Odom Test", &odomTest);
    selector.addRoutine("Async Example", &asyncExample);
    selector.addRoutine("Path Example", &pathExample);
    selector.addRoutine("Trajectory Example", &trajectoryExample);
    selector.addRoutine("Wall Reset Test", &wallResetTest);
    selector.addRoutine("Lift Testing", &liftTesting);
}
