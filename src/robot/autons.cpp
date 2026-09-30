/**
 * \file autons.cpp
 *
 * Every autonomous routine, and registerAutons(), which lists them on the
 * brain screen. autonomous() runs whichever one is picked there (see
 * runSelectedAuton() in screen.cpp); so does B+DOWN off the field.
 *
 * Team 96671H: Hitmen
 */

#include "robot/autons.hpp"

#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "robot/config.hpp"
#include "robot/devices.hpp"

namespace robot {

using sapphirelib::gui::AutonSelectorPage;

namespace {

void doNothingAuton() {}

// The lift's own motors, driven directly (the driver macros' lift loop only
// runs from opcontrol, so the two never command them at once).
pros::Motor winch(ports::kLiftA);
pros::Motor winch2(ports::kLiftB);
void liftTesting() {
    winch.set_encoder_units_all(pros::v5::MotorUnits::degrees);
    winch2.set_encoder_units_all(pros::v5::MotorUnits::degrees);
    // winch is reversed (port -20), so its position counts UP while moving at
    // +127. Zero it so the thresholds are relative to where the lift starts.
    winch.tare_position();
    winch2.tare_position();
    for (int i = 0; i < 18; i++) {
        winch.move(127);
        winch2.move(127);
        while (winch.get_position() < 1000) {
            pros::delay(2);
        }
        winch.move(-127);
        winch2.move(-127);
        while (winch.get_position() > 15) {
            pros::delay(2);
        }
        winch.move(0);
        winch2.move(0);
        pros::delay(6000);
    }
}

void driveForwardAuton() { drivetrain().driveDistance(24); }

void turnTestingAuton() {
    drivetrain().turnToHeading(90);
    pros::delay(1000);
    drivetrain().turnToHeading(0);
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
}

} // namespace robot
