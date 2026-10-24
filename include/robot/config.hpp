/**
 * \file robot/config.hpp
 *
 * Where everything on 96671H's robot is plugged in, plus the drivetrain and
 * odometry geometry. Each port is written here and nowhere else: the
 * drivetrain, the tracking wheels, the driver macros, the Lift Testing auton,
 * the telemetry and the Diagnostics page all read these, so moving a cable
 * means changing one number, and adding or removing a lift motor one entry in
 * kLiftMotors. As everywhere in PROS, a negative smart port reverses the
 * device on it.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <array>
#include <cstdint>

namespace robot::ports {

// --- Drivetrain (see devices.cpp) ---
// The four mecanum/X-drive corners, plus the Asterisk's straight-facing
// center pair. The right side is reversed: with the left reversed instead,
// as on the test chassis, the whole drive ran backward. With the wheels off
// the ground, full forward on the left stick must spin all six forward; if
// one wheel doesn't, flip just its sign.
constexpr std::int8_t kFrontLeft = 9;
constexpr std::int8_t kFrontRight = -19;
constexpr std::int8_t kBackLeft = 2;
constexpr std::int8_t kBackRight = -20;
constexpr std::int8_t kMiddleLeft = 5;
constexpr std::int8_t kMiddleRight = -7;
constexpr std::uint8_t kImu = 4;

// --- Odometry tracking wheels (rotation sensors) ---
// Vertical is reversed, as it had to be on the test chassis; see its
// constructor in devices.cpp. TODO: confirm both on this robot: on the Odom
// page, driving forward must raise Y and strafing right must raise X.
// Negate whichever one goes the wrong way.
constexpr std::int8_t kVerticalTracking = 6;
constexpr std::int8_t kHorizontalTracking = 3;

// --- Lift, claw, and extender (macros.cpp) ---
// The lift's motors, all driven together, as many as the lift has: add or
// remove one here and the lift, the Diagnostics page, the telemetry and the
// controller's motor readouts (LEFT) all follow. They're lettered A, B, C...
// in this order everywhere. + must be up for every one: motors geared
// straight to each other spin opposite ways, so one of each such pair is
// reversed, and with the signs wrong they fight each other and lock up. If L1
// in debug mode drives the lift down, negate them all.
//
// There's no lift sensor: the position is these motors' own encoders, in
// motor degrees, zeroed by the bottom switch below.
//
// After moving one, check each motor on its own (Diagnostics, then the LEFT
// readout's volts in debug mode).
constexpr std::array<std::int8_t, 3> kLiftMotors{14, -13, -10};
// A limit switch the lift presses at the bottom of its travel, on a 3-wire
// port. Each press zeroes the lift's encoders, and a lift heading for the
// bottom drives down until it's pressed.
constexpr char kLiftBottomSwitch = 'A';
// Sees a pin in the claw: autoClaw closes on it at the bottom, and the Branch
// autons pick their branch by it.
constexpr std::uint8_t kClawDistance = 15;
// The two pistons, on 3-wire ports (not smart ports). R1 opens and closes the
// claw, a score opens it, and autoClaw closes it; Y pushes the extender out
// and pulls it back in (see macros.hpp).
constexpr char kClawPiston = 'B';
constexpr char kExtenderPiston = 'H';

// --- Localization distance sensors (devices.cpp) ---
// One per side, for Monte Carlo localization. The team calls them by
// compass point, with north the robot's front.
constexpr std::uint8_t kDistanceFront = 21;  // north
constexpr std::uint8_t kDistanceRight = 8; // east
constexpr std::uint8_t kDistanceBack = 11;  // south
constexpr std::uint8_t kDistanceLeft = 12;   // west

} // namespace robot::ports

namespace robot {

// --- Drivetrain and odometry geometry ---
constexpr double kDriveWheelDiameterIn = 4.0;

// TODO: measure your actual tracking wheels and set this to their real
// diameter (2.75in is just a common off-the-shelf omni size), odometry
// distance is directly proportional to this value.
constexpr double kTrackingWheelDiameterIn = 2.75;

// Each tracking wheel's mounting offset from the tracking center, in inches
// (see OdometryConfig). TODO: these are the test chassis's numbers, so they're
// wrong for the competition robot until it's measured: run the Odom page's
// "Calibrate Offsets", which applies what it measures to the running odometry
// and shows the numbers, but doesn't change this file: copy them in here to
// keep them. Orbit mode (driver.cpp) leans on these too: they're how the
// tracking wheels tell turning from moving, so wrong ones make every turn of
// an orbit read as drifting off the circle, and the radius hold "corrects" it.
//
// Mind the vertical wheel's sign, which is the opposite of what you'd guess
// (see OdometryConfig): positive is LEFT of center. A vertical wheel on the
// robot's right side rolls backward on a right turn, so it needs a negative
// offset. The horizontal wheel is the intuitive way round: positive is
// ahead. So 3.59 below says the vertical wheel is 3.59in left of center.
// Nothing records whether it was calibrated (which gets the sign right on
// its own) or measured by hand; if the wheel is really on the right, it
// should be -3.59, and every 90 degree turn is putting about 11in of
// phantom travel into the pose. TODO: check once on the robot: on the Odom
// page, spin in place a few turns and watch x/y, which should barely move;
// or run "Calibrate Offsets" and compare its signs with these.
constexpr double kVerticalWheelOffsetIn = 3.59;
constexpr double kHorizontalWheelOffsetIn = 4.18;

// Where each localization distance sensor's face sits, measured from the
// tracking center (see DistanceSensorMount): inches forward (negative is
// behind) and right (negative is left). Measure to the face of the sensor,
// the spot its reading starts from; an inch off here is an inch of bias in
// the localizer's estimate along that sensor's direction. TODO: measure them;
// these are placeholders for sensors centered on each side of a ~14in chassis.
// Or measure them roughly and let the Odom page's "Calibrate Sensors" find
// them: it applies what it finds until the program restarts, and the terminal
// logs each sensor's {.forwardIn, .rightIn} to copy here. From a wall or a
// corner (no full field) it keeps one length from here, so measure it across
// the robot, face to face: front to back (kFrontSensorForwardIn -
// kBackSensorForwardIn) plus right to left (kRightSensorRightIn -
// kLeftSensorRightIn). It finds everything else, the tracking center too.
constexpr double kFrontSensorForwardIn = 7.0;
constexpr double kFrontSensorRightIn = 0.0;
constexpr double kRightSensorForwardIn = 0.0;
constexpr double kRightSensorRightIn = 7.0;
constexpr double kBackSensorForwardIn = -7.0;
constexpr double kBackSensorRightIn = 0.0;
constexpr double kLeftSensorForwardIn = 0.0;
constexpr double kLeftSensorRightIn = -7.0;

} // namespace robot
