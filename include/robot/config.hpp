/**
 * \file robot/config.hpp
 *
 * Where everything on 96671H's robot is plugged in, plus the drivetrain and
 * odometry geometry. Each port is written here and nowhere else: the
 * drivetrain, the tracking wheels, the driver macros, the Lift Testing auton
 * and the Diagnostics page all read these, so moving a cable means changing
 * one number. As everywhere in PROS, a negative smart port reverses the
 * device on it.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <cstdint>

namespace robot::ports {

// --- Drivetrain (see devices.cpp) ---
// The four mecanum/X-drive corners, plus the Asterisk's straight-facing
// center pair. The right side is reversed: with the left reversed instead,
// as on the test chassis, the whole drive ran backward. With the wheels off
// the ground, full forward on the left stick must spin all six forward; if
// one wheel doesn't, flip just its sign.
constexpr std::int8_t kFrontLeft = 3;
constexpr std::int8_t kFrontRight = -15;
constexpr std::int8_t kBackLeft = 14;
constexpr std::int8_t kBackRight = -6;
constexpr std::int8_t kMiddleLeft = 2;
constexpr std::int8_t kMiddleRight = -20;
constexpr std::uint8_t kImu = 19;

// --- Odometry tracking wheels (rotation sensors) ---
// Vertical is reversed, as it had to be on the test chassis; see its
// constructor in devices.cpp. TODO: confirm both on this robot: on the Odom
// page, driving forward must raise Y and strafing right must raise X.
// Negate whichever one goes the wrong way.
constexpr std::int8_t kVerticalTracking = -11;
constexpr std::int8_t kHorizontalTracking = 1;

// --- Intake, roller claw, and lift (macros.cpp) ---
// Intaking (+) spins the intake and the roller claw "clockwise" in the
// driver's sense of it; the intake's port is reversed so that's +. If either
// spins outward on R1, negate its port.
constexpr std::int8_t kIntake = -21;
constexpr std::int8_t kClawMotor = 17; // the roller claw
// The two lift motors are geared straight to each other, so they spin
// opposite ways: A clockwise going up, B counterclockwise. B is reversed so
// + is up for both; with both the same sign they fight each other and lock
// up. If L1 in debug mode drives the lift down, negate both.
constexpr std::int8_t kLiftA = 7;
constexpr std::int8_t kLiftB = -9;
// Negate this if the sensor's reading goes down as the lift goes up.
constexpr std::int8_t kLiftSensor = 16;
// Sees a pin in the roller claw.
constexpr std::uint8_t kClawDistance = 12;
// The two pistons, on the brain's 3-wire ports (not smart ports). G deploys
// the claw, the moment L1 or Y takes the lift off the bottom. H deploys once
// the lift reaches the top of its travel: its goal's top level. In debug
// mode, A and X toggle them by hand (see macros.hpp).
constexpr char kClawPiston = 'G';
constexpr char kTopPiston = 'H';

// --- Localization distance sensors (devices.cpp) ---
// One per side, for Monte Carlo localization. The team calls them by
// compass point, with north the robot's front.
constexpr std::uint8_t kDistanceFront = 8;  // north
constexpr std::uint8_t kDistanceRight = 10; // east
constexpr std::uint8_t kDistanceBack = 18;  // south
constexpr std::uint8_t kDistanceLeft = 5;   // west

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
// logs each sensor's {.forwardIn, .rightIn} to copy here.
constexpr double kFrontSensorForwardIn = 7.0;
constexpr double kFrontSensorRightIn = 0.0;
constexpr double kRightSensorForwardIn = 0.0;
constexpr double kRightSensorRightIn = 7.0;
constexpr double kBackSensorForwardIn = -7.0;
constexpr double kBackSensorRightIn = 0.0;
constexpr double kLeftSensorForwardIn = 0.0;
constexpr double kLeftSensorRightIn = -7.0;

} // namespace robot
