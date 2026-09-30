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
// center pair.
constexpr std::int8_t kFrontLeft = -3;
constexpr std::int8_t kFrontRight = 8;
constexpr std::int8_t kBackLeft = -17;
constexpr std::int8_t kBackRight = 10;
constexpr std::int8_t kMiddleLeft = -2;
constexpr std::int8_t kMiddleRight = 9;
constexpr std::uint8_t kImu = 7;

// --- Odometry tracking wheels (rotation sensors) ---
// Vertical is reversed on purpose; see its constructor in devices.cpp before
// changing the sign.
constexpr std::int8_t kVerticalTracking = -13;
constexpr std::int8_t kHorizontalTracking = 14;

// --- Intake, claw, and lift (macros.cpp) ---
// TODO: only the lift motor ports are real (the same winch motors the Lift
// Testing auton uses). Every other port here is a placeholder; set it to
// where that device is actually plugged in, then uncomment its entry in the
// Diagnostics page's list (screen.cpp).
constexpr std::int8_t kIntake = -18;   // reversed, so intaking (+) spins it counterclockwise
constexpr std::int8_t kClawMotor = 15; // intaking (+) spins it clockwise
constexpr std::int8_t kLiftA = -20;
constexpr std::int8_t kLiftB = 19;
// Negate this if the sensor's reading goes down as the lift goes up.
constexpr std::int8_t kLiftSensor = 16;
constexpr std::uint8_t kClawDistance = 5;
constexpr char kClawPiston = 'A'; // a 3-wire port on the brain, not a smart port

// --- Localization distance sensors (devices.cpp) ---
// One per side, for Monte Carlo localization. TODO: these are placeholders on
// free ports; set each to where that sensor is really plugged in, then
// uncomment its entry in the Diagnostics page's list (screen.cpp). A port with
// no distance sensor on it just reads nothing, and the localizer leaves
// odometry alone without readings, so a wrong port is harmless, only useless.
constexpr std::uint8_t kDistanceFront = 1;
constexpr std::uint8_t kDistanceRight = 4;
constexpr std::uint8_t kDistanceBack = 6;
constexpr std::uint8_t kDistanceLeft = 11;

} // namespace robot::ports

namespace robot {

// --- Drivetrain and odometry geometry ---
constexpr double kDriveWheelDiameterIn = 4.0;

// TODO: measure your actual tracking wheels and set this to their real
// diameter (2.75in is just a common off-the-shelf omni size), odometry
// distance is directly proportional to this value.
constexpr double kTrackingWheelDiameterIn = 2.75;

// Each tracking wheel's mounting offset from the tracking center, in inches
// (see OdometryConfig). TODO: re-check these whenever a wheel is remounted,
// measure them, or use the Odom page's "Calibrate Offsets", which applies
// what it measures to the running odometry and shows the numbers, but
// doesn't change this file: copy them in here to keep them.
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
constexpr double kFrontSensorForwardIn = 7.0;
constexpr double kFrontSensorRightIn = 0.0;
constexpr double kRightSensorForwardIn = 0.0;
constexpr double kRightSensorRightIn = 7.0;
constexpr double kBackSensorForwardIn = -7.0;
constexpr double kBackSensorRightIn = 0.0;
constexpr double kLeftSensorForwardIn = 0.0;
constexpr double kLeftSensorRightIn = -7.0;

} // namespace robot
