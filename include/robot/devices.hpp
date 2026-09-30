/**
 * \file robot/devices.hpp
 *
 * The robot's drivetrain, tracking wheels, and odometry, shared by every
 * file of the robot program (autons, driver control, the screen, tuning,
 * telemetry) through accessors instead of global pointers.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include "sapphirelib/chassis/holonomic_drivetrain.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/rotation_tracking_wheel.hpp"

namespace robot {

/// Constructs every device in a fixed order, starts the odometry task, and
/// zeroes the lift (see macros::initialize()). Call once from initialize(),
/// right after robot::screen(): the drivetrain blocks for ~2-3s on IMU
/// calibration, and the header should already be painted by then, so a
/// fault here reads as "initialize() stopped here" rather than "the GUI is
/// broken".
void initDevices();

/// Each one is built on first use (a function-local static), so there is
/// never a null to check, and initDevices() just makes "first use" happen at
/// a known moment. Don't move them to namespace scope: static initialization
/// runs before initialize() and before the screen paints, and the drivetrain's
/// constructor blocks on IMU calibration.
///
/// First use isn't guarded against two tasks racing to construct the same
/// one, which is fine as long as initDevices() runs first: every task that
/// calls these (odometry, the GUI's routines, autonomous, opcontrol) starts
/// after it returns.
sapphirelib::chassis::HolonomicDrivetrain& drivetrain();
sapphirelib::odom::Odometry& odometry();
sapphirelib::odom::RotationTrackingWheel& verticalWheel();
sapphirelib::odom::RotationTrackingWheel& horizontalWheel();

} // namespace robot
