/**
 * \file robot/telemetry.hpp
 *
 * The robot's SD-card log: which channels it records and where the files go.
 * The file format, and what every channel below means for tuning, is in
 * docs/TELEMETRY_FORMAT.md.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include "sapphirelib/input/controller.hpp"
#include "sapphirelib/telemetry/logger.hpp"

namespace robot {

/// The one Logger for this program. Files go to /usd/sl/SLnnnnnn.CSV, one per
/// program run. Make the "sl" folder at the card's root on a computer once
/// (the V5 can't create folders); without it, logs land in the root instead
/// and the Home page says so.
sapphirelib::telemetry::Logger& logger();

/// Registers every channel this robot logs: the drivetrain's three PIDs
/// ("drive", "turn", "hold"), the odometry pose ("odom"), the drivetrain's
/// applied axis volts ("chassis"), the battery ("batt"), every motor's health
/// ("motor.*"), the driver's controller ("driver", see logDriver()), and the
/// macros' channels (macros::attachTelemetry(): "lift", "lift.act", "mech"),
/// then starts the logger's tasks.
///
/// Call once from initialize(), after initDevices() and before anything can
/// run a motion or opcontrol: attaching a PID's observer isn't synchronized
/// with a task running that PID, and registration takes a mutex that a
/// competition task deleted mid-call would leave locked. start() itself only
/// spawns two tasks (the card check and file open happen on the logger's
/// own writer task), so a missing card costs initialize() nothing.
void startTelemetry();

/// Records this tick's controller sample in the "driver" channel: both
/// sticks, every button held (as a bitmask, bit i = input::Button i), and
/// whether the controller is connected. Call once per opcontrol tick, right
/// after controller.update(), a robot that stops answering its driver is
/// either this (connected drops to 0) or everything downstream of it, and the
/// log should say which.
void logDriver(const sapphirelib::input::Controller& controller);

} // namespace robot
