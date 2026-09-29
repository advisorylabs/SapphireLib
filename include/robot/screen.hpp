/**
 * \file robot/screen.hpp
 *
 * The brain screen: SapphireLib's GUI with this robot's pages. Other files
 * never see a page pointer; they go through the few things the robot
 * actually needs from the screen, below.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/gui/gui.hpp"

namespace robot {

/// The GUI. The first call builds it and paints the branded header, so make
/// that the first thing initialize() does after sapphirelib::initialize().
sapphirelib::gui::Gui& screen();

/// Adds every page — Home, Auton, Diagnostics, PID tuning, Odom — and starts
/// the screen refreshing. Call once from initialize(), after initDevices()
/// (the pages need the devices) and startTelemetry() (the Home page shows
/// the SD card's state, and the tuner's Auto-Tune axes log into channels).
void buildScreen();

/// Runs the routine picked on the Auton tab, logging its start and end (to
/// the terminal, and as `auton` telemetry events the log reader cuts runs
/// on). Does nothing before buildScreen().
void runSelectedAuton();

/// True while a page is driving the chassis on its own: the Odom page's
/// offset calibration spin, or a PID tuner test or Auto-Tune run. Driver
/// control must leave the drivetrain alone meanwhile — its every-tick drive
/// call would immediately overwrite the routine's commanded voltage with
/// whatever the (likely centered) sticks read, breaking the routine (see
/// OdometryPage::isCalibrating()'s comment for the failure mode in detail).
bool screenBusy();

} // namespace robot
