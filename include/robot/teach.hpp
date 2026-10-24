/**
 * \file robot/teach.hpp
 *
 * Teach mode: drive a route by hand once, and replay it as an autonomous
 * routine (docs/TEACH.md).
 *
 * X+A in driver control (pressed together; off the field only) starts
 * recording, and X+A again stops it; so does the Home page's Teach button.
 * What's recorded is where the robot went (odometry's pose, every 20ms tick)
 * and every button the driver held, not the sticks: the replay drives the
 * same path as a trajectory, with the feedback, odometry and the localizer
 * correcting it like any other, so a low battery or a start a little off
 * doesn't send it somewhere else. The buttons go back into the driver macros
 * on the trajectory's clock, so the claw, lift and extender do what the
 * driver did, where they did it, even if the robot was held up on the way.
 *
 * Stopping saves the drive to the SD card as TEACH.TXT (next to the logs, or
 * the card's root), and the next power-on loads it again. The "Taught Route"
 * auton replays it. Start teaching where the routine will start, with the
 * mechanisms as a match starts them (lift down, claw closed on a preload,
 * extender in), and put the robot back on that spot before replaying.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <string>

#include "sapphirelib/input/controller.hpp"

namespace robot::teach {

/// Loads TEACH.TXT, if there's one, so Taught Route can replay it. Call once
/// from initialize(), before startTelemetry(): the logger then has the card
/// to itself.
void load();

/// Every opcontrol tick, after drive(): X+A (or the Home page's button)
/// starts and stops recording, and while recording, this tick's pose and
/// buttons are kept. A restart (autonomous, a disable) ends a recording.
void update(sapphirelib::input::Controller& controller);

/// Whether a drive is being recorded. Safe from any task.
bool recording();

/// Asks update() to start or stop recording on its next tick, as X+A does.
/// For the Home page's button; safe from any task.
void requestToggle();

/// The Home page's "Teach:" line, e.g. "Teach: 12.4s, replays at 87%".
/// Safe from any task.
std::string statusText();

/// The "Taught Route" auton: replays the drive taught last.
void replayAuton();

/// The "SD Stress Test" auton, for the bench: ten rounds of writing the
/// biggest drive teach mode saves (75s, about 150KB) and reading it back,
/// while the logger records (it starts a recording if none is running, and
/// stops it after). The terminal and the log's "sdtest" event say how many
/// rounds read back as written, and whether the logger kept logging to the
/// same file; then `python tools/telemetry/slt_read.py` on that log should
/// say 0 lines skipped. Writes SDTEST.TXT, never TEACH.TXT.
void sdStressAuton();

} // namespace robot::teach
