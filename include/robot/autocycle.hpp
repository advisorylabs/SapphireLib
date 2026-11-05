/**
 * \file robot/autocycle.hpp
 *
 * The auto cycle: A in driver control runs pins from the matchloader to an
 * alliance goal and back, over and over, while the driver keeps the lift and
 * the claw on the usual macros (macros.hpp).
 *
 * Start it on the matchloader to the right of the drive team as they stand at
 * their station, facing the loader with a pin in the claw: the tracking
 * center 9.5in out from the loader's face and 11.5in off the end wall. Red or
 * blue, it's the same spot (the field's the same turned half way round).
 * Pressing A tells odometry the robot is there (setPose()), so press it
 * there. Then each trip:
 *
 *   1. Waits for the claw to be shut on a pin: already, the first time; the
 *      driver's R1 every time after (or autoClaw, with clawSpeed on).
 *   2. Backs straight out to just past the goal's line, turns 90 degrees
 *      right, drives up to 2in short of the goal without stopping, and creeps
 *      the last 2in into it at 6V, pressing on it a little.
 *   3. Waits there while the driver raises the lift and scores (R2), until
 *      the score opens the claw (R1 opening it counts too).
 *   4. Backs off the goal the way it came, which is what lowers the lift (see
 *      R2 in macros.hpp), turns back to face the loader, and drives up to it.
 *
 * Steps 2 and 4 are chained: the robot keeps moving from one point to the
 * next, and only stops at the goal and the loader.
 *
 * The lift, claw and side piston buttons work the whole time, as in driver
 * control. Any drive stick ends the cycle that same tick, and the sticks
 * drive on from there; so does A again. So does anything that stops driver
 * control (a disable, autonomous), a GUI routine taking the chassis, or X+UP.
 * A motion that times out (blocked by another robot, say) doesn't end it: the
 * next step carries on from wherever the robot got to.
 *
 * The route's numbers are in autocycle.cpp, written as an autonomous routine
 * is. Every motion logs to the SD card, and so does where the robot stopped
 * at the goal and the loader (`autocycle` events, docs/TELEMETRY_FORMAT.md),
 * for tuning them.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include "sapphirelib/input/controller.hpp"

namespace robot::autocycle {

/// Every opcontrol tick, after macros::update() and ahead of drive(): A
/// starts or ends the cycle, a drive stick ends it, and while it runs this
/// sets the controller screen's line 1. Returns whether the cycle has the
/// chassis this tick; when it doesn't, drive() should.
bool update(sapphirelib::input::Controller& controller);

/// Ends the cycle, if one is running, and waits for its motion to stop (10ms
/// or so). For anything about to drive the chassis itself: X+UP's routine
/// would otherwise wait on the cycle forever, since two motions never drive
/// at once.
void cancel();

/// Whether the cycle is running.
bool active();

} // namespace robot::autocycle
