/**
 * \file robot/autocycle.hpp
 *
 * The auto cycle: A in driver control runs pins from the matchloader to an
 * alliance goal and back, over and over, with the driver only working the
 * lift.
 *
 * Start it with the robot on the matchloader on the right, as the driver
 * stands at their station: the tracking center in line with the middle of
 * the loader, 9.5in out from its face, facing it, with a pin in the claw and
 * the claw closed. The field is the same turned half way round for the other
 * alliance, so the same A works from either station's right-hand loader.
 * From there, each cycle:
 *
 *   1. Backs straight away from the loader until the tracking center is in
 *      line with the alliance goal on the left (the one nearer the wall),
 *      turns 90 degrees right to face it, and drives up until the tracking
 *      center is 7in short of the goal's middle, measured by the back
 *      distance sensor off the wall behind (autocycle.cpp says why). Running
 *      into the goal on the way counts as getting there.
 *   2. Holds there while the driver raises the lift (L1/L2) and scores (R2),
 *      as in driver control (macros.hpp).
 *   3. As the score opens the claw, it heads back the same way: backs off the
 *      goal (the score lowers the lift once it's 3in off), turns to face the
 *      loader, and drives back to it. With the lift down, it creeps into the
 *      loader and closes the claw as soon as the claw's sensor reads the pin
 *      within 45mm, without stopping: it carries on 2in further in (by the front
 *      distance sensor) with the claw shut, then starts the next cycle.
 *
 * Pressing A tells odometry the robot is on that spot (Odometry::setPose()),
 * so it must be pressed there. Every target is a spot on the field, driven
 * to by getPose(), which the localizer keeps correcting from the distance
 * sensors, so the drift doesn't build up cycle after cycle. Started with the
 * claw open, it goes straight to picking up a pin.
 *
 * A again, or any drive stick, ends it, and the sticks drive the robot on
 * that same tick. So does anything that stops opcontrol (autonomous, a
 * disable), a GUI routine taking the chassis, or one step taking too long
 * (blocked by another robot, say), which also rumbles. A step pushing
 * without moving just moves on to the next. autoClaw is off while
 * it runs, so the claw closes at 45mm and nowhere else.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include "sapphirelib/input/controller.hpp"

namespace robot::autocycle {

/// Every opcontrol tick, after macros::update() and in place of drive(): A
/// starts or ends the cycle, and while it runs, this drives the chassis and
/// sets the controller screen's line 1. Returns whether it drove the chassis
/// this tick; when it didn't, drive() should.
bool update(sapphirelib::input::Controller& controller);

/// Ends the cycle, if one is running, without touching the drive. For
/// opcontrol(), while a GUI routine has the chassis.
void cancel();

/// Whether a cycle is running.
bool active();

} // namespace robot::autocycle
