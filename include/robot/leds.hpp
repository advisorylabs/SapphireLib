/**
 * \file robot/leds.hpp
 *
 * The LED strands, run by HitLib from the Pattern Studio design in
 * include/hitlib_studio.hpp (ports, lengths and animations all come from
 * there: edit the design and export it again, not that file). Every strand
 * shows the same mode at once:
 *
 *   Idle       the rest of the time, disabled included.
 *   LiftGauge  for a second each time the lift is sent up a level, by any
 *              means: L1's step, its jump to the lift's memory, RIGHT moving
 *              it to a taller level on the new goal, or a routine's
 *              setLiftLevel(). And each time L2 steps it down a level, down
 *              to the bottom, where none light. Not other trips down: a
 *              score's end, or RIGHT onto a lower level. Each strand is split
 *              into as many sections as the goal has levels, and the level's
 *              number of them light up from the start of the strand, each
 *              solid: the bottom two green, the next two orange, and the top
 *              two red. ALLIANCE's six are exactly that; MEDIUM's five have
 *              one red, and CENTER's four one orange and one red, so the top
 *              level is always red. L2 of ALLIANCE's six lights the bottom
 *              two of six, both green. MEDIUM's first level (its L2) lights
 *              one of its five. Not in debug mode, where the lift ignores its
 *              levels. Defined here rather than in the design, see leds.cpp.
 *   AutoClaw   while autoClaw is on: clawSpeed in driver control (R1+R2, but
 *              not in debug mode, where autoClaw is off), or setAutoClaw()'s
 *              mode in autonomous (macros.hpp).
 *   Goal       for a second each time RIGHT picks a goal (macros.hpp), its
 *              color: ALLIANCE solid blue split into 6 segments, every other
 *              one red, swapping every 200ms; MEDIUM solid orange; CENTER
 *              solid green. A second RIGHT inside it shows the next goal and
 *              starts the second again. Not a routine's setLiftLevel() onto
 *              another goal.
 *   MemoryReset  two purple blinks (175ms on, 175ms off, 175ms on) each time
 *              L1+L2 forgets the lift's memory (macros.hpp), whether or not
 *              it had one, so the driver sees the combo land. Not RIGHT,
 *              which forgets it too.
 *   Scoring    while R2's score runs, from the press until the robot has
 *              backed off the goal and the lift heads down, or L2 lowers it
 *              early. Not R2 at the bottom, which drops the pin rather than
 *              scoring it.
 *
 * And the driver's own, from the controller in driver control (driver.hpp's
 * shortcuts()), over everything the robot shows by itself:
 *
 *   Showcase   DOWN: a light show, about 15 seconds once through (a white
 *              scanner, red-blue breathing, racing stripes, a chopped
 *              rainbow, confetti, police lights, and a strobe hit into solid
 *              red that closes to dark), then back to whatever the robot
 *              shows. DOWN again stops it early.
 *   Strobe     UP: white strobe, on until UP again.
 *   Dark       X: every LED off, until X again, over even Strobe and
 *              Showcase. Turning them off ends a showcase.
 *
 * With more than one on, the highest priority shows: Dark (255), Strobe
 * (220), Showcase (200), Scoring (30), MemoryReset (25), Goal (24), AutoClaw
 * (20), LiftGauge (15), then Idle (10). The timed ones (LiftGauge's second,
 * Goal's, MemoryReset's blinks, Showcase) keep running under whatever beats
 * them, so one that ends inside them shows them for the rest. So RIGHT onto a
 * taller level shows Goal, not LiftGauge: both start then and last a second. Every mode but the
 * design's own Idle, AutoClaw and Scoring is defined in leds.cpp.
 *
 * The strands hold their last frame when the program stops: PROS runs no code
 * as it does, so nothing can turn them off then. X turns them off first.
 *
 * Team 96671H: Hitmen
 */

#pragma once

namespace robot::leds {

/// Call once from initialize(): starts every strand and its refresh task,
/// showing Idle.
void start();

/// Shows AutoClaw on every strand (true), or stops showing it (false). Safe
/// from any task; a call that changes nothing does nothing, so the macros
/// call it every tick.
void showAutoClaw(bool on);

/// The same for Scoring.
void showScoring(bool on);

/// What LiftGauge shows: `level` of `levels` sections lit (the lift's level
/// in its goal's table, 0 the bottom, and the goal's top level). Only stores
/// it, so it's cheap enough for every tick: update() calls it every tick, so a
/// gauge still showing follows the lift down when a score or RIGHT lowers it,
/// without showing for longer. Safe from any task.
void setLiftGauge(int level, int levels);

/// Shows LiftGauge on every strand for a second from now, whatever
/// setLiftGauge() last said: a second call before it's over starts the second
/// again. Safe from any task.
void showLiftGauge();

/// Shows Goal on every strand for a second from now, in `goal`'s color: 0
/// ALLIANCE, 1 MEDIUM, 2 CENTER (macros::Goal's order, the ladder's table
/// index). A second call before it's over shows its goal instead and starts
/// the second again. Safe from any task.
void showGoal(int goal);

/// Blinks MemoryReset (purple) on every strand from now, over Idle, LiftGauge
/// and AutoClaw: a second call before it's over pushes its end out. Safe from
/// any task.
void showMemoryReset();

/// DOWN: starts Showcase from the top, or stops it if it's still playing.
/// Safe from any task.
void toggleShowcase();

/// UP: turns Strobe on, or off. Safe from any task.
void toggleStrobe();

/// X: turns every LED off (Dark), or back on to whatever they'd be showing.
/// Safe from any task.
void toggleLeds();

/// Back to Idle, from AutoClaw, Scoring, LiftGauge, Goal and MemoryReset: for
/// disabled(), where nothing runs. Leaves the driver's Showcase, Strobe and
/// Dark as they are. Whatever is still on shows again once it's next told (driver
/// control's first update(), or a routine's setAutoClaw()).
void showIdle();

} // namespace robot::leds
