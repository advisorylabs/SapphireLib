# Quick start: writing autonomous routines

Everything you need to write, pick and check an autonomous routine, on one page. The functions are
`sapphirelib/auton/auton.hpp`'s; the routines live in [`src/robot/autons.cpp`](../src/robot/autons.cpp).

Contents: [First time](#first-time) · [A routine](#a-routine) ·
[Which function do I use?](#which-function-do-i-use) · [Motion options](#motion-options) ·
[Coordinates](#coordinates) · [Background motions](#background-motions) ·
[Trajectories](#trajectories) · [Localization](#localization-mcl-and-wall-resets) ·
[Mechanisms](#mechanisms) · [Picking a routine](#picking-a-routine) ·
[The simulator](#trying-it-in-the-simulator) · [Common mistakes](#common-mistakes) ·
[Every function](#every-function)

## First time

Do these in order. Each one depends on the one before.

1. **Ports.** Set every motor and sensor in [`include/robot/config.hpp`](../include/robot/config.hpp).
   Drive forward on the sticks: if a wheel spins backward, negate its port.
2. **Measurements.** Wheel sizes, tracking wheel offsets and distance sensor mounts, also in
   `config.hpp`. [`NEW_CHASSIS.md`](NEW_CHASSIS.md) walks through each, with the brain screen
   buttons that measure most of them for you.
3. **Tune.** PID page, **Auto-Tune**: it measures the drive and designs every drive controller. Then
   pick **Tuning Test** on the Auton tab and run it: a 48 in drive, a ladder of turns, and back. Each
   motion should stop on target without creeping or oscillating. [`TUNING.md`](TUNING.md) has the rest.
4. **Check the motions.** **Odom Test** drives a 24 in square by odometry. **Wall Reset Test** checks
   the distance sensors against one wall.
5. **Write your own** (below), and try it in the [simulator](#trying-it-in-the-simulator) first.

## A routine

A routine is a plain function: `setPose()` at the spot the robot really starts, then motions, one
after another. Each motion blocks until it's done.

```cpp
void rightSide() {
    setPose(48, -60, 0);                  // where the robot starts, facing +y
    moveToPoint(48, -24);
    turnTo(270);
    if (!moveToPose(24, -24, 270).settled()) return; // blocked: stop here
    macros::setClawClosed(false);         // score
    drive(-12);
}
```

Then add it to `registerAutons()` at the bottom of `autons.cpp`, in the order the Auton tab lists
them:

```cpp
selector.addRoutine("Right Side", &rightSide);
```

That's all. Routines in `autons.cpp` sit outside `namespace robot` with
`using namespace sapphirelib::auton;` and `namespace macros = robot::macros;` at the top, so the calls
above need no prefixes.

## Which function do I use?

| I want to... | Use |
| --- | --- |
| Say where the robot starts | `setPose(x, y, heading);` |
| Drive straight (negative = backward) | `drive(24);` |
| Turn to a heading | `turnTo(90);` |
| Turn one way, like a 180 that has to go right | `turnTo(180, clockwise);` (or `counterclockwise`) |
| Turn to face a point | `turnToPoint(24, 24);` |
| Drive to a point | `moveToPoint(24, 48);` |
| Drive to a point and end facing a heading | `moveToPose(24, 48, 90);` |
| Strafe sideways (holonomic) | `strafe(12);` (negative = left) |
| Follow a curvy path through points | `followPath({{0, 0}, {0, 24}, {24, 48}});` |
| Drive a path drawn in the simulator, planned and fast | `followTrajectory(kRush);` ([Trajectories](#trajectories)) |
| Run a mechanism partway through a motion | `runAsync()` and the waits ([Background motions](#background-motions)) |
| Fix x or y off a wall | `resetBack();` and friends ([Localization](#localization-mcl-and-wall-resets)) |
| Know where the robot is | `pose()` (`.xIn`, `.yIn`, `.headingDeg`) |
| Stop the routine if a motion didn't get there | `if (!moveToPoint(0, 24).settled()) return;` |

On a holonomic drive, `moveToPoint()`, `strafe()` and `followPath()` hold the heading the robot
starts them with and slide there; a tank drive steers to face the point instead. `drive()` holds the
heading it starts on.

## Motion options

Every motion takes an optional last argument. Pass only what you change:

```cpp
drive(24, {.maxVolts = 6});                  // gently
moveToPoint(0, 48, {.timeoutMs = 1500});     // give up sooner
turnTo(90, {.exitError = 1});                // stop within 1 degree, not 2
moveToPoint(0, 24, {.chain = true});         // keep moving into the next motion...
moveToPoint(24, 48);                         // ...which ends stopped
```

| Option | Default | What it does |
| --- | --- | --- |
| `timeoutMs` | 3000 (4000 for `moveToPose()`) | Give up after this long, so a stuck robot can't freeze the routine |
| `maxVolts` | 12 | Top motor voltage. Lower is gentler. The whole output scales down, so the robot still heads the same way |
| `chain` | `false` | End as soon as it's close, without settling, and leave the motors running: the next motion starts at speed. Faster, a little less exact |
| `exitError` | 1 in / 2 degrees (3 in / 5 degrees chaining) | How close counts as there |

- **A lower `maxVolts` is slower**, so give it a longer `timeoutMs`: 4V can take twice as long as
  the default 3 seconds allows.
- **Never chain a routine's last motion.** Nothing after it would stop the robot. (The selector
  stops the drive when a routine returns, but don't count on it mid-routine.)
- `followPath()` takes its own: `{.timeoutMs = 10000, .maxVolts = 8, .lookaheadIn = 10, .chain = false}`.
  `maxVolts` is the speed it follows the path at; a bigger `lookaheadIn` is smoother but cuts corners.

Every motion returns a `MotionResult`: `.settled()` is true if it got there, `.reason` says why it
ended (`settled`, `timedOut`, `aborted`, `interrupted`), `.finalError` how far off it was, and
`.elapsedMs` how long it took.

## Coordinates

Inches and degrees. The middle of the field is (0, 0) and the walls are at plus and minus 70.25.
Heading 0 faces +y and headings go clockwise, so 90 faces +x, 180 faces -y and 270 faces -x.

**Start every routine with `setPose()`** at the robot's real spot. It puts odometry, every turn and
the localizer in the field frame. Without it, the robot is at (0, 0) facing 0 wherever it sat when the
program started, and the localizer can't help. The test routines in `autons.cpp` do exactly that on
purpose (`setPose(0, 0, 0)`), so they run anywhere with room.

## Background motions

`runAsync()` starts motions on a task of their own, and the routine carries on. Time a mechanism
against them with the waits:

```cpp
runAsync([] { moveToPoint(0, 48); });
waitUntilTraveled(12);          // 12 in into the move...
macros::setExtended(true);      // ...push the extender out
waitUntilNear(0, 48, 6);        // 6 in from the end...
macros::setExtended(false);     // ...pull it back in
waitForMotion();                // until it's done
```

| Call | Waits until |
| --- | --- |
| `waitUntilTraveled(inches)` | the running motion has driven this far (around corners too) |
| `waitUntilTurned(degrees)` | it has turned this far, either way |
| `waitUntilNear(x, y, radius)` | the robot is this close to a point |
| `waitUntilWaypoint(n)` | a background `followTrajectory()`'s plan reaches its point `n` |
| `waitForMotion()` | the background motions are done |

Each wait returns `true` when it happens, or `false` if the background motions ended first. To stop
them early, on a sensor say, `cancelMotion()`: the running motion stops where it is, and the rest of
the `runAsync()` block is skipped.

```cpp
runAsync([] { moveToPoint(0, 48); });
sapphirelib::waitUntil([] { return !isMotionRunning() || macros::pinInClaw(); }, 3000);
cancelMotion();                 // stop once the claw's sensor sees a pin
macros::setClawClosed(true);
```

- Put several motions in one `runAsync()` and they run in order.
- **A motion called while background motions run waits for them first**, so two never drive at
  once. So does a second `runAsync()`.
- Call the waits from the routine, not from inside the `runAsync()` block.

## Trajectories

On a holonomic drive, a trajectory is a path planned ahead of time from the measured drive models:
straight diagonals, smooth speed-up and slow-down, the same time every run, turning while driving,
and no stops except where you ask. [`TRAJECTORIES.md`](TRAJECTORIES.md) has the details.

1. Draw it in the simulator's **Paths** tab (`tools/sim/index.html`). Give points headings, split it
   into sections with their own speed limits, and put **Claw** and **Extender** actions on points.
2. **Copy C++** and paste it into `autons.cpp`. It writes the path and a routine that drives it:

```cpp
// Sapphire Sim path "rush": 3.42 s planned with the open TUNE.CFG's models
const std::vector<sapphirelib::motion::TrajectorySection> kRush{
    {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},
                   {.xIn = -40, .yIn = -30}}},
    {.waypoints = {{.xIn = -24, .yIn = -12, .headingDeg = 45}},
     .stopAtEnd = true},
};

// in registerAutons(): selector.addRoutine("rush", &rush);
void rush() {
    setPose(-48, -60, 0);
    runAsync([] { followTrajectory(kRush); });
    if (!waitUntilWaypoint(1)) return;
    macros::setExtended(true);
    if (!waitUntilWaypoint(2)) return;
    macros::setClawClosed(false);
    waitForMotion();
}
```

3. Add the routine to `registerAutons()`.

Points are numbered across the sections in order, 0 being the start. The robot doesn't slow down
for an action; end a section at that point with **Stop at its end** for it to be at rest first. If
the robot is held up until the motion times out, the routine ends at the point it never reached, so
the claw doesn't open short of the goal.

`followTrajectory()` plans the first time it's driven, from Auto-Tune's models or `TUNE.CFG`'s, and
keeps the plan. Planning takes a few milliseconds: `planTrajectory(kRush)` at the very start of the
routine gets it done before the robot moves. Without measured models it can't plan: it returns
`aborted`, and the terminal says why. **Paste C++** reads the path and the routine's actions back into
the tab.

## Localization: MCL and wall resets

Odometry drifts over a routine. Two things keep it honest, both on the four distance sensors:

**MCL** (Monte Carlo localization) runs all the time, from startup: it compares what the sensors see
with the field walls and eases odometry back toward where the robot really is. It does nothing until
a routine's `setPose()` puts odometry on the field, so start every routine with one. The Auton tab's
top line shows it: how sure it is (`+-` inches), how many sensors it can read, and what each update
costs. Particle counts, noise and the rest are tuned in the simulator and loaded from `TUNE.CFG`
([`LOCALIZATION.md`](LOCALIZATION.md)).

```cpp
setMcl(false);   // e.g. while pushed up against a goal it could mistake for a wall
setMcl(true);
```

**Wall resets** are one-shot: the sensor facing a wall sets x or y from it, right now.

```cpp
resetBack();     // the back sensor, off the wall behind
resetLeft();     // the left sensor: x from the left wall
```

`resetFront()`, `resetBack()`, `resetLeft()` and `resetRight()` each trace their sensor's beam to the
wall it heads for and set the one coordinate across that wall. They return `true` if they reset, and
skip the reading (changing nothing, the terminal says why) when:

- the beam meets the wall more than 30 degrees off square,
- it lands within 18 in of a corner, where Override's loaders are,
- it would move the robot more than 8 in, which is likelier a game element than the wall, or
- there's no reading, or no `setPose()` yet.

The 18 in and the 8 in are set in [`devices.cpp`](../src/robot/devices.cpp), in `auton::use()`.
Reset when the robot is still and square to a clear wall, a few feet out: the middle of a wall is the
cleanest. A reset starts MCL's particles over around the new spot too.

## Mechanisms

During autonomous the driver macros aren't running, so move things directly:

| Call | Does |
| --- | --- |
| `macros::setClawClosed(true / false)` | close or open the claw |
| `macros::setExtended(true / false)` | push the extender out or pull it in |
| `macros::pinInClaw()` | whether the claw's sensor sees a pin |
| `macros::liftAtBottom()` | whether the lift is on its bottom switch |
| `macros::liftMechanism()` | the lift itself (see Lift Testing in `autons.cpp`) |

Driver control carries on with the claw and extender wherever autonomous left them.

## Picking a routine

- **On the brain screen**: the Auton tab. Tap one.
- **On the controller**: while the robot is disabled, **LEFT** and **RIGHT** step through them; the
  controller screen shows the one picked.
- **It remembers**: the pick is saved on the SD card (`AUTON.TXT`), so it's still picked after a
  restart.
- **Off the field**: **X+UP** in driver control runs the picked routine (never with a competition
  switch or the field connected).

The first routine, **Do Nothing**, is picked when nothing else is, so it stays harmless. When a
routine returns, the selector waits for its background motions and stops the drive.

## Trying it in the simulator

Open `tools/sim/index.html` in a browser (no install). The **Drive** tab runs routines from
`tools/sim/js/routines.js` on a simulated robot running the library's own odometry, localizer and
motions, so what you see is what the robot does. Write a routine there the way you would in
`autons.cpp`; the calls are the same, as generators:

| `autons.cpp` | `routines.js` |
| --- | --- |
| `setPose(-48, -60, 0);` | `a.setPose(-48, -60, 0);` |
| `moveToPoint(-48, -24, {.maxVolts = 8});` | `yield* a.moveToPoint(-48, -24, { maxVolts: 8 });` |
| `turnTo(180, clockwise);` | `yield* a.turnTo(180, a.clockwise);` |
| `followPath({{0, 0}, {0, 24}});` | `yield* a.followPath([[0, 0], [0, 24]]);` |
| `runAsync([] { moveToPoint(0, 48); });` | `a.runAsync(function* () { yield* a.moveToPoint(0, 48); });` |
| `waitUntilTraveled(12);` | `yield* a.waitUntilTraveled(12);` |
| `pros::delay(250);` | `yield* a.delay(250);` |

Once it works there, copy it into `autons.cpp`, dropping the `yield*`s and the `a.`s.
[`TOOLS.md`](TOOLS.md) covers the rest of the simulator.

## Common mistakes

- **No `setPose()`.** Field coordinates mean nothing to odometry without one, and MCL and the wall
  resets stay off.
- **A timeout too short for the move.** A long drive, or a low `maxVolts`, needs more than the
  default 3 seconds. The `MotionResult` says `timedOut`.
- **Chaining the last motion.** Nothing stops the robot after it.
- **Waiting with nothing in the background.** The waits return `false` at once without a
  `runAsync()` running; the terminal says so.
- **A trajectory without measured models.** Run Auto-Tune (or load `TUNE.CFG`) first, or
  `followTrajectory()` aborts.
- **Forgetting `registerAutons()`.** A routine that isn't added doesn't show on the Auton tab.

## Every function

All in `sapphirelib::auton` (`#include "sapphirelib/auton/auton.hpp"`).

| Function | Returns |
| --- | --- |
| `use(drivetrain, odometry, &localizer, settings)` | once, in `initialize()` (`devices.cpp` does it) |
| `setPose(x, y, heading)` | |
| `pose()` | `odom::Pose` |
| `drive(inches, options)` | `MotionResult` |
| `turnTo(heading, options)`, `turnTo(heading, clockwise / counterclockwise, options)` | `MotionResult` |
| `turnToPoint(x, y, options)` | `MotionResult` |
| `moveToPoint(x, y, options)` | `MotionResult` |
| `moveToPose(x, y, heading, options)` | `MotionResult` |
| `strafe(inches, options)` | `MotionResult` (holonomic) |
| `followPath(points, pathOptions)` | `MotionResult` |
| `followTrajectory(sections, followConfig)` | `MotionResult` (holonomic) |
| `planTrajectory(sections)` | `bool`: planned |
| `runAsync(function)` | |
| `isMotionRunning()` | `bool` |
| `waitForMotion()` | |
| `cancelMotion()` | |
| `waitUntilTraveled(inches)`, `waitUntilTurned(degrees)`, `waitUntilNear(x, y, radius)`, `waitUntilWaypoint(n)` | `bool`: it happened |
| `resetFront()`, `resetBack()`, `resetLeft()`, `resetRight()` | `bool`: reset |
| `setMcl(on)` | |

The drivetrain's own motions (`drivetrain().moveToPoint()` and the rest) are still there for
anything these don't cover, such as branching between trajectories at speed
([`examples/trajectory.cpp`](../examples/trajectory.cpp)).
