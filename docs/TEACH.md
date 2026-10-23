# Teach mode: drive it once, replay it as an auton

Drive a route by hand in driver control, and the **Taught Route** auton replays it: the same path,
at the driver's pace, with the intake, claw and lift doing what the driver did where they did it.

It doesn't replay the sticks. Replaying stick inputs is open loop: the same volts don't give the
same path, so a start 1° off ends 2.5 in off after 12 ft, and a low battery, wheel slip or a bump
adds to that with nothing to correct it. Teach mode records where the robot *went* (odometry's
pose) and replays that as a trajectory, through the same follower as every other trajectory
(`docs/TRAJECTORIES.md`): feedforward from the axis models, a PD per axis on the error, the
localizer correcting odometry against the walls, and the plan waiting for the robot if it's held
up.

```
X+A ── drive ── X+A ──▶ TEACH.TXT on the SD card ──▶ power-on loads it ──▶ Taught Route auton
                              (pose + buttons every 20ms)        trajectoryFromRecording() ─▶ followTrajectory()
                                                                 buttons ─▶ ButtonPlayback ─▶ macros::update()
```

Contents: [Teaching](#teaching) · [Replaying](#replaying) · [How a drive becomes a plan](#how-a-drive-becomes-a-plan) ·
[Limits](#limits) · [The code](#the-code)

## Teaching

1. Off the field (no competition switch or field cable), put the robot where the routine will
   start, with the mechanisms as a match starts them: stowed. The drive is recorded in whatever
   frame odometry is in: field coordinates if an auton's `setPose()` put it there earlier this
   power-on, otherwise inches from where the robot sat at power-on. The replay sets that same
   start, so either works with the robot put back on the spot, but only in field coordinates can
   the localizer also correct the replay against the walls.
2. **X+A** (pressed together) starts recording, or the Home page's **Teach** button. The
   controller gives one long rumble, and its middle line reads `TEACH REC: X+A`.
3. Drive the route, with the macros as usual. Waiting before you start and after you finish is
   trimmed, so there's no hurry.
4. **X+A** again (or **Teach**) stops: two short rumbles. Driver control stopping (a disable, a
   routine run with X+UP) also ends it, as of driver control's next tick, and it stops on its own
   after 75 s.

Stopping saves the drive to `TEACH.TXT` on the SD card, in `sl/` beside the logs (or the card's
root without that folder), on a background task, then reads it back: it only counts as saved if the
card has exactly what was written. The Home page's `Teach:` line then says how long it is, whether
it's saved, and how fast it will replay: `Teach: 12.4s, saved, replays at 87%` (or `SAVE BAD` if
it didn't read back right, `NOT SAVED` with no card).
At the next power-on, `teach::load()` reads it back, so a drive taught in the pits is there for the
match.

A new recording replaces the last; there's one taught drive at a time.

## Replaying

Pick **Taught Route** on the auton selector, put the robot back on the spot the drive started
from, and run it (in a match, or X+UP off the field). It:

1. plans the drive (`trajectoryFromRecording()`, a few milliseconds),
2. sets odometry to where the drive started,
3. follows it on a `MotionQueue`, and meanwhile
4. plays the driver's buttons into `macros::update()` on a `Controller` of its own, every 20 ms,
   on the trajectory's clock: if the plan waits for a blocked robot, the buttons wait too.

When the trajectory ends, the routine stops the mechanisms (`macros::stop()`).

The replay runs at the driver's own speed when it can. A driver at full stick asks for all 12 V,
which leaves the feedback nothing to correct with, so the whole drive is slowed evenly until the
plan fits the 10 V budget (`ReplayConfig::voltageBudget`), as a planned trajectory's does. That's
the "replays at 87%" on the Home page: 87% of the driver's speed, everywhere, buttons included.

## How a drive becomes a plan

`trajectoryFromRecording()` (`motion/recorded_trajectory.hpp`):

1. **Trim**: from just before the first sample that moved (0.05 in, 0.25°) or changed a button, to
   just after the last. What's kept either side is the smoothing's reach, 0.2 s, so the plan starts
   and ends at rest.
2. **Resample** onto an even 10 ms grid, the heading unwrapped so a turn through 0° is a turn.
3. **Smooth** the pose with a Gaussian (`smoothingS`, 0.06 s), then differentiate it for the
   velocity and acceleration the feedforward needs, in the field and in the robot's frame.
   Odometry's small steps would otherwise become kicks in the feedforward.
4. **Fit the budget**: each sample needs friction + s·kV·v + s²·kA·a volts per axis for a replay at
   speed s, and the worst wheel (their sum) has to fit the budget. The largest s up to
   `maxSpeedScale` that fits everywhere wins.
5. **Build the trajectory**: a `TrajectorySample` every 10 ms of the drive, timed at that speed.
   Friction pushes the way the robot moves (or starts to move); where the robot was still, none
   does, so it holds on feedback instead of creeping.

The buttons come out alongside: the held buttons from each moment they changed, on the plan's
clock. `input::ButtonPlayback` plays them back a tick at a time without ever skipping a change, so a
press the driver held for a single tick still reaches the macros.

Unlike a drawn path, a taught drive can turn in place: it's planned in time, not along a path.

## Limits

- **The start spot matters.** The replay starts from where the drive started. With the localizer
  and a field-coordinates `setPose()`, a start a little off gets corrected against the walls;
  without them, the whole replay is shifted by however far off the robot was put.
- **Mechanism state isn't recorded**, only the buttons. Start teaching and replaying from the same
  state (stowed, the same goal selected), or the same presses do something else.
- **Only the driver macros' buttons replay.** The drive itself comes from the trajectory, so B
  (invert) and DOWN (orbit) don't matter, and X+UP isn't replayed.
- **A long drive outlives autonomous**: a taught drive longer than the autonomous period is cut off
  when it ends, like any motion.
- **Saving shares the SD card with the logger, taking turns.** PROS's file functions go straight
  into VEXos with no lock between tasks (its SD driver calls `vexFileWrite()` and `vexFileSync()`
  bare), so the program never lets two tasks at the card at once: the logger holds
  `telemetry::SdCardLock` for each of its writer's passes, and teach mode a 4 KB chunk at a time
  (`telemetry::writeCardFile()`), so neither waits long. `tools/telemetry/check_sd_access.py`
  (in CI) fails if any card call in the program is made without the lock. Not yet run on the brain:
  the **SD Stress Test** auton (bench only) saves and reads back the biggest drive teach mode keeps,
  ten times, while the logger records, then says in the terminal and the log's `sdtest` event how
  many rounds read back as written and whether the logger kept logging to the same file;
  `python tools/telemetry/slt_read.py` on that log should then say 0 lines skipped.

## The code

| | |
|---|---|
| `motion/recorded_trajectory.hpp` | `RecordedSample`, `ReplayConfig`, `ReplayPlan`, `trajectoryFromRecording()`, and the `TEACH.TXT` format (`kRecordingHeader`, `formatRecordedSample()`, `parseRecording()`) |
| `input/button_playback.hpp` | `TimedButtons`, `ButtonPlayback` |
| `input/controller.hpp` | `Controller::replay()`: a tick from a recording instead of the controller |
| `telemetry/sd_card.hpp` | `SdCardLock`, and `readCardFile()`/`writeCardFile()`, which hold it a chunk at a time |
| `src/robot/teach.cpp` | recording in driver control, saving and loading, and the Taught Route auton |

The library parts are pure and tested on the host (`tests/motion/recorded_trajectory_test.cpp`
replays a recorded drive in closed loop on a simulated chassis; `tests/input/button_playback_test.cpp`
checks every press survives playback).

**On the robot**: not run yet. It needs the axis models from Auto-Tune or TUNE.CFG, like any
trajectory.
