# Trajectories: planned holonomic motion

`moveToPoint()`, `moveToPose()` and `followPath()` steer toward a target and let one PID decide how
fast to go. A trajectory is planned instead: a smooth path, and the speed and heading at every
moment along it, worked out ahead of time from the chassis's measured axis models. The follower then
drives that plan with feedforward and uses a PD per axis only for what the feedforward missed. On a
holonomic chassis that buys:

- **Straight diagonals.** Strafing is slower per volt than driving forward, and the Asterisk center
  wheels only push forward, so splitting one PID's volts by direction bends a diagonal. The plan
  gives each axis its own speed and turns it into volts through that axis's own model.
- **Planned speed-up and slow-down**, instead of a 12V step at the start and the P term winding down
  at the end: less wheel slip, less overshoot, a short settle.
- **The same time every run.** Velocity feedback holds the plan's clock, battery or not, so a
  mechanism can be timed off the trajectory's progress.
- **Turning while driving, together.** The heading is part of the plan, so the robot faces its
  target as it arrives, not whenever the turn PID gets there.
- **No stop between pieces.** A chain of sections, each with its own limits, is driven through at
  speed; stops happen only where you ask for them.

```
   Sapphire Sim Paths tab ── Copy C++ ──▶ autons.cpp ──▶ generateTrajectory() ──▶ followTrajectory()
            ▲                                   (axis models from Auto-Tune / TUNE.CFG)       │
            └────── analyzer: the plan over the replay, the tracking error ◀── SD log ◀───────┘
```

Contents: [Drawing a path](#drawing-a-path) · [In autons.cpp](#in-autonscpp) ·
[Branching](#branching) · [How the plan is made](#how-the-plan-is-made) ·
[How it's followed](#how-its-followed) · [Tuning](#tuning) · [Reading a run](#reading-a-run) ·
[The code](#the-code)

## Drawing a path

Open Sapphire Sim (`tools/sim/index.html`) and its **Paths** tab.

- **Click** the field to add a point at the end, or on the path to insert one there. **Drag** a point
  to move it (it snaps to half inches). **Right-click** it, or select it and press `Del`, to remove
  it.
- A point's **heading** is where the robot faces on reaching it. Tick it in the Points table and drag
  its handle on the field (`Shift` for 15° steps). The first point always has one: it's where the
  robot starts. Between headings the robot turns smoothly as it drives, the shortest way round; after
  the last one it holds.
- **Split** ends a section at a point. Each section has its own **voltage budget** and caps on each
  axis's speed and acceleration (0 for none), so a path can be fast to the goal and gentle through
  the intake. **Stop at its end** brings the robot to rest at the section's last point; the path may
  then turn sharply there (drive in, stop, back out at an angle). Without it the junction is driven
  through at speed. **Join next** merges a section into the one before it.

The path is colored by its planned speed (grey slow, blue fast, amber fastest), with the robot's
footprint along it so you can see it clear the walls. Under the table: the duration, when each
section starts, the peak speed on each axis, and a chart of speed and the worst wheel's volts. Red
dashes instead of a colored path mean it can't be planned; the status line says why.

**Run** puts the simulated robot at the first point and follows the trajectory, the plan's reference
drawn as a dashed outline it chases. **Reverse** turns the chain around.

**Plan with the robot's models.** The plan depends on the axis models. By default the tab uses the
Robot tab's chassis model; with the robot's TUNE.CFG open in the Tune tab, pick **Open TUNE.CFG's**
and the preview is what the robot will plan.

## In autons.cpp

**Copy C++** writes the chain:

```cpp
// Sapphire Sim path "rush": 3.42 s planned with the open TUNE.CFG's models
const std::vector<sapphirelib::motion::TrajectorySection> kRush{
    {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},
                   {.xIn = -40, .yIn = -30}}},
    {.waypoints = {{.xIn = -24, .yIn = -12, .headingDeg = 45}},
     .constraints = {.voltageBudget = 7},
     .stopAtEnd = true},
};
```

**Paste C++** reads that format back into the tab, so a path in `autons.cpp` can be edited again.

The robot plans it itself, at the start of the routine, from the axis models it has (Auto-Tune, or
TUNE.CFG's `model.*` lines):

```cpp
void rushAuton() {
    static sapphirelib::motion::TrajectoryResult rush; // the motion reads it to the end
    rush = sapphirelib::motion::generateTrajectory(kRush, drivetrain().axisModels());
    if (!rush.ok) return; // rush.error says why, e.g. a model that isn't measured
    odometry().setPose({.xIn = -48, .yIn = -60, .headingDeg = 0}); // its first point
    drivetrain().followTrajectory(rush.trajectory);
}
```

Planning takes a few milliseconds and allocates, so do it before the motion, never in a loop.
`followTrajectory()` returns a `MotionResult` like every motion: `settled` on the last point,
`timedOut` if it ran `overtimeMs` past the plan (held in place, say), `aborted` for an empty
trajectory, no odometry, or no axis models.

**Timing a mechanism.** `trajectoryProgress()` says which section the plan is in, how far along it
is, and whether it's waiting, from any task. Run the trajectory on a `MotionQueue` and wait on it:

```cpp
queue.enqueue([] { drivetrain().followTrajectory(rush.trajectory); });
queue.run();
sapphirelib::waitUntil([] { return drivetrain().trajectoryProgress().section >= 1; }, 3000);
intake().spin(); // as the robot enters section 1, without slowing down
queue.waitUntilDone();
```

A section starts at its first point, so `section >= 1` comes true as the robot reaches the junction.
`examples/trajectory.cpp` is a complete routine; `src/robot/autons.cpp` has a **Trajectory Test**
routine for checking it on the robot (it needs about 40 in ahead and 30 in to the right).

## Branching

A routine can pick its next trajectory from a sensor partway through: drive to where a pin should
be, and if the claw's sensor sees it go and score, if not skip to the next one. Each branch is a
trajectory of its own, planned up front with `TrajectoryOptions::from`, which starts it exactly
where another ends (its point, heading, speed and direction of travel). Its sections list only the
points after that one, as every section after the first of a chain does.

**With a stop.** The trunk ends at rest; the routine reads the sensor there and follows the branch
it picks. Works with any trajectory, and costs the stop and the settle:

```cpp
static TrajectoryResult trunk, score, skip;
trunk = generateTrajectory(kToPin, models);
score = generateTrajectory(kScore, models, {.from = &trunk.trajectory});
skip = generateTrajectory(kSkip, models, {.from = &trunk.trajectory});
if (!drivetrain().followTrajectory(trunk.trajectory).settled()) return;
drivetrain().followTrajectory(macros::pinInClaw() ? score.trajectory : skip.trajectory);
```

**At speed.** The trunk is planned to end moving (`endSpeedInPerS`, the most it may be there), each
branch carries on from it at that speed, and `TrajectoryFollowConfig::next` picks one as the robot
reaches the junction. The robot drives straight through: the clock runs on, the feedback isn't
reset, and the motion ends at the end of the branch:

```cpp
trunk = generateTrajectory(kToPin, models, {.endSpeedInPerS = 24});
score = generateTrajectory(kScore, models, {.from = &trunk.trajectory});
skip = generateTrajectory(kSkip, models, {.from = &trunk.trajectory});
drivetrain().followTrajectory(trunk.trajectory, {.next = [](const Trajectory&) {
    return macros::pinInClaw() ? &score.trajectory : &skip.trajectory;
}});
```

- `next` runs inside the control loop, on the motion's task, as the plan's clock reaches the
  trunk's end: read a sensor and pick a plan that's already made. Never plan in it.
- It's only asked of a trajectory that ends moving, with the one that's ending, so a branch that
  also ends moving gets asked about in turn: one function can choose at every junction of a tree
  by checking which trajectory it was given.
- A branch must carry on from the trajectory it follows. One that doesn't start where it ended
  (planned without `from`) is refused, as is `nullptr`: the robot then settles on the trunk's last
  point from speed, which overshoots, and the log says `nocontinue`.
- The junction speed is a promise every branch must keep. A branch that can't take it (a turn
  right after the junction too tight for that speed) fails to plan, saying so, before the robot
  moves: lower `endSpeedInPerS`. The trunk brakes to it in time, and planning a little under what
  the branches could take loses almost nothing.
- `trajectoryProgress()` counts across the chain: `section` runs on from the trunk's into the
  branch's, `timeS` and `distanceIn` from the trunk's start, and `piece` is 1 once it's on the
  branch.
- In the analyzer the chain is one `followTrajectory` motion, its plan the trunk and the branch
  driven together.

`src/robot/autons.cpp` has both: **Branch Test** (with a stop) and **Branch At Speed**, each
picking a branch on the claw's pin sensor and driving home after. Sapphire Sim's **Branch at
speed** routine branches on its front distance sensor: turn the defender on and it swerves.

## How the plan is made

`generateTrajectory()` (`motion/trajectory_generator.hpp`):

1. **The path.** A natural cubic spline through the waypoints, parameterized by chord length, so
   curvature changes smoothly through every point: no jumps in the acceleration the feedforward
   asks for. It's split into separate splines at each stop. Sampled every half inch of arc length.
2. **The heading.** Between waypoints with a heading it eases over arc length with smootherstep
   (6u⁵ − 15u⁴ + 10u³): no turn rate and no turn acceleration at the waypoints themselves.
3. **The speed.** At each sample the direction of travel relative to the robot fixes how fast each
   axis moves for a given path speed, and how its acceleration depends on path acceleration (plus a
   speed² term from the path curving and the heading turning: what slows corners and turning while
   driving). The binding limit is the **voltage budget**: the forward, strafe and turn volts the
   models need, `kS·share + kV·v + kA·a` per axis, must add up to no more than the budget, because
   that sum is what the worst wheel gets after mixing. The per-axis caps apply on top. Then the usual
   two passes: accelerate as hard as each step allows (averaging what its two ends allow), then
   brake in time for everything slower ahead. Speed is 0 at the start, at each stop and at the end.

A trajectory that starts moving (`TrajectoryOptions::from` one that ends at speed) has its spline
leave the first point along the direction the robot is already going, instead of the natural
spline's free direction, and starts its speed passes at that speed; one that ends moving caps the
last point's speed at `endSpeedInPerS` instead of stopping there.

Static friction is shared between the axes by how much of the motion each one has, in fractions of
its top speed (`frictionVolts()`), so a diagonal pays friction once, and an axis that's barely moving
doesn't jump to its full kS.

The default budget is 10V: what's left of 12 is headroom for the feedback and for battery sag.
Lower it for a gentler section; raise it toward 12 and the feedback has nothing left to correct with.

## How it's followed

`HolonomicDrivetrain::followTrajectory()` runs `motion::TrajectoryFollower` every 10 ms:

- **The reference** is the plan at the plan's clock. The clock runs with time, except while the robot
  is more than `maxLagIn` (4 in) behind along the path: then it **waits**, so a robot that was
  blocked or bumped catches up at the planned speed instead of chasing a reference that ran away.
  Time spent waiting counts against `overtimeMs`, so a robot held in place gives up.
- **Feedforward**: the plan's velocity and acceleration through the models (`trajectoryFeedforward()`),
  rotated by the heading error so a lagging heading still pushes the right way on the field.
- **Feedback**: a PD per axis on the error in the robot's frame (`trackingForwardPID()`,
  `trackingStrafePID()`, `trackingTurnPID()`). The D term is the measured velocity error, from
  odometry's own velocity (`Odometry::Snapshot::rawVelocity`), not a difference of the error.
- **At a junction** of a chain (a trajectory that ends moving), the follower asks `next` for the
  trajectory to carry on into and switches to it on the same tick, the clock running on into it;
  the overtime allowance grows by its duration.
- **After the plan ends** the robot settles on the last point on feedback alone, until it's within
  `positionErrorThresholdIn` and `headingErrorThresholdDeg` for `settleTimeMs`.

## Tuning

1. **Auto-Tune the drivetrain**: PID page, select any drivetrain controller, **Auto-Tune**. One run
   measures Fwd, Strafe and Turn and designs every controller on them, **Track F**, **Track S** and
   **Track T** included, each from its own axis. With the feedforward cancelling the motion, the
   tracking error is exactly the spring and damper `designPositionGains()` places poles for, to
   `kTrackingResponse` in `src/robot/tuning.cpp` (0.4 s), slowed if the axis's delay needs it. The
   plan is only as good as the models, so save them and the gains to TUNE.CFG (the analyzer's Tune
   tab writes the `model.*` and `pid.*` lines).
2. **Check it**: **Run Test** on any Track controller drives a short out-and-back trajectory that uses
   all three axes at once.
3. **Budget** each section: start at the default and lower it where the robot falls behind its plan
   (the analyzer says so, below), or where it slips.

The three gains are in TUNE.CFG as `pid.trackFwd`, `pid.trackStrafe` and `pid.trackTurn`.

## Reading a run

The robot logs a trajectory three ways (`docs/TELEMETRY_FORMAT.md`): the plan once as it starts
(`traj.plan`), the reference and feedforward every tick (`traj`), and the three tracking PIDs.

In the analyzer:

- **Replay** draws the plan under the robot during the motion (the **Trajectory plan** toggle): the
  path colored by planned speed, rings where sections end, squares at stops, red dots where the plan
  waited, and the reference it was chasing as a dashed outline joined to the robot. The **Trajectory:
  behind its plan** chart is the lag.
- **Motions** shows each trajectory's worst and RMS distance from its plan, its heading error, how
  long it waited, and its time against the plan, with a chart of the distance, how far behind and
  how far to the side.
- **Overview** findings: off the plan by more than 3 in (a warning, saying whether the tracking loops
  were at full power, which means the budget is too high or the models are stale, or not, which means
  the gains or models undershoot), waiting more than half a second (blocked or bumped), and taking
  over 15% longer than planned beyond the waiting and a settle (stale models, hot motors, a low
  battery).

The simulator's recorder (`tools/sim/js/recorder.js`) logs the same channels and motion events, so a
simulated run reads in the analyzer like the robot's. The analyzer's demo pit session ends with a
Trajectory Test, to see all of this without a robot.

## The code

| | |
|---|---|
| `motion/trajectory_config.hpp` | `TrajectoryWaypoint`, `AxisLimits`, `TrajectoryConstraints`, `TrajectorySection`, `TrajectoryOptions`, `TrajectoryFollowConfig` |
| `motion/trajectory_path.hpp` | `samplePath()`: the spline, the heading easing, arc-length samples; `PathEnds` for a moving start or end |
| `motion/trajectory_generator.hpp` | `generateTrajectory()`, `voltageAccelRange()` |
| `motion/trajectory.hpp` | `Trajectory`: samples, `sample(t)`, sections |
| `motion/holonomic_feedforward.hpp` | `frictionVolts()`, `holonomicFeedforward()`, `trajectoryFeedforward()` |
| `motion/holonomic_tracking.hpp` | `trackingErrors()`, the clock, `TrajectoryFollower` (and its chain), `TrajectoryObserver`, `TrajectoryProgress` |
| `chassis/holonomic_drivetrain.hpp` | `followTrajectory()`, `trajectoryProgress()`, the tracking PIDs, `setTrajectoryObserver()` |

All but the drivetrain are pure and tested on the host (`tests/motion/trajectory_path_test.cpp`,
`trajectory_generator_test.cpp`, `holonomic_tracking_test.cpp`); `tools/sim/js/trajectory.js` is their
port, held to the same golden values.

**On the robot**: not driven yet. It needs the axis models from Auto-Tune and odometry, and neither
has been verified on the real chassis (`docs/ROADMAP.md`).
