# Sapphire Sim: the localization simulator

A simulator of 96671H's robot for trying out odometry and Monte Carlo localization without the robot,
tuning the localizer, showing people how MCL works, and planning autonomous routines for
`autons.cpp` on Override's field, drawn to scale from the game manual (the Paths tab). Plain HTML and
JavaScript: no install and no build step. It works offline; the only thing it fetches is a web font,
and it falls back to a system font.

It runs the robot program's own math, ported line for line from the library: odometry and its
correction, the particle filter and `MonteCarloLocalizer`, the holonomic drivetrain's motions
(`moveToPoint()`, `moveToPose()`, `turnToHeading()`, `followPath()`, `followTrajectory()` and its
planner), and Auto-Tune's gain design.
The chassis it drives is the model Auto-Tune measures. So what you see here is what the robot does,
up to how well the simulated world matches the real one. [`docs/LOCALIZATION.md`](../../docs/LOCALIZATION.md)
explains the localizer itself.

## Opening it

Open `index.html` in Chrome, Edge, Firefox or Safari: double-click it. It also loads a few of the
analyzer's files (`../analyzer/js/`), so keep it inside the repo's `tools` folder. To hand it to
someone as one file, `node tools/sim/build.js` writes `tools/sim/dist/sapphire-sim.html`, with
everything inlined.

It opens running the Field tour routine. `claw.html`, beside it, is the claw timing page: autoClaw's grab simulated at every speed, for tuning `kGrabMm` and `kClawLeadMs` (see [`docs/TOOLS.md`](../../docs/TOOLS.md#claw-timing)). The **?** button (or the `?` key) lists the controls and
units; hover a label for what it means. **How to use it: [`docs/TOOLS.md`](../../docs/TOOLS.md)**:
the screen, the controls, each tab (Drive, Show, Events, Robot, MCL, Tune), and a few recipes for
showing someone how MCL works.

## How it's built

`js/` holds one file per concern, loaded in order by `index.html` as plain scripts. All but
`render.js`, `app.js` and `tuneview.js` also load under Node, for the tests:

| File | What |
|---|---|
| `mcl.js` | Ports of the localization math: `Rng` (with the ziggurat), `LookupTable`, `FieldMap` and `ParallelRayCaster`, the sensor model and `ReadingScorer`, `ParticleFilter`, `sampleParticles()`, `computeOdometryDelta()`, `correctionStep()` |
| `world.js` | The physical world: the chassis (three `PlantSim` axes from the analyzer's `model.js`), tracking wheels, IMU, distance sensors, collisions, bumps, the defender |
| `field.js` | Override's field (2026-27) from the game manual's Appendix A: tile seams, tape and zones, Goals, Loaders, Toggles, the starting Cups and Pins; and the Paths tab's checks against it (the robot's footprint against the Goals, Loaders and walls, a start against SG1), and `wallResetCheck()`: whether a wall reset's beam cone sees clean wall there, or a Goal, a Loader, the Cups and Pins along the walls, or a Toggle |
| `trajectory.js` | Ports of the trajectory planner and follower math: `samplePath()`, `generateTrajectory()` (with `from` and `endSpeedInPerS`, for branching), `Trajectory`, the holonomic feedforward, `trackingErrors()`, and a chain's junction checks |
| `robot.js` | The robot program: `Imu`, `Odometry` (with its odometers), `MonteCarloLocalizer` (and `resetFromWall()`), the drivetrain and its motions (generators that yield where the C++ calls `pros::delay(10)`, with the voltage cap, chaining and turn direction), `TrajectoryFollower`, and Auto-Tune's gain design |
| `auton.js` | The auton functions (`sapphirelib/auton/auton.hpp`) for a simulated robot: the same calls `src/robot/autons.cpp` makes, as generators, with background motions taking turns with the routine a tick at a time |
| `macros.js` | The lift, the claw and the extender for routines (`include/robot/macros.hpp`): `setLiftLevel()`, `liftScoreDeg()`, `waitForLift()`, `setClawClosed()`, `setAutoClaw()` and the rest, on `macros.cpp`'s ladder and loop, with the field's Pins picked up and stacked on Goals, and autoClaw's lead (`clawtiming.js`'s rule) on a claw sensor that measures every 33 ms, 15 ms late. The lift's model, the pistons' speed and where the claw holds a Pin are guesses, said so at the top |
| `clawtiming.js` | autoClaw's timing, one grab at a time: the claw sensor's samples, latency and noise, the program's tick, odometry's speed and the valve, with `macros.cpp`'s lead rule (`ruleFires()`) and the two before it; sweeps over speed, the best lead, and what spreads the grabs. The claw timing page and `macros.js` both use it |
| `routines.js` | The autonomous routines, written with `auton.js` and `macros.js` the way `src/robot/autons.cpp` writes them, including Branch at speed, which picks its branch on the front distance sensor (turn on the defender to see it swerve), and Triple Pin (paths), `autons.cpp`'s `triplePinPaths()`: three Pins stacked on a Goal, every drive a Paths-tab trajectory, with averaged wall resets where the field leaves a sensor clean wall. Add your own here |
| `sim.js` | A world and a robot, run in 10 ms ticks, with the error history the page charts |
| `recorder.js` | A run, logged the way the robot logs it (`odom`, `chassis`, `mcl`, `mcl.beams`, `mcl.state`, `mcl.pts`, `traj.plan`, `traj`, `followTrajectory()`'s motion events, the `#meta` lines) |
| `tuner.js` | The MCL auto-tuner: the cases, open-loop replays, the search, the holdout check, and calibrating the world from logs |
| `render.js`, `app.js`, `tuneview.js`, `pathview.js` | The canvas drawing, the page, the Tune tab, and the Paths tab (its path document and C++ both ways also load under Node) |
| `clawview.js` | The claw timing page (`claw.html`): your robot's numbers, and the charts `clawtiming.js` makes of them |

It also loads four of the analyzer's files: `slt.js` (reading logs), `demo.js` (writing them),
`tunefile.js` (`TUNE.CFG`) and `mclcal.js` (measuring the localizer's world from logs).

The simulated chassis is the three axis models Auto-Tune measures, with the six motors and the
Asterisk center wheels folded in, as they are when Auto-Tune measures the whole chassis at once.
Per-motor effects (thermal derating, the center wheels' drift correction) aren't simulated. The
tracking wheels are mounted where `OdometryConfig`'s offsets say, under the sign convention its arc
correction uses (see the note in `world.js`).

```bash
node --test tools/sim/test/*.test.js
```

runs the tests (Node 20 or later; CI runs them). `mcl.test.js` holds the ports to the C++ unit tests'
golden values, down to a seeded particle filter run landing on the same particles. `sim.test.js` checks
the claims the page makes: every routine finishes within an inch and is closer on average with MCL
than without, worn wheels and bumps are corrected, unmapped elements are shrugged off, nothing is
corrected before `setPose()`, a closed `followPath()` lap drives the whole lap (and a lap's last side
passing beside its first doesn't end it early), a spin in place leaves odometry still (so the
simulated tracking wheels follow `OdometryConfig`'s sign), and `followTrajectory()` lands within an
inch near the planned time, drives a diagonal straight where `moveToPoint()` bends, reports each
section as the plan enters it, waits for a robot the defender blocks, and logs what the robot logs.
It also checks Triple Pin (paths): three Pins at L1, L2 and L3 inside 15 s, with the simulator's chassis and the
robot's; every reset it takes reads clean wall on the real field an inch and 3 degrees either way, and
the stops it passes over couldn't; a bump on the way to the third Pin costs it that Pin without the
resets and nothing with them; four averaged readings beat one; and its paths clear the Goals for
robots up to 17in. `macros.test.js` checks the lift, the claw and autoClaw against `macros.cpp`'s numbers, and that the lead holds a Pin from 3V to full speed where any one fixed trigger misses at one end. `clawtiming.test.js` checks the timing model: its rule is `autoClawAim()`'s, exact lags shut the jaws on `kGrabMm` at any speed, the best lead is the sensor's latency plus the valve whatever the tick, and the valve's spread sets the top speed. `trajectory.test.js` holds the planner's port to `tests/motion/trajectory_generator_test.cpp`'s
golden values, `auton.test.js` checks the auton functions against the simulated robot (background
motions caught on the way and cancelled, chaining, forced turn directions, the voltage cap, wall
resets, `waitUntilWaypoint()`), `pathview.test.js` pins the Paths tab's C++ both ways (and that Paste C++ reads `autons.cpp`'s
paths, Triple Pin's the same as `routines.js` drives) and its other-alliance and other-quadrant twins, and `field.test.js` pins the field to the manual's drawings and checks the
footprint and start checks. `tuner.test.js` checks the tuner on worlds
it can't see: calibration finds the noise, blocked and missing readings, wheel scale and a
mis-measured mount of a world it's only given logs of, the matched sensor delay lands near the
truth, and a search started from bad settings climbs out and holds on seeds it never saw; and that a
recorded run logs `mcl.state` and `mcl.pts` the way the robot does. Change a port's C++ side, change
the port.

Team 96671H: Hitmen
