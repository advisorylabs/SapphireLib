# Sapphire Sim: the localization simulator

A simulator of 96671H's robot for trying out odometry and Monte Carlo localization without the robot,
tuning the localizer, and showing people how MCL works. Plain HTML and JavaScript: no install and no
build step. It works offline; the only thing it fetches is a web font, and it falls back to a system
font.

It runs the robot program's own math, ported line for line from the library: odometry and its
correction, the particle filter and `MonteCarloLocalizer`, the holonomic drivetrain's motions
(`moveToPoint()`, `moveToPose()`, `turnToHeading()`, `followPath()`), and Auto-Tune's gain design.
The chassis it drives is the model Auto-Tune measures. So what you see here is what the robot does,
up to how well the simulated world matches the real one. [`docs/LOCALIZATION.md`](../../docs/LOCALIZATION.md)
explains the localizer itself.

## Opening it

Open `index.html` in Chrome, Edge, Firefox or Safari: double-click it. It also loads a few of the
analyzer's files (`../analyzer/js/`), so keep it inside the repo's `tools` folder. To hand it to
someone as one file, `node tools/sim/build.js` writes `tools/sim/dist/sapphire-sim.html`, with
everything inlined.

It opens running the Field tour routine. The **?** button (or the `?` key) lists the controls and
units; hover a label for what it means. **How to use it: [`docs/TOOLS.md`](../../docs/TOOLS.md)**:
the screen, the controls, each tab (Drive, Show, Events, Robot, MCL, Tune), and a few recipes for
showing someone how MCL works.

## How it's built

`js/` holds one file per concern, loaded in order by `index.html` as plain scripts. All but the
last three also load under Node, for the tests:

| File | What |
|---|---|
| `mcl.js` | Ports of the localization math: `Rng` (with the ziggurat), `LookupTable`, `FieldMap` and `ParallelRayCaster`, the sensor model and `ReadingScorer`, `ParticleFilter`, `sampleParticles()`, `computeOdometryDelta()`, `correctionStep()` |
| `world.js` | The physical world: the chassis (three `PlantSim` axes from the analyzer's `model.js`), tracking wheels, IMU, distance sensors, collisions, bumps, the defender |
| `robot.js` | The robot program: `Imu`, `Odometry`, `MonteCarloLocalizer`, the drivetrain and its motions (generators that yield where the C++ calls `pros::delay(10)`), and Auto-Tune's gain design |
| `routines.js` | The autonomous routines, written like `src/robot/autons.cpp`. Add your own here |
| `sim.js` | A world and a robot, run in 10 ms ticks, with the error history the page charts |
| `recorder.js` | A run, logged the way the robot logs it (`odom`, `chassis`, `mcl`, `mcl.beams`, `mcl.state`, `mcl.pts`, the `#meta` lines) |
| `tuner.js` | The MCL auto-tuner: the cases, open-loop replays, the search, the holdout check, and calibrating the world from logs |
| `render.js`, `app.js`, `tuneview.js` | The canvas drawing, the page, and the Tune tab |

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
passing beside its first doesn't end it early), and a spin in place leaves odometry still (so the
simulated tracking wheels follow `OdometryConfig`'s sign). `tuner.test.js` checks the tuner on worlds
it can't see: calibration finds the noise, blocked and missing readings, wheel scale and a
mis-measured mount of a world it's only given logs of, the matched sensor delay lands near the
truth, and a search started from bad settings climbs out and holds on seeds it never saw; and that a
recorded run logs `mcl.state` and `mcl.pts` the way the robot does. Change a port's C++ side, change
the port.

Team 96671H: Hitmen
