# Sapphire Sim: the localization simulator

A simulator of 96671H's robot for trying out odometry and Monte Carlo localization without the robot,
and for showing people how MCL works. Plain HTML and JavaScript: no install and no build step. It
works offline; the only thing it fetches is a web font, and it falls back to a system font.

It runs the robot program's own math, ported line for line from the library: odometry and its
correction, the particle filter and `MonteCarloLocalizer`, the holonomic drivetrain's motions
(`moveToPoint()`, `moveToPose()`, `turnToHeading()`, `followPath()`), and Auto-Tune's gain design.
The chassis it drives is the model Auto-Tune measures. So what you see here is what the robot does,
up to how well the simulated world matches the real one. [`docs/LOCALIZATION.md`](../../docs/LOCALIZATION.md)
explains the localizer itself.

## Opening it

Open `index.html` in Chrome, Edge, Firefox or Safari: double-click it. It also loads the analyzer's
`../analyzer/js/model.js` (the PID and gain design it shares), so keep it inside the repo's `tools`
folder. To hand it to someone as one file, build the single-file version:

```bash
node tools/sim/build.js
```

That writes `tools/sim/dist/sapphire-sim.html`, with everything inlined.

It opens running the Field tour routine, so there's something to watch straight away.

## What's on the screen

The field, with the robot where it **really** is (solid), where **odometry alone** thinks it is
(orange, dashed), and the **corrected pose** the motions drive by (blue). Around it, the particle cloud,
the estimate (×) and its 2σ uncertainty ellipse, and each distance sensor's beam. **Follow ×4** and
**×12** zoom in on the robot; the cloud is usually smaller than the robot at full-field scale.

Under the field: how far odometry alone and the corrected pose are from the truth right now, the
particle spread, how many sensors agree with the walls, and whether the localizer is correcting
odometry. The chart shows the last minute of the same errors.

## The tabs

- **Drive**: run an autonomous routine (from [`js/routines.js`](js/routines.js)), or drive it
  yourself (W/S forward and back, A/D strafe, Q/E or ←/→ turn, Shift for slow, Space pauses). Switch
  the motions between **odometry + MCL** and **odometry alone** (`setCorrectionEnabled()`) and run the
  same routine both ways. The gains are **Auto-Tuned** (designed from the chassis model with the
  robot's own specs) or the robot's **hand-set** ones from `devices.cpp`.
- **Show**: the visualizer toggles. Particles colored by how well they explain the last readings
  (amber best, grey worst) and sized by weight; the uncertainty ellipse; sensor beams; **expected vs
  measured**, which draws what the map says each sensor should read from the estimate (dashed) against
  what it did read (a tick), green when they agree and red when they don't; odometry alone, the
  corrected pose and the correction between them; trails; the running motion's target; the tracking
  wheels and sensors on the robot.
- **Learn**: how MCL works, in three steps, and a **step-through** that pauses the simulation and runs
  the next update one step at a time (predict, weigh, resample), with the field showing what each
  step did and a caption saying why. Click any particle to **inspect** it: where its beams would hit,
  what each reading scores from there, and why it weighs what it does. And the sensors' readings
  against the map right now, with why the localizer is or isn't correcting.
- **Events**: things odometry never sees. **Bump** shoves the robot 6 in without its tracking wheels
  turning; **pick it up and move it** drops it where you click; **relocalize globally** scatters every
  particle over the field. A **defender** robot to drag around or set patrolling, and generic **field
  elements** that can be in the world, in the localizer's map, both, or neither. Scenarios set up
  worn tracking wheels, mis-measured offsets, a vertical offset with the wrong sign (the wheel is on
  the robot's right but the offset was entered as positive: every turn drags the pose sideways, and
  MCL can't fully rescue it), noisy sensors, or a crowded field in one click.
- **Robot**: the world's errors, which the robot program doesn't know about: tracking wheel diameter
  error, slip and offset error, IMU drift and scale error, distance sensor noise, delay and dropouts.
  And the chassis model (kS, kV, kA per axis, plus delay) that Auto-Tune would measure. **Auto-Tune**
  designs gains from it with the robot's pole placement.
- **MCL**: every `LocalizerConfig` setting that matters, live. It writes what you changed two ways:
  as `TUNE.CFG` lines (the robot loads them from its SD card at startup) and as the
  `LocalizerConfig{...}` for `localizerSettings()` in `src/robot/tune.cpp`.
- **Tune**: the localizer, auto-tuned. A search over `LocalizerConfig` on a suite of runs: the
  simulator's scenarios, or **your robot**, a world calibrated from the robot's own logs (open the
  `SL*.CSV` files from its card; **Demo** uses a simulated robot's) with the paths it really drove
  replayed. Every candidate replays the same paths with the same noise, the winner is checked on
  runs it never saw, and **Save TUNE.CFG** writes the next revision of the robot's tuning file,
  keeping the analyzer's PID lines. How it all works: [`docs/TUNING.md`](../../docs/TUNING.md).

## Showing it to someone

1. Let the Field tour run on **Whole field**. The orange ghost slowly wanders off the real robot; the
   blue one doesn't.
2. **Events → Bump the robot.** Orange jumps and stays wrong. Blue jumps too, then eases back within
   a second or two. The chart shows both.
3. **Follow ×4**, then **Learn → Step: predict / weigh / resample**. Click a particle far from the
   robot and look at why it scored badly.
4. **Events → Crowded field.** The defender turns a sensor's check red; the estimate holds.
5. **Drive → Odometry alone**, run a routine, then **Odometry + MCL** and run it again. Compare where
   each one ended.

## How it's built

`js/` holds one file per concern, loaded in order by `index.html` as plain scripts. The first five
also load under Node, for the tests:

| File | What |
|---|---|
| `mcl.js` | Ports of the localization math: `Rng`, `FieldMap`, the sensor model, `ParticleFilter`, `computeOdometryDelta()`, `correctionStep()` |
| `world.js` | The physical world: the chassis (three `PlantSim` axes from the analyzer's `model.js`), tracking wheels, IMU, distance sensors, collisions, bumps, the defender |
| `robot.js` | The robot program: `Imu`, `Odometry`, `MonteCarloLocalizer`, the drivetrain and its motions (generators that yield where the C++ calls `pros::delay(10)`), and Auto-Tune's gain design |
| `routines.js` | The autonomous routines, written like `src/robot/autons.cpp`. Add your own here |
| `sim.js` | A world and a robot, run in 10 ms ticks, with the error history the page charts |
| `recorder.js` | A run, logged the way the robot logs it (`odom`, `chassis`, `mcl`, `mcl.beams`, the `#meta` lines) |
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
corrected before `setPose()`, a closed `followPath()` lap drives the whole lap, a spin in place leaves
odometry still (so the simulated tracking wheels follow `OdometryConfig`'s sign), and stepping through
an update by hand gives the same run as ticking. `tuner.test.js` checks the tuner on worlds it can't
see: calibration finds the noise, blocked and missing readings, wheel scale and a mis-measured mount
of a world it's only given logs of, the matched sensor delay lands near the truth, and a search
started from bad settings climbs out and holds on seeds it never saw. Change a port's C++ side, change the port.

Team 96671H: Hitmen
