# Localization: odometry with Monte Carlo localization on top

Odometry is good, but it only ever adds. Tracking wheels read a percent or two long or short, slip
when the robot is hit, and the IMU drifts a degree or two a minute, and every one of those errors
stays in the pose for the rest of the match. The first motion of a routine lands where it should; the
tenth lands a few inches off, and a hard hit in the middle can move it off by several more.

Monte Carlo localization (MCL) fixes that with distance sensors that can see the field walls. It never
replaces odometry. It runs beside it, checks it against what the sensors see, and eases odometry's
pose back whenever the two disagree. Nothing else changes: `moveToPoint()`, `moveToPose()` and
`followPath()` read `odometry().getPose()` exactly as before, with whatever gains Auto-Tune gave them,
and now that pose is corrected.

**Try it without a robot first.** [`tools/sim/`](../tools/sim/) is a simulator of 96671H's robot:
the same odometry, localizer and motions (ported line for line and checked against these C++ tests),
against a simulated field with realistic sensor noise, slip, bumps and a defender. Open
`tools/sim/index.html` (see its [README](../tools/sim/README.md)). Its Learn tab steps through one
update at a time, which is the quickest way to see what the rest of this page describes.

Contents: [How it works](#how-it-works) · [Setting it up](#setting-it-up) ·
[Autonomous](#autonomous) · [What it won't do](#what-it-wont-do) · [Tuning](#tuning) ·
[Telemetry](#telemetry) · [Checking it on the robot](#checking-it-on-the-robot) ·
[The code](#the-code)

## How it works

The localizer keeps a few hundred *particles*, each one a guess at where the robot is (x and y;
heading always comes from the IMU, which on a V5 robot is far better than anything the distance
sensors could work out). Every 50 ms it runs three steps:

1. **Predict.** Move every particle by the distance odometry measured since the last update, plus a
   little random noise, because odometry isn't perfect. The cloud follows the robot and spreads out a
   bit.
2. **Weigh.** For each particle, work out what each distance sensor *would* read from there (a
   raycast against the map), and compare it with what the sensor *did* read. Particles that explain
   the readings get heavy; the rest get light. A reading nothing explains (another robot in the way)
   counts as an outlier: every particle explains it equally badly, so it stops telling them apart
   instead of dragging the cloud toward the wrong answer.
3. **Resample.** Once the weights get lopsided, copy heavy particles and drop light ones, so the
   cloud gathers where the robot really is.

The cloud's weighted average is the *estimate*. Then the localizer decides whether to trust it, and
only corrects odometry when all of these hold:

- `setPose()` has run at least once, so odometry is in the field frame the map describes
  (`LocalizerConfig::waitForSetPose`);
- the cloud is tight: its spread is under `maxCorrectionSpreadIn` (3 in);
- at least `minAgreeingSensors` (2) of the readings match the map from the estimate, within
  `agreementSigmas` (3σ). A sensor blocked by a defender disagrees; the walls the others see still
  agree.

The correction is an offset: odometry's pose becomes its raw pose plus that offset
(`Odometry::setPositionCorrection()`), and the offset *eases in*, at most `maxCorrectionRateInPerS`
(4 in/s), rather than jumping. A jump in the pose is a spike in every motion's derivative term: an
Auto-Tuned drive kD is around 0.7 V per in/s, so a 0.2 in jump in one 10 ms tick would be 14 V. At
4 in/s the motion sees at most a steady 2.7 V nudge while a correction is still easing in, and
ordinary drift needs far less than that.

Recovery: if the readings suddenly stop matching (a hard hit shoves the robot and the tracking wheels
never saw it), the filter notices its fit dropping and scatters a share of fresh particles within
18 in of the estimate (`RecoveryConfig`). The next few readings sort them out. In the simulator the
corrected pose is back within an inch about 1.5 s after a 6 in shove mid-routine, most of that the
correction easing in at 4 in/s. Without recovery it takes two to three times as long while the robot
is moving, and a robot standing still never finds its way back.

## Setting it up

You need distance sensors that can see the walls. 96671H's robot uses four, one per side. More
sensors see more walls at once, and two perpendicular ones are enough to pin the position down.

**Mount them** low enough to see the perimeter walls (not over them) and high enough to miss game
elements where you can. The V5 Distance Sensor ranges from 20 mm to 2000 mm (0.8 in to 78 in) with
about ±15 mm of noise up close and ±5% past 200 mm, so readings of the far wall from the middle of a
140 in field are out of range. That's fine: the localizer uses whichever readings it gets.

**Measure each sensor's mount** from the tracking center (the point odometry tracks) to the sensor's
*face*: inches forward (negative behind), inches right (negative left), and which way it faces
(0 forward, 90 right, 180 back, 270 left). An inch of error here is an inch of bias along that
sensor's beam.

**Build the localizer** on your odometry, after it:

```cpp
#include "sapphirelib/api.hpp"

using namespace sapphirelib::localization;

MonteCarloLocalizer& localizer() {
    static MonteCarloLocalizer instance(
        odometry(),
        {
            DistanceSensorConfig{.port = 1, .mount = {.forwardIn = 7.0, .rightIn = 0.0, .facingDeg = 0}},
            DistanceSensorConfig{.port = 4, .mount = {.forwardIn = 0.0, .rightIn = 7.0, .facingDeg = 90}},
            DistanceSensorConfig{.port = 6, .mount = {.forwardIn = -7.0, .rightIn = 0.0, .facingDeg = 180}},
            DistanceSensorConfig{.port = 11, .mount = {.forwardIn = 0.0, .rightIn = -7.0, .facingDeg = 270}},
        },
        FieldMap::centered(), // the perimeter, origin in the middle of the field
        LocalizerConfig{});   // the defaults suit most robots
    return instance;
}

void initialize() {
    // ...
    odometry().startTask();
    localizer().startTask(); // after odometry, which it reads
    drivetrain().setOdometry(&odometry());
}
```

96671H's is `localizer()` in [`src/robot/devices.cpp`](../src/robot/devices.cpp), with its ports and
mounts in [`include/robot/config.hpp`](../include/robot/config.hpp).
[`examples/localization.cpp`](../examples/localization.cpp) is a complete small program.

**The map.** `FieldMap::centered()` is a standard field's perimeter, 140.5 in wall to wall (the game
manual allows ±0.5 in; measure your practice field), with the origin in the middle, +x right, +y
downfield, the same frame as `odom::Pose`. Add solid, fixed structures the sensors will see with
`addBox()` or `addSegment()`. Leave out anything that gets pushed around. A thing in the world but not
in the map only produces outliers, which the model shrugs off; a thing in the map that isn't there
makes the localizer expect a wall that isn't, which is worse. The simulator's Events tab shows both.

## Autonomous

Start every routine with a `setPose()` at the robot's real place on the field, in field coordinates:

```cpp
void twoGoalAuton() {
    odometry().setPose({.xIn = -48, .yIn = -60, .headingDeg = 270});
    if (!drivetrain().moveToPoint(-60, -60).settled()) return;
    drivetrain().moveToPose(-60, -24, 0);
}
```

That one call puts odometry in the map's frame, and the localizer notices it and starts its particles
there (within `startSpreadIn`, 2 in). Until the first `setPose()`, odometry's pose is relative to
wherever the robot sat at `initialize()`, so the localizer corrects nothing. In the simulator, a
localizer allowed to correct anyway *did* find the robot eventually, but on the way briefly latched
onto a wrong spot over 30 in away.

To compare, `localizer().setCorrectionEnabled(false)` leaves odometry alone while the filter keeps
running; drive the same route both ways.

## What it won't do

- **Correct heading.** The IMU owns heading. Calibrate its scale (`sensors::calibrateHeadingScale()`)
  so a long match doesn't drift.
- **Work without seeing walls.** With no usable readings (all out of range, all blocked, or turning
  faster than 200°/s), it predicts from odometry and holds the last correction. The estimate spreads
  out until the sensors see something again.
- **Fix bad odometry.** It corrects drift, not a wrong offset or a reversed wheel. Get odometry right
  first; a pose that's badly wrong on every turn will fight the localizer all match. The usual
  culprit is the vertical wheel's offset sign, which is backward from what you'd guess: positive is
  *left* of center (see `OdometryConfig`, and the simulator's "Vertical offset sign wrong" scenario
  for what the mistake looks like).
- **Know the time a reading was taken.** The sensor measures at about 30 Hz, so a reading is a few
  tens of milliseconds old when it's read. `sensorLatencyMs` (30) shifts each reading by odometry's
  speed along the beam to compensate. At 60 in/s, 30 ms is nearly 2 in.

## Tuning

The defaults were chosen in the simulator across clean, worn-wheel, noisy-sensor, bumped and
crowded-field runs. Change them there first: its MCL tab has every setting, and its **Tune** tab
searches them for you, on the simulator's scenarios or on a simulated robot calibrated from your
robot's own logs (sensor noise, blocked and missing readings, sensor delay, mount errors, tracking
wheel scale), replaying the paths it really drove. What it finds goes to the robot as `mcl.*` lines
in `TUNE.CFG` on the SD card, no rebuild needed. The whole loop is
[`docs/TUNING.md`](TUNING.md). The ones that matter most:

| Setting | Default | Raise it when | Lower it when |
|---|---|---|---|
| `filter.particleCount` | 300 | the estimate is jumpy, or for `relocalizeGlobally()` (try 1500) | the brain is short on CPU |
| `filter.motionNoise.perInch` | 0.05 | odometry's error outruns the cloud (worn wheels, slip): the corrected pose lags behind | the estimate wanders between readings |
| `filter.beam.outlierProbability` | 0.1 | the field is crowded and readings are often blocked | the sensors always see clean walls and you want sharper corrections |
| `maxCorrectionRateInPerS` | 4 | corrections after a bump take too long to land | motions twitch while a correction eases in |
| `minAgreeingSensors` | 2 | a blocked sensor ever pulls the pose | the robot rarely has two walls in range |
| `sensorLatencyMs` | 30 | readings lag the robot at speed | never set it above what you've measured |

A few are easy to get wrong: a `maxCorrectionRateInPerS` of 0 applies corrections in one step, which
the motions see as a spike. Too few particles make the estimate noisy, and recovery can only search
as far as `recovery.radiusIn`.

## Telemetry

While recording, 96671H's robot logs every localizer update from the localizer's own task:

- `mcl`: estimate, spread, effective particles, sensors used and agreeing, whether it corrected,
  the correction, the update's time in microseconds, and raw odometry.
- `mcl.beams`: each sensor's reading next to what the map says it should read and its closing
  speed.

These sit next to `odom`, the corrected pose. The correction is how far raw odometry had drifted,
so a match's log shows how much the localizer earned its keep. Every file's `#meta` lines record
the settings it ran with and the sensor mounts. See [`docs/TELEMETRY_FORMAT.md`](TELEMETRY_FORMAT.md);
the simulator calibrates from these channels ([`docs/TUNING.md`](TUNING.md)).

`MonteCarloLocalizer::setUpdateCallback()` hands any code the same per-update data
(`LocalizerUpdate`: the estimate, the status, raw odometry, and a `BeamSample` per sensor).

Auto-Tune measures its translation axes from the raw pose (`odometry().snapshot().rawPose`), not
`getPose()`, since a correction easing in during a characterization run would read as speed the
chassis never had.

## Checking it on the robot

None of this has run on the real robot yet. The math is unit-tested on a desktop compiler and matched
bit for bit by the simulator, and the simulator's physics is the same axis model Auto-Tune measures,
but real sensors, real walls and real wheel slip are what count. Before trusting it in a match:

1. **Sensor mounts.** Set the pose where the robot really sits and leave it still. In
   `localizer().status()` (or the `mcl` log), every sensor that sees a wall should agree, and the
   correction should be well under an inch. A sensor that never agrees has a wrong mount, or faces
   the wrong way.
2. **Drift without it.** Run a long routine with `setCorrectionEnabled(false)` and measure how far off
   the robot ends. That's what the localizer has to fix.
3. **Drift with it.** The same routine with correction on. It should end within an inch or so.
4. **Bumps.** Shove the robot mid-routine. `mcl.corr_x`/`corr_y` in the log should jump by about the
   shove, and the routine should still land.
5. **A crowded field.** Park another robot in front of a sensor. `agree` should drop by one while the
   estimate holds.

## The code

| Piece | Where | Tested by |
|---|---|---|
| `FieldMap`: walls, boxes, raycasting | `localization/field_map.hpp` | `tests/localization/field_map_test.cpp` |
| `LocalizerConfig`, and every field by name and range (for `TUNE.CFG`) | `localization/localizer_config.hpp` | `tests/localization/localizer_config_test.cpp` |
| The sensor model: mounts, the beam likelihood, reading conversion, latency | `localization/sensor_model.hpp` | `tests/localization/sensor_model_test.cpp` |
| `ParticleFilter`: predict, weigh, resample, recovery | `localization/particle_filter.hpp` | `tests/localization/particle_filter_test.cpp`: unit checks, plus closed-loop runs on a simulated field against biased odometry, a blocked sensor and a bump |
| `Rng`: seeded xoshiro128** | `util/random.hpp` | `tests/util/random_test.cpp` |
| `MonteCarloLocalizer`: sensors, gating, the correction | `localization/monte_carlo_localizer.hpp` | on the robot; its logic runs in the simulator |
| Odometry's correction: `snapshot()`, `setPositionCorrection()`, the ease-in | `odom/odometry.hpp`, `odom/odometry_math.hpp` | `tests/odom/odometry_math_test.cpp` (`correctionStep()`) |

Everything but `MonteCarloLocalizer` and `Odometry` is pure (no PROS). The simulator's
[`js/mcl.js`](../tools/sim/js/mcl.js) is a line-for-line port held to the C++ tests' golden values,
including a seeded filter run that has to land on the same particles.
