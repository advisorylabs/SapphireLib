# Tools: quick reference

How to use the simulator (`tools/sim`) and the telemetry analyzer (`tools/analyzer`). Both are
plain HTML: double-click `index.html`, no install, works offline. Keep them inside the repo's
`tools` folder (each loads a few of the other's files), or build a single file to hand around:
`node tools/sim/build.js`, `node tools/analyzer/build.js` (written to each tool's `dist/`).

Both have a **?** button (or press `?`) with the controls and units, and a theme button.

Contents: [Units](#units) · [Simulator](#simulator) · [Analyzer](#analyzer) ·
[Reading the localizer in a log](#reading-the-localizer-in-a-log) ·
[Particle count](#particle-count-what-it-costs-what-it-buys) · [What gets logged](#what-gets-logged)

## Units

| | |
|---|---|
| Distance | inches |
| Angles | degrees; headings clockwise from +y (`odom::Pose`) |
| Field frame | origin at the field's center, +x right, +y downfield (up the screen); walls at ±70.25 in (140.5 in square). The analyzer's demo uses a 0 to 144 in corner frame. |
| Uncertainty ellipse | 2σ of the particle cloud |
| Volts, amps, °C, rpm | motors and chassis, as the V5 reports them |
| `us` | microseconds of wall time: an upper bound on CPU (another task preempting counts too) |
| Time (analyzer) | match time: `A0:12` is 12 s into autonomous, `1:03` is driver control |

## Simulator

The robot's own odometry, Monte Carlo localizer, motions and Auto-Tune gain design (JavaScript
ports, held to the C++ tests' golden values) on a simulated field. It opens running the Field
tour routine.

**On screen:** the real robot (solid), odometry alone (orange dashed), the corrected pose the
motions use (blue), the particles (amber explains the readings best, grey worst; size is weight),
the estimate (×) and its ellipse, and each sensor's beam. Tiles under the field: odometry's and the
corrected pose's error from the truth, the cloud's spread, how many sensors agree with the map, and
whether it's correcting (and if not, why). The chart is the last 60 s of the same errors.

**Controls:**

| | |
|---|---|
| `W` `S` | drive forward, back |
| `A` `D` | strafe |
| `Q` `E` or `←` `→` | turn |
| `Shift` | slow (40%) |
| `Space` | pause, play |
| Drag | the defender robot |
| Click | where to put the robot, after Events → Pick up |
| Follow ×4, ×12 | zoom on the robot (scale bar in the corner) |

Driving stops a running routine. Hover a label or slider for what it means.

**Tabs:**

- **Drive**: run a routine (`js/routines.js`, written like `src/robot/autons.cpp`), drive it
  yourself, switch the motions between odometry + MCL and odometry alone
  (`setCorrectionEnabled()`), Auto-Tuned or hand-set PID gains.
- **Show**: which layers the field draws.
- **Events**: bump (6 in, wheels don't see it), pick up and put down, relocalize globally, the
  defender (sensors see it, the map doesn't), field elements in the world and/or the map, and
  scenarios (worn wheels, wrong offsets, noisy sensors, crowded field) that reset the world.
- **Robot**: the world's errors the robot program doesn't know about (wheel scale and slip, IMU
  drift, sensor noise, delay, dropouts, outliers), and the chassis model Auto-Tune measures.
- **MCL**: every `LocalizerConfig` setting, live. The panel below writes the changes as
  `TUNE.CFG` lines and as C++ for `localizerSettings()` in `src/robot/tune.cpp`.
- **Tune**: auto-tunes `LocalizerConfig`. Standard suite (the scenarios), or **Your robot**: open
  the `SL*.CSV` logs from the SD card (plus its `TUNE.CFG`), and it calibrates a simulated world
  to match them (noise, dropouts, latency, mounts, wheel scale) and replays the robot's own paths.
  **Save TUNE.CFG** writes the next revision, keeping the analyzer's PID lines. The loop is in
  [`TUNING.md`](TUNING.md).

**Recipes:**

- *Show someone MCL:* let the tour run, then Events → Bump. Orange stays off; blue eases back in
  a second or two. Follow ×4 to see the cloud.
- *MCL vs odometry alone:* Robot → wheel diameter error 3%, run Square laps both ways from the
  Drive tab, compare the end errors in the result line.
- *Try a particle count:* MCL tab → Particles. Then Events → Relocalize globally and watch how
  long it takes to find the robot.
- *Blocked sensors:* Events → Crowded field. Beams through the defender turn red; the estimate
  holds.

## Analyzer

Drop the `SL*.CSV` files from the SD card's `sl` folder on the page (or **Open logs**). Several
at once is fine. It opens on a demo (a simulated pit session and a match with faults planted).
Pick the run and the match at the top; **Whole run** shows everything.

**Tabs:**

- **Overview**: findings, worst first. Click a title for why; **Replay** and **Chart** jump to the
  moment. Motor temperatures and a motor table below.
- **Replay**: the match played back: the field (with the localizer, below), the lift, motors,
  battery, controller and what was happening, over synced charts. ▼ on the timeline are findings.
- **Charts**: any column of any channel on one time axis, with presets (motors, PIDs, battery,
  odometry, **Localizer**, **Distance sensors**). The table under the charts shows values at the
  cursor; the visible range copies out as CSV.
- **PID responses**: every step response, its overshoot and settle time, and P/I/D terms.
- **Motions**: every drivetrain motion, its result, its path and its PIDs' error.
- **Tune**: refit a system's model from an Auto-Tune run or match driving, design gains the way the
  robot does, replay this log's targets with old and new gains, copy the C++.
- **Refine**: fine-tune the drive and turn PIDs from every logged motion; **Save TUNE.CFG**. Run
  by run shows what each run ran and what changed it.

**Controls:**

| | |
|---|---|
| `Space` | play, pause (with the replay timeline focused) |
| `←` `→` | step 0.5 s; with `Shift`, 5 s |
| Click a chart | set the cursor; every view follows |
| Drag on a chart | zoom to that span; double-click zooms out; Ctrl+wheel zooms |
| Follow ×4 | the replay's field zooms on the robot |

## Reading the localizer in a log

Replay's **Field** draws, at the cursor (toggles above the field):

| Mark | Is |
|---|---|
| The robot | the corrected pose, what the motions drove by (`odom`) |
| Orange dashed square, arrow | raw odometry (tracking wheels and IMU alone), and the correction to the corrected pose |
| × and ellipse | the localizer's estimate and its 2σ uncertainty |
| Blue dots | 24 particles picked by weight (4 times a second, moved with the estimate between) |
| Dashed beam, tick | the map's distance for that sensor from the estimate, and what it read: green agrees (within `agreementSigmas`·σ), red doesn't, grey read nothing |

The **Localizer** panel beside it: correcting or not (and why), **drift** (how far raw
odometry is off), **spread** and effective particles, sensors agreeing of those reading, **fit**
(how well the particles explain the readings, 1 is perfect, and that against its usual level:
recovery scatters fresh particles when it drops), particles recovery scattered, and the update's
time. Then each sensor: what it read, what the map says, and the difference.

**Why it isn't correcting** (`blocked`), and what to do:

| Says | Means | Look at |
|---|---|---|
| no setPose() yet | odometry isn't in the field frame | `setPose()` at the start of autonomous |
| spinning | turning faster than `maxTurnRateDegPerS`; readings skipped | normal mid-turn |
| no readings | no sensor had a usable reading | out of range (78 in), unplugged, low confidence |
| too spread out | the cloud disagrees (spread over `maxCorrectionSpreadIn`) | right after a bump or relocalize it's normal; if it lasts, motion noise too high or readings too few |
| too few agree | under `minAgreeingSensors` readings match the map | a blocked sensor, a wrong mount in `config.hpp`, or something in the map that isn't on the field |
| refused | a `setPose()` landed during the update | harmless |

**Findings it raises:** the localizer not correcting for 3 s or more (with the main reason),
odometry losing inches at once (a hit or a wheel off the floor), a sensor that disagrees with the
map a quarter of the time (steady offset: its mount is wrong; scattered: something blocks it), a
sensor that reads nothing half the time with a wall in range, and updates over 5 ms.

**Charts:** the replay's Localizer chart has drift, spread and the correction actually applied;
the Localizer preset in Charts adds sensors used/agreeing, fit and update time.

## Particle count: what it costs, what it buys

The particle filter uses lookup tables (a ziggurat for its noise, a softplus table for the sensor
model) and a division-free raycast. Estimated per update on the brain (a Cortex-A9 `-Os` build's
instruction counts under QEMU, weighted by the A9's cycle costs; the `mcl` channel's `us` column
is the real number):

| Particles | Before the tables | Now | Share of the brain at 20 Hz, now |
|---|---|---|---|
| 300 | 1.1–2.1 ms | 0.34–0.76 ms | 0.7–1.5% |
| 1000 | 3.5–6.5 ms | 1.1–2.4 ms | 2.1–4.8% |
| 3000 | 10–19 ms | 3.2–7.1 ms | 6–14% |

In the simulator (the same paths and noise at each count), tracking error barely changes past 300:
sensor noise and the 4 in/s correction limit set it. More particles do find the robot faster
after a bump (median 13 updates at 300, 7 at 1000, 4 at 3000, standing still) or a global
relocalization (2.7 s, 1.0 s, 0.1 s).

So: **1000 now costs what 300 used to**, and buys faster recovery; set
`mcl.filter.particleCount=1000` in `TUNE.CFG` and check `us` in the next log. Thousands are only
worth it if you relocalize globally often.

## What gets logged

Per localizer update (20 Hz): `mcl` (estimate, spread, sensors, correction, `us`, raw odometry),
`mcl.beams` (each sensor's reading, the map's distance, closing speed), and `mcl.state`
(covariance, fit, recovery, resampled, why it didn't correct, beam heading). Every 5th update:
`mcl.pts` (24 particles as offsets from the estimate). `odom` carries `raw_x,raw_y` beside the
corrected pose, which the analyzer's fits use (a correction easing in isn't motion).

That's about 36 rows a second more than before `mcl.state`, `mcl.pts` and the raw odometry
columns: about 5% more rows and 12% more values (around 4 KB/s). Recording a row is a 64 byte
copy; formatting and SD writes run on the logger's lowest-priority task. To log less, raise
`kParticleEvery` or drop `mcl.pts` in `src/robot/telemetry.cpp`. The full format is
[`TELEMETRY_FORMAT.md`](TELEMETRY_FORMAT.md).
