# Tuning: from Auto-Tune to real matches, and back to the robot

Every number the robot runs on comes from somewhere: Auto-Tune measures the chassis and designs the
PID gains; the localizer's settings were chosen in the simulator. Both are guesses about the real
robot, made from a few seconds of measurement or from a simulated one. This page is the loop that
makes them better with every practice session: record what the robot really did, let the tools
work out what that says about the settings, accept the changes you agree with, and put them on the
SD card, where the robot picks them up at its next start, no rebuild needed.

```
   robot ── SD card logs ──▶ simulator Tune tab ──▶ MCL settings ─┐
     ▲                    └─▶ analyzer Refine tab ──▶ PID gains ───┤
     │                                                            ▼
     └──────────────── TUNE.CFG on the SD card ◀──── you accept ──┘
```

Contents: [Logging](#logging) · [TUNE.CFG](#tunecfg) · [Tuning the localizer in the simulator](#tuning-the-localizer-in-the-simulator) ·
[Calibrating from the robot's logs](#calibrating-from-the-robots-logs) ·
[Refining PIDs from real motions](#refining-pids-from-real-motions) ·
[Checking it on the robot](#checking-it-on-the-robot) · [The code](#the-code)

## Logging

96671H's robot records on demand, not from power-on:

- **In the pits**, the Home page has a **Start log** button next to its SD line. Tap it before a
  test run and **Stop log** after; each recording is its own file (`/usd/sl/SLnnnnnn.CSV`). Stop
  writes everything recorded and closes the file, so once the line reads "SD: ready, not logging"
  the card is safe to pull.
- **In a match**, recording starts on its own when the robot is plugged into a competition switch
  or the field, and stops 5 s after it's unplugged (so a tether that drops for a moment doesn't
  split the match into two files). The SD line reads "SD: logging SL000042 (match)".

The Home page's SD line also says "no card", "waiting for card" (recording, no card yet), "no
folder, root" (make the `sl` folder on a computer), or "FAULT".

**What it costs.** Recording a row copies 64 bytes into a ring buffer on the task that records it.
Polling runs on a task one step below default priority, and formatting and SD writes on the
lowest-priority task, so they only use what the control loops leave. The logger measures itself:
every file's `H` rows carry `samp_us` and `fmt_us`, the microseconds its two tasks spent in the last
second, and the `mcl` channel carries `us`, each localizer update's time. Both are wall time, so an
upper bound. The analyzer's Refine tab and the simulator's calibration show them as a share of the
brain. Check the first real match's numbers before trusting the estimate here: expect the logger
under a few percent and the localizer around a percent at 300 particles and 20 Hz (0.3 to 0.8 ms an
update, estimated from the filter's instruction counts; [`docs/TOOLS.md`](TOOLS.md) has the table).

Other robots: `LoggerConfig::recordAtStart` (true by default, one file per program run) and
`recordUnderCompetition` choose the policy; `Logger::startRecording()`/`stopRecording()` are safe
from any task. The format is [`TELEMETRY_FORMAT.md`](TELEMETRY_FORMAT.md).

## TUNE.CFG

`TUNE.CFG` is a text file in the `sl` folder (or the card's root) that the robot reads once at
startup, before it builds the localizer or runs a PID:

```
format=1
rev=8
note=PID refinement: Turn
pid.turn=0.442,0,0.0849        # kP,kI,kD
model.turn=0.6,0.045,0.00992   # kS,kV,kA
mcl.periodMs=50
mcl.filter.motionNoise.perInch=0.07
```

- One `key=value` per line; `#` starts a comment.
- 96671H's robot accepts:
  - `pid.drive`, `pid.turn`, `pid.hold`, `pid.lift`
  - `model.fwd`, `model.strafe`, `model.turn` (Auto-Tune's axis models, which velocity driving uses)
  - `lift.gravityVolts`, `mcl.periodMs`
  - every `LocalizerConfig` field as `mcl.<path>`, for example `mcl.filter.beam.outlierProbability`

  The list is `schema()` in [`src/robot/tune.cpp`](../src/robot/tune.cpp); the ranges are the
  library's ([`localizer_config.cpp`](../src/sapphirelib/localization/localizer_config.cpp)).
- **All or nothing.** One bad line (an unknown key, a value out of range, a key twice) and the
  robot uses none of the file. The Home page says "Tune: TUNE.CFG REJECTED, line 4", the terminal
  and every log's `#meta,tune.error` say why, and the robot runs the code's values. It never runs
  half a profile.
- **The code stays the fallback.** No card, no file or a rejected one: the robot runs exactly what's
  compiled in. Anything the file leaves out comes from the code too.
- **Every log says which revision it ran** (`#meta,tune.rev`), and the Home page shows it ("Tune:
  TUNE.CFG r8, 6 settings"), so a run is never a mystery.
- **The tools write it.** The simulator's Tune tab saves the `mcl.*` lines and the analyzer's Refine
  tab the `pid.*` and `model.*` lines. Each opens the current file (copy it off the card with the
  logs), keeps every line it doesn't own, adds one to `rev`, and notes what changed in a changelog
  at the bottom (comments, so the robot ignores them):
  ```
  # --- changelog, newest first ---
  # r8 2026-10-02 analyzer: pid.turn 0.885,0,0.107 -> 0.442,0,0.0849; model.turn ...
  # r7 2026-09-30 sim tuner: mcl.filter.motionNoise.perInch 0.05 -> 0.07
  ```
- Once values have settled, move them into the code (the analyzer's Tune tab and the simulator's MCL
  tab print the C++) and delete the lines, so the code and the robot agree again.

Gains you set on the PID page, and Auto-Tune's results, still only last until the robot restarts.
The analyzer's history shows when a restart lost them; save the ones worth keeping through the
Refine tab.

## Tuning the localizer in the simulator

[`tools/sim`](../tools/sim/), **Tune** tab. The search changes the localizer's settings to keep
the corrected pose (what every motion drives by) as close as it can to where the robot really is,
over a suite of simulated runs:

- **Standard suite**: a clean robot, worn tracking wheels, two bumps, noisy sensors, and a crowded
  field, each with one of the routines. Settings that hold up across all of them are a sound start
  for any robot. The library's defaults were chosen this way, and a search from them finds a few
  percent at most: they're close to the best the standard suite can tell apart.
- **Your robot**: the simulated world calibrated from the robot's own logs (next section), with two
  routines and up to three paths the robot really drove, replayed. This is where a real robot that
  differs from the defaults (slower sensors, more noise, a crowded field, worn wheels) gains.

**How it searches.** A coordinate pattern search: each setting a step up and down from the best so
far, keeping a change only when it lowers the score; once a full sweep finds nothing, the steps
halve. Three things keep it honest:

- **Open loop.** Each case is driven once to record its true path, and every candidate replays
  that path. Driven closed loop, each candidate would steer a slightly different path, the
  simulated noise would land differently, and that noise (about 10% per case) is bigger than most
  settings' effects. On one path, the world's random draws are identical for every candidate, so
  only the localizer differs.
- **Significance.** A change is kept only if, run for run against the current best (same case,
  same seed), it's better by more than one standard error, and by at least 1%.
- **A holdout.** The winner is run on seeds the search never saw. "Better" means it beat the start
  there by more than two standard errors; "within noise" or "no better" means keep what you have.

**The score** is relative: each case's error with the candidate over its error with the starting
settings, averaged, so 1.0 is no change and 0.8 is 20% less error. A case's error is the corrected
pose's mean distance from the truth, plus half its 95th percentile, a quarter of where it ended,
and half a second per second a bump took to recover. A case more than 5% worse than it started
costs extra, so the search can't buy a better average by giving up one kind of match.

**What it leaves alone by default**, and why:

- `sensorLatencyMs`: in the simulator, compensating the full sensor delay often does worse than
  under-compensating, because compensation also widens each reading's uncertainty by half the shift
  it applies. Search it once the world is calibrated from logs.
- `maxCorrectionRateInPerS`: faster corrections land sooner but push the motions harder, which the
  score can't see.
- `filter.particleCount` and the update period: they cost CPU on the brain. Searched, they pay 15%
  of a point per extra share of the starting CPU.

**What to do with it.** "Use in the simulator" runs the result (and the MCL tab shows it as C++ and
as `TUNE.CFG` lines). "Save TUNE.CFG" writes the next revision.

## Calibrating from the robot's logs

Same tab, **Calibrate from the robot's logs**. Open the `SL*.CSV` files (and `TUNE.CFG`) from the
card. It needs the `mcl` and `mcl.beams` channels, a few minutes of driving after an autonomous
`setPose()` (so the pose is in field coordinates), and walls in the sensors' range. **Demo** runs it
on a simulated robot whose true errors it shows alongside.

From each update where the localizer trusted its estimate, a reading's error against the map
(`m - e` in `mcl.beams`) is the sensor's error plus a little of the estimate's:

- **Sensor noise**: the robust spread of that error, binned by distance and fitted to the beam
  model's shape. Becomes the simulated sensors' noise.
- **Blocked readings**: readings well off the wall (and how many were short of it: something in
  front). Corrected for the ones too near the wall to tell from noise.
- **Missing readings**: no reading with a wall comfortably in range, not mid-spin.
- **Sensor delay**: matched by simulating, not measured directly. A reading taken L seconds before
  it's used reads v·L long at closing speed v, and the localizer takes off `sensorLatencyMs` of
  that. But a stale reading also drags the estimate back with it, so the error left over shows only
  part of the delay. So the robot's own paths are simulated with its own localizer settings at
  delays from 0 to 100 ms, the leftover is measured the same way on each, and the delay that
  reproduces the robot's is the world's. About ±8 ms from a few minutes of logs, better with more.
- **Sensor bias**: each sensor's typical error, fitted together with the latency (otherwise a
  sensor mostly driven away from reads short from latency alone). A mount measured an inch wrong
  shows as about half an inch here (the estimate gives some ground to it), on that sensor alone. The
  table flags anything past 0.3 in and says roughly how far off the mount is. To fix it, run the
  Odom page's **Calibrate Sensors** on the robot, which measures the mounts directly
  ([`docs/LOCALIZATION.md`](LOCALIZATION.md#calibrating-the-mounts)).
- **Tracking wheels**: how far raw odometry drifts from the estimate per inch driven, along each
  wheel, in the robot's frame. "+2.1%" means the wheel reads 2.1% long: its real diameter is
  1/1.021 of the configured one, and the table gives the factor to multiply
  `kTrackingWheelDiameterIn` by.
- **CPU**: the localizer's update time and the logger's share, from the same logs.

**Use this world** gives the simulated robot those errors (the Robot tab shows them), sets the MCL
tab to the settings the robot ran, and makes **Your robot** available to search.

## Refining PIDs from real motions

[`tools/analyzer`](../tools/analyzer/), **Refine** tab, over every loaded log.

Auto-Tune designs gains from a few seconds of open-loop ramps and steps. Every motion the robot logs
after that is a step response with known gains: the `turn` and `drive` PID channels record each
step's error over time, with the gains in effect.

1. **Collect** each controller's steps (3 or more, the most recent 16), leaving out continuous loops
   (heading hold), steps under three exit thresholds, and steps that never got halfway (blocked,
   not dynamics).
2. **Identify**: replay each step through the axis model with the gains it ran, and find the model
   (inertia and damping scaled, delay changed; friction kept) whose replays best match what the
   robot really did. The starting model is the newest Auto-Tune measurement in the logs, else
   `TUNE.CFG`'s, else a fit to the logs' driving.
3. **Redesign** from that model exactly as the robot's PID page designs (pole placement, slowed to
   keep 50° of phase margin with the delay), and replay the same steps with the new gains to
   predict the effect.
4. **Decide**, and say why:
   - **Retune** when the refined model explains the steps clearly better than the old one, and
     the new gains do clearly better on the same steps without making them much slower. Each gain
     moves at most 2× per round; the next round can go further.
   - **No change** when the gains are doing what they were designed to.
   - **No change, model only** when the model was off but new gains wouldn't help. Big turns at
     12 V are limited by the motors, not the gains. The refined model can still be saved:
     velocity driving uses it.
   - **Unexplained** when no rescaling of the model explains the steps (mismatch over 8% of the
     step): something else is going on, like a blocked turn, contact with another robot, or a
     different load. Look at those steps in **PID responses**.

The panel shows the reasons, the recorded steps against what the old model promised and what the
refined model predicts, and one step at a time with the replays overlaid. Tick the proposals you
accept, then **Save TUNE.CFG** (open the card's current one first, to keep its other lines).

**Run by run** is the history: every loaded run in order, with each controller's gains, its steps'
median overshoot and settle time, timeouts, the localizer's drift correction and CPU, and what
changed the gains since the run before:

- `TUNE.CFG r8`: a new revision loaded.
- Auto-Tune on the robot.
- The PID page, by hand.
- A restart that lost the gains Auto-Tune set last run.
- A rebuild.

After a change, the next run's numbers are the check.

## Checking it on the robot

None of this has run on the real robot yet; the C++ builds with the PROS toolchain, and the pure
parts (the tune file reader, the recording policy, the settings table, the encoder) are unit-tested
on a desktop compiler, and the tools are tested against simulated robots whose truth is known.
Before relying on it:

1. **Recording.** Tap Start log, drive, tap Stop log: one new file, ending in `rec,stop,manual`.
   Plug into a competition switch: a new file starts by itself; unplug, and it closes about 5 s
   later.
2. **CPU.** In a match log, `H` rows' `samp_us + fmt_us` per second, and `mcl`'s `us` against its
   50 ms period. Both are upper bounds; if either looks high, that's the first thing to report.
3. **TUNE.CFG.** Put the file on the card: the Home page shows its revision, and the PID page shows
   its gains. Then break a line on purpose (`pid.turn=1,2`): the Home page shows REJECTED with the
   line, and the robot runs the code's gains.
4. **Calibration.** Drive a few minutes of practice after an autonomous `setPose()`, then calibrate
   from it. Flagged sensor biases are worth a tape measure before anything else.

## The code

| Piece | Where | Tested by |
|---|---|---|
| `LocalizerConfig` by name: every field's path and range | `localization/localizer_config.hpp` | `tests/localization/localizer_config_test.cpp` |
| `TUNE.CFG` parsing and checking | `tuning/tune_profile.hpp` | `tests/tuning/tune_profile_test.cpp`, with `tune_profile_golden.cfg` |
| Loading and applying it on 96671H's robot | `src/robot/tune.cpp` | on the robot |
| When the logger records | `telemetry/recording_policy.hpp` | `tests/telemetry/recording_policy_test.cpp` |
| Recording on and off, files per recording, CPU in `H` rows, `#meta` lines | `telemetry/logger.hpp` | on the robot; the gate in `tests/telemetry/channel_test.cpp`, the rows in `csv_format_test.cpp` |
| The localizer's per-update callback and timing | `localization/monte_carlo_localizer.hpp` | on the robot; its port in the simulator |
| Reading and writing `TUNE.CFG` in the tools | `tools/analyzer/js/tunefile.js` | `tools/analyzer/test/tunefile.test.js` (the same golden file, and the C++ table) |
| Calibrating from logs | `tools/analyzer/js/mclcal.js` | `tools/sim/test/tuner.test.js`, on simulated logs of a known world |
| The MCL search, open-loop replay, latency matching | `tools/sim/js/tuner.js` | `tools/sim/test/tuner.test.js` |
| PID refinement and history | `tools/analyzer/js/refine.js` | `tools/analyzer/test/refine.test.js`, on logs of a robot whose true model is known |
