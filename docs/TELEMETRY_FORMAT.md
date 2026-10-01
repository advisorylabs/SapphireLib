# SapphireLib telemetry format (SLT v1)

`sapphirelib::telemetry::Logger` records PID steps, pose, mechanism commands, motor health and
events to the V5's SD card while the robot runs, one file per recording. This document is the
contract the telemetry analyzer (`tools/analyzer/`) and any other reader code against. The encoder (`src/sapphirelib/telemetry/csv_format.cpp`) is
golden-tested byte for byte against it (`tests/telemetry/csv_format_test.cpp`), and
`tools/telemetry/slt_read.py` is the reference reader, standard library only, with a
`--selftest` that parses the same golden file.

Contents: [File](#file) · [Lines and fields](#lines-and-fields) · [Directives](#directives) ·
[Rows](#rows) · [Ordering and robustness](#ordering) · [Events](#events) ·
[Channels on 96671H's robot](#channels-on-96671hs-robot) · [Motor channels](#motor-channels) ·
[Meaning, for tuning](#meaning-for-tuning) ·
[Refitting an axis offline](#refitting-an-axis-offline-from-char-rows) ·
[Worked example](#worked-example)

## File

- ASCII text. Every line ends in LF (`\n`). No BOM, no CR.
- One file per recording, plus a new file after each recovered SD fault (card pulled and
  reinserted, say). When a recording runs is up to the program (`LoggerConfig`):
  - `recordAtStart` (the library's default): from `start()`, so one file per program run.
  - `startRecording()` / `stopRecording()`, e.g. the Home page's Start log button. A stop writes
    everything recorded before it and closes the file, so the card is safe to pull once the Home
    page says "SD: ready, not logging".
  - `recordUnderCompetition` (on by default): connecting competition control (a field, or a
    competition switch) starts a recording if none is running, and it ends once competition
    control has been disconnected for `competitionStopDelayMs` (5 s), so a tether that drops for a
    moment doesn't split a match.

  96671H's robot waits for the button or a match (`recordAtStart = false`). Between recordings
  nothing is recorded at all: rows are turned away before they reach a buffer, and nothing counts
  as dropped. Every file in one program run shares its clock (`t_us`).
- Name: `SLnnnnnn.CSV`: `SL`, six zero-padded digits, `.CSV`, in `/usd/sl/` (the `sl` folder at
  the card's root). Always a legal 8.3 name: the V5 has no confirmed long-file-name support.
- The brain has no clock, so files are numbered, not dated: each new file takes the next number
  after the highest already in the folder, starting at `SL000001.CSV`. Newer runs have higher
  numbers. Readers match names case-insensitively.
- If the `sl` folder is missing, files go to the card's root instead. The V5 can't create folders
  (PROS's `mkdir` is unimplemented), so make the folder on a computer once. Empty it now and then:
  a folder with thousands of files makes every open slower.
- The card must be FAT32.

## Lines and fields

**Line 1** is `#SLT,<major>`, so `#SLT,1` for this version.

- A reader must refuse any other major version.
- Minor evolution never bumps the major version. It covers new `#meta` keys, new `#` directives,
  new row types, new `H` keys, new event tags and new event messages. Readers **must ignore** any
  line whose first field they don't recognize.

**Fields**

- Fields are comma-separated, with no quoting or escaping.
- Only an `E` row's message can contain commas. It is the last field and runs to the end of the
  line.
- Channel and column names match `[A-Za-z0-9_.-]{1,31}`. Event tags follow the same rule with at
  most 15 characters. Event messages are printable ASCII (anything else, CR and LF above all, is
  written as a space), at most 191 characters.

**Numbers**

- Integers are plain decimal.
- `S` values are fixed-point with the channel's decimals (`-?\d+(\.\d+)?`, trailing zeros trimmed,
  `-0` written as `0`), or `nan`, `inf`, `-inf`, or `d.dddddde±xx` when the magnitude is at least
  1e12. They were floats on the robot: about 7 significant digits.
- `G`/`C` values are `%.9g` (full double precision), so they may use exponent form.
- Every value parses with Python's `float()` or C's `strtod`.

**`t_us`**

- Unsigned integer microseconds since the program started.
- The same monotonic clock is used for every row of every file in one program run. It does not
  reset between competition phases, and it never wraps.
- It is stamped when the row was recorded on the robot, not when it was written.

## Directives

| Line | Meaning |
|---|---|
| `#meta,<key>,<value>` | The value is the rest of the line. v1 keys: `writer` (`sapphirelib <version>`), `kernel` (PROS version), `build` (when the robot program was linked), `robot` (`LoggerConfig::robotName`), `file` (this file's name), `dir` (`/usd/sl`, or `/usd` in the root fallback), `open_us` (`t_us` when the file opened), `clock` (`us_since_program_start`). |
| `#meta,<key>,<value>` (the program's own) | Lines the program adds with `Logger::addMeta()`, after the v1 keys, in every file. Keys are `[A-Za-z0-9_.-]{1,63}`; values are cut to 63 characters. 96671H's: `tune` (`loaded`, `rejected`, `none` or `no_card`: what its `TUNE.CFG` came to), `tune.rev`, `tune.path`, `tune.note`, `tune.error` (`line <n>: <why>` for a rejected file), every `LocalizerConfig` field as `mcl.<path>` (`mcl.filter.motionNoise.perInch`, ..., the settings the localizer ran with, file or code), `mcl.periodMs`, and `mcl.sensor<i>` = `<forward>,<right>,<facing>` for each distance sensor's mount. See [`docs/TUNING.md`](TUNING.md). |
| `#chan,<id>,<name>,<kind>,<decimals>[,<col>...]` | Declares a channel. `id` is 0–65535 and unique in the file. `kind` is `samples`, `pid` or `events`. Columns exclude `t_us`. For `pid` the columns are exactly `target,meas,err,p,i,d,u_raw,out,dt,flags`; for `events` there are none. A channel's `#chan` line always comes before any row that uses its id. Ids 0 (`sys`) and 1 (`events`) are always the event channels. **Match channels by name** across files: ids depend on registration order. |

## Rows

| Row | Fields | Meaning |
|---|---|---|
| `S` | `S,<id>,<t_us>,<v1>,...,<vN>` | One sample. N equals the channel's column count; a value the robot didn't supply is `nan`. For `pid` channels the last column `flags` is an integer bitfield: 1 = output clamped (saturated), 2 = slew-limited, 4 = integration rolled back by anti-windup, 8 = **first step after construction or reset (a new response starts here)**, 16 = the dt passed in was invalid and the nominal dt was used. |
| `G` | `G,<id>,<t_us>,<kP>,<kI>,<kD>` | The gains in effect for this PID's `S` rows that follow it in the file, until the next `G`. These are continuous-time gains: kI is per second, and kD is output per (error unit per second). They are written before the PID's first `S` in each file and again at the first step after the gains change (PidTunerPage's buttons and Auto-Tune change them mid-session). |
| `C` | `C,<id>,<t_us>,<integral_limit>,<output_limit>,<slew_rate>,<derivative_on_measurement>,<nominal_dt_s>` | The PID's configuration. A limit of 0 means disabled. `derivative_on_measurement` is 0 or 1. Written once per file, before the first `S`. |
| `R` | `R,<id>,<t_us>` | `PID::reset()` cleared real state: a motion ended or was abandoned, or a loop restarted. A loop that resets every tick while idle writes one `R`, not one per tick. |
| `E` | `E,<t_us>,<tag>,<message>` | An event. The message is the rest of the line and may be empty. See [Events](#events). |
| `D` | `D,<t_us>,<id>,<dropped_full>,<dropped_contended>` | Cumulative rows lost from that channel since the program started: lost to a full buffer (the writer fell behind), or to two tasks recording on the channel at the same instant. Written at the health interval when a count changed; each new file restates any nonzero counts at its first health interval. |
| `H` | `H,<t_us>,<key>=<int>,...` | Writer health, every second while logging. Ignore unknown keys. |

`H` keys in v1:

- `rows`: rows put into this file so far (the header and other `#` lines excluded).
- `bytes`: bytes of this file written to the card so far.
- `writes`: write calls to the card for this file. Each is also a filesystem sync.
- `wmax_us`, `wavg_us`: slowest and mean write duration since the previous `H` row (0 if none).
- `drops`: total rows dropped across all channels since the program started.
- `unlogged`: rows drained while no file was open (no card yet, or after a fault), since start.
- `resyncs`, `breaks`: buffer recoveries and producer-gate reclaims since start. Both should stay 0;
  a `break` means a task was deleted mid-record (the competition switch does that), costing one row.
- `faults`: SD write failures since start.
- `samp_us`: microseconds the sampler task spent since the previous `H` row (polling sources and
  watching the competition state).
- `fmt_us`: microseconds the writer task spent since the previous `H` row outside its SD writes
  (draining and formatting rows); the writes themselves are `wmax_us`/`wavg_us`.

  Both are wall time, so time another task spent preempting them counts too: an upper bound on
  the logger's own CPU. Per second, `(samp_us + fmt_us) / 10000` is a percentage of the brain.

<a name="ordering"></a>
## Ordering and robustness

**Ordering**

- Within one channel, rows appear in the order they were recorded, and `t_us` never decreases.
- Across channels, the writer merges rows by `t_us` within each batch it writes. Global order is
  best-effort, so sort by `t_us` if it matters.
- Rows the writer adds itself (`H`, `D`, and the `E` rows at file open) are stamped when written.

**Rules for readers**

- A final line with no LF was torn by power loss: discard it.
- Any line that fails to parse (wrong field count, unknown id) is skipped; never abort the file.
- A pid channel's `S` rows before its first `G` in a file have unknown gains. This only happens to
  rows in flight at the moment the file opened.
- Look for gaps with `D` rows and with `t_us` jumps.
- What survives a sudden power-off: everything up to the last write (at most 250 ms old), and
  everything before the latest competition phase change, which forces a write.

## Events

`E,<t_us>,<tag>,<message>`. The format doesn't record which channel an event came from; tags say
what it is.

### System tags

| Tag | Message | When |
|---|---|---|
| `file` | `open,<name>` | First row after the header of every file. |
| `phase` | `<disabled\|autonomous\|opcontrol>,comp=<0\|1>,field=<0\|1>` | At every file open, and on every competition state change. `comp` is "competition control connected", `field` is "field control (not a switch)". Disabled wins over autonomous. |
| `sd` | `dir_missing,<dir>` | The log folder is missing; this file is in the card's root. |
| `sd` | `open_failed,<dir>` | A file couldn't be created in the folder (usually also a missing folder: PROS reports every failed open the same way); this file is in the root. |
| `sd` | `reopened,prev=<name>,faults=<n>` | This file follows a write fault on `<name>`. At most 8 faults per run; after that the robot stops logging until it restarts. |
| `rec` | `start,<manual\|competition\|startup>` | At every file open except a reopen after a fault: a recording starts, and why (the button or `startRecording()`, competition control connecting, or `recordAtStart`). |
| `rec` | `stop,<reason>` | The last row of a recording's file, with the reason it started. Missing when the power went first. |

### Motion events

Every blocking drivetrain motion (both `HolonomicDrivetrain` and `TankDrivetrain`) logs a start
and an end:

```
E,<t_us>,motion,start,<kind>,<key>=<value>,...
E,<t_us>,motion,end,<kind>,reason=<settled|timeout|aborted>,error=<finalError>,ms=<elapsedMs>
```

`kind` is one of `driveDistance`, `turnToHeading`, `moveToPoint`, `moveToPose`, `followPath`.
Floats are written `%.3f`, milliseconds as integers.

| kind | start keys |
|---|---|
| `driveDistance` | `target_in,threshold,settle_ms,timeout_ms` |
| `turnToHeading` | `target_deg,threshold,settle_ms,timeout_ms` |
| `moveToPoint` | `x,y,hold_deg,threshold,settle_ms,timeout_ms` (a tank drivetrain steers to face the point, so it has no `hold_deg`) |
| `moveToPose` | `x,y,heading_deg,pos_threshold,heading_threshold,settle_ms,timeout_ms` |
| `followPath` | `waypoints,lookahead_in,cruise_v,timeout_ms` |

- Units: inches and degrees. `threshold` is the exit tolerance (inches for drive/point motions,
  degrees for turns); `settle_ms` is how long the error must stay inside it; a `timeout_ms` of 0
  means none.
- `error` is the motion's error on its last tick, as an absolute value: inches for
  `driveDistance`, degrees for `turnToHeading`, distance to the target in inches for the pose
  motions and `followPath` (whose result is its final approach's).
- `ms` is the motion's whole duration.
- A motion that can't start (no odometry set, an empty path) logs a `start` with no keys, then an
  `end` with `reason=aborted`.
- A motion cut short by a competition state change (PROS deletes the autonomous task wherever it
  is) has a `start` and no `end`; the `phase` event marks where it stopped.
- Look keys up by name and tolerate missing or extra ones. Pair each `end` with the most recent
  open `start` of the same kind (motions don't nest today, but a reader shouldn't assume that).
  `slt_read.py`'s `motions()` does all of this.

Which pid channels a motion drives (see the channel table below):

| kind | pid channels |
|---|---|
| `driveDistance` | `drive` (heading correction is a plain kP, not a PID) |
| `turnToHeading` | `turn` |
| `moveToPoint`, `moveToPose` | `drive` (distance to the target) and `turn` (heading hold, or steering on a tank) |
| `followPath` | `turn` while pursuing; then `drive` and `turn` for the final approach |

### Device events

The GUI's diagnostics page (`gui::DiagnosticsPage`) re-checks every device about once a second
while it's installed, whichever page is showing, and logs each change of verdict:

```
E,<t_us>,device,missing,port=<n>,label=<label>,found=<what it found>
E,<t_us>,device,lost,port=<n>,label=<label>,found=<what it found>
E,<t_us>,device,back,port=<n>,label=<label>
```

`missing` is a device that failed its very first check, `lost` one that was fine and then failed
(a cable worked loose mid-match), `back` one that answers again. `label` and `found` have their
commas replaced, so the message splits cleanly on `,` and `=`. A motor channel (below) going to all
NaN says the same thing for a motor, at 100 ms resolution; the device events cover every sensor,
and say which port.

### Tuning events

Auto-Tune (`PidTunerPage`, wired in `src/robot/tuning.cpp`) marks each axis it measures:

```
E,<t_us>,tune,start,<Fwd|Strafe|Turn|Lift>
E,<t_us>,tune,model,<axis>,kS=<V>,kV=<V/unit/s>,kA=<V/unit/s²>,r2=<r²>,delay_ms=<ms>
E,<t_us>,tune,model,Lift,kS=<V>,kV=<V/unit/s>,kA=<V/unit/s²>,kG=<V>,r2=<r²>,delay_ms=<ms>
```

`start` comes just before the axis's first `char.*` row; `model` is what the robot fitted from
them, once the run ends (a run that was stopped or failed its fit logs no `model`). Units are the
axis's: inches for Fwd/Strafe, degrees for Turn and the lift. A mechanism's model adds `kG`, the
volts that hold it against gravity.

### Other tags

- `auton`: `start,<routine name>` / `end,<routine name>` around each autonomous routine (a routine
  the competition switch cuts short has no `end`).
- Suggested for team code: `mark` (a driver's "that looked wrong" button), `macro`.

## Channels on 96671H's robot

Wired in `src/robot/telemetry.cpp`, `src/robot/macros.cpp` and `src/robot/tuning.cpp`. The
`#chan` lines in each file are the authority (decimals, and columns, may change); these are the
names to match on.

| Channel | Kind | Columns | Source |
|---|---|---|---|
| `sys` | events | - | Competition phases (`phase`). |
| `events` | events | - | Everything else: `motion`, `auton`, `tune`, ... |
| `drive` | pid | pid columns | The drivetrain's `drivePID()`: distance loops. |
| `turn` | pid | pid columns | The drivetrain's `turnPID()`: turns, heading hold/steering in point and path motions. |
| `hold` | pid | pid columns | The drivetrain's `headingHoldPID()`: driver-control heading hold. |
| `odom` | samples | `x,y,heading,raw_x,raw_y` | Odometry pose (inches, inches, degrees 0–360) every 10 ms while enabled. The corrected pose, the one every motion drives by: raw odometry plus whatever correction the localizer has eased in (see `mcl`). `raw_x,raw_y` are raw odometry's position from the same snapshot, before any correction: what fitting the chassis from a log should differentiate, since a correction easing in (up to `maxCorrectionRateInPerS`) moves `x,y` without the motors doing anything. Older logs have only `x,y,heading`. |
| `mcl` | samples | `x,y,spread,neff,used,agree,correcting,corr_x,corr_y,us,raw_x,raw_y` | The Monte Carlo localizer, one row per update (every 50 ms by default) while enabled, recorded by the localizer's own task ([`docs/LOCALIZATION.md`](LOCALIZATION.md)): its estimate (inches), the particles' RMS spread around it (inches), the effective particle count, how many of the four distance sensors gave a usable reading and how many of those agree with the walls, 1/0 for whether that update set odometry's correction, the correction odometry is easing toward (inches), how long the update took (`us`, microseconds of wall time: an upper bound on its CPU), and raw odometry's position (inches) before any correction. The correction is how far raw odometry had drifted; `x,y` minus `raw_x,raw_y` is the same thing, for every update rather than only correcting ones. |
| `mcl.beams` | samples | `m0,e0,v0,m1,e1,v1,m2,e2,v2,m3,e3,v3` | Each distance sensor in that update, in the order the localizer was given them (96671H: front, right, back, left): `m` the reading in inches after latency compensation (`nan` when unusable: nothing in range, low confidence, dropped, or skipped mid-spin), `e` what the map says it should read from the estimate (inches; `inf` when the beam would see nothing in range), and `v` how fast the sensor was closing on what it points at (in/s, by odometry). `m - e` is the reading's error against the map. The simulator's Tune tab calibrates the localizer's world from these (sensor noise, blocked and missing readings, the latency left uncompensated, mount errors and the tracking wheels' scale): [`docs/TUNING.md`](TUNING.md). Four sensors fill 12 of a row's 13 columns. |
| `mcl.state` | samples | `sxx,syy,sxy,fit,fit_ratio,recovered,resampled,blocked,hdg` | The rest of each update, for the analyzer's replay: the particles' weighted covariance (`sxx`, `syy` variances and `sxy` covariance, in²: the uncertainty ellipse), recovery's fit (`fit`, how well the particles explain the readings as a share of a perfect fit per reading, 0 to 1) and its fast average over its slow one (`fit_ratio`: recovery starts below `filter.recovery.triggerRatio`), how many particles recovery replaced, 1/0 for whether the update resampled, why it didn't correct odometry (`blocked`, bits below; 0 when it did), and the heading the beams were cast at (`hdg`, degrees: odometry's, wound back by `sensorLatencyMs` of turning). |
| `mcl.pts` | samples | `dx0,dy0,...,dx5,dy5` | A sample of the particle cloud: every 5th update, 24 particles picked by weight (`sampleParticles()`, systematic), as four rows of six (x, y) offsets from that update's estimate, in inches. The four rows of one sample are recorded within microseconds of each other; group rows less than 10 ms apart. Add the `mcl` row's `x,y` for field positions. |
| `chassis` | samples | `fwd_v,strafe_v,turn_v` | The volts the drivetrain last commanded on each axis, before each motor's ±12 V clamp, every 10 ms while enabled. A snapshot, possibly one tick torn across the three fields; `strafe_v` is always 0 on a tank. |
| `batt` | samples | `volts,pct,amps,temp` | Battery voltage, charge (%), current drawn (A) and pack temperature (°C), every 200 ms. A failed read is `nan`. |
| `motor.<name>` | samples | `volts,amps,temp,rpm,eff,faults` | One per motor, every 100 ms: see [Motor channels](#motor-channels). 96671H logs `motor.fl`, `fr`, `bl`, `br`, `ml`, `mr` (drivetrain), `motor.liftA`, `liftB`, `intake` and `claw`. |
| `driver` | samples | `lx,ly,rx,ry,buttons,connected` | The controller, every opcontrol tick (20 ms): sticks in −1…1, the held buttons as a bitmask (bit *i* = `input::Button` *i*: L1, L2, R1, R2, Up, Down, Left, Right, X, B, Y, A), and 1/0 for whether the controller is connected. |
| `lift` | pid | pid columns | The lift's position PID. |
| `lift.act` | samples | `target,pos,volts,law` | Every lift update: target and position in degrees (`pos` is `nan` if the rotation sensor didn't answer), the volts the motors were actually sent (after gravity feedforward and the ±12 V clamp; 0 while braking; `nan` while Auto-Tune has the motors), and which branch of the control law ran (`law`, below). |
| `mech` | samples | `intake_v,claw_v,piston,piece_mm,level,mode,phase,deployed` | The macro system, every opcontrol tick: volts sent to the intake and claw, the claw piston (1 extended), the claw's distance sensor (mm; `nan` if it didn't answer), the lift's preset level and scoring mode (0 alliance, 1 medium, 2 center), the running score/reseat sequence step (−1 none; else 0 scoreDescend, 1 scoreOuttake, 2 reseatLower, 3 reseatRetract, 4 reseatDeploy), and whether the claw is deployed. |
| `char.fwd`, `char.strafe`, `char.turn`, `char.lift` | samples | `volts,pos` | Auto-Tune characterization runs, one row per sample the runner takes. Declared in every file; rows appear only while Auto-Tune runs. See [Refitting an axis offline](#refitting-an-axis-offline-from-char-rows). |

`mcl.state`'s `blocked` is `LocalizationStatus::blockedBy`, a sum of these bits; a reader should
ignore bits it doesn't know:

| bit | Name | The update didn't correct because |
|---|---|---|
| 1 | correction off | `setCorrectionEnabled(false)` |
| 2 | no setPose | `waitForSetPose`, and no `Odometry::setPose()` yet |
| 4 | no readings | no sensor had a usable reading |
| 8 | too spread | the particles' spread was over `maxCorrectionSpreadIn` |
| 16 | too few agree | fewer than `minAgreeingSensors` readings agreed with the map |
| 32 | refused | a `setPose()` landed during the update |
| 64 | spinning | turning faster than `maxTurnRateDegPerS`, so every reading was skipped (comes with 4) |

`lift.act`'s `law` is `mechanism::PositionLaw` as an integer:

| law | Name | Meaning |
|---|---|---|
| 0 | track | PID on the target plus gravity feedforward |
| 1 | seat | heading for the floor, driving down at least the seat volts |
| 2 | rest | at the floor: 0 V, loop reset |
| 3 | no sensor | no position reading: motors braked, loop reset |
| 4 | manual | open-loop volts |
| 5 | off | braked: stopped, or disabled |
| 6 | external | something else has the motors: an Auto-Tune run (`beginExternalControl()`); `volts` is `nan` |

### Motor channels

`Logger::motor(name, port)` polls one motor, every 100 ms by default, into six columns:

| Column | Unit | From |
|---|---|---|
| `volts` | V | `motor_get_voltage()`: what the motor is applying |
| `amps` | A | `motor_get_current_draw()` |
| `temp` | °C | `motor_get_temperature()`; the V5 reports it in 5 °C steps |
| `rpm` | RPM | `motor_get_actual_velocity()`, at the output shaft (after the cartridge) |
| `eff` | % | `motor_get_efficiency()`: 100 is free-spinning, 0 is stalled while powered |
| `faults` | bits | `motor_get_faults()`: 1 over temperature, 2 H-bridge fault, 4 over current, 8 H-bridge over current |

- A single reading that failed is `nan`. A motor that answers nothing at all (unplugged, or a dead
  cable) logs a whole row of `nan`, never zeros: read a run of all-`nan` rows as "disconnected"
  (and a channel that's all `nan` from the start as "never plugged in", or a placeholder port).
- V5 motors protect themselves from heat by limiting their own current: to 50 % at 55 °C, 25 % at
  60 °C, 12.5 % at 65 °C, and 0 at 70 °C. Nothing else in the log says so: a mechanism that
  weakens late in a match shows it here, as `temp` climbing through those steps while `amps`
  flattens under a full-power command. The over-temperature fault bit comes on at the top end.

## Meaning, for tuning

- `err` is always the controller's true error. `target` and `meas` are exactly what the caller
  passed. SapphireLib's turn loops and its `moveToPoint`/`moveToPose`/`followPath` distance and
  heading loops pass the error as `target` and `0` as `meas`, so for those, read `err`, and get
  the physical position from `odom`.
- `p`, `i`, `d` are the three terms in output units (kP·err, kI·∫err, kD·derivative); `u_raw` is
  their sum after anti-windup, before slew limiting and clamping; `out` is what the PID returned.
  The terms can differ from `u_raw` in the last bit.
- `out` is in volts for the drivetrains and the lift. What actually reached the motors, after
  feedforward, mixing and clamping, is in `chassis` (per drivetrain axis) and `lift.act`.
- `dt` is the timestep the PID used. The real loop period is the `t_us` difference between
  successive `S` rows; a gap much larger than `dt` means the loop was held up. The `hold` PID
  passes a measured dt, so there the two agree.
- A step response starts at each `S` row with flag 8 and runs to the next one (or to an `R`).
  Motion events bracket each response with its target and exit conditions, and say how it ended.
- Gains in `G` rows are exact (doubles). `S` values are floats rounded to the channel's decimals:
  a value below 10^-decimals prints as 0.
- `batt` matters for feedforward: battery sag under load skews any kV fitted from a log.

## Refitting an axis offline from `char.*` rows

Auto-Tune (`PidTunerPage`) characterizes an axis on the robot with
`tuning::runCharacterization()`, fits a feedforward model to the samples, and designs gains from
it. `telemetry::tapCharacterization()` mirrors every sample into a `char.<axis>` channel, so the
app can refit offline from real runs and compare against what the robot fitted (which the `tune`
events record).

Each row is a superset of `tuning::CharacterizationSample{timeMs, volts, position}`: `volts` is
the voltage applied at that tick and `pos` the position measured just before it: inches along
the axis for `char.fwd`/`char.strafe`, cumulative (unwrapped) degrees for `char.turn`, and the
lift's own degrees for `char.lift`. To rebuild the runner's `CharacterizationData`:

1. Take the rows of one run. One characterization is four segments, in order: ramp forward, ramp
   back, step forward, step back (up, down, up, down for the lift). Separate runs are seconds
   apart; each starts just after its `tune,start,<axis>` event.
2. Split into segments at gaps: within a segment rows are one sample period (10 ms) apart; between
   segments the runner waits at least 100 ms for the axis to stop, and logs nothing meanwhile.
3. Within each segment, `timeMs` is `(t_us - first t_us) / 1000`. The first ~100 ms are the
   pre-roll at 0 V (so velocity has a full differentiation window when the voltage starts). A
   mechanism is held still (braked) instead, and its held rows have `nan` volts: the fit skips
   them as samples, but they still give the velocity window.
4. A segment cut short by its travel limit ends with one extra row: 0 V (`nan` for a mechanism)
   at the position that was out of range, which the runner measured but didn't store. A drive
   axis's is a genuine (volts, position) pair, so keeping it barely changes a fit; drop it to
   match the robot exactly (the last row of a segment, when it is 0 V or `nan` and the row before
   it isn't).
5. Ramps (volts rising slowly) pin down kS and kV; steps pin down kA and the response delay; for a
   mechanism, the difference between going up and coming down pins down kG. Differentiate
   position within a segment, never across the gap between two. Feed the result to the same math
   the robot uses (`tuning::fitFeedforward()`/`characterizeAxis()`, or `characterizeMechanism()`
   with the lift's gravity shape, which is constant for 96671H's lift;
   `include/sapphirelib/tuning/characterization_math.hpp`).

`slt_read.py`'s `characterization_segments()` does steps 2–3. The telemetry analyzer
(`tools/analyzer/`, its Tune tab) does all five, with a JavaScript port of that math that
reproduces the robot's fits, and designs gains from the result.

## Worked example

This is the golden file from `tests/telemetry/csv_format_test.cpp` (and `slt_read.py
--selftest`), an excerpt of a run with the `...` gaps closed up:

```
#SLT,1
#meta,writer,sapphirelib 0.1.0
#meta,kernel,4.2.2
#meta,build,Sep 27 2026 14:02:11
#meta,robot,96671H
#meta,file,SL000042.CSV
#meta,dir,/usd/sl
#meta,open_us,2104331
#meta,clock,us_since_program_start
#chan,0,sys,events,0
#chan,1,events,events,0
#chan,2,drive,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags
#chan,3,turn,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags
#chan,5,odom,samples,4,x,y,heading
#chan,6,batt,samples,2,volts,pct
#chan,7,lift,pid,3,target,meas,err,p,i,d,u_raw,out,dt,flags
#chan,8,lift.act,samples,3,target,pos,volts,law
E,2104502,file,open,SL000042.CSV
E,2104502,phase,disabled,comp=1,field=1
H,3104771,rows=4,bytes=1034,writes=1,wmax_us=21873,wavg_us=21873,drops=0,unlogged=2,resyncs=0,breaks=0,faults=0,samp_us=1840,fmt_us=2615
E,15003114,phase,autonomous,comp=1,field=1
S,5,15003201,0,0,0
S,6,15003201,12.61,87
E,15003390,auton,start,Turn Testing
E,15003400,motion,start,turnToHeading,target_deg=90.000,threshold=2.000,settle_ms=200,timeout_ms=3000
C,3,15003511,0,12,0,0,0.01
G,3,15003512,0.35,0,0.0002
S,3,15003514,90,0,90,31.5,0,0,31.5,12,0.01,13
S,5,15013203,0,0,0.12
S,3,15013604,89.59,0,89.59,31.3565,0,-0.0082,31.3483,12,0.01,5
S,3,15023690,88.63,0,88.63,31.0205,0,-0.0192,31.0013,12,0.01,5
S,3,15612874,1.84,0,1.84,0.644,0,-0.0012,0.6428,0.6428,0.01,0
E,15612900,motion,end,turnToHeading,reason=settled,error=0.412,ms=609
E,16871020,motion,start,turnToHeading,target_deg=0.000,threshold=2.000,settle_ms=200,timeout_ms=3000
R,3,16871021
S,3,16871027,-89.97,0,-89.97,-31.4895,0,0,-31.4895,-12,0.01,13
E,30001022,phase,opcontrol,comp=1,field=1
C,7,31540205,0,12.7,0,1,0.02
G,7,31540206,0.2,0,0.01
S,7,31540210,150,0.37,149.63,29.926,0,0,29.926,12.7,0.02,13
S,8,31540236,150,0.37,12,0
S,5,31540301,23.4512,-11.0833,91.87
S,7,31560198,150,3.12,146.88,29.376,0,-1.375,28.001,12.7,0.02,5
S,8,31560221,150,3.12,12,0
E,48220114,mark,driver
D,61000044,1,0,1
```

How to read it:

- **Turn rows.** Turn Testing's `turnToHeading(90)` runs the `turn` PID with kP 0.35 and kD
  0.0002, the error folded into `target` (so `meas` is 0). Its `C` and `G` rows come just before
  its first `S`.
  - First step: flags 13 = first step (8) + integration rolled back (4) + saturated (1). The
    output is clamped to the 12 V limit.
  - d = 0.0002 × (89.59 − 90) / 0.01 = −0.0082.
  - The motion's `end` says it settled 609 ms after it started, 0.412° off.
- **R row.** The second motion logs its `start`, then its `turnPID().reset()` produces the `R`
  row, and its first `S` carries flag 8 again. That motion has no `end`, and the routine no
  `auton,end`: the switch to driver control (the `phase` event) deleted the autonomous task
  mid-turn, before either could be logged.
- **Lift rows.** The lift heads for 150°.
  - Derivative on measurement (`C` row, 4th value 1): d = 0.01 × −(3.12 − 0.37) / 0.02 = −1.375.
  - The PID output is clamped to 12.7; with gravity feedforward the motors were sent the full
    12 V, which is what `lift.act` shows, with law 0 (track).
- **H row.** One second after the file opened: 4 rows and 1034 bytes so far in one write that took
  22 ms, and 2 rows drained before the file was open. The sampler spent 1.8 ms of that second and
  the writer 2.6 ms outside its write: under half a percent of the brain.
- **D row.** One row was lost from the `events` channel because two tasks logged an event at the
  same instant.
