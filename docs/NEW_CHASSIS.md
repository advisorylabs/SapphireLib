# New chassis: every calibration, in order

A new chassis starts with every number the robot runs on wrong: the IMU's scale, the tracking
wheels' size and offsets, the distance sensors' mounts, every PID gain, and the lift's gravity
volts. Each has a tool that measures it, on the robot or in the browser tools, but the order matters,
because each one trusts the ones before it. Calibrate Offsets measures its spin with the IMU, so a
1% IMU error is a 1% offset error. Calibrate Sensors trusts odometry's record of its spin. Auto-Tune
measures the chassis in inches from the tracking wheels. Refine and the MCL calibration need all of
that right before their logs mean anything.

```
 macros, lift heights ─▶ lift Auto-Tune        (needs nothing below)

 IMU scale ─▶ wheel signs, diameter ─▶ wheel offsets ─┬─▶ sensor mounts ───┬─▶ practice logs
                                                      └─▶ drive Auto-Tune ─┘   (Refine, MCL)
                                                                                     │
                                                                 checks ◀─ hardcode ◀┘
```

Every on-robot calibration applies what it finds to the running program and **saves nothing**: a
restart goes back to the code. So every step below ends with where to copy the numbers, then a
rebuild, then a check that the code now does what the calibration did.

Contents: [Before you start](#before-you-start) · [1. Macros and lift heights](#1-macros-and-lift-heights) ·
[2. Lift](#2-lift-auto-tune-then-hardcode) · [3. Odometry](#3-odometry) ·
[4. Sensor mounts](#4-mcl-sensor-mounts-calibrate-sensors) · [5. Drive PIDs](#5-drive-pids-auto-tune-then-hardcode) ·
[6. Practice logs](#6-practice-logs-refine-the-pids-calibrate-the-localizer) ·
[7. Hardcode and clear TUNE.CFG](#7-hardcode-what-has-settled-and-clear-tunecfg) · [8. Final checks](#8-final-checks) ·
[Reading an Auto-Tune result](#reading-an-auto-tune-result) · [Where every number lives](#where-every-number-lives) ·
[What has no calibration routine yet](#what-has-no-calibration-routine-yet)

## Before you start

- [ ] Build with the PROS toolchain ([`SETUP.md`](SETUP.md)). A mixed-toolchain image black-screens.
- [ ] **Take `TUNE.CFG` off the SD card** (rename it, keep it for reference). It was tuned for the old
  chassis, and the robot applies it at every startup over the code's values: `pid.*`, `model.*`,
  `lift.gravityVolts` and `mcl.*`. The Home page should read "Tune: code values (no TUNE.CFG)".
- [ ] Ports in [`config.hpp`](../include/robot/config.hpp), then uncomment each real one in the
  Diagnostics list in [`screen.cpp`](../src/robot/screen.cpp). The Diagnostics page is all green.
- [ ] **Turn stick driving back on.** It's switched off in `drive()` in
  [`driver.cpp`](../src/robot/driver.cpp) right now (the "DISABLED" comment): uncomment `turn` and the
  `holonomicFieldCentricHeadingHold()` call, and drop the two `[[maybe_unused]]`. Steps 3, 5 and 6
  spin and drive the robot with the sticks. The right stick steers a heading, held by the heading-hold
  PID that step 5 tunes, so expect turning to feel soft or loose until then.
- [ ] `kDriveWheelDiameterIn` and the drivetrain's gear ratio in
  [`devices.cpp`](../src/robot/devices.cpp). Push each stick: every wheel turns the way it should.
- [ ] The `sl` folder exists on the card; the Home page's SD line says "ready".
- [ ] The robot is in match configuration (everything mounted, match weight), with a charged battery
  for every Auto-Tune run.
- [ ] Tune on the surface you compete on (field tiles). Friction (`kS`) on a hard floor is different.

Two ways to get numbers off the robot, used all through this page:

- **The SD log.** Home page, **Start log** before a run, **Stop log** after (wait for "SD: ready, not
  logging" before pulling the card). Copy the `sl` folder to the computer and drop the files on the
  analyzer ([`tools/analyzer/index.html`](../tools/analyzer/)). Auto-Tune runs, calibration results and
  every PID step are in it.
- **The terminal.** `pros terminal` with the brain or controller plugged in over USB. Calibrations
  print what they found there, ready to copy.

Units: the lift is in **degrees of its rotation sensor**, counted up from where it was zeroed, past
360 (720 is two turns of the sensor, not of the lift). Driving is in inches, turning in degrees.

## 1. Macros and lift heights

Yours to finish first. Everything here is a constant at the top of
[`macros.cpp`](../src/robot/macros.cpp):

| Constant | Now | What it is |
|---|---|---|
| `ladder` | placeholders | each scoring mode's level heights, in lift sensor degrees. Entry 0 is level 0 (lift down, claw deployed); each entry after it is one L1 press, so the count sets how many levels the mode has. Names show on the controller, 8 characters at most |
| `kPieceDetectMm` | 50 | the claw's distance sensor reading at or under which a piece counts as in the claw |
| `kIntakeMv`, `kClawMv` | 12700 | intake and claw speed, in millivolts (12000 is full) |
| `kClawHoldMv` | 12700 | claw spin while it holds a detected piece. It's stalled most of that time: turn this down first if the claw motor runs hot |
| `kPistonDeployMs` | 400 | how long the claw piston gets to deploy before anything outtakes past it |
| `kHeightOuttakeMs` | 500 | how long the claw outtakes on a score above level 0 |
| `kReseatRetractMs`, `kReseatDeployMs` | 250, 250 | the re-seat after releasing Y: piston retracted this long, then redeployed this long |
| `kLiftToleranceDeg` | 15 | how close the lift must get before a sequence waiting on it moves on (the score's dip, the re-seat's lowering) |
| `kLiftDescendTimeoutMs` | 1000 | how long a sequence waits for the lift before moving on anyway |

- [ ] **Lift sensor sign.** Raising the lift makes the controller's `LIFT` reading grow. If it
  shrinks, negate `kLiftSensor` in `config.hpp`. Backward, Auto-Tune can't fit the lift ("no fit"),
  since a positive voltage has to make the reading grow.
- [ ] **Zero.** `macros::initialize()` zeroes the lift sensor wherever the lift is when the program
  starts. Always start with the lift all the way down, and never restart with it up.
- [ ] **Level heights.** The controller's third line shows `LIFT reading/target`. The macros have no
  manual lift control, so either iterate (guess a height, step to it with L1, look, adjust, rebuild),
  or add a temporary hold-to-raise button with `lift.setVolts()` while held and `lift.holdPosition()`
  on release ([`MACROS.md`](MACROS.md#positionmechanism) has the pattern) and read the number off
  line 3 at each goal.
- [ ] **The top of travel.** Read the sensor with the lift at its highest safe point. Step 2's window
  is set from it.
- [ ] **`kPieceDetectMm`.** Start log, sit a piece in the claw, take it out, a few times. In the
  analyzer's **Charts**, the `mech` channel's `piece_mm` shows both readings; pick a value between.

Measure the heights again after step 2: a lift that settled short with the old gains lands on its
target now, so what you read is where it really is.

## 2. Lift: Auto-Tune, then hardcode

On the bench; it needs no field and nothing from the drivetrain steps, so do it whenever the heights
are in.

### What Auto-Tune does to the lift

`macros::liftExperiment()` takes the lift's motors from its loop and drives four segments, each from
rest:

1. **Ramp up**: volts climb at 6 V/s (to 10 V at most) until the reading passes `kLiftTuneUpperDeg`.
2. **Ramp down**: the same downward, until it passes `kLiftTuneLowerDeg`.
3. **Step up**: straight to `kLiftTuneUpVolts`, until it passes `kLiftTuneUpperDeg`.
4. **Step down**: straight to `kLiftTuneDownVolts` downward, until it passes `kLiftTuneLowerDeg`.

A segment also ends after 2 s, or if the sensor stops answering. Between segments the motors brake
(the lift's brake mode is hold) and it waits up to 1.5 s for the lift to stop. It samples every 10 ms,
and only samples moving faster than 10°/s count.

Then it fits `volts = kS·sign(speed) + kG + kV·speed + kA·acceleration`, plus the delay before the
lift responds. Moving both ways is what tells friction (`kS`, which flips with direction) from gravity
(`kG`, which doesn't). The PID page designs the Lift gains from that model and `kLiftResponse`, sets
them, and installs `kG` as the lift's gravity volts. It all lasts until the program restarts.

### The constants

All in `macros.cpp`:

| Constant | Now | What it does | How to choose it |
|---|---|---|---|
| `kLiftTuneLowerDeg` | 60 | the window's bottom: a downward segment stops the moment the reading is at or below it | above the bottom stop by more than the lift coasts while braking on the way down |
| `kLiftTuneUpperDeg` | 720 | the window's top: an upward segment stops the moment the reading is at or above it | below the top of travel by more than the lift coasts while braking on the way up, at full speed |
| `kLiftTuneUpVolts` | 8 | the up step's volts | enough to get near top speed inside the window. Lower it if the lift slams |
| `kLiftTuneDownVolts` | 4 | the down step's volts. Gravity helps on the way down, so it's less | raise it if the down step crawls (under 10°/s doesn't count) |
| `kLiftTestHighDeg`, `kLiftTestLowDeg` | 450, 150 | where **Run Test** sends the lift: high, then low | two levels you really score at |
| `kLiftTestMoveTimeoutMs` | 2500 | how long Run Test waits for each move | |
| `kLiftTestDwellMs` | 500 | how long it pauses at each | |

**Why the window matters.** The segments stop where the reading *passes* a limit, and only then
brake, so the lift always coasts past both limits. Too close to a hard stop and it hits the stop with
volts still on, which reads as a huge deceleration and ruins the fit (and is hard on the lift). Too
narrow and each segment spends its time speeding up and slowing down, with little at speed: `kV` and
`kA` come out poorly, or "no fit". Cover the travel the presets use: a lift with rubber bands or a
spring has a load that changes with height, and the constant `kG` is an average over the window.

**Finding it:**

1. [ ] Start conservative: `kLiftTuneLowerDeg` about 10% of the top of travel, `kLiftTuneUpperDeg`
   about 70% of it.
2. [ ] Run Auto-Tune (below) with logging on.
3. [ ] Analyzer, **Charts**, the `char.lift` channel's `pos`: the highest reading minus
   `kLiftTuneUpperDeg` is how far it coasts going up; `kLiftTuneLowerDeg` minus the lowest is how far
   going down.
4. [ ] Widen the window to the top of travel minus the upward coast, and the downward coast above 0,
   each with about half the coast again to spare. Rebuild and run it again.

### Run it

1. [ ] Lift all the way down. The first segment goes up from wherever the lift is; started above
   `kLiftTuneUpperDeg`, it would end at once.
2. [ ] Home page: **Start log**.
3. [ ] PID page, select **Lift**, tap **Auto-Tune**. The readout says "Measuring Lift"; the run takes
   10 to 15 s. **Stop** (or disabling) ends it and applies nothing.
4. [ ] Read the result ([Reading an Auto-Tune result](#reading-an-auto-tune-result)). The kG it shows
   is the gravity volts it just installed.
5. [ ] **Run Test**, in driver control (the lift's loop runs in opcontrol). It sends the lift to
   `kLiftTestHighDeg`, waits until it's within `kLiftToleranceDeg` (or the timeout), pauses, then
   `kLiftTestLowDeg`, then back to its level. Then step through the levels with the macros.
6. [ ] Home page: **Stop log**. Copy the card.

### Judge it and hardcode it

In the analyzer ([`tools/analyzer`](../tools/analyzer/)):

1. [ ] **PID responses**, the `lift` PID: each Run Test move's overshoot and settle time. **Replay**'s
   lift view shows the target, the position and the volts: a lift that hangs below every target needs
   more gravity volts; one that rings needs a slower spec.
2. [ ] **Tune** tab, system **Lift**, source the Auto-Tune run, gravity **constant** (an elevator).
   "Does the model match?" plays the recorded volts through the model: the two lines should lie on
   each other.
3. [ ] **Controller**: the design from a settle time, damping and phase margin (the robot's are 0.5 s,
   1 and 50°). **What if** replays the log's real lift targets with the gains it ran, the designed
   ones, and any you type.
4. [ ] **Copy** under "Paste into the robot program". It looks like:

   ```cpp
   // Lift: designed by the telemetry analyzer from SL000014.CSV (Auto-Tune run 1)
   //   model kS 0.42 kV 0.0081 kA 0.00031 kG 1.18 (constant), delay 35 ms, R² 0.981
   constexpr sapphirelib::PIDGains kLiftGains{.kP = 0.153, .kI = 0, .kD = 0.0042};
   constexpr double kLiftGravityVolts = 1.18;
   ```

   (Made-up numbers.) Those two lines replace `kLiftGains` and `kLiftGravityVolts` in `macros.cpp`.
   Copy from the analyzer, not the screen: the PID page shows gains to three decimals, and kD often
   lives in the fourth. The analyzer refits the logged run with a port of the robot's own math, so its
   numbers are the robot's, in full.
5. [ ] **Seat volts.** If the lift heading for level 0 hangs just above its bottom stop instead of
   resting on it, raise `kLiftSeatVolts` (0 now). Heading for level 0 it drives down with at least
   that many volts, without gravity volts, until it's within `kLiftRestDeg` (2) of the bottom, then
   cuts power and rests on the stop.
6. [ ] Rebuild, flash, Run Test again: the same behavior, now from the code.

**Tweaking.** `kLiftResponse` in [`tuning.cpp`](../src/robot/tuning.cpp) is what the design aims for:
0.5 s to settle, damping 1. The lift's loop runs every 20 ms against Auto-Tune's 10 ms samples, and
the design counts the difference as 5 ms of extra delay. The knobs work as for the drive
([Tweaking the design](#tweaking-the-design)).

## 3. Odometry

### 3a. IMU heading scale

There's no button for this yet (see [the end](#what-has-no-calibration-routine-yet)); the math is
`sensors::calibrateHeadingScale()`. It comes first: the offset calibration, the Turn Auto-Tune, and
every heading after that are measured with the IMU, and the scale multiplies all of them.

1. [ ] With `/*imuHeadingScale=*/1.0` in `devices.cpp` (as it is now), line a straight edge of the robot
   up with a field wall or a tile seam, and start the program there. The Odom page's `H` reads 0.
2. [ ] Turn the robot clockwise in place exactly **10 turns**, at about match speed with the sticks,
   and stop square on the same line. Read `H`.
3. [ ] Its error is `H` as -180 to 180: `356.4` is -3.6°, `3.6` is +3.6°. The IMU measured
   `10 + error/360` turns, so the scale is `10 / (10 + error/360)`. (356.4: it measured 9.99 turns,
   read short, scale 1.001.)
4. [ ] Again counterclockwise: there the IMU measured `10 - error/360` turns. Average the two scales.
5. [ ] Put it in `devices.cpp`'s `imuHeadingScale`, rebuild, and repeat once: under a degree in 10 turns.

Ten turns so that a degree of error squaring the robot up again is only 0.03% of the turn. If a scale
is already in the code when you measure, multiply them: the new scale is the old one times the one
you just worked out.

### 3b. Tracking wheels: directions and diameter

There's no button for the diameter either; a tape measure does it.

1. [ ] **Directions**, on the Odom page from a fresh start (heading 0): push the robot straight forward
   and `Y` grows; push it right and `X` grows; turn it clockwise and `H` grows. A wheel that reads
   backward gets its port's sign flipped in `config.hpp` (the vertical one is reversed today; check
   it again on this chassis).
2. [ ] **Diameter.** Tape out a long straight line, 96 in or more. Start the program at one end and slide
   the robot along a wall to the other, heading held still, and read `Y`. The diameter is
   `kTrackingWheelDiameterIn × actual / Y`. Sideways with `X` for the horizontal wheel. Do each a
   few times and average.
3. [ ] Put it in `kTrackingWheelDiameterIn` in `config.hpp`. Both wheels share it; if they come out more
   than about half a percent apart, give each its own (they're separate constructor arguments in
   `devices.cpp`).

Holding the heading still matters: the offsets aren't calibrated yet, so a turn mid-push would put
their error into the reading. Step 6's log calibration checks the diameter again from real driving.

### 3c. Wheel offsets: Calibrate Offsets

**What it does.** It notes both wheels' distance, spins in place clockwise at 0.35 of full power
(about 4.2 V) until the IMU has turned 8 full turns, stops, and divides how far each wheel rolled by
the radians turned. That's each wheel's offset: how far it rolls per radian of turn, which odometry
subtracts to tell turning from driving. It applies them to the running odometry
(`Odometry::setConfig()`) and shows them.

1. [ ] Rebuild with 3a and 3b in. Clear floor, room to spin.
2. [ ] Odom page: **Calibrate Offsets**. **Watch it: it has no timeout.** A spin that snags and never
   finishes its turns never stops, and keeps driver control locked out. Restart the program to get
   out.
3. [ ] The status line shows "Vertical: 3.59in  Horizontal: 4.18in (applied)".
4. [ ] Check: spin in place a few turns with the sticks. `X` and `Y` should move a fraction of an inch
   at most. Before the calibration they'd wander by inches.
5. [ ] Copy them into `kVerticalWheelOffsetIn` and `kHorizontalWheelOffsetIn` in `config.hpp`. Rebuild
   and repeat the spin check with no calibration.

**The signs.** The vertical wheel's is backward from what you'd guess: **positive is left** of center
(`OdometryConfig`). The horizontal wheel's is the intuitive way: positive is ahead. The calibration
gets both right on its own; if its sign differs from what you'd have measured, believe the
calibration.

**The tracking center**, the point odometry tracks and the robot spins about, is where these put it:
the vertical offset to the **right** of the vertical wheel (left if negative), and the horizontal
offset **behind** the horizontal wheel (ahead if negative). With the numbers above: 3.59 in right of the
vertical wheel's line, 4.18 in behind the horizontal wheel's. Mark it on the robot: step 4 measures
every sensor from it, and step 6's `setPose()` places it.

Run it again whenever a wheel is remounted or the diameter or IMU scale changes.

## 4. MCL sensor mounts: Calibrate Sensors

Needs step 3 (the fit trusts odometry's record of the spin), the distance sensors' real ports in
`config.hpp`, and their Diagnostics entries uncommented. Independent of step 5.

### What it does

**The spin.** One turn clockwise, a half-second stop, one turn back. It holds about 45°/s on its own
(it starts at 0.1 power and adjusts, never past 0.5), and gives up with "the spin didn't finish:
blocked?" if a turn takes three times as long as it should plus 3 s. Every 30 ms it reads every
sensor, paired with where raw odometry had the robot `sensorLatencyMs` (30 ms) earlier, since a
reading describes the robot as it was when the sensor measured. Turning both ways cancels most of
the delay that's left.

**The fit** (`localization::fitSensorMounts()`) solves for everything at once against the walls in
view: where the robot was and which way it faced when the spin started, and each sensor's offset
**along its beam** and **sideways** from it. It uses only readings that hit a wall within 20° of
square on (a slanted beam's cone reads short), and drops readings that match no wall as it goes.

| | Fitted? | Why |
|---|---|---|
| Along the beam | always | it decides what the sensor reads from a wall it faces. The readings pin each sensor's against the others; your spacing between opposite sensors (or a full field's width) pins the rest |
| Sideways | only when pinned down | it shows only in slanted readings, and every sensor shifted sideways the same way looks like the robot facing a little differently. It stays near your measurement (within about an inch) unless the readings clearly say otherwise |
| Facing | never | a sensor's facing and the robot's heading can't be told apart, and a degree of facing barely changes a square-on reading |

**The one length that's yours.** From walls on one or two sides, a robot an inch nearer them with
every sensor an inch further out reads exactly the same. The readings pin everything else. Where
the tracking center sits among the sensors shows up: moved forward, it shortens the front sensor's
offset and lengthens the back one's, and the two read the same wall half a turn apart. What they
can't show is how far apart opposite sensors are. So the fit keeps that from your mounts: their
average along-beam offset stays where `config.hpp` puts it, which is the same as keeping
front-to-back plus right-to-left, and it finds their difference and the rest. A tape measure across
the robot gets that length to well under a quarter inch, which is the hard part (the tracking
center) left to the robot.

On a full field, from the halfway line, a sensor reads the left wall (`L`) and half a turn later
the right wall (`R`) from the same spot: `L + R = 140.5 − 2 × along`, so there the field's width
gives the spacing too. The fit uses it whenever a sensor sees both walls of a pair.

### Where to put the robot

```
   a corner (best)                    one wall

   ━━━━━━━━━━━━━━━━━━━━━━━━┓          ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
            ▲               ┃                         ▲
          1–3 ft            ┃                       1–3 ft
            ▼               ┃                         ▼
            ●  ◀─ 1–3 ft ─▶ ┃                         ●
                            ┃
                            ┃          ● the tracking center, facing any way
```

- **A corner beats one wall.** Two field perimeter pieces at a right angle on the tiles, or a corner
  of the room. Twice the readings, and things around the room have nothing to pass for. Different
  distances to the two walls (say 1½ ft from one and 2½ from the other) give the sideways offsets
  their best chance of showing.
- **1 to 3 ft from each wall** (the tracking center, marked in step 3c). Closer reads quieter, but
  leave room to spin: the robot's corners sweep a circle. Past 3 ft the readings get noisier and the
  rest of the room starts to come into the sensors' 78 in reach.
- **Walls**: straight, flat, and long enough that a sensor 20° off square still hits them. At 2 ft,
  4 ft of wall is plenty. They must be square to each other, as field perimeter corners are.
- **The view of the walls clear**: nothing between the robot and them. A flat object square to a wall
  can pass for the wall itself. The rest of the room doesn't matter much: the readings off it match
  no wall and drop out, and one flat thing across the room that the fit takes for a second wall is
  left out once it's more than 3° out of square with the first.
- **Which way it faces doesn't matter.** The fit solves for the robot's position and heading itself;
  it uses odometry only for how far the robot turned and drifted during the spin. No `setPose()`
  needed.
- **On a full field**, anywhere within about a foot of a center line works too (there every sensor
  sees an opposite pair, and the field's width also checks your spacing). A corner of the full field
  is just a corner.

### Before you run it

- [ ] **Mark the tracking center** (step 3c tells you where it is).
- [ ] **Measure each sensor**, from the tracking center to the sensor's **face** (where its reading
  starts): inches forward (negative behind) and right (negative left). Put them in `config.hpp`:

  | Sensor | Forward | Right | Facing |
  |---|---|---|---|
  | Front | `kFrontSensorForwardIn` | `kFrontSensorRightIn` | 0 |
  | Right | `kRightSensorForwardIn` | `kRightSensorRightIn` | 90 |
  | Back | `kBackSensorForwardIn` | `kBackSensorRightIn` | 180 |
  | Left | `kLeftSensorForwardIn` | `kLeftSensorRightIn` | 270 |

  How careful to be:
  - **The spacing, carefully.** Measure front face to back face and right face to left face across
    the robot. `kFrontSensorForwardIn − kBackSensorForwardIn` must be the first, and
    `kRightSensorRightIn − kLeftSensorRightIn` the second. Each of them an eighth of an inch long puts
    every sensor a sixteenth too far out.
  - **Where the tracking center sits between them, roughly.** Within 2 or 3 in is fine: the fit finds
    it. (The along-beam numbers on a full field can be off by up to 6.)
  - **Sideways** is usually kept, so measure it as well as you can. The facing (set in `devices.cpp`)
    must be the right side.
- [ ] **On a full field only: measure it**, inside face to inside face of the walls, both ways.
  `FieldMap::centered()` assumes 140.5 in, and from the halfway line the fit takes the field's width
  as known, so a wrong width goes straight into the mounts: roughly half the error into each sensor
  of that pair. Off by more than a quarter inch, pass the real size in `localizer()` in
  `devices.cpp`: `FieldMap::centered(141.0)`. From a wall or a corner the size doesn't matter.
- [ ] Rebuild.

### Run it

1. [ ] Robot's tracking center 1 to 3 ft from a wall, or from both walls of a corner
   ([Where to put the robot](#where-to-put-the-robot)), any heading. Nothing between it and the
   walls.
2. [ ] Home page: **Start log** (the results go in the log too), and `pros terminal` if you have a
   cable.
3. [ ] Odom page: **Calibrate Sensors**. Under 20 s.

### Read the result

The status line: "Sensors, fwd,right in (applied):" then each sensor's numbers, `Front 7.12,0.00`, or
`Front --` for one left as it was. The terminal has more: a summary, then one line per sensor.

```
sensor mounts: 1193 readings, 476 used, from walls on one or two sides: spacing kept from the mounts
sensor 0 (front, port 1): calibrated, {.forwardIn = 7.12, .rightIn = 0.00, .facingDeg = 0}, +0.12 in along the beam (+-0.04), sideways kept
```

"spacing kept from the mounts" says your spacing between opposite sensors stood in for a field's
width (it's missing when a full field's width was used). `+0.12 in along the beam` is how far the face moved from what
you measured (positive is farther out), and `+-0.04` its standard error. With the spacing kept, the
four changes average to 0: the fit moved the tracking center between the sensors, not their span.
The log has the same as `mcl,mount,...` events.

| It says | Means | Do |
|---|---|---|
| calibrated | good to use | copy it |
| no readings | the sensor never read anything usable | port, cable, or nothing in its 78 in range |
| too few readings of a wall | under 30 readings hit a wall square on | something blocks it, it only sees walls at a slant, or its mount is several inches off |
| no opposite sensor saw a wall | (whole run: "no two opposite sensors saw a wall") the sensors that saw a wall all face one or two neighboring ways, so there's no spacing to keep | check the sensors facing the other way: unplugged, blocked |
| no two opposite walls seen | on a full field, other sensors saw a pair of walls, but this one only walls they didn't | start nearer the halfway line, or use a corner |
| too uncertain | the along-beam offset's standard error is over 0.25 in | something moving in view, a wobbly mount, walls too far. Run it again, closer |
| moved too far, view blocked? | over 6 in from your measurement | something in front of a wall fooled it, or your measurement is badly off |
| (whole run) no sensor saw a wall | nothing matched a wall 30 times | start within 3 ft of a wall |
| the spin didn't finish: blocked? | (whole run) a turn timed out | clear the robot, check the drive |

### Hardcode it

1. [ ] Copy each calibrated sensor's `.forwardIn` and `.rightIn` into its two constants in the table
   above. Sensor 0 is the front, 1 the right, 2 the back, 3 the left: the order `localizer()` builds
   them in. Facing stays.
2. [ ] Rebuild, and run it once more: the changes should be about 0.1 in, and "sideways kept" is
   normal.

The check with the robot standing still at a known pose is in step 6, once there's an autonomous
routine that sets one.

## 5. Drive PIDs: Auto-Tune, then hardcode

Needs step 3: translation is measured in inches from the tracking wheels, the turn in degrees from
the IMU.

### What Auto-Tune does to the drivetrain

Selecting Drive, Turn or Hold and tapping Auto-Tune measures three axes in order: **Fwd**, **Strafe**,
**Turn**. Each gets four segments, each from rest after 100 ms at 0 V:

1. **Ramp** one way: volts climb at 4 V/s, to 8 V at most.
2. **Ramp** back the other way.
3. **Step** one way: straight to 6 V (`kTranslationStepVolts`, `kTurnStepVolts`).
4. **Step** back.

A translation segment ends once it's 30 in (`kTranslationTravelIn`) from where it started, or after
2.5 s; a turn segment after 2.5 s. Between segments it waits up to 1.5 s for the robot to stop.
Translation is read from raw odometry (the tracking wheels and IMU, before any localizer correction),
projected onto the way the robot faced when that axis began. Turning is the IMU's unwrapped heading.
Only samples faster than 2 in/s (5°/s turning) count.

Each axis is fitted to `volts = kS·sign(speed) + kV·speed + kA·acceleration` plus its delay, and its
model is installed for velocity sticks (the PID page's "Sticks" toggle) as soon as it fits. If an axis
fails, the run stops there and no gains change. Once all three fit, every controller is designed from
its axis: Drive from Fwd, Turn and Hold from Turn, each with its own spec.

**The design** places the closed loop's poles, so the gains follow from the model and the spec:

- `kP = kA·ω²` and `kD = 2·ζ·ω·kA − kV` (never below 0), and kI is always 0.
- `ζ` is the spec's damping ratio; `ω` comes from its settle time: with damping 1, `ω ≈ 5.83 / settleTimeS`.
- If the measured delay leaves less than the spec's phase margin, `ω` is lowered until it doesn't,
  and the readout says `lag-capped`.

So halving the settle time quadruples kP, and more than doubles kD. A kD of exactly 0 means the spec
is softer than the motors' own back-EMF already damps, and the real response will be slower than the
readout's `settle`.

### The constants

All in [`tuning.cpp`](../src/robot/tuning.cpp):

| Constant | Now | What it does |
|---|---|---|
| `kTranslationTravelIn` | 30 | how far each Fwd and Strafe segment may go from where it started, in inches |
| `kTranslationStepVolts` | 6 | the Fwd and Strafe steps' volts |
| `kTurnStepVolts` | 6 | the Turn steps' volts |
| `kDriveResponse` | 0.6 s, damping 1 | the Drive controller's spec (`driveDistance()`, and the drive term of the point motions) |
| `kTurnResponse` | 0.5 s, damping 1 | the Turn controller's spec (`turnToHeading()`) |
| `kHeadingHoldResponse` | 0.9 s, damping 1 | the heading hold's spec: softer on purpose, since the driver steers its target |

Each spec also has `minPhaseMarginDeg` (50 unless set). More travel or volts gives the fit more
samples at speed; fewer volts if the robot lurches or tips.

### Floor space

Fwd goes forward first: up to 30 in plus coasting, then back about as far, then the same again. Strafe
does the same to the right, then back. It ends about where it started. So start with about 4 ft clear
**ahead** and to the **right**, a foot or two behind and to the left, and nothing within reach of a
spin. A wall hit mid-segment, with volts still on, ruins that axis's fit. The middle of a field,
facing along it, works.

### Run it

1. [ ] Lift down, charged battery, robot where it has room (above).
2. [ ] Home page: **Start log** (`char.fwd`, `char.strafe`, `char.turn`).
3. [ ] PID page, select **Drive**, tap **Auto-Tune**. "Measuring Fwd", then Strafe, then Turn; under a
   minute. **Stop** (or disabling) ends it and applies nothing.
4. [ ] Read the result for Drive, then select Turn and Hold to see theirs
   ([Reading an Auto-Tune result](#reading-an-auto-tune-result)). A sanity check: `(12 − kS) / kV` is
   the top speed the model predicts, in in/s for Fwd and °/s for Turn. Far from what the robot really
   does, suspect the wheel diameter or the IMU scale.
5. [ ] **Run Test** on Drive: `driveDistance(24)`, then `driveDistance(-24)`. On Turn:
   `turnToHeading(90)`, then back to 0. Hold has no test: it's what the right stick drives through.
   Let go of the stick and push the robot's nose aside: it should come back without ringing.
6. [ ] Home page: **Stop log**. Copy the card.

### Judge it and hardcode it

In the analyzer:

1. [ ] **PID responses**: each Run Test step's overshoot and settle time, with its P and D terms.
   **Motions**: each motion's result (settled, or timed out), final error and time. `driveDistance()`
   counts as arrived within 1 in, `turnToHeading()` within 2°, for 200 ms.
2. [ ] **Tune** tab, system **Fwd**, source the Auto-Tune run. "Does the model match?" for each axis
   (switch the system to Strafe and Turn). Then **Controller** and **What if**, as for the lift.
3. [ ] **Copy** under "Paste into the robot program". One paste covers the drivetrain:

   ```cpp
   // Drivetrain: designed by the telemetry analyzer from SL000015.CSV
   // Drive → drivePIDConfig: settles in 0.60s, phase margin 63°, friction band ±0.21
   PID::Config{.gains = {.kP = 2.84, .kI = 0, .kD = 0.153}, .outputLimit = 12.0},
   // Turn → turnPIDConfig: settles in 0.50s, phase margin 58°, friction band ±0.37
   PID::Config{.gains = {.kP = 0.0981, .kI = 0, .kD = 0.00612}, .outputLimit = 12.0},
   // Hold → headingHoldPID() (setGains): settles in 0.90s, phase margin 71°, friction band ±1.1
   drivetrain().headingHoldPID().setGains({.kP = 0.0303, .kI = 0, .kD = 0.00108});
   // Axis models, for DriverInputMode::velocity (initDevices()):
   drivetrain().setAxisModels({
       .forward = {.kS = 0.61, .kV = 0.262, .kA = 0.0291}, // kS 0.61 kV 0.262 kA 0.0291, delay 30 ms, R² 0.990
       .strafe = {.kS = 0.88, .kV = 0.371, .kA = 0.0412}, // ...
       .turn = {.kS = 0.6, .kV = 0.045, .kA = 0.00992}, // ...
   });
   ```

   (Made-up numbers.) Where each goes in [`devices.cpp`](../src/robot/devices.cpp):
   - The **Drive** `PID::Config` replaces the one after `/*drivePIDConfig=*/` in `drivetrain()`, and the
     **Turn** one the one after `/*turnPIDConfig=*/`. Update the comment above them while you're there.
   - The **Hold** and **axis model** lines go in `initDevices()`, after `chassis.setOdometry(...)` and
     **before** `applyTuneProfile()`, so a `TUNE.CFG` can still override them. `chassis` is
     `drivetrain()` there, either works. Without the Hold line, heading hold runs the *turn* gains:
     it's built from `turnPIDConfig`.
4. [ ] Rebuild, flash, Run Test both: the same behavior as after Auto-Tune, now from the code.

### Tweaking the design

Auto-Tune doesn't search for gains. It fits a model and places the poles to meet a `ResponseSpec`.
So tweaking means changing the spec, not the gains:

| You see | The readout says | Change |
|---|---|---|
| Slower than it needs to be | long `settle`, not `lag-capped` | lower `settleTimeS` in `kDriveResponse` / `kTurnResponse` |
| Asking for faster changes nothing | `lag-capped` | the measured delay limits it. Lowering `minPhaseMarginDeg` (50) buys speed for ringing: 45 to 60 is usual |
| You'd trade a little overshoot for speed | | `dampingRatio` under 1 (0.8, say) |
| Motions stop a hair short, or time out just outside the threshold | `fric ±` wider than the exit threshold (1 in, 2°) | friction can hold the error at kS/kP, and there's no kI: a faster spec (more kP), or a wider `errorThreshold` on that motion |
| Heading hold feels twitchy, fights the driver | | raise `kHeadingHoldResponse.settleTimeS` |
| No fit, or a low `R2` | | sensor signs, then more travel or volts |
| Overshoots although damping is 1 | | the model is off: delay, or a different load. Step 6's Refine finds the model the real motions imply |

Try a spec in the analyzer's **Tune** tab first: **What if** replays the log's real targets through
the model with the old gains and the new ones, before anything is flashed. Then put the spec you
chose into `tuning.cpp`, so the next Auto-Tune on the robot designs the same way.

### The Asterisk's correction gains

No tuner; by hand once the PIDs are in, in `devices.cpp`:

- `headingCorrectionKP` (0.4) keeps `driveDistance()` straight with the IMU. Run Test on Drive: the
  robot should end the 24 in on the heading it started on.
- `driftCorrectionKP` (0.5, volts per in/s of drift) has the center wheels cancel forward/back drift
  while strafing. Strafe a few tiles: raise it until the drift is gone, and stop before the center
  wheels fight an intentional strafe.
- `turnContribution` (1.0) is how much the center wheels help turn. Walk it down if the 5.5 W center
  motors saturate or drag on fast turns.

## 6. Practice logs: refine the PIDs, calibrate the localizer

Needs steps 4 and 5 in the code. This is the loop in [`TUNING.md`](TUNING.md).

### An autonomous routine that sets the pose

None of the routines in [`autons.cpp`](../src/robot/autons.cpp) calls `setPose()` today. Without one
the localizer never corrects (odometry isn't in the field's frame), and the log calibration has
nothing to use. Pick a start you can put the robot on exactly, and work out where its tracking center
is in field coordinates (origin in the middle, +x right, +y downfield, walls at ±70.25 in):

```cpp
// Back against the near wall, tracking center on the x = -24 tile seam (measure
// where the seam really is on your field), facing downfield.
// kBackToCenterIn: the robot's back face to the tracking center, measured.
void fieldCheck() {
    odometry().setPose({.xIn = -24.0, .yIn = -70.25 + kBackToCenterIn, .headingDeg = 0});
    drivetrain().moveToPoint(-24, -24);
    drivetrain().turnToHeading(90);
    drivetrain().moveToPoint(24, -24);
}
// in registerAutons(): selector.addRoutine("Field Check", &fieldCheck);
```

Off the field, B+DOWN runs the selected routine from driver control.

### The loop

1. [ ] **Static check of the mounts.** Start log, run the routine, let it finish, leave the robot still
   a few seconds. In the analyzer's **Replay**, the Localizer panel: every sensor that sees a wall
   agrees (a green tick on its beam), and the drift (the correction) is well under an inch. A sensor
   that never agrees has a wrong mount or faces the wrong way.
2. [ ] **Record a session.** Log on, run the routine, then drive a few minutes with walls in the
   sensors' range: speed up and slow down, turn, go near the walls. Run motions too (autonomous
   routines, Run Test): every one is a step response for Refine. Stop log. Copy the `sl` folder and
   `TUNE.CFG` (if there is one) off the card.
3. [ ] **Analyzer, Refine.** It replays each logged step through the axis model with the gains it ran,
   finds the model that explains what the robot really did, and designs again from it. Each controller
   gets one of:
   - **Retune**: the refined model explains the steps clearly better, and its gains do clearly better
     on the same steps. Each gain moves at most 2× a round.
   - **No change**: the gains are doing what they were designed to.
   - **No change, model only**: the model was off, but new gains wouldn't help (a big turn at 12 V is
     limited by the motors). Save the model anyway: velocity sticks use it.
   - **Unexplained**: something the model can't describe (a blocked turn, another robot). Look at
     those steps in **PID responses**.

   Tick the ones you accept, **Save TUNE.CFG** (open the card's current one first, to keep its lines).
4. [ ] **Simulator** ([`tools/sim`](../tools/sim/)), **Tune** tab, **Calibrate from the robot's
   logs**: open the `SL*.CSV` files and `TUNE.CFG`. It needs the `mcl` and `mcl.beams` channels, driving
   after the routine's `setPose()`, and walls in range. Read the table:

   - **Sensor noise**, **blocked** and **missing readings**, **sensor delay**: what the simulated robot
     will get.
   - **Sensor bias** flagged past 0.3 in: that mount is off by about twice it. Re-run Calibrate Sensors
     (step 4), or a tape measure.
   - **Tracking wheels**: the factor to multiply `kTrackingWheelDiameterIn` by ("+2.1%" reads 2.1%
     long). Past about 1%, fix the diameter and re-run Calibrate Offsets (3c). The PIDs don't need
     redoing for a percent.
   - **CPU**: the localizer's update time and the logger's share.
5. [ ] **Use this world**, then search on **Your robot**. Keep the result only when the holdout says
   "better" (not "within noise" or "no better"). **Save TUNE.CFG**: it keeps the analyzer's PID lines.
6. [ ] Put `TUNE.CFG` in the card's `sl` folder. The Home page shows "Tune: TUNE.CFG rN" and how many
   settings. If it says REJECTED, the terminal and the log's `#meta,tune.error` say which line and
   why; the robot runs the code's values meanwhile.
7. [ ] The next session's logs are the check: Refine's **Run by run** shows each run's gains, its
   steps' overshoot and settle time, the localizer's drift, and what changed since the run before.

Repeat 2 to 7 over a few practice sessions, until Refine says No change and the search finds nothing
better.

## 7. Hardcode what has settled, and clear TUNE.CFG

Move `TUNE.CFG`'s lines into the code and delete them from the file, so the code and the robot agree
and a lost card changes nothing:

| `TUNE.CFG` line | Goes to |
|---|---|
| `pid.drive`, `pid.turn` (`kP,kI,kD`) | `drivePIDConfig`, `turnPIDConfig` in `devices.cpp` |
| `pid.hold` | `drivetrain().headingHoldPID().setGains(...)` in `initDevices()` |
| `model.fwd`, `model.strafe`, `model.turn` (`kS,kV,kA`) | `drivetrain().setAxisModels({...})` in `initDevices()` |
| `pid.lift`, `lift.gravityVolts` | `kLiftGains`, `kLiftGravityVolts` in `macros.cpp` |
| `mcl.*` | `localizerSettings()` in [`tune.cpp`](../src/robot/tune.cpp): the simulator's MCL tab prints the C++ |
| `mcl.periodMs` | `kDefaultLocalizerPeriodMs` in `tune.cpp` |

Then rebuild and flash. With the whole file gone, the Home page reads "Tune: code values (no
TUNE.CFG)". The next log's **Run by run** should show the same gains as the run before.

## 8. Final checks

- [ ] [`LOCALIZATION.md`](LOCALIZATION.md#checking-it-on-the-robot): a long routine with
  `setCorrectionEnabled(false)` and then with it on (it should end within an inch or so), a shove
  mid-routine (`mcl`'s correction jumps by about the shove, and the routine still lands), and another
  robot parked in front of a sensor (one fewer sensor agrees, and the estimate holds).
- [ ] [`TUNING.md`](TUNING.md#checking-it-on-the-robot): recording on and off, the CPU numbers in a
  match log, and a deliberately broken `TUNE.CFG` line (the Home page shows REJECTED and the robot runs
  the code).

## Reading an Auto-Tune result

The PID page shows the selected controller's axis model and design. "(not saved)" is a reminder that
it lasts until the program restarts.

| Line | Means | Look for |
|---|---|---|
| `kS` | volts to break friction | a volt or so. Several volts: something binds or drags |
| `kV` | volts per unit/s of steady speed | `(12 − kS) / kV` (minus `kG` for the lift) is about the top speed you see |
| `kA` | volts per unit/s² | positive |
| `kG` (lift) | volts to hold the lift still | becomes the gravity volts |
| `R2` | the share of the volts the model explains | over 0.95 is clean; under 0.8 is "no fit" |
| `lag` | how long before the axis responds, ms | tens of ms |
| `settle` | the settle time the design achieves | what the spec asked, unless `lag-capped` |
| `PM` | the design's phase margin with the lag | at least the spec's minimum (50°) |
| `lag-capped` | the design was slowed for the lag | asking for faster won't help |
| `fric ±` | `kS / kP`: how far from the target friction alone can hold it | smaller than the motion's exit threshold (or the lift's tolerance) |

"No fit" shows `R2` and how many moving samples it had, and names the usual causes: the sensor's sign
(a positive voltage must make the reading grow), too little travel (or for the lift, too narrow a
window), or too few volts to move it well past friction.

## Where every number lives

| Number | Measured by | Lives in |
|---|---|---|
| IMU heading scale | 10 turns by hand (3a) | `imuHeadingScale`, `devices.cpp` |
| Tracking wheel diameter | tape measure (3b); checked by the simulator's log calibration | `kTrackingWheelDiameterIn`, `config.hpp` |
| Tracking wheel offsets | Odom page, Calibrate Offsets | `kVerticalWheelOffsetIn`, `kHorizontalWheelOffsetIn`, `config.hpp` |
| Distance sensor mounts | Odom page, Calibrate Sensors | `k*Sensor*In`, `config.hpp` |
| Field size | tape measure | `FieldMap::centered()` in `localizer()`, `devices.cpp` |
| Drive, turn, hold gains; axis models | PID page Auto-Tune, analyzer Tune; then Refine | `devices.cpp` |
| Drive Auto-Tune specs and run | by hand | `tuning.cpp` |
| Lift gains, gravity volts | PID page Auto-Tune (Lift), analyzer Tune | `macros.cpp` |
| Lift Auto-Tune window, seat volts, levels, macro timing | by hand | `macros.cpp` |
| Asterisk correction gains | by hand | `devices.cpp` |
| Localizer settings | simulator Tune tab, from the logs | `localizerSettings()`, `tune.cpp` |

The geometry (IMU scale, diameter, offsets, mounts, field) is never in `TUNE.CFG`: it's always the
code's.

## What has no calibration routine yet

- **IMU heading scale.** Only the math, `sensors::calibrateHeadingScale()`; nothing on the robot runs the
  turns or reads the result.
- **Tracking wheel diameter.** Only indirectly, from the simulator's log calibration, which needs the
  mounts and the localizer already working. Nothing measures it on the robot.
- **Calibrate Offsets has no timeout** ([`ROADMAP.md`](ROADMAP.md)): a blocked spin never ends, and a
  non-finite IMU reading ends it at once with NaN offsets in the live odometry.
- **The lift's Auto-Tune window** has to be found by a first run and a look at the log.
- **The Asterisk's correction gains, the lift's seat volts, and the macros' numbers**: by hand.
