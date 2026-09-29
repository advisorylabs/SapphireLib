# SapphireLib — Development Roadmap

**Team 96671H — Hitmen**
Standalone PROS library for VEX V5, custom API, rewritten from StratagemV2.0 (previously LemLib-based).

---

## Design Principles

- **No external framework dependency.** SapphireLib owns its own odometry, motion control, and utility math — no LemLib.
- **Flexible localization.** Odometry supports:
  - IMU + drive motor encoders only (no tracking wheels)
  - IMU + single vertical tracking wheel
  - IMU + single horizontal tracking wheel
  - IMU + vertical + horizontal tracking wheels
  - Configuration is declarative — you tell SapphireLib what sensors exist, it picks the odometry math automatically.
- **Custom API**, not a LemLib/EZ-Template clone.
- **Season-durable**: needs to survive 2 seasons, so testability, documentation, and tuning tools matter as much as raw features.

---

## Phase 0 — Foundation & Repo Setup
**Goal:** A clean, buildable PROS project skeleton the whole team can pull and build on day one.

- [x] Repo structure: `include/sapphirelib/`, `src/sapphirelib/`, `docs/`, `examples/`
- [x] README, LICENSE, CONTRIBUTING, issue templates
- [x] `.gitignore`, `.clang-format`
- [x] CI workflow skeleton (build check)
- [x] PROS kernel template pulled locally and merged in (`pros conduct new`) — kernel@4.2.2 +
      liblvgl@9.2.0, merged into repo root alongside `include/sapphirelib/` and `src/sapphirelib/`
- [x] Decide C++ standard + namespace, confirm they compile cleanly against kernel — `gnu++20` /
      `gnu17` pinned explicitly in the root `Makefile` (rather than trusting the kernel template's
      bleeding-edge default of gnu++26/gnu23), `sapphirelib` namespace confirmed. Verified with a
      manual `arm-none-eabi-g++ -c` compile of every `.cpp` against the merged kernel headers.
- [x] Basic logging/telemetry macro system (used by every later phase) —
      `include/sapphirelib/util/log.hpp`, `SAPPHIRELIB_LOG_{DEBUG,INFO,WARN,ERROR}`, wired into
      `sapphirelib::initialize()`

**Deliverable:** Empty library that compiles and links into a PROS project, with CI green.

---

## Phase 1 — Chassis Control (MVP core) — released as `v0.1.0`
**Goal:** Reliable closed-loop drive and turn.

- [x] Motor group abstraction (wraps `pros::MotorGroup`, handles per-side/per-corner grouping, gearing,
      brake modes) — `sapphirelib::chassis::MotorGroup`
- [x] Generic PID controller class (kP/kI/kD, integral windup guard, derivative-on-measurement option,
      slew rate limiting) — `sapphirelib::PID`, unit tests in `tests/control/pid_test.cpp`. Gains are
      *continuous-time*: `update()` scales the integral by its timestep and the derivative by its
      reciprocal, so a gain set survives a change of loop period and textbook tuning rules apply
      directly. (Before, both terms were raw per-tick sums, which silently made auto-tuned gains wrong
      by the loop period in both directions at once — kI 100× too large, kD 100× too small at 10ms.)
      Setting `outputLimit` also enables conditional-integration anti-windup, so an auto-tuned kI can't
      accumulate charge the output can't express.
- [x] Drivetrain classes: `driveDistance()`, `turnToHeading()`, configurable exit conditions, timeout
      failsafe — `sapphirelib::chassis::TankDrivetrain` (2-side differential) and
      `sapphirelib::chassis::HolonomicDrivetrain` (4-corner mecanum/X-drive), sharing
      `DrivetrainConfig`/`ExitConditions`
- [x] Tank + arcade + holonomic driver control modes with joystick curve/expo scaling —
      `TankDrivetrain::tank()`/`arcade()`, `HolonomicDrivetrain::holonomic()`,
      `sapphirelib::curveJoystick()` (cubic curve), unit tests in
      `tests/control/joystick_curve_test.cpp`
- [x] Unit-testable PID module (and angle-wrapping helper — `sapphirelib::wrapDegrees180`, unit tests in
      `tests/util/angle_test.cpp`)

**Deliverable:** Robot drives straight and turns to heading using only IMU + drive encoders — no tracking
wheels required — on either a differential or a holonomic chassis. Implementation compiles cleanly
against the kernel (verified via manual `arm-none-eabi-g++ -c`); **on-bot tuning and validation
(drivePID/turnPID gains, headingCorrectionKP, wheel diameter/gear ratio for the actual chassis) is still
outstanding** before this phase is truly done — see `examples/tank_chassis.cpp` and
`examples/holonomic_chassis.cpp` for wiring examples to start from.

---

## Phase 2 — Sensor Abstraction & Odometry
**Goal:** Flexible, swappable localization system.

- [x] Sensor interface layer: `sapphirelib::odom::TrackingWheel` (abstract), implemented by
      `RotationTrackingWheel` (dedicated tracking wheel via `pros::Rotation`) and
      `MotorGroupTrackingWheel` (drive-encoder fallback, adapts `chassis::MotorGroup`)
- [x] Odometry configuration struct for each supported sensor combination — `OdometryConfig`
      (`verticalOffsetIn`/`horizontalOffsetIn`); which of the four combos is active is purely a
      function of which `TrackingWheel` pointers are passed to `Odometry::Sensors`
- [x] Odometry math per configuration (custom-written, not assuming tracking wheels are standard) —
      `sapphirelib::odom::computeOdometryDelta()`, the standard tracking-wheel-and-IMU ("arc") method,
      framework-agnostic and unit-tested in `tests/odom/odometry_math_test.cpp`
- [x] Pose class (x, y, heading) with documented field-coordinate convention — `sapphirelib::odom::Pose`
- [x] Background odometry task (PROS task at fixed Hz, thread-safe pose access) — `Odometry::startTask()`,
      pose guarded by `pros::MutexVar<Pose>`
- [x] Odometry calibration/tuning routine — `calibrateTrackingWheelOffsetIn()` (spin-in-place
      calibration for a tracking wheel's offset from the tracking center)

**Deliverable:** Accurate pose tracking on any supported sensor config, verified against a taped-out
field. The odometry math is implemented and unit-tested for all four sensor configs (IMU + drive
encoders only, IMU + vertical wheel, IMU + horizontal wheel, IMU + both), and compiles cleanly against
the kernel; **on-bot verification against a taped-out field is still outstanding** for every config —
the test robot now runs the IMU + vertical + horizontal wheel config (`src/robot/devices.cpp`), but
none of them has been measured against a taped-out field yet.

---

## Phase 3 — Motion Algorithms
**Goal:** Pose-aware autonomous motion.

- [x] `moveToPose()` — boomerang-style controller on `TankDrivetrain` (carrot-point curve into the final
      heading); on `HolonomicDrivetrain` it's just moveToPoint()'s translation control plus
      turnToHeading()'s heading control running together, since a holonomic chassis doesn't need the
      carrot trick — translation and rotation don't interfere with each other
- [x] `moveToPoint()` — simpler odom-based point drive, on both drivetrains
- [x] Path following (pure pursuit) for multi-point paths — `sapphirelib::motion::Path`/`PursuitConfig`,
      `followPath()` on both drivetrains, lookahead-circle math unit-tested in
      `tests/motion/pure_pursuit_math_test.cpp`
- [x] Motion chaining / queuing — `sapphirelib::motion::MotionQueue`, runs any sequence of blocking
      motions (driveDistance/turnToHeading/moveToPoint/moveToPose/followPath) one after another
- [x] Async, non-blocking motion execution — `MotionQueue::run()` processes the queue on a background
      PROS task; `autonomous()` can enqueue a full routine and keep running (e.g. to manage a mechanism)
      instead of blocking

**Deliverable:** Full odometry-driven autonomous motion. Implemented and compiles cleanly against the
kernel for all three motion primitives on both drivetrains, with the pure geometry (local-frame rotation,
pure-pursuit lookahead search) unit-tested; **on-bot tuning and verification is still outstanding** —
none of this has been driven on a real chassis yet, and depends on Phase 2's odometry also getting
on-bot verification first (see Phase 2's caveat above). The boomerang lead percentage, pursuit lookahead
distance, and the reused drive/turn PID gains in particular will need real tuning, not just the
defaults shipped here.

---

## Phase 4 — Tuning & Developer Tools ⬅ *current*
**Goal:** Usable and debuggable by the whole team.

- [x] IMU multi-turn drift correction — `sapphirelib::sensors::Imu` wraps `pros::Imu`, tracking
      cumulative (unwrapped) rotation and applying a calibrated `headingScale` before re-wrapping to a
      0-360 heading (raw 0-360 readings can't be scaled directly — there's no sane way to "scale" a value
      that wraps). `calibrateHeadingScale()` derives the scale from a known number of physical turns vs.
      what the IMU measured over them; the cumulative-tracking math is pure and unit-tested in
      `tests/sensors/imu_scale_math_test.cpp`. `TankDrivetrain`, `HolonomicDrivetrain`, and
      `odom::Odometry` all take/use a `sensors::Imu` now instead of a raw `pros::Imu`, so the correction
      applies everywhere heading is read, not just in one place — both drivetrains gained an
      `imuHeadingScale` constructor parameter (default `1.0`, so this is non-breaking) and an `imu()`
      accessor so an externally-built `Odometry` can share the same calibrated instance instead of
      opening a second sensor object on the same physical port.
- [x] Brain-screen GUI system — `sapphirelib::gui`, a branded, tab-based UI built directly on the
      vendored liblvgl (bypassing LLEMU, which is deprecated upstream and wasn't rendering for us
      anyway). `Gui` owns a persistent branding header plus an `lv_tabview`; `Page` is the extension
      point — implement `title()`/`build()`/`update()` to add a tab, whether it's one of SapphireLib's
      own default pages or a team's custom one. Entirely opt-in: `sapphirelib::initialize()` never
      touches the screen, so a team that wants their own UI (or none) just never constructs a `Gui` —
      that's the whole "easy to disable/replace" story. Refresh runs on an `lv_timer`, not a `pros::Task`
      — LVGL isn't thread-safe, and `lv_timer` callbacks are guaranteed to run on the same context LVGL's
      own display task already drives, where a raw background task wouldn't be.
  - [x] Odometry visualization on brain screen — `gui::OdometryPage`: numeric x/y/heading readout plus a
        live position dot and heading line on a scaled field rectangle. The field/screen pixel-mapping
        math is pure and unit-tested in `tests/gui/field_view_math_test.cpp`; the field size defaults to
        a 144x144in (12x12ft) VRC field and is overridable per game.
  - [x] Autonomous selector — `gui::AutonSelectorPage`: register named routines with `addRoutine()`, tap
        one on the brain screen to select it, `autonomous()` calls `run()`. Built on `lv_list`, not a
        hand-rolled layout.
  - [x] Telemetry to brain screen — `gui::HomePage` shows battery %, competition connection/mode
        status, (if given an IMU) heading, and — once handed the SD logger with `setTelemetry()` —
        its state (`SD: logging SL000042`, `SD: waiting for card`, `SD: FAULT`, ...), so a missing,
        full or pulled card is noticed in the pits rather than after the match whose data it lost.
  - [x] SapphireLib's own default pages plug into the same `addPage()` a team's custom pages use — the
        requested "~2 custom pages, easy to expand" story is just calling `addPage()` a couple more
        times; nothing about the framework caps or special-cases the built-in ones.
- [x] PID tuning helper — `gui::PidTunerPage`, both manual and automatic:
  - Manual: adjust a registered `PID`'s kP/kI/kD with +/- touch buttons and immediately re-run a bound
    test motion to see the effect, no re-flashing. The entry selector (Drive/Turn/...) lives in a static
    right-hand column of buttons, not a scrolling list — touch-scrolling on the brain screen proved
    "incredibly hard" to use reliably in testing, so nothing on this page requires it.
  - Automatic: `sapphirelib::tuning`, model-based auto-tune — system identification plus pole
    placement. (Replaced a Ziegler-Nichols relay-feedback auto-tune, which itself replaced an earlier
    Twiddle search. Relay tuning measured each controller at a single frequency, assumed a sinusoidal
    response a drivetrain's friction and backlash don't give, and produced one gain set per plant —
    which couldn't be right for both an autonomous turn and driver heading hold sharing that plant, and
    had nothing to say about pathing.) Auto-Tune now works on *axes* rather than controllers: each axis
    registered with `PidTunerPage::addAxis()` (forward, strafe, turn) is driven through a ramp and a step
    each way (`tuning::runCharacterization()`, travel-limited so a translation axis stays on the field),
    and `tuning::characterizeAxis()` fits `V = kS·sign(v) + kV·v + kA·a` plus the axis's response delay.
    The fit deliberately avoids differentiating position twice — acceleration noise biases kA toward
    zero (regression dilution) — and instead regresses the exact discrete-time solution
    `v[k+m] = α·v[k] + β·u + γ·sign(v)` over intervals whose two velocity windows share no samples.
    Delay is measured by comparing the step's time-to-half-speed against what the delay-free model
    predicts, and the fit is then redone with commands shifted by that delay, since fitting delayed data
    as if it weren't inflates kS and shrinks kA. Every controller registered with `addController()` names
    its axis and a `tuning::ResponseSpec` (settle time, damping ratio, minimum phase margin), and
    `tuning::designPositionGains()` places its poles: `kP = kA·ω²`, `kD = 2ζω·kA − kV` (clamped at 0),
    no kI — feedforward's kS already removes the friction an integrator usually exists for. Delay is
    what textbook pole placement ignores and a V5 loop can't: the design checks the phase margin the
    measured delay leaves and backs ω off until it meets the spec, reporting that it did. One tap
    therefore measures the robot once and tunes every controller from it — `src/robot/tuning.cpp` gives Drive,
    Turn, and the new driver heading-hold PID (`HolonomicDrivetrain::headingHoldPID()`, split off from
    `turnPID()`) three different specs designed from two axis measurements. If any axis fails to fit
    (reversed sensor, too little travel, voltage under kS), the run stops and every controller keeps its
    old gains. The readout shows the model, R², lag, and achieved settle time/phase margin so they can be
    copied into source; nothing is persisted. Pure math unit-tested against a simulated axis with known
    kS/kV/kA, delay, and sensor noise in `tests/tuning/characterization_math_test.cpp`, and the designs
    checked in closed loop against the same simulation in `tests/tuning/gain_design_test.cpp`.
  - Mechanisms: lifts and arms Auto-Tune the same way (`PidTunerPage::addMechanismAxis()`). Gravity
    is the difference: it pulls the same way whichever way the mechanism moves, so
    `tuning::runMechanismCharacterization()` ramps and steps *up and down* between absolute limits,
    braking the mechanism between segments, and `characterizeMechanism()` fits
    `V = kS·sign(v) + kG·g(x) + kV·v + kA·a` with `g` constant (an elevator) or `cos(angle)` (an arm,
    matching `GravityFeedforward`) — the up/down asymmetry is what separates kG from kS. Held samples
    record NaN volts and the fit skips them. The result drops straight into the mechanism
    (`MechanismModel::gravityFeedforward()`, `PositionMechanism::setGravity()`), and its controller is
    designed from the same model with gravity cancelled. While the run has the motors,
    `PositionMechanism::beginExternalControl()` puts the mechanism in a new `external` law, so the
    driver macros can keep calling `update()` without fighting it. 96671H's lift is registered (Auto-Tune
    measures only the selected controller's group, so tuning the lift never drives the chassis), with
    a Run Test that moves it between two heights through the macros' own `update()`.
  - A Stop button aborts a run between samples (and the robot being disabled does too); the new
    gains, and each axis's finish hook, are applied only once the whole run succeeded. The runners
    moved onto the `util/clock.hpp` seam, so `tests/tuning/characterization_runner_test.cpp` runs
    runner → fit → design → closed loop end to end on simulated drive and lift axes, and the tap test
    drives the real runner. The design now adds half the loop's own period to the measured delay, and
    the readout shows the friction band (`GainDesign::staticErrorBound`, kS/kP): how far short of
    the target static friction can stop a controller with no integrator.
  - Driver stick mode toggle (`chassis::DriverInputMode`, the button under Auto-Tune): `voltage` (stick
    = fraction of 12V, the old behavior and still the default) or `velocity` (stick = fraction of the
    axis's top speed, turned into volts through the measured model via `MotorFeedforward`, so the
    friction deadzone at the bottom of the stick is gone and stick position maps linearly to speed).
    Feedforward only for now — it doesn't yet correct for battery sag or motor heat, which needs
    measured-velocity feedback. `HolonomicDrivetrain::holonomicVolts()` is the new raw-volts entry point that
    characterization, calibration spins, and autonomous motions use, so none of them are affected by the
    toggle.
  - Gain adjustments, entry selection, and new test/auto-tune launches are all locked out while a run is
    in progress, since a run and the tuning UI would otherwise read/write the same `PID` object
    concurrently. Every run (manual test or auto-tune trial) happens on a background `pros::Task`, never
    the LVGL-owning context, so it can't freeze the screen — and that task only ever writes to
    `std::atomic` state (a running flag, or the gains it just tried), never an LVGL widget directly;
    `update()` is the only thing that turns those atomics into label text. (An earlier draft of the
    auto-tune path called into a label-setting helper directly from the background task — caught and
    fixed before it shipped, by rereading the same thread-safety rule the rest of the page already
    followed.) Both drivetrains gained `drivePID()`/`turnPID()` accessors (same pattern as `imu()`) so the
    page can reach the controllers to tune.
- [x] Telemetry/logging to SD card — `sapphirelib::telemetry::Logger`, the data source for an
      off-robot tuning app. It records every step of any `PID` (target, measurement, error, the P/I/D
      terms, raw and final output, dt, flags) through a new `PID::setObserver()` hook — so neither the
      drivetrains nor `PidTunerPage` had to change — plus the odometry pose, channels of your own
      (`Channel::record()` from wherever the values are computed, or `poll()` on the logger's sampler
      task), and low-rate events: each motion's start and end with its result, competition phase
      changes, and markers of your own via `telemetry::event()`. One file per program run
      (`/usd/sl/SLnnnnnn.CSV`, numbered because the brain has no clock), in a line-oriented CSV-style
      format specified in `docs/TELEMETRY_FORMAT.md` and read by the standard-library-only
      `tools/telemetry/slt_read.py`, which also pairs motion events into segments and slices a PID's
      rows by them.
      The rule it's built around is that logging never slows the code it records. A producer copies a
      64-byte record into its channel's lock-free ring and moves on; a row that can't be taken right
      now is dropped and *counted* (the counts land in the file) rather than waited for, since a
      stalled control loop is worse than a gap a tuning app can see and step around. There's no mutex
      anywhere on that path, for the same reason as everywhere else: PROS deletes competition tasks on
      every mode change, and a mutex held at that moment stays locked. All SD work happens on a
      low-priority writer task that merges the channels by timestamp and writes in large chunks every
      250ms — on PROS 4.2.2 every write is a filesystem sync, so many small writes would cost far more
      card time — and a sampler task polls sources and the competition state, forcing a write on every
      phase change so the end of a match is on the card moments after the robot is disabled. It rides
      out a card being pulled or filling up (a new file once the card is back, with `H`/`D` health rows
      recording write latency and drops), falls back to the card's root when the `sl` folder is
      missing (PROS can't create folders), and repeats each PID's config and gains at the top of every
      file and again whenever they change, so gains the tuner page or Auto-Tune set mid-session are in
      the log. `telemetry::tapCharacterization()` mirrors Auto-Tune's characterization runs into
      `char.*` channels — a superset of `tuning::CharacterizationSample` — so an axis can be refit
      offline from real runs without touching `tuning/`. The pure parts (the ring, channels and the PID
      probe, number formatting golden-tested byte for byte against the format doc, file naming, the
      characterization tap) are unit-tested under `tests/telemetry/`. **It hasn't written to a real
      card yet**: real SD write latency, and what the V5 does when a card is pulled mid-write, have only
      been exercised against a fake card on disk.
- [x] Startup diagnostic checks (sensor connectivity) — `sapphirelib::diag`: `SensorCheck` (label + port +
      expected `DeviceKind`) checked against PROS's device registry (`pros::c::registry_get_plugged_type`)
      via `runCheck()`/`runChecks()`, without needing the device to already be constructed. `gui::
      DiagnosticsPage` re-runs every registered check on a 250ms poll — not just at startup, so a
      sensor knocked loose mid-match shows up too — and lists failures with what's actually plugged in
      instead; optionally raises a red header banner via `Gui::showWarning()`/`clearWarning()` so a bad
      sensor is visible from any tab. Every change of verdict is also logged (`device` events:
      missing, lost, back), so a sensor that came loose mid-match is in the SD log with its port.
- [x] Motor health telemetry — `Logger::motor(name, port)` logs any motor's volts, current,
      temperature, speed, efficiency and fault bits every 100 ms, as a whole row of NaN when the motor
      doesn't answer (unplugged), never plausible zeros. The robot logs all ten motors, plus battery
      current and temperature, the driver's sticks, buttons and controller connection (`driver`), and
      the macros' state (`mech`). V5 motors cut their own current as they heat (to half at 55 °C,
      nothing at 70 °C) and report it nowhere else, so this is the evidence for a mechanism that fades
      mid-match. Pure part (turning PROS's error values into NaN) tested in
      `tests/telemetry/motor_row_test.cpp`. Not on the brain screen yet: the diagnostics page still
      checks ports, not motor faults.
- [x] Telemetry analyzer — `tools/analyzer/`, a browser app with no install or server (open
      `index.html`, drop the SD card's logs on it; it works offline). Plain JavaScript with a Node test
      suite that CI runs. It reads SLT files (merging a run split across files by `sd,reopened`), cuts
      them into matches and practice sessions, and:
  - lists what went wrong, each finding with its time and a jump to it: motors overheating (with the
    derating step reached), disconnecting, stalling or losing speed per volt; a mechanism stuck
    short of its target at full power, or settling below its targets; battery sag; a controller
    dropping out; sensors unplugged; autonomous motions timing out (and whether they were still
    pushing when they did); PIDs oscillating or their loops stalling; odometry jumps; SD faults and
    dropped rows;
  - replays a match on a small field view (robot pose, trail, targets) and a side view of the lift
    (target and actual height, its motors' temperatures), with the controller's sticks, battery,
    motor tiles and "happening now", next to synced charts;
  - charts any column of any channel on one shared time axis, with presets, zoom, a values table,
    and CSV export; lists every PID step response and every motion with its metrics;
  - tunes offline — the practical form of "a tuner in the app": the robot has to be driven, but a
    log already holds what Auto-Tune measures. It refits Auto-Tune's `char.*` runs with a
    JavaScript port of `tuning/` (matching the C++ to 1e-12 on its golden cases), or fits a model
    from ordinary match driving (the volts each system was sent, next to where it went); designs
    each controller the way the robot does; replays the log's real targets through the fitted model
    with the gains the robot had, the designed ones, and hand-entered ones; and prints the C++ to
    paste. A demo (a simulated match with an overheating lift, a loose cable and a controller
    dropout, plus a pit session with an Auto-Tune run) loads on start, so every view can be seen
    without a robot.

- [x] GUI refresh-cost pass — the default pages were cheap individually but the refresh model wasn't:
      every registered page's `update()` ran on every tick regardless of which tab was showing, and each
      one rewrote its labels unconditionally. `lv_label_set_text()` reallocates and invalidates even when
      the new text is byte-identical, so five pages' worth of unchanged readouts kept LVGL redrawing the
      whole 480x240 ARGB8888 surface at the timer rate. Now: `Gui::tick()` refreshes only the active
      tab's page, with `Page::updatesWhenHidden()` as the opt-in for pages whose `update()` has an
      off-tab side effect (only `DiagnosticsPage`, for its screen-wide warning banner — and it throttles
      itself to 250ms rather than leaning on the timer period); a shared `gui::setLabelText()` compares
      before writing, so a label invalidates on change instead of on tick; `OdometryPage` repositions its
      robot dot and heading line only when they'd land on a different pixel. That page also carried a
      real lifetime bug: `lv_line_set_points()` stores only the *address* of the point array, and it was
      being handed a stack local from `update()`, so LVGL drew the heading line from a dead stack frame
      on every refresh — the array is now a member. Its field view was also 15px taller than the tab
      content area, putting the bottom of the field behind a scroll gesture; shrinking it to 150px lets
      both it and the tab container drop `LV_OBJ_FLAG_SCROLLABLE`, which stops LVGL recomputing scroll
      extents every time the dot moves.
- [x] Asterisk center wheels now drive turns — they were commanded off the recovered *throttle*
      component alone, which is identically zero for a pure turn mix, so the 5th/6th motors coasted
      through every `turnToHeading()` and every stick turn. `setWheelVoltages()` now also recovers the
      rotation component and applies it differentially (left `+`, right `−`), scaled by the new
      `AsteriskConfig::turnContribution` (default `1.0`; `0` restores the old coast-through behavior).
      This is the physically correct thing for them to do — in a point turn about the chassis center a
      wheel on the left or right flank travels purely fore/aft, exactly the direction a straight-mounted
      center wheel rolls — and it also puts them behind `driveDistance()`'s heading correction, which
      reaches the same function as a small rotation term riding on the drive output.

- [x] Thermal compensation on the Asterisk center wheels — a V5 motor derates its own available power
      as it heats (roughly half by 55C, shutdown by 70C) and reports nothing back up the command path.
      On a holonomic chassis that's worse than "slower": the four corners' contributions are meant to
      cancel in every axis but the one being driven, so a *single* derated corner breaks the
      cancellation and the chassis picks up motion nobody asked for — most visibly as a strafe that
      creeps forward or back and twists as it goes. Straight-mounted center wheels face exactly the
      right way to cancel that.
      `chassis::thermalPowerFraction()` estimates what a motor at a given temperature still delivers;
      `chassis::centerThermalCorrection()` multiplies each corner's shortfall by its commanded voltage
      and projects the four onto the axes the center wheels can push on ([+ + + +] for forward,
      [+ - + -] for yaw, no strafe pattern because nothing mounted fore/aft can help there). One
      expression covers both regimes: corners derating *together* under throttle reinforce into a
      drive-harder boost, while *unevenly* derated corners under a strafe produce the drift correction.
      `HolonomicDrivetrain` caches the per-corner fractions at 2Hz (temperature moves over tens of
      seconds) but recomputes the correction every tick from live corner voltages — necessarily, since
      the same derating means a shortfall while driving and a drift while strafing, and only the
      current command distinguishes them. Per corner, not averaged: two hot corners on a strafe's
      forward diagonal double the drift while one on each diagonal cancels, and an average can't tell
      those apart. Pure and unit-tested in `tests/chassis/thermal_math_test.cpp`, including that
      asymmetry case checked against an independently computed drift. Tunable via
      `AsteriskConfig::thermalCompensation` (0 disables) and `maxThermalCorrectionVolts`, faded out as
      the *center* motors heat up themselves so this can't turn a four-motor overheat into a six-motor
      one. Feedforward, and deliberately complementary to `driftCorrectionKP`'s reactive tracking-wheel
      loop — this corrects before the chassis has moved, that one catches the remainder plus everything
      derating isn't. What stays out of reach is sideways thrust: a strafe on hot corners holds its
      line but still loses speed.

- [x] Heading-hold driver control — the turn stick now steers a *heading* instead of commanding a turn
      rate: `sapphirelib::advanceHeldHeadingDeg()` sweeps a held heading from the stick, and
      `HolonomicDrivetrain::holonomicFieldCentricHeadingHold()` closes the turn PID on it every tick,
      mixing the result in on top of translation rather than replacing it. Releasing the stick leaves
      the chassis pointed somewhere definite instead of coasting on through, and anything that knocks
      it off heading mid-move — a collision, an uneven strafe, a corner motor derating — gets corrected
      without the driver reacting. The raw-rate `holonomic()`/`holonomicFieldCentric()` are untouched
      and still what the autonomous paths use.
      Three things make it behave. A deadband, because a V5 stick at rest reports a count or two that
      `curveJoystick()` passes straight through and a held heading would integrate all match (input past
      it is rescaled so there's no jump at the edge). A lead cap, because a rate-steered target
      otherwise sweeps at its full slew rate while the chassis lags by whatever error the PID needs —
      a debt the chassis pays off by over-rotating after the stick is released; capping how far the
      target may run ahead bounds that, at the cost of making the cap, not the slew rate, the real
      limit on sustained turn rate. And a resume check: a gap longer than a quarter second means driver
      control wasn't running (autonomous, a PID tuner test, a calibration spin), so the held heading
      re-adopts the live one and the PID is reset rather than steering back to a pre-interruption
      target. Pure part unit-tested in `tests/control/heading_hold_test.cpp`, including the 0/360 seam
      and a property check that the held heading can never drift outside the cap.

- [x] Drift correction measures actual drift — `driftCorrectionKP` used to treat *all* forward/back
      motion on the vertical tracking wheel as drift whenever strafing dominated, so it fought any
      intentional diagonal (a strafe with some throttle mixed in, or a field-centric push with the
      chassis rotated ~45°) and fought turning while strafing, since a wheel 3.59in off center rolls
      ~0.06in per degree of rotation. `chassis::strafeDriftIn()` now removes the wheel's rotation arc
      (using `OdometryConfig::verticalOffsetIn`, read live via `setDriftSource()`'s new `odometry`
      argument) and the forward travel the command asked for (the corner encoders' measured strafe
      travel scaled by the command's throttle/strafe ratio). It deliberately does *not* subtract the
      encoders' own forward reading — corners spinning unevenly under the same command is the most
      common strafe drift and exactly what `thermalCompensation`'s feedforward leaves for this loop to
      catch. Pure and unit-tested in `tests/chassis/drift_math_test.cpp`.
- [x] `moveToPoint()`/`followPath()` hold heading — both passed a zero turn command, so a holonomic
      chassis could yaw freely on the way (translation still arrived, since `toLocalFrame()` uses the
      live heading). They now hold the heading the motion started with through the turn PID, the same
      way `moveToPose()` holds its target, and `followPath()` carries its starting heading through the
      final `moveToPoint()` approach.

**Deliverable:** Tools that make tuning and debugging fast during practice. **The brain-screen GUI has
been confirmed working on real hardware** — tab bar height has been bumped twice in response to that
testing (28 → 31 → 43px total). The robot program (`src/robot/screen.cpp`) wires up `Gui` with
`HomePage` + `AutonSelectorPage` + `DiagnosticsPage` + `PidTunerPage` + `OdometryPage` against the real
test chassis, including a real `odom::Odometry` (on two rotation-sensor tracking wheels) so the
auto-tune and odometry pages have live data to work with. IMU drift correction, PID tuning (manual and automatic), and sensor-port diagnostics are all
implemented and compile clean against the kernel, and — critically, now that a host compiler is available
in this environment — every pure-math module's unit tests (`odom`, `motion`, `sensors`, `gui`, `tuning`)
have actually been *run*, not just type-checked; that pass also caught and fixed a real bug where CI was
failing to link three of them due to unlisted cross-module dependencies (see
`.github/workflows/build.yml`). Still outstanding: the IMU scale factor needs calibrating on the real
robot (default `1.0` is a no-op); the new `DiagnosticsPage`/`PidTunerPage` widgets, and the auto-tune flow
specifically, haven't had on-hardware time yet the way the rest of the GUI has — the identification and
gain-design math is verified against a simulated axis (including delay and sensor noise), but the
characterization voltages/travel in `src/robot/tuning.cpp` are unmeasured starting points, and how the fit holds
up against real odometry noise, wheel slip, and backlash hasn't been checked. Still to come on the tuning
side: motion profiles with feedforward and measured-velocity feedback in `moveToPoint()`/`followPath()`
(per-axis gains, so strafing stops borrowing the forward axis's), and an automated path-tracking
validation run — for which the SD telemetry now records the data (see above), though it hasn't run
against a real card yet. The lift's Auto-Tune and the new telemetry are verified on the host
(simulated axes, fake clock) and compile-checked only; the analyzer has been run against simulated
logs, not a real match's.

---

## Phase 5 — Subsystem & Utility Support ⬅ *in progress, alongside Phase 4*
**Goal:** Everything else a competition robot needs.

- [x] Generic subsystem/mechanism class pattern (intake, arm, lift, etc.) — `sapphirelib::mechanism`,
      built as primitives rather than a framework: no subsystem base class, scheduler, or command
      groups, because those add indirection and ordering rules to learn while removing nothing from a
      robot's file. A driver macro system already has the right shape — one per-tick function whose
      `else if` chain states the robot's priorities, with every output recomputed from state each tick
      — so each primitive replaces one piece of the machinery teams otherwise hand-roll around that
      function. `docs/MACROS.md` is the guide, and `examples/macros.cpp` a complete example.
  - `PositionMechanism` — a lift, an arm, or anything else a motor group drives to a position read off
    a sensor: PID plus gravity feedforward (constant for an elevator, scaled by `cos(angle)` for an
    arm), an optional seat-and-rest at a hard stop (drive down onto it, then rest at 0V instead of
    pushing into it forever), braking when the sensor stops answering, a manual volts override, and
    settle detection. Commands and queries go through lock-free atomics, so any task may call them
    and a deleted competition task can't orphan a lock. The control law itself
    (`computePositionCommand()`) is pure, and is checked bit for bit against the hand-written lift
    loop it replaced, over 1.2 million simulated ticks with sensor dropouts, in
    `tests/mechanism/position_control_test.cpp`.
  - `Piston` — a pneumatic that knows how long it's been in its current state ("outtake once the claw
    has had 400ms to deploy"). `Roller` — an intake or conveyor with an optional pure `JamDetector`: a
    roller told to spin that isn't turning gets a short reverse pulse, then goes back to the command
    (`tests/mechanism/jam_detector_test.cpp`). `PresetLadder` — named tables of preset positions,
    stepped a level at a time (one table per scoring mode, say).
- [x] Async task utilities for mechanism control alongside drive/auton (the mechanism side):
  - `PositionMechanism::startTask()` runs a mechanism on its own fixed-period task, so a lift keeps
    holding while autonomous blocks on a drivetrain motion, and `moveTo()`/`waitUntilSettled()` wait
    on it with a timeout. Or drive it from your own loop with `update(now)` — the same class, no task.
    While disabled the task brakes with its PID cleared, so nothing winds up while VEXos ignores the
    motors.
  - `Sequence<StepId>` — timed step programs ("dip the lift, outtake for 500ms, go up a level")
    advanced from driver control one tick at a time instead of blocking it, with at most one
    transition per update so every step gets at least one tick; the same program runs to completion
    from autonomous with `runBlocking()`. Checked against a verbatim copy of the robot's hand-rolled
    phase machine over 10,000 random driver traces in `tests/util/sequence_test.cpp`.
  - `waitUntil()` — a blocking wait that always has a timeout, since a wait with no way out is how a
    robot sits frozen for the rest of a match when a sensor comes unplugged.
  - Underneath them: `util/timing.hpp` (wrap-safe `elapsedMs()`, `Stopwatch`, `TimedFlag`,
    `GapDetector` — none of which read a clock, so "take now once per tick and hand it to everything"
    is the only way to use them) and `util/clock.hpp`, the seam that lets `waitUntil()` and
    `runBlocking()` run under host tests with a fake clock.
  - Async *drivetrain* motions are still open — see "Deferred cleanup" below.
- [ ] Math/geometry utility library (angle wrapping, vector math, spline helpers) — angle wrapping
      exists (`util/angle.hpp`, from Phase 1); vector math and spline helpers don't yet.
- [x] Controller input utilities (button macros, rumble feedback, deadzone handling) —
      `sapphirelib::input::Controller`: `update()` samples every button and stick once per tick, then
      `pressed()`/`released()`/`held()`/`heldMs()`/`longPressed()`/`repeated()`/`combo()` can be asked
      as often as you like. Edges come from comparing this tick's sample with the last one
      (`ButtonTracker`, pure), not from PROS's `get_digital_new_press()`, whose "already seen" flag
      only updates when a button is *read* — so a button skipped for a while fires late, two readers
      steal presses from each other, and a two-button combo written with `&&` only works in one order
      (the bug behind the robot's old B+DOWN shortcut). Sampled every tick, it reports exactly what
      `get_digital_new_press()` would have, checked against a copy of the kernel's latch logic over
      400,000 random ticks. `resumed()` flags the loop having stopped and restarted (autonomous, a
      disable), so stale sequence and PID state can be dropped. `ControllerScreen` keeps three lines
      of text plus a rumble queue, and sends at most one write per 60ms and only what changed, since
      V5 controllers silently drop text sent faster than about every 50ms. Sticks come normalized to
      [-1, 1] and read 0 while the controller is disconnected; `applyDeadband()`, next to
      `curveJoystick()`, zeroes the count or two a stick reads at rest.

**Deliverable:** A robot's full software stack can be built on SapphireLib alone. 96671H's own
intake/claw/lift macros (`src/robot/macros.cpp`) now run on these primitives, and the port was checked
piece by piece against the code it replaced: the lift law bit for bit, the score/re-seat sequences
phase for phase, the controller screen write for write, and button edges against the kernel's latch.
Still outstanding: none of it has had hardware time beyond those equivalence tests, and task mode
(`startTask()`) isn't exercised by the robot at all yet — its lift is still driven from opcontrol, as
before, so nothing holds it during autonomous.

---

## Library Cleanup (LemLib/EZ-Template-inspired)
**Goal:** Make the existing API cleaner and more intuitive — closer to what a LemLib or EZ-Template
user expects — without dropping features or changing how any motion drives.

- [x] `MotionResult` from every blocking motion — `driveDistance()`, `turnToHeading()`,
      `moveToPoint()`, `moveToPose()` and `followPath()` on both drivetrains returned `void`, so a
      routine couldn't tell a motion that arrived from one that timed out against a wall. They now
      return `motion::MotionResult` (settled / timed out / aborted, the final error, and how long it
      ran), and their shared settle/timeout bookkeeping moved into a pure `motion::ExitTracker`,
      checked against a verbatim copy of the old inline loop in `tests/motion/exit_tracker_test.cpp`.
      A simulator harness replaying both drivetrains' old and new code confirmed that every motor
      command, and when it was sent, is unchanged in every scenario it covered. Each motion also logs
      a `motion` start/end event to the SD telemetry, when a logger is running.
- [x] `setOdometry()` — set the pose source once, then write `moveToPoint(24, 24)` instead of passing
      the odometry to every call, the way LemLib users expect. The overloads that take an `Odometry`
      work unchanged; with none set, the new ones log an error and return `aborted` without moving.
- [x] Sane exit defaults — `ExitConditions::errorThreshold` had no initializer, so `{}` or
      `{.timeoutMs = 1500}` meant a threshold of 0: a motion that could never settle and always ran to
      its timeout. It now defaults to 1.0 (inches; `turnToHeading()`'s default argument uses 2.0
      degrees), `PoseExitConditions` to 1in and 2°, and `moveToPoint()`/`moveToPose()` gained default
      arguments. Every threshold documents its units.
- [x] `driveDistance()` measures from wherever the encoders read when it starts, instead of taring
      them. A tare is device-level, so it also zeroed a `MotorGroupTrackingWheel` on the same motors
      (the drive-encoder odometry config) and jumped the pose back by everything driven so far.
- [x] `followPath()` guards — an empty path was undefined behavior (`.back()` on an empty vector) and
      now returns `aborted`; `PursuitConfig::timeoutMs` (default 10s, added as the last field so
      existing initializers stay valid) caps the pursuit phase, which used to end only by reaching the
      final approach, so a blocked robot chased the path for the rest of autonomous.
- [x] Robustness against disconnected devices and task races:
  - `sensors::Imu` skips non-finite reads. One `PROS_ERR_F` (unplugged, or recalibrating after a
    brownout) used to turn heading into NaN for the rest of the program, taking turns, odometry and
    heading hold with it. A failed read also drops the baseline, so when the sensor comes back reading
    near 0 that restart isn't counted as a turn — heading loses only what the chassis actually turned
    during the gap. Its cumulative tracking is now lock-free atomics: it's read from the
    odometry task, the GUI and competition tasks at once, and two readers racing on the old code could
    count one rotation twice. `calibrated()` reports whether calibration worked, and a failure is
    logged.
  - `RotationTrackingWheel` holds its last good reading while the sensor isn't answering (an
    unplugged sensor used to move the pose ~515,000in), and `MotorGroup::getPositionDegrees()`/
    `getVelocityRPM()` skip motors that aren't answering (one unplugged drive motor made
    `driveDistance()`'s error -inf). A group where *nothing* answers — every holonomic corner is a
    one-motor group — holds its last position instead of reading 0, since a jump to 0 from however far
    the motor had turned is just as much a runaway once `driveDistance()` measures from a start
    reading.
  - `Odometry::startTask()`, `MotionQueue::run()` and `Gui::start()` ignore a second call instead of
    starting a second worker, and `SensorCheck` reports an out-of-range port as a config error.
- [x] Smaller additions — `appliedAxisVolts()` on both drivetrains (the forward/strafe/turn volts
      last commanded, for telemetry and model fitting); `Page::isBusy()` and `Gui::anyPageBusy()`, so
      driver control can stand aside while a GUI routine (an offset calibration, a tuner run) drives
      the chassis; `AutonSelectorPage::selectedName()`; `DeviceKind::distance`/`optical` in
      diagnostics; and `PID::setObserver()`/`lastStep()`/`config()`, which telemetry builds on.
- [x] Project layout — the robot program moved out of one long `main.cpp` into `include/robot/` +
      `src/robot/` (namespace `robot`): every port written once, in `config.hpp`; devices behind
      accessors backed by function-local statics, built in a known order from `initialize()` rather
      than during static initialization, which is no place for the IMU's blocking calibration;
      autonomous routines in `autons.cpp`; tuning, telemetry, driver control and the macros each in a
      file of their own; and `main.cpp` down to the competition callbacks. The `Makefile` now keeps
      everything under `src/` except `src/sapphirelib/` out of the library archive — an allowlist, so a
      new robot file can't silently ship inside `sapphirelib.a`.
- [x] Tooling — `.clang-format` now matches the house style (it indented namespace contents, which the
      code never has, so the CI format gate reported thousands of violations and couldn't pass), with
      `tuning/` left out of the CI check until Auto-Tune landed (it's in now); `make check-examples` compiles every
      example against the current headers so they can't silently rot; and CI's test loop handles
      header-only modules and tests that need extra sources.
- [x] One field heading frame — `Odometry::setPose()` (and the constructor's `startPose`) used to
      ignore the heading: every update overwrote it with the raw IMU heading, so the field frame was
      locked to wherever the robot faced at calibration, and an autonomous that started at
      `{x, y, 270}` got rotated x/y axes and turn targets 90° off. `sensors::Imu` now has a heading
      offset (`setHeadingDeg()`/`headingOffsetDeg()`) that `getHeadingDeg()` applies, and `setPose()`
      sets it on the Imu odometry shares with the drivetrain — so the pose, `turnToHeading()`,
      `moveToPose()`, and the Home page all read the same field heading after one call. Two frames,
      each where it belongs: `getCumulativeHeadingDeg()` stays unshifted rotation since construction,
      so everything that only takes differences of it (Auto-Tune's turn experiment, the offset
      calibration spin, Asterisk drift correction) can't see a re-frame; and the things that track a
      physical direction — field-centric "forward", driver heading hold, `driveDistance()`'s heading
      correction — work in that rotation frame too, so a `setPose()` never moves the driver's forward
      or makes heading hold spin to chase a target that just jumped. Odometry re-expresses its
      previous heading in the current frame, so a re-frame between two updates isn't read as a turn
      (which would have fired the tracking wheels' arc correction), and travel from before a
      `setPose()` never lands in the new pose, even with the chassis moving: `setPose()` takes the
      readings the next update measures from, and an update already in flight is discarded. The offset math and the
      re-framing property are unit-tested in `tests/sensors/imu_scale_math_test.cpp` and
      `tests/odom/odometry_math_test.cpp`. Robots that start at heading 0 (this one does) see no
      change beyond the IMU's few milliseconds of drift between chassis and odometry construction.

### Deferred cleanup
Worth doing, but each one changes behavior, touches every call site, or wants on-robot validation
first:

- [ ] Config-struct constructors for the drivetrains — eleven positional arguments with `/*name=*/`
      comments is exactly the complaint about LemLib's stable API, and a `Config` struct would also
      make explicit that `headingHoldPID()` is silently built from the turn PID's config. Add it as a
      delegating constructor first (non-breaking) and deprecate the positional one later.
- [ ] Per-motion params and unified exit types — `chassis::ExitConditions` and
      `motion::PoseExitConditions` live in different namespaces, and `motion_config.hpp` and the
      chassis headers include each other. Move the exit types to `motion/` (keeping an alias) and add
      LemLib-style per-call params (`maxVolts`, `minVolts`, `earlyExitIn`, `reverse`): today the only
      speed limit is each PID's shared `outputLimit`.
- [ ] Explicit `calibrate()` — move the IMU's blocking calibration out of the drivetrain constructor,
      so the chassis can be an ordinary global and the accessors become optional. It changes
      constructor semantics, so it goes with the config-struct change.
- [ ] Async motions on one persistent motion task per drivetrain — `moveToPoint(..., {.async =
      true})`, then `waitUntil(inches)` to fire a mechanism mid-path, `waitUntilDone()`, `cancel()`.
      The request is copied into the task, never captured by reference; there's one long-lived task,
      not one per motion; and it cancels itself on any competition-state change — each of those avoids
      a known LemLib or EZ-Template bug. `MotionQueue` becomes a thin layer over it, and its lifetime
      hazard (a queue declared inside `autonomous()` leaves its worker running on freed memory) goes
      away. Needs the two items above first.
- [ ] Shorter names through C++20 inline namespaces — `sapphirelib::HolonomicDrivetrain` alongside
      `sapphirelib::chassis::HolonomicDrivetrain`, source-compatible. It touches every file's namespace
      line; `tuning/` has had its format pass now, so nothing's waiting on it.
- [ ] Driver-control API consolidation — `holonomic` / `holonomicFieldCentric` / `holonomicHeadingHold`
      / `holonomicFieldCentricHeadingHold` is combinatorial naming. One call with options
      (`{.fieldCentric = true, .headingHold = true}`), or a stored mode, would replace them. Add first,
      remove later.
- [ ] `PID::Config::slewRate` per second — it's per call while the gains are per second, so the same
      number ramps twice as fast in a 10ms loop as in a 20ms one. Add a per-second field, then
      deprecate the old one.
- [ ] Kill-safe shared pose and config — `Odometry`'s pose and the drivetrain's axis models are
      `MutexVar`s that competition tasks lock. If PROS deletes the task inside that microsecond window,
      the odometry task blocks forever: a tiny probability with a catastrophic outcome. Replace them
      with atomics or a seqlock, with or before the async motions.
- [ ] Tank `moveToPoint()` near the target — the bearing is the `atan2` of a vanishing vector, and the
      steering flips direction whenever the target crosses 90°, so the robot can oscillate or spin in
      the settle window. The usual fix: inside a small radius, freeze the heading and drive on
      `distance·cos(headingError)`.
- [ ] `OdometryPage` calibration timeout — the offset-calibration spin has none, so a blocked spin
      never ends and keeps driver control locked out (through `isBusy()`) until restart, and a
      non-finite IMU reading ends it at once and writes NaN offsets into the live odometry.
- [ ] Keyed GUI warnings — the header has a single warning slot, and `DiagnosticsPage` clears it every
      250ms while its checks pass, wiping anyone else's `showWarning()`.
- [ ] Gains written while a loop runs — `PID::setGains()` from the tuner page's task lands while
      another task may be inside `update()`. The gains are plain doubles, so one step can use a mix
      of old and new kP/kI/kD (and on the Cortex-A9 a double itself isn't guaranteed to be written in
      one piece). Harmless in practice — it lasts one tick — but the fix is cheap: stage new gains in
      an atomic slot that `update()` adopts at the top of its next step.

---

## Phase 6 — Documentation, Testing, and v1.0 Release
**Goal:** Ship a stable, documented v1.0.

- Full API documentation (Doxygen or docs site)
- Example autonomous programs for each odometry configuration
- Migration notes from the old LemLib-based StratagemV2.0 code
- Field-testing checklist and known-issues list
- Tag `v1.0.0`, changelog, semantic versioning going forward

**Deliverable:** SapphireLib v1.0 — ready to be the software foundation for the season.

---

## Sequencing Notes

- Phase 1 comes before Phase 2 intentionally — the team gets working PID drive/turn immediately, before odometry exists, so early practice matches aren't blocked.
- Phase 2's multi-configuration odometry is the biggest architectural risk — no direct LemLib equivalent to reference, since LemLib assumes tracking wheels are standard. Worth prototyping the sensor-abstraction interface during Phase 1 so Phase 2 isn't a rewrite.
- Phases 4 and 5 can run partially in parallel with 2–3 once the team is comfortable with the codebase.
