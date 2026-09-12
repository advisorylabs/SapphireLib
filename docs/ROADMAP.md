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
the current test robot (`src/main.cpp`) has no tracking wheels wired up yet, so only the IMU +
drive-encoder-fallback config can even be exercised on real hardware right now.

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
  - [x] Telemetry to brain screen (partial) — `gui::HomePage` shows battery %, competition
        connection/mode status, and (if given an IMU) heading. SD-card logging is still open.
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
    therefore measures the robot once and tunes every controller from it — `src/main.cpp` gives Drive,
    Turn, and the new driver heading-hold PID (`HolonomicDrivetrain::headingHoldPID()`, split off from
    `turnPID()`) three different specs designed from two axis measurements. If any axis fails to fit
    (reversed sensor, too little travel, voltage under kS), the run stops and every controller keeps its
    old gains. The readout shows the model, R², lag, and achieved settle time/phase margin so they can be
    copied into source; nothing is persisted. Pure math unit-tested against a simulated axis with known
    kS/kV/kA, delay, and sensor noise in `tests/tuning/characterization_math_test.cpp`, and the designs
    checked in closed loop against the same simulation in `tests/tuning/gain_design_test.cpp`.
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
- [ ] Telemetry/logging to SD card
- [x] Startup diagnostic checks (sensor connectivity) — `sapphirelib::diag`: `SensorCheck` (label + port +
      expected `DeviceKind`) checked against PROS's device registry (`pros::c::registry_get_plugged_type`)
      via `runCheck()`/`runChecks()`, without needing the device to already be constructed. `gui::
      DiagnosticsPage` re-runs every registered check on a 250ms poll — not just at startup, so a
      sensor knocked loose mid-match shows up too — and lists failures with what's actually plugged in
      instead; optionally raises a red header banner via `Gui::showWarning()`/`clearWarning()` so a bad
      sensor is visible from any tab. Motor *fault* checking (stalls/over-temp, as opposed to wrong-port
      detection) is still open — `MotorGroup` doesn't currently expose per-motor fault flags.

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
testing (28 → 31 → 43px total). `src/main.cpp` wires up `Gui` with `HomePage` + `AutonSelectorPage` +
`DiagnosticsPage` + `PidTunerPage` + `OdometryPage` against the real test chassis, now including a real
`odom::Odometry` (drive-encoder fallback) so the auto-tune and odometry pages have live data to work
with. IMU drift correction, PID tuning (manual and automatic), and sensor-port diagnostics are all
implemented and compile clean against the kernel, and — critically, now that a host compiler is available
in this environment — every pure-math module's unit tests (`odom`, `motion`, `sensors`, `gui`, `tuning`)
have actually been *run*, not just type-checked; that pass also caught and fixed a real bug where CI was
failing to link three of them due to unlisted cross-module dependencies (see
`.github/workflows/build.yml`). Still outstanding: the IMU scale factor needs calibrating on the real
robot (default `1.0` is a no-op); the new `DiagnosticsPage`/`PidTunerPage` widgets, and the auto-tune flow
specifically, haven't had on-hardware time yet the way the rest of the GUI has — the identification and
gain-design math is verified against a simulated axis (including delay and sensor noise), but the
characterization voltages/travel in `src/main.cpp` are unmeasured starting points, and how the fit holds
up against real odometry noise, wheel slip, and backlash hasn't been checked. Still to come on the tuning
side: motion profiles with feedforward and measured-velocity feedback in `moveToPoint()`/`followPath()`
(per-axis gains, so strafing stops borrowing the forward axis's), and an automated path-tracking
validation run. SD-card telemetry and motor-fault diagnostics haven't been started.

---

## Phase 5 — Subsystem & Utility Support
**Goal:** Everything else a competition robot needs.

- Generic subsystem/mechanism class pattern (intake, arm, lift, etc.)
- Async task utilities for mechanism control alongside drive/auton
- Math/geometry utility library (angle wrapping, vector math, spline helpers)
- Controller input utilities (button macros, rumble feedback, deadzone handling)

**Deliverable:** A robot's full software stack can be built on SapphireLib alone.

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
