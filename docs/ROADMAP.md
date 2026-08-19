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
      slew rate limiting) — `sapphirelib::PID`, unit tests in `tests/control/pid_test.cpp`
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
  - Automatic: `sapphirelib::tuning`, a Twiddle (coordinate-ascent hill-climbing) search over the three
    gains — not automatic relay/Ziegler-Nichols tuning, which works by deliberately driving the system
    into sustained oscillation; Twiddle instead just tries nearby gains and keeps whatever measurably
    did better, so every trial is an ordinary bounded test motion. `measureOvershoot()` turns a series of
    odometry samples taken during one test motion into an overshoot/final-error pair;
    `tuning::runAndSample()` is what collects those samples (runs the motion on a background task while
    polling odometry from the caller); `autoTuneCost()` combines them into the scalar Twiddle minimizes.
    The Twiddle state machine itself, the cost function, and convergence are all pure and unit-tested in
    `tests/tuning/auto_tune_math_test.cpp` — including a synthetic-cost-landscape test that actually
    checks the search converges toward a known minimum, not just that it runs. `src/main.cpp` wires the
    drive controller's test cycle to alternate forward/backward ~2ft (so repeated trials don't walk the
    robot away) and the turn controller's to two safely-off-seam headings (90°/30°) since turning in
    place doesn't accumulate drift the way driving does. Once a search finishes (or hits its 40-trial
    cap), the tuned gains are applied immediately *and* displayed on the page so they can be copied into
    source — this only tunes what's live in memory; it does not persist or hardcode anything itself.
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
      DiagnosticsPage` re-runs every registered check on every refresh tick — not just at startup, so a
      sensor knocked loose mid-match shows up too — and lists failures with what's actually plugged in
      instead; optionally raises a red header banner via `Gui::showWarning()`/`clearWarning()` so a bad
      sensor is visible from any tab. Motor *fault* checking (stalls/over-temp, as opposed to wrong-port
      detection) is still open — `MotorGroup` doesn't currently expose per-motor fault flags.

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
specifically, haven't had on-hardware time yet the way the rest of the GUI has — the Twiddle search itself
is verified against a synthetic cost function, but nothing about how well it performs against *real*
overshoot noise, sensor lag, or the drive-encoder-fallback odometry's own accuracy has been checked; SD-card
telemetry and motor-fault diagnostics haven't been started.

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
