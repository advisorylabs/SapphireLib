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

## Phase 3 — Motion Algorithms ⬅ *current*
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

## Phase 4 — Tuning & Developer Tools
**Goal:** Usable and debuggable by the whole team.

- PID auto-tuning / live-tuning helper
- Telemetry/logging to SD card or brain screen
- Odometry visualization on brain screen
- Autonomous selector (LVGL-based)
- Startup diagnostic checks (sensor connectivity, motor faults)

**Deliverable:** Tools that make tuning and debugging fast during practice.

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
