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

## Phase 1 — Chassis Control (MVP core) ⬅ *current*
**Goal:** Reliable closed-loop drive and turn.

- Motor group abstraction (wraps `pros::MotorGroup`, handles left/right sides, gearing, brake modes)
- Generic PID controller class (kP/kI/kD, integral windup guard, derivative-on-measurement option, slew rate limiting)
- Drivetrain class: `driveDistance()`, `turnToHeading()`, configurable exit conditions, timeout failsafe
- Tank + arcade driver control modes with joystick curve/expo scaling
- Unit-testable PID module

**Deliverable:** Robot drives straight and turns to heading using only IMU + drive encoders — no tracking wheels required.

---

## Phase 2 — Sensor Abstraction & Odometry
**Goal:** Flexible, swappable localization system.

- Sensor interface layer: `TrackingWheel` (optional, vertical or horizontal), `Imu`, drive encoders as fallback
- Odometry configuration struct for each supported sensor combination
- Odometry math per configuration (custom-written, not assuming tracking wheels are standard)
- Pose class (x, y, heading) with documented field-coordinate convention
- Background odometry task (PROS task at fixed Hz, thread-safe pose access)
- Odometry calibration/tuning routine

**Deliverable:** Accurate pose tracking on any supported sensor config, verified against a taped-out field.

---

## Phase 3 — Motion Algorithms
**Goal:** Pose-aware autonomous motion.

- `moveToPose()` — boomerang-style controller
- `moveToPoint()` — simpler odom-based point drive
- Path following (pure pursuit) for multi-point paths, if scope allows
- Motion chaining / queuing
- Async, non-blocking motion execution

**Deliverable:** Full odometry-driven autonomous motion.

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
