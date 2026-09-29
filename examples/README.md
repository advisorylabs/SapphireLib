# Examples

Example robot programs showing how SapphireLib's pieces fit together, from a bare drivetrain up to
driver macros and SD telemetry. Each example notes which odometry/sensor configuration it assumes.

They aren't part of the program — the Makefile builds `src/`, not `examples/` — so copy the relevant
pieces into your own files rather than expecting these to run as-is. They do stay current, though:
`make check-examples` compiles every file here against the current headers (syntax only), so an API
change that breaks one shows up at once. Run it after changing any public header.

- [`tank_chassis.cpp`](tank_chassis.cpp) — Phase 1: closed-loop differential (tank) drive with
  `driveDistance()` / `turnToHeading()`, checking a `MotionResult` in autonomous, and arcade driver
  control through `input::Controller` with a deadband and joystick curve. Assumes IMU + drive motor
  encoders only, no tracking wheels.
- [`holonomic_chassis.cpp`](holonomic_chassis.cpp) — Phase 1: the same on a mecanum/X-drive, with
  field-centric holonomic (throttle/strafe/turn) driver control and a button to re-zero "forward".
  Assumes IMU + drive motor encoders only, no tracking wheels.
- [`gui.cpp`](gui.cpp) — Phase 4: wiring up SapphireLib's default brain-screen GUI (`HomePage`,
  `AutonSelectorPage`, `OdometryPage`) on a holonomic chassis, including the minimal `odom::Odometry`
  setup (drive-encoder fallback, no dedicated tracking wheels) needed to feed `OdometryPage`, and a
  routine that drives to field points with `setOdometry()` + `moveToPoint(x, y)`.
- [`telemetry.cpp`](telemetry.cpp) — Phase 4: SD-card logging with `telemetry::Logger` on a tank
  chassis with one vertical tracking wheel: the drivetrain's PIDs, the pose, the commanded axis volts,
  the battery, a channel of your own, event markers, and the `SD:` status line on `HomePage`. See
  [`docs/TELEMETRY_FORMAT.md`](../docs/TELEMETRY_FORMAT.md) for what lands in the file.
- [`macros.cpp`](macros.cpp) — Phase 5: a small driver macro system for a hypothetical robot — an arm
  with cosine gravity feedforward on its own task, a clamp piston, an intake roller with anti-jam,
  preset levels, and a score `Sequence` that also runs from autonomous. It's the complete example
  [`docs/MACROS.md`](../docs/MACROS.md) walks through.

The real robot program in `src/robot/` is a larger worked example of the same pieces: ports in one
config file, devices behind accessors, autonomous routines in their own file, and the intake/claw/lift
macros in `src/robot/macros.cpp`.
