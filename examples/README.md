# Examples

Example autonomous programs and usage snippets land here starting in Phase 1 (chassis control) and
expand through Phase 3 (motion algorithms). Each example notes which odometry/sensor configuration it
assumes.

These files are illustrative only — the Makefile compiles `src/`, not `examples/`, so copy the relevant
pieces into your own `src/main.cpp` rather than expecting these to build as-is.

- [`tank_chassis.cpp`](tank_chassis.cpp) — Phase 1: closed-loop differential (tank) drive with
  `driveDistance()` / `turnToHeading()` and curved arcade driver control. Assumes IMU + drive motor
  encoders only, no tracking wheels.
- [`holonomic_chassis.cpp`](holonomic_chassis.cpp) — Phase 1: closed-loop mecanum/X-drive with
  `driveDistance()` / `turnToHeading()` and curved holonomic (throttle/strafe/turn) driver control.
  Assumes IMU + drive motor encoders only, no tracking wheels.
