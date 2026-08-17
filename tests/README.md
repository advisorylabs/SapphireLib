# Tests

Host-side unit tests for the parts of SapphireLib that have no PROS/embedded dependency (pure math:
`sapphirelib::PID`, `sapphirelib::curveJoystick`, ...). These build and run with a normal desktop
compiler — no PROS CLI or ARM toolchain needed — and run in CI on every push (see
`.github/workflows/build.yml`).

Chassis/odometry code that touches `pros::` types isn't unit-testable this way; that gets validated
on-bot per `CONTRIBUTING.md`'s testing guidance.

## Running locally

Each test file documents its own build command in a header comment. For example:

```bash
g++ -std=c++20 -Iinclude tests/control/pid_test.cpp src/sapphirelib/control/pid.cpp -o pid_test && ./pid_test
```

## Layout

Mirrors `include/sapphirelib/` / `src/sapphirelib/`: a test for `src/sapphirelib/control/pid.cpp` lives at
`tests/control/pid_test.cpp`.
