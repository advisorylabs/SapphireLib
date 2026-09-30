# Contributing to SapphireLib

Internal library for Team 96671H: Hitmen. This guide is for team members working on SapphireLib itself
(not for teams consuming it as a dependency). [`docs/SETUP.md`](docs/SETUP.md) covers getting a build
going.

## Workflow

1. Branch off `master`: `git checkout -b phase-<n>/<short-description>` (e.g. `phase-1/pid-controller`)
2. Keep PRs scoped to one roadmap item where possible: see `docs/ROADMAP.md` for current phase.
3. Format what you change with `clang-format` (`.clang-format` is in the repo root and matches the
   house style below). The library has had its one-time format pass, so `clang-format -i` on any file
   under `include/sapphirelib/` or `src/sapphirelib/` changes only what you wrote. Elsewhere, format
   only the lines you touched (`git clang-format` does exactly that), so your diff stays about your
   change.
4. Build from the PROS toolchain, never a bare `make` from a shell with another ARM GCC on `PATH` (see
   `docs/SETUP.md` step 4 for why). If you changed a public header, also run `make check-examples`.
5. Open a PR into `master`. CI must pass before merge: the build, the host-side unit tests, the
   telemetry reader's self-test, the telemetry analyzer's tests
   (`node --test tools/analyzer/test/*.test.js`), and the library's formatting check.
6. At least one other team member should review before merging: two sets of eyes catches a lot before
   it hits a competition robot.

## Code Style

- Namespace everything under `sapphirelib::`, most modules in a nested namespace of their own
  (`sapphirelib::chassis`, `sapphirelib::mechanism`, ...). No indentation inside namespaces; close
  them with `} // namespace sapphirelib::chassis`.
- Public headers live in `include/sapphirelib/`, implementation in `src/sapphirelib/`, mirroring the same
  subfolder structure. This robot's own code lives in `include/robot/` + `src/robot/` (namespace
  `robot`), never in the library.
- 4-space indent, 100 columns. `constexpr` constants are `kCamelCase`; private members end in `_`.
  Config structs have default member initializers and are built with designated initializers
  (`PID::Config{.gains = {.kP = 1.2}, .outputLimit = 12.0}`).
- Prefer explicit, documented public APIs over clever templates: this library needs to be readable by
  teammates joining mid-season
- Comments follow LemLib's style. Every public class and function gets a `/** */` block: a `@brief`
  line (capitalized, no period), `@param name description` and `@return` lines in lowercase, `@note`
  for what bites (which task may call it, blocking, lifetimes), and a `@b Example` with
  `@code {.cpp}` on the main API. Struct fields get a one-line `/** ... */`. Say the units and what
  happens at the edges (an unplugged sensor, a timeout of 0). Inline comments are short and lowercase
  (`// reset the PID after a gap`). No file banners, no em dashes, plain ASCII.
- **Pure vs PROS.** Math and decision logic goes in pure modules that include no `pros/` header, even
  indirectly, so they build and test on a desktop compiler. Pure code gets time as an argument
  (`nowMs`) or through `sapphirelib/util/clock.hpp`, which tests replace with a fake clock. PROS-facing
  classes are thin shells around them.
- **Nothing that can stall a control loop.** No heap allocation per tick, and no mutex on any path a
  control loop or competition task runs: PROS deletes the autonomous/opcontrol task on every
  competition-mode change, and a mutex held at that moment stays locked forever. Share state across
  tasks with `std::atomic` or lock-free structures instead.

## Commit Messages

Short, present-tense, scoped to what changed. Reference the roadmap phase where relevant:

```
[phase1] add slew-rate limiting to PID controller
[phase2] implement 2-wheel + IMU odometry math
```

## Testing Changes

Pure modules get a host-side unit test at `tests/<module>/<name>_test.cpp`, with its exact `g++`
build line in a header comment (see [`tests/README.md`](tests/README.md)). CI finds new tests on its
own; if a test needs more than its own module's `.cpp`, add a `case` entry for it in
`.github/workflows/build.yml` to match that build line. When you change behavior that a test pins
down, keep a copy of the old logic in the test and check the new code against it, the way
`tests/mechanism/position_control_test.cpp` does for the lift.

Anything that touches `pros::` types is tested on-bot. When you open a PR touching motion/control
code, note in the PR description:

- What robot/config you tested on
- What sensor configuration was used
- Any tuning constants that changed

## Reporting Issues

Use the issue templates under `.github/ISSUE_TEMPLATE/`. Bug reports should include the sensor/robot
config and, if possible, PROS terminal output, and the SD telemetry log for the run, if the robot was
logging (`docs/TELEMETRY_FORMAT.md`).
