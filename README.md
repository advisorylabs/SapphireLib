# SapphireLib

Standalone motion & control library for VEX V5 robots, built on [PROS](https://pros.cs.purdue.edu/).
Developed by **Team 96671H: Hitmen**, as the successor to StratagemV2.0.

SapphireLib is a from-scratch rewrite, it does not depend on LemLib or EZ-Template. It's built around a
flexible sensor model, so the same library supports robots with tracking wheels, robots without them, or
any mix in between.

## Status

🚧 Pre-1.0. Phases 1-3 (chassis control, odometry, pose-aware motion) are implemented, along with most
of Phase 4's developer tools (brain-screen GUI, PID tuning and model-based Auto-Tune, sensor
diagnostics, SD-card telemetry) and Phase 5's driver-macro and mechanism building blocks. Only Phase 1
has been released as a template (`v0.1.0`); everything since lives in this repo but isn't in a release
yet.

The pure math under all of it is unit-tested on a desktop compiler, but most of the library hasn't had
on-robot validation yet: odometry, the pose motions, Auto-Tune, and SD logging in particular. On-bot
PID tuning is still per-robot work you'll do after pulling it in. See
[`docs/ROADMAP.md`](docs/ROADMAP.md) for the phase-by-phase plan and what each phase still needs.

## What's in the box

- **Chassis** — `TankDrivetrain` and `HolonomicDrivetrain` (mecanum/X-drive, with optional "Asterisk"
  center wheels): `driveDistance()`/`turnToHeading()`, arcade/tank/holonomic driver control,
  field-centric and heading-hold modes, and `curveJoystick()`/`applyDeadband()` stick shaping.
- **Odometry** — `odom::Odometry` on a background task, for any of the four sensor configurations below.
- **Motion** — `moveToPoint()`, `moveToPose()` (boomerang on tank), and pure-pursuit `followPath()` on
  both drivetrains, plus `MotionQueue`. Every blocking motion returns a `MotionResult`, so a routine
  can tell a motion that settled from one that timed out against something.
- **GUI** — a tab-based brain-screen UI: home/status, autonomous selector, live odometry, sensor
  diagnostics, and a PID tuner. Opt-in, and your own pages plug into the same `addPage()`.
- **Tuning** — manual PID tuning from the brain screen, and Auto-Tune: it measures each drive axis's
  model — or a lift's or arm's, gravity included — and places every controller's poles from it. Also
  IMU heading-scale calibration.
- **Macros & mechanisms** — `input::Controller` (button edges, hold times, combos, a throttled
  controller screen), `PositionMechanism` (lifts and arms: PID plus gravity feedforward, on your loop
  or its own task), `Piston`, `Roller` with anti-jam, `PresetLadder`, `Sequence`, and `waitUntil()`.
  See the guide, [`docs/MACROS.md`](docs/MACROS.md).
- **Telemetry** — `telemetry::Logger` records PID steps, pose, motor health (temperature, current,
  derating, disconnects), your own channels, and events to the SD card, one file per program run,
  without ever blocking the code it records. Format:
  [`docs/TELEMETRY_FORMAT.md`](docs/TELEMETRY_FORMAT.md); reader: `tools/telemetry/slt_read.py`.
- **Telemetry analyzer** — [`tools/analyzer/`](tools/analyzer/): open `index.html` in a browser (no
  install, works offline) and drop the SD card's logs on it. It lists what went wrong in a match
  (overheating, derating, disconnects, stalls, battery sag, timed-out motions), replays the match on a
  small field-and-lift view next to synced charts, and tunes controllers from the log: it refits
  Auto-Tune runs or ordinary driving, designs gains, shows how they'd have done against the match's
  real targets, and writes the C++ to paste.

## Supported Odometry Configurations

- IMU + drive motor encoders only (no tracking wheels required)
- IMU + single vertical tracking wheel
- IMU + single horizontal tracking wheel
- IMU + vertical + horizontal tracking wheels

## Getting Started

### Prerequisites

- [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/) installed
- A PROS V5 project (kernel template), SapphireLib is added as a library on top of it, it is not a
  standalone PROS project itself once integrated into a robot repo

### Using SapphireLib in your own robot project (recommended)

Grab the `sapphirelib` template `.zip` from the [Releases](../../releases) page and pull it into your
own PROS kernel project:

```bash
pros conduct fetch path/to/sapphirelib@0.1.0.zip
pros conduct apply sapphirelib
```

Then `#include "sapphirelib/api.hpp"`. A minimal tank robot looks like this:

```cpp
#include "main.h"
#include "sapphirelib/api.hpp"

using namespace sapphirelib;

// At namespace scope, never a local in opcontrol(): its button history has to
// survive opcontrol() restarting (see input::Controller).
input::Controller master(pros::E_CONTROLLER_MASTER);

// Built on the first call, from initialize(): the constructor blocks while the
// IMU calibrates, which is no job for static initialization.
chassis::TankDrivetrain& drivetrain() {
    static chassis::TankDrivetrain instance(
        /*leftPorts=*/{1, -2, 3}, /*rightPorts=*/{-4, 5, -6}, chassis::Gearset::green,
        /*imuPort=*/10,
        chassis::DrivetrainConfig{.wheelDiameterIn = 3.25, .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/PID::Config{.gains = {.kP = 1.2, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/PID::Config{.gains = {.kP = 0.35, .kD = 0.0002}, .outputLimit = 12.0});
    return instance;
}

void initialize() {
    sapphirelib::initialize();
    drivetrain(); // blocks ~2-3s while the IMU calibrates
}

void autonomous() {
    drivetrain().driveDistance(24.0); // within 1in for 200ms, or a 3s timeout
    if (!drivetrain().turnToHeading(90.0).settled()) return; // stuck: stop here
    drivetrain().driveDistance(-24.0, {.timeoutMs = 1500});
}

void opcontrol() {
    while (true) {
        master.update(); // one sample of every button and stick per tick
        drivetrain().arcade(master.axis(input::Axis::leftY), master.axis(input::Axis::rightX));
        pros::delay(20);
    }
}
```

[`examples/`](examples/) builds on this: a holonomic chassis, the GUI with odometry and pose motions,
a driver macro system, and SD telemetry.

### Building this repo / building the template yourself

This repository holds the SapphireLib source plus the PROS kernel it builds against, and 96671H's own
robot program on top. See [`docs/SETUP.md`](docs/SETUP.md) for local setup, which toolchain to build
with (it matters), and how to build the distributable template `.zip`.

## Project Structure

```
SapphireLib/
├── include/
│   ├── sapphirelib/       # Public library headers: the template ships exactly these
│   └── robot/             # 96671H's robot program: ports, devices, autons, macros, ...
├── src/
│   ├── sapphirelib/       # Library implementation, compiled into sapphirelib.a
│   ├── robot/             # The robot program's implementation (never in the library)
│   └── main.cpp           # PROS competition callbacks, handing off to src/robot/
├── tests/                 # Host-side unit tests for the pure modules (desktop g++)
├── examples/              # Usage examples, compile-checked with `make check-examples`
├── tools/telemetry/       # slt_read.py, the reference reader for SD telemetry logs
├── tools/analyzer/        # Browser telemetry analyzer: diagnostics, match replay, offline tuning
├── docs/                  # Roadmap, setup guide, macros guide, telemetry format
└── .github/               # CI workflow, issue templates
```

Library headers and sources mirror each other by module: `chassis/`, `control/`, `odom/`, `motion/`,
`sensors/`, `input/`, `mechanism/`, `telemetry/`, `tuning/`, `gui/`, `diag/`, `util/`.

## License

MIT — see [LICENSE](LICENSE).

## Team

Team 96671H — Hitmen
