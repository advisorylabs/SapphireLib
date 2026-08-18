# SapphireLib

Standalone motion & control library for VEX V5 robots, built on [PROS](https://pros.cs.purdue.edu/).
Developed by **Team 96671H — Hitmen**, as the successor to StratagemV2.0.

SapphireLib is a from-scratch rewrite — it does not depend on LemLib or EZ-Template. It's built around a
flexible sensor model, so the same library supports robots with tracking wheels, robots without them, or
any mix in between.

## Status

🚧 Early development — Phase 1 (chassis/drivetrain control) complete and available as the `v0.1.0`
template release. Phase 2+ (odometry, pose-aware motion) is not implemented yet, so this release covers
tank/holonomic drive, driver control, and PID drive/turn only. On-bot PID/tuning is still per-robot work
you'll do after pulling the template in. Not yet ready for competition use.

See [`docs/ROADMAP.md`](docs/ROADMAP.md) for the full phase-by-phase development plan.

## Supported Odometry Configurations

- IMU + drive motor encoders only (no tracking wheels required)
- IMU + single vertical tracking wheel
- IMU + single horizontal tracking wheel
- IMU + vertical + horizontal tracking wheels

## Getting Started

### Prerequisites

- [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/) installed
- A PROS V5 project (kernel template) — SapphireLib is added as a library on top of it, it is not a
  standalone PROS project itself once integrated into a robot repo

### Using SapphireLib in your own robot project (recommended)

Grab the `sapphirelib` template `.zip` from the [Releases](../../releases) page and pull it into your
own PROS kernel project:

```bash
pros conduct fetch path/to/sapphirelib@0.1.0.zip
pros conduct apply sapphirelib
```

Then `#include "sapphirelib/api.hpp"` and build `TankDrivetrain`/`HolonomicDrivetrain` from
`examples/tank_chassis.cpp` / `examples/holonomic_chassis.cpp` as a starting point.

### Building this repo / building the template yourself

This repository holds the SapphireLib source only. To build/test it against a real kernel, or to produce
your own template `.zip`:

```bash
# From the repo root, pull the PROS kernel template (requires PROS CLI):
pros conduct new . --no-default-libs

# Then the include/ and src/ directories here layer directly on top of the
# generated kernel project structure.
```

See [`docs/SETUP.md`](docs/SETUP.md) for full local setup instructions, including how to build the
distributable template `.zip`.

## Project Structure

```
SapphireLib/
├── include/sapphirelib/   # Public library headers
├── src/sapphirelib/       # Library implementation
├── docs/                  # Roadmap, setup guide, API docs (added over time)
├── examples/              # Example autonomous programs / usage
└── .github/               # CI workflows, issue templates
```

## License

MIT — see [LICENSE](LICENSE).

## Team

Team 96671H — Hitmen
