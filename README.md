# SapphireLib

Standalone motion & control library for VEX V5 robots, built on [PROS](https://pros.cs.purdue.edu/).
Developed by **Team 96671H — Hitmen**, as the successor to StratagemV2.0.

SapphireLib is a from-scratch rewrite — it does not depend on LemLib or EZ-Template. It's built around a
flexible sensor model, so the same library supports robots with tracking wheels, robots without them, or
any mix in between.

## Status

🚧 Early development (Phase 0 — foundation). Not yet ready for competition use.

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

### Setup (for this repo)

This repository holds the SapphireLib source only. To build/test it against a real kernel:

```bash
# From the repo root, pull the PROS kernel template (requires PROS CLI):
pros conduct new . --no-default-libs

# Then the include/ and src/ directories here layer directly on top of the
# generated kernel project structure.
```

See [`docs/SETUP.md`](docs/SETUP.md) for full local setup instructions.

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
