# Contributing to SapphireLib

Internal library for Team 96671H — Hitmen. This guide is for team members working on SapphireLib itself
(not for teams consuming it as a dependency).

## Workflow

1. Branch off `main`: `git checkout -b phase-<n>/<short-description>` (e.g. `phase-1/pid-controller`)
2. Keep PRs scoped to one roadmap item where possible — see `docs/ROADMAP.md` for current phase.
3. Run `clang-format` before committing (`.clang-format` is in the repo root).
4. Open a PR into `main`. CI must pass (build check) before merge.
5. At least one other team member should review before merging — two sets of eyes catches a lot before
   it hits a competition robot.

## Code Style

- Namespace everything under `sapphirelib::`
- Public headers live in `include/sapphirelib/`, implementation in `src/sapphirelib/`, mirroring the same
  subfolder structure
- Prefer explicit, documented public APIs over clever templates — this library needs to be readable by
  teammates joining mid-season
- Doc-comment (Doxygen-style `/** ... */`) all public classes and functions

## Commit Messages

Short, present-tense, scoped to what changed. Reference the roadmap phase where relevant:

```
[phase1] add slew-rate limiting to PID controller
[phase2] implement 2-wheel + IMU odometry math
```

## Testing Changes

Since this is embedded robot code, most testing happens on-bot. When you open a PR touching
motion/control code, note in the PR description:

- What robot/config you tested on
- What sensor configuration was used
- Any tuning constants that changed

## Reporting Issues

Use the issue templates under `.github/ISSUE_TEMPLATE/`. Bug reports should include the sensor/robot
config and, if possible, PROS terminal output.
