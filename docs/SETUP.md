# Local Setup Guide

SapphireLib's source (`include/`, `src/`) is framework-agnostic PROS library code. To actually build and
flash it to a V5 brain, you need a full PROS **kernel project** underneath it — the kernel template
contains VEX's firmware bindings and isn't something we vendor in this repo.

## 1. Install the PROS CLI

Follow the official install guide: https://pros.cs.purdue.edu/v5/getting-started/1-installation.html

Or via VS Code: install the **PROS extension**, which bundles the CLI.

## 2. Clone this repo

```bash
git clone https://github.com/<your-org>/SapphireLib.git
cd SapphireLib
```

## 3. Pull the PROS kernel template

This repo ships with SapphireLib's own `include/sapphirelib/` and `src/sapphirelib/` folders already in
place. Run:

```bash
pros conduct new . --no-default-libs
```

This fetches the current PROS kernel (firmware, `project.pros`, `Makefile`, `common.mk`, the base
`include/main.h` / `src/main.cpp`) directly into this directory, alongside the existing SapphireLib
folders. `--no-default-libs` skips pulling in okapilib/liblvgl duplicates if we already manage those
ourselves.

> If `pros conduct new .` complains the directory isn't empty, use `pros conduct new . --force` — it will
> not overwrite `include/sapphirelib/` or `src/sapphirelib/` since those are new, unrelated paths.

## 4. Build

```bash
pros make
```

## 5. Upload to a brain

```bash
pros upload
```

## Building the distributable template (release)

The root `Makefile` is configured as a PROS library project (`IS_LIBRARY=1`, `LIBNAME=sapphirelib`),
which enables PROS's built-in templating target. From a repo checkout with the kernel already pulled in
(steps 1–3 above):

```bash
pros make template
```

This compiles `src/sapphirelib/**` into `bin/sapphirelib.a` and runs `pros c create-template` to stage a
`template/` directory containing the public headers (`include/sapphirelib/**/*.hpp`) plus the compiled
archive — this is what a consumer's `pros conduct apply` pulls in, so `src/sapphirelib/**` itself is never
shipped. Zip the contents of the resulting `template/` directory and attach it to a GitHub Release (tag
`v<version>`, matching `SAPPHIRELIB_VERSION` in `include/sapphirelib/version.hpp` and `VERSION` in the
root `Makefile`) so others can download it and run:

```bash
pros conduct fetch path/to/sapphirelib@<version>.zip
pros conduct apply sapphirelib
```

inside their own kernel project. Exact flag names can drift between PROS CLI versions — check
`pros conduct --help` (or `pros c --help`) against what you have installed if a command above doesn't
match.

Since Phase 2+ isn't implemented yet, whatever is currently in `include/sapphirelib/` and
`src/sapphirelib/` at release time — chassis, control, and util only — is the whole template. There's
nothing extra to strip out to get a "drivetrain-only" release; that's just what exists right now.

## Updating the kernel later

```bash
pros conduct info-project   # see current kernel version
pros conduct apply <kernel-version>
```

## Troubleshooting

- **"kernel does not support kernel version None"** — your PROS CLI couldn't reach
  `pros.cs.purdue.edu` to resolve available kernel templates. Check your network/firewall; VEX's template
  server occasionally rate-limits or blocks automated environments (e.g. CI, sandboxed dev containers).
- **VS Code PROS extension** is the easiest path if the CLI gives you trouble — it handles kernel
  resolution through the same backend but with better error surfacing.
