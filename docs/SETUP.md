# Local Setup Guide

SapphireLib's source (`include/sapphirelib/`, `src/sapphirelib/`) is framework-agnostic PROS library
code. To actually build and flash it to a V5 brain, it needs a full PROS **kernel project** underneath
it. This repo already is one: the PROS kernel (4.2.2) and liblvgl (9.2.0) are merged into the repo
root, and 96671H's own robot program (`src/main.cpp`, `src/robot/`, `include/robot/`) sits on top of
the library.

## 1. Install the PROS CLI

Follow the official install guide: https://pros.cs.purdue.edu/v5/getting-started/1-installation.html

Or via VS Code: install the **PROS extension**, which bundles the CLI and the ARM toolchain.

## 2. Clone this repo

```bash
git clone https://github.com/advisorylabs/SapphireLib.git
cd SapphireLib
```

The default branch is `master`.

## 3. Check the kernel files are there

There's nothing to pull: `common.mk`, `project.pros`, `include/pros/`, `include/liblvgl/` and the
kernel's `firmware/` archives are all committed. The one exception is `firmware/liblvgl.a`, which
`.gitignore`'s `firmware/*.a` rule catches (the kernel's own archives were committed before that rule
existed). If a fresh clone fails to link with undefined `lv_...` symbols, re-apply the liblvgl template
`project.pros` records (`pros conduct apply liblvgl@9.2.0`), or copy the file from a teammate's
checkout.

## 4. Build: from the PROS toolchain

**Always build with the ARM toolchain that ships with PROS** (currently GCC 14.3.1): from VS Code's
PROS integrated terminal, or a shell with the PROS toolchain's `usr/bin` first on `PATH`. Check with:

```bash
arm-none-eabi-g++ -dumpfullversion   # should print the PROS toolchain's version, e.g. 14.3.1
```

Bare `make` from a shell where some other `arm-none-eabi-g++` comes first on `PATH` (a package
manager's GCC 15, say) mixes objects from two compiler versions into one image. It links without
complaint and then faults on the brain before the screen paints anything, which looks like a dead
program rather than a build problem. The `Makefile` guards against this: it stamps the compiler
version into `bin/` and refuses to build with a different one. If you hit that error, switch back to
the PROS toolchain, or run `make clean` to rebuild everything with the new one on purpose.

```bash
pros make
```

To check that the examples still compile against the current headers (they aren't part of the
program, so nothing else would notice them going stale), run this after changing any public header:

```bash
make check-examples
```

The library's pure modules also have host-side unit tests that build with any desktop `g++`, see
[`tests/README.md`](../tests/README.md). CI runs them on every push.

## 5. Upload to a brain

```bash
pros upload
```

## Building the distributable template (release)

The root `Makefile` is configured as a PROS library project (`IS_LIBRARY=1`, `LIBNAME=sapphirelib`),
which enables PROS's built-in templating target. From a repo checkout (steps 1–3 above):

```bash
pros make template
```

This compiles `src/sapphirelib/**` into `bin/sapphirelib.a` and runs `pros c create-template` to stage a
`template/` directory containing the public headers (`include/sapphirelib/**/*.hpp`) plus the compiled
archive; this is what a consumer's `pros conduct apply` pulls in, so `src/sapphirelib/**` itself is never
shipped. Zip the contents of the resulting `template/` directory and attach it to a GitHub Release (tag
`v<version>`, matching `SAPPHIRELIB_VERSION` in `include/sapphirelib/version.hpp` and `VERSION` in the
root `Makefile`) so others can download it and run:

```bash
pros conduct fetch path/to/sapphirelib@<version>.zip
pros conduct apply sapphirelib
```

inside their own kernel project. Exact flag names can drift between PROS CLI versions; check
`pros conduct --help` (or `pros c --help`) against what you have installed if a command above doesn't
match.

The template is every module under `include/sapphirelib/` (chassis, control, odom, motion, sensors,
input, mechanism, telemetry, tuning, gui, diag and util) and nothing else. The robot program never
ships: the `Makefile` keeps everything under `src/` except `src/sapphirelib/` out of the library
archive (an allowlist, so a new robot file can't slip in by accident), and only
`include/sapphirelib/` headers are template files, so `include/robot/` stays behind too. A consumer's
project needs liblvgl, which the GUI is built on; PROS 4's default project already has it.

## Updating the kernel later

```bash
pros conduct info-project   # see current kernel version
pros conduct apply <kernel-version>
```

## Troubleshooting

- **"bin/ was built with arm-none-eabi-g++ X but Y is first on PATH"**: the toolchain guard from step
  4. Build from the PROS toolchain, or `make clean` if you meant to switch.
- **Builds and uploads, then the screen stays black**: if it isn't your own code hanging in
  `initialize()`, suspect a mixed-toolchain image: the guard can only compare against a stamp, and a
  `bin/` built before the guard existed has none. `make clean`, then rebuild from the PROS toolchain.
- **"kernel does not support kernel version None"**: your PROS CLI couldn't reach
  `pros.cs.purdue.edu` to resolve available kernel templates. Check your network/firewall; VEX's template
  server occasionally rate-limits or blocks automated environments (e.g. CI, sandboxed dev containers).
- **VS Code PROS extension** is the easiest path if the CLI gives you trouble; it handles kernel
  resolution through the same backend but with better error surfacing.
