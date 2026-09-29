# Sapphire Telemetry — the telemetry analyzer

A browser app for the robot's SD-card logs: what went wrong in a match, a replay of it, charts of
anything logged, and tuning from the log. Plain HTML and JavaScript — no install, no server, no
build step, and it works offline (the only thing it fetches is a web font, and it falls back to a
system font without one). Logs never leave the computer.

## Opening logs

1. Copy the `sl` folder off the robot's SD card (files `SL000001.CSV`, `SL000002.CSV`, ...).
2. Open `index.html` in Chrome, Edge, Firefox or Safari — double-click it.
3. Drop the files on the page, or use **Open logs**. Several at once is fine.

It opens on a demo (a simulated match and a pit session), marked as such, so every view can be
explored without a robot. **Demo** brings it back.

Each file is one program run (a run the card faulted in the middle of is merged back together).
The picker at the top chooses the run, then a match or practice session in it — **Whole run**
shows everything. A match is an autonomous followed by driver control under competition control;
the clock reads in match time (`auton 0:12.4`, `driver 1:03.0`).

To hand the analyzer to someone as one file: `node tools/analyzer/build.js` writes
`tools/analyzer/dist/sapphire-telemetry.html`, with everything inlined.

## The tabs

- **Overview** — findings, worst first, each with a **Replay** and a **Chart** button that jump to
  the moment: motors overheating (and how far the V5's self-protection has cut their current),
  disconnecting, stalling or losing speed per volt; the lift stuck at full power or sagging below
  its targets; battery sag; the controller dropping out; sensors coming unplugged; autonomous
  motions that timed out (and whether they were still pushing); PID loops oscillating or stalling;
  SD faults and dropped rows. Below them, a temperature strip for every motor and a motor table.
- **Replay** — the match played back: the robot on the field (pose, trail, the current motion's
  target), a side view of the lift (target, position, control law, volts), a tile per motor
  (temperature, current, derating, unplugged), the battery, the driver's sticks and buttons, and
  "happening now", over synced charts. Findings are marked on the timeline; with it focused, Space
  plays and pauses and the arrow keys step (Shift for bigger steps).
- **Charts** — any column of any channel, stacked on one time axis. Presets for motor
  temperatures and current, the lift, each PID, the battery, the driver and odometry. Drag across
  a chart to zoom, double-click to reset, Ctrl+wheel to zoom, click to set the cursor; the table
  underneath shows every plotted value at the cursor, and the visible range copies out as CSV.
- **PID responses** — every step response of every PID in range: the motion it belonged to, its
  first and last error, length, time saturated, swings, and the gains it ran with. Select one to
  chart its error and its P, I and D terms.
- **Motions** — every drivetrain motion with its target, result, final error and time; select one
  to see its path on the field and its PIDs' error against the exit threshold.
- **Tune** — tuning from the log, below.

## Tuning from a log

The analyzer can't drive the robot, and doesn't need to: a log already has what tuning needs.

1. **Model.** Pick a system (the drivetrain's Fwd, Strafe and Turn axes, and the lift) and a source:
   - an **Auto-Tune run** — the robot's PID page logs every characterization it runs (`char.*`
     channels). The analyzer refits it with a port of the robot's own math and shows the robot's
     numbers next to its own; they agree, up to the rounding of the logged values.
   - **driving in a match or practice** — the volts each system was sent, next to where it went,
     fitted the same way. No extra runs needed; it needs some variety (speeding up, slowing down,
     both directions). For the lift, choose how gravity loads it (constant for an elevator, cosine
     for an arm).

   "Does the model match?" plays the recorded volts through the model against where the system
   really went.
2. **Controller.** Each controller is designed from the model the way the robot designs it: a
   settle time, a damping ratio, and a minimum phase margin in, kP and kD out, backed off for the
   measured delay. Or type your own gains.
3. **What if.** The log's real targets — the lift's presets through the match, each autonomous
   turn — are replayed through the model with the gains the robot had, the designed ones, and
   yours, with how long each takes to settle, how far it overshoots, and where it ends up.
4. **Paste.** The C++ for the chosen design, naming where each line goes. Flash it, run, and open
   the next log to check.

For a fresh measurement, select the controller on the robot's PID page and tap Auto-Tune with the
SD card in (the lift's measures only the lift). The robot applies its own result too; the analyzer
is for checking it, comparing runs, and trying other specs without the robot.

## What it reads

The [SLT v1 format](../../docs/TELEMETRY_FORMAT.md). Channels are recognized by name and columns,
so another robot's logs work as long as they use the library's own:

| Needs | For |
|---|---|
| `pid` channels (any name) | PID responses; `drive`/`turn`/`hold` also for tuning and motions |
| samples with `volts,amps,temp,rpm` (`Logger::motor()`) | motor health, derating, disconnects |
| `<name>.act` with `target,pos,volts,law` | a mechanism: its replay, sag, stuck, and tuning (with the `<name>` PID) |
| `odom` (`x,y,heading`) | the field replay, motions' paths, odometry jumps |
| `chassis` (`fwd_v,strafe_v,turn_v`) with `odom` | fitting the drivetrain from driving |
| `char.fwd`, `char.strafe`, `char.turn`, `char.<mechanism>` | Auto-Tune refits |
| `batt`, `driver`, `mech` | battery, controller, and the macros' state |

## Code and tests

`js/` holds one file per concern, loaded in order by `index.html` as plain scripts (each also
loads under Node where it can, for the tests):

| File | What |
|---|---|
| `model.js` | Ports of the robot's math: the characterization fits, gain design, `PID`, the position-control law, gravity feedforward, thermal derating; a plant simulator |
| `slt.js` | The SLT parser, run merging, and slicing: phases, matches, motions, Auto-Tune runs |
| `analysis.js` | The findings and per-system summaries |
| `tuning.js` | Offline tuning: fits from logs, designs, what-if replays, C++ snippets |
| `demo.js` | The demo: a simulated robot writing SLT logs |
| `charts.js`, `ui.js` | Canvas time charts (shared cursor and zoom), DOM helpers |
| `app.js`, `replay.js`, `explorer.js`, `tuneview.js` | The page and its tabs |

```
node --test tools/analyzer/test/*.test.js
```

runs the tests (Node 20 or later; CI runs them). They check the ports against the C++ unit tests'
golden values, the parser against the format doc's worked example, the findings against the demo
match's known faults, and the fits against the demo's known physics. List the files — `node --test
<directory>` doesn't find them.

Team 96671H — Hitmen
