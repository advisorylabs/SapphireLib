# Building a driver macro system

A driver macro system is the code between the controller and a robot's mechanisms: L1 raises the lift
a level, R1 toggles the clamp, one press of A runs "dip, release, stow" on its own while the driver
keeps driving. SapphireLib gives you the building blocks for one, so your robot's file holds only
your ports, your numbers, and your rules.

It deliberately isn't a framework. There's no subsystem base class, scheduler, command group, or
button-callback registry: those add indirection and ordering rules to learn, and remove nothing from
your file. The pattern that works is already the simple one: one function your opcontrol loop calls
every tick, with an `if`/`else if` chain that states your priorities, and each primitive below
replaces one piece of the machinery teams otherwise rebuild by hand around it: new-press bookkeeping,
controller-screen throttling, `now - startMs` timers, a lift loop with its edge cases, a phase state
machine.

[`examples/macros.cpp`](../examples/macros.cpp) is the complete example this guide builds up to, and
it's [at the end](#a-complete-example) too. [`src/robot/macros.cpp`](../src/robot/macros.cpp) is
96671H's real one: three scoring modes, a claw with a distance sensor, and two sequences.

Contents: [The primitives](#the-primitives) · [The per-tick pattern](#the-per-tick-pattern) ·
[Where things live](#where-things-live-namespace-scope) · [Mechanisms](#mechanisms) ·
[Sequences](#sequences) · [Autonomous](#autonomous) · [Telemetry](#telemetry) ·
[A complete example](#a-complete-example)

## The primitives

Everything is in `sapphirelib/api.hpp`.

| Primitive | Header | What it does for you |
|---|---|---|
| `input::Controller` | `input/controller.hpp` | Samples every button and stick once per tick: presses, releases, hold times, auto-repeat, two-button combos, sticks normalized to [-1, 1], "the loop just restarted", and a throttled controller screen and rumble. |
| `mechanism::PositionMechanism` | `mechanism/position_mechanism.hpp` | A lift or arm on a rotation sensor: PID plus gravity feedforward, an optional rest on a hard stop, braking when the sensor is lost, a manual override, and settle detection. Runs from your loop or on its own task. |
| `mechanism::Piston` | `mechanism/piston.hpp` | A pneumatic that knows how long it's been extended or retracted. |
| `mechanism::Roller` | `mechanism/roller.hpp` | An intake or conveyor, with optional anti-jam (a short reverse pulse when it's told to spin but isn't turning). |
| `mechanism::PresetLadder` | `mechanism/preset_ladder.hpp` | Tables of preset positions, stepped a level at a time; one table per scoring mode, say. |
| `Sequence<StepId>` | `util/sequence.hpp` | Timed multi-step actions ("dip, outtake 500ms, go up a level") advanced one tick at a time from driver control, or run to completion from autonomous. |
| `waitUntil()` | `util/wait.hpp` | A blocking wait for autonomous that always has a timeout. |
| `elapsedMs()`, `Stopwatch`, `TimedFlag`, `GapDetector` | `util/timing.hpp` | Wrap-safe timing for anything of your own: "has this been true for 200ms", debouncing, noticing a gap between calls. |
| `applyDeadband()`, `curveJoystick()` | `control/joystick_curve.hpp` | Stick shaping for a manual mechanism control, same as for driving. |

The decision logic in most of these is pure (no PROS, time passed in as an argument) and
unit-tested on a desktop compiler (`tests/input/`, `tests/mechanism/`, `tests/util/`), in several
cases against a copy of the hand-written code it replaced. `Controller`, `PositionMechanism`, `Piston`
and `Roller` are the thin PROS-facing shells.

## The per-tick pattern

```cpp
void updateMacros() {
    const std::uint32_t now = master.now(); // the tick's one "now"

    // The loop was stopped (autonomous, a disable) and has restarted: drop
    // anything whose timing went stale.
    if (master.resumed()) sequence.cancel();

    // One chain, in priority order. Exactly one branch runs each tick.
    if (sequence.active()) {
        sequence.update(now);
    } else if (master.pressed(Button::a) && levels.level() > 0) {
        sequence.start(kScore, now);
    } else {
        if (master.repeated(Button::l1, 400, 150)) levels.up();
        if (master.repeated(Button::l2, 400, 150)) levels.down();
        if (master.pressed(Button::r1)) clamp.toggle(now);
        arm.setTarget(levels.levelPosition());
    }

    // Outputs from state, every tick.
    intake.spin(master.held(Button::r2) ? 12.0 : 0.0, now);

    // The screen last: say what each line should show, then send at most one write.
    master.screen().setLine(0, "ARM %d", levels.level());
    master.flushScreen();
}

void opcontrol() {
    while (true) {
        master.update(); // sample once, at the top
        updateMacros();
        // drive code here, reading the same sample
        pros::delay(20);
    }
}
```

Four rules make this work:

1. **Sample once, at the top of the tick.** `master.update()` reads all twelve buttons, the four
   sticks and the connection state, and stamps the tick's time. Every query after that
   (`pressed()`, `held()`, `heldMs()`, `longPressed()`, `repeated()`, `combo()`) is a pure function of
   this sample and the last one, so it doesn't matter what order you ask in, how often, or whether you
   ask at all. That's the difference from PROS's `get_digital_new_press()`, whose "already seen" flag
   only updates when a button is *read*: a button your code skips for a while fires late, two places
   reading one button steal presses from each other, and `get_digital_new_press(B) &&
   get_digital_new_press(DOWN)` only ever works in one order. Use `master.now()` for everything that
   takes a time that tick; comparing a start-of-tick "now" with a timestamp taken later in the tick is
   how `now - stamp` underflows to about 49 days.
2. **One `else if` chain, in priority order.** It *is* your robot's rules, readable top to bottom:
   here a running sequence owns the arm and clamp, then starting a score, then the manual buttons.
   Because exactly one branch runs, a sequence and the manual buttons can never both command the arm
   in one tick. When `resumed()` is true, cancel sequences and anything else timed across the gap.
3. **Derive outputs from state, every tick.** Recompute what each mechanism should be doing from the
   current state, rather than commanding it once on an edge and hoping nothing interrupts. A missed
   edge or a restart then can't leave a motor stuck on. The primitives are built for this:
   `setTarget()` with the same target doesn't restart settle timing, `Piston::set()` with the same
   value is a no-op that keeps its time-in-state, and `Roller::spin()` *must* run every tick (0V
   included), since jam detection only works while it's fed.
4. **Flush the screen once, at the end.** Set every line's text every tick if you like,
   `flushScreen()` sends at most one write per 60ms, and only for a line that changed, because V5
   controllers silently drop text that arrives faster than about every 50ms. `master.rumble(".")`
   goes through the same queue. Lines are 15 characters.

**Never block in the per-tick function.** `waitUntil()`, `moveTo()` and `runBlocking()` are for
autonomous: called from opcontrol they freeze every other macro and the drive until they return. If
a development shortcut really must run something blocking from opcontrol (96671H's runs the selected
autonomous with B+DOWN, off the field only), stop the mechanisms first; nothing updates them until it
returns, and `resumed()` will report the gap afterwards.

## Where things live: namespace scope

**Declare the `Controller` at namespace scope (or `static`), never as a local in `opcontrol()`.** PROS
deletes and recreates the opcontrol task on every disable and enable. A local `Controller` would start
over each time believing nothing was held, so a button held through a re-enable reads as a fresh press
on the first tick back: holding L1 through a re-enable would raise the lift. (PROS's own new-press
flags don't have this problem because they're kernel globals, not part of a `pros::Controller`.)
Constructing one at namespace scope is safe: it makes no PROS calls until `update()`.

```cpp
sapphirelib::input::Controller master(pros::E_CONTROLLER_MASTER,
                                      {.resumeGapMs = 100, .screenIntervalMs = 60});
```

`resumeGapMs` is how long a gap between `update()`s counts as a restart; keep it comfortably longer than your
loop period (5 × a 20ms tick by default). A `Controller` isn't thread-safe: update it, query it, and
flush it from one task.

The mechanisms belong at namespace scope too. Their constructors only build PROS objects without
commanding any device, so that's safe; a `PositionMechanism` with a running task must never be
destroyed; and `PresetLadder` allocates when it's built, so build it once, never per tick. Sequence
programs have to live there as well, see [Sequences](#sequences).

## Mechanisms

### PositionMechanism

```cpp
pros::Rotation liftSensor(16);

const PositionConfig kLiftConfig{
    .pid = {.gains = {.kP = 0.2, .kD = 0.01},
            .outputLimit = 12.0,
            .derivativeOnMeasurement = true,
            .nominalDtS = 0.02}, // update() every 20ms tick
    .gravity = {.constantVolts = 0.8},
    .seat = {.enabled = true, .floor = 0.0, .seatVolts = 2.0, .restBand = 2.0},
    .tolerance = 15.0,
    .settleTimeMs = 100,
};

PositionMechanism lift({-20, 19}, [] { return readRotationDeg(liftSensor); }, kLiftConfig);
```

- **The position source** returns a reading in the same units as your targets, usually
  `readRotationDeg()`, which gives NaN when the sensor isn't answering. With no reading the mechanism
  brakes and resets its PID instead of driving blind, and `isNear()`/`settled()` are false, so a
  sequence waiting on it falls through to its timeout. (To fall back to a second sensor, return its
  reading when the first gives NaN.) Set the brake mode it uses yourself, in `initialize()`:
  `lift.motors().set_brake_mode_all(pros::v5::MotorBrake::hold)`.
- **`pid`** works in position units in, volts out. `derivativeOnMeasurement` stops a preset change (a
  step in the target) from kicking the output. `nominalDtS` must match how often `update()` really
  runs.
- **`gravity`** is added to the loop's output so the PID only corrects error instead of also holding
  the load up. Without it, a proportional loop settles short of every target. `constantVolts` suits
  an elevator-style lift, whose load is the same at every height: raise it until the lift stops
  settling below its targets. An arm's load falls off as it swings toward vertical, so use
  `cosineVolts` (the volts to hold it level) with `horizontalPosition` (the reading when it's level).
- **`seat`** is for a mechanism that rests on a hard stop at the bottom. With plain PID it settles a
  few units short there, or pushes into the stop forever. With `seat` enabled, a target at or below
  `floor` drives down with at least `seatVolts` until within `restBand` of it, then cuts to 0V and
  rests on the stop.
- **`tolerance`** and **`settleTimeMs`** define `isNear()`/`atTarget()` and `settled()`.
  `resetAfterGapMs` (default 100) resets the PID when `update()` calls are further apart than that, so
  its derivative doesn't kick off a reading from before the gap.

Tune it with the robot on the bench: gravity feedforward first, with `kP` small, then `kP` until it
arrives briskly without oscillating, then `kD` to damp it. `positionLawName(lift.law())` on a
controller line shows which branch is in charge (`track`, `seat`, `rest`, `no sensor`, `manual`,
`off`).

**Manual or task mode.** There are two ways to run it, and the choice matters for autonomous:

| | Manual | Task |
|---|---|---|
| How | call `lift.update(now)` once per tick from your per-tick function | `lift.startTask(10)` once, in `initialize()` |
| `nominalDtS` | your tick: 0.02 for a 20ms opcontrol loop | the task period: 0.01 for `startTask(10)` |
| During autonomous | nothing drives it | keeps holding while autonomous blocks on drive motions |
| `update()` | drives it | ignored, with a one-time warning (two loops on one motor group fight) |
| While disabled | nothing runs | brakes with its PID cleared, so nothing winds up while VEXos ignores the motors |

Manual mode is the smallest change from a hand-written loop (96671H's lift works this way). But
nothing drives the mechanism while your loop isn't running, and its motors keep whatever was last
sent, so stop it explicitly before anything blocks the loop. Use task mode whenever autonomous moves
the mechanism.

In both modes, commands and queries are safe from any task: they go through lock-free atomics,
never a mutex, so a competition task PROS deletes mid-call can't leave a lock held. `setTarget()`
for closed loop; `setVolts()` for a manual override; `holdPosition()` to go back to closed loop on
wherever it is right now, without a jump; `stop()` to brake. A manual override is usually "open loop
while held, then hold where the driver left it until they pick a preset again":

```cpp
if (master.pressed(Button::l1)) {
    levels.up();
    manualHold = false;
}
if (master.held(Button::up)) {
    lift.setVolts(6.0); // open loop while held
    manualHold = true;
} else if (master.released(Button::up)) {
    lift.holdPosition(); // closed loop wherever it stopped: no jump
} else if (!manualHold) {
    lift.setTarget(levels.levelPosition());
}
```

### Piston

```cpp
Piston clamp('A');                                   // or {smart port, 'A'} on a 3-wire expander
clamp.set(master.held(Button::r1), now);             // level-triggered: fine every tick
const bool clampReady = clamp.extendedFor(400, now); // it's had 400ms to close
```

`extendedAtStart` and `extendedIsLow` (for a valve plumbed the other way round) are constructor
arguments. Only drive the piston through the class, or its time-in-state goes wrong.

### Roller

```cpp
Roller intake({11, -12}, {.enabled = true, .stallMs = 300});
intake.spin(volts, now); // every tick, 0V included
```

With anti-jam enabled, a roller commanded at `minCommandVolts` (4V) or more that turns slower than
`stallRpm` (5RPM) for `stallMs` (250ms) gets `reverseVolts` (6V) the other way for `reverseMs`
(200ms), then goes back to the command; a piece still stuck gets another pulse `stallMs` later.
`stallMs` must be longer than the roller takes to spin up, or every start looks like a jam. Letting
go of the button (or reversing) ends a pulse at once. `jammed()` is true while it's reversing, handy
for one short rumble as each jam starts.

### PresetLadder

```cpp
PresetLadder levels({
    {"ALLIANCE", {0, 150, 300, 450}},
    {"CENTER", {0, 225, 450}},
});
if (master.pressed(Button::l1)) levels.up();          // stops at the top
if (master.pressed(Button::right)) levels.nextTable(); // keeps the level, capped
lift.setTarget(levels.levelPosition());
```

`midpointBelow(level)` answers "halfway to the level below", a common way to score. It holds numbers
only; feeding them to a mechanism is your code's job. It's the most optional primitive here: if your
modes and levels read better as plain code, keep them there.

## Sequences

A `Sequence` answers "which step of this timed action are we in, and is it time to move on?" one
tick at a time, so driver control never blocks.

```cpp
enum class Phase { dip, outtake };

const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{
    {.id = Phase::dip,
     .until = [] { return lift.isNear(levels.midpointBelow(levels.level())); },
     .timeoutMs = 1000},
    {.id = Phase::outtake, .timeoutMs = 500},
}};
const Sequence<Phase>::Program kScore{.steps = kScoreSteps, .onFinish = [] { levels.up(); }};

Sequence<Phase> sequence;
```

- A step exits on the first `update()` where its `until` is true, or once `timeoutMs` has passed
  since it was entered (0 = no limit). `until` is checked first, so arriving on the tick the timeout
  runs out still counts as arriving. A step with only a timeout is a fixed-length step. **Give every
  step that waits on a sensor a timeout**, so an unplugged sensor costs a second rather than the rest
  of the match.
- `update()` makes **at most one transition per call**: a step entered on this call gets its first
  chance to exit on the next, so every step lasts at least one tick, and its timeout counts from the
  `now` it was entered with.
- `onEnter` runs once as a step begins; `onFinish` runs after the last step exits, once the sequence
  is already idle, so it may start the next program. `cancel()` stops without running `onFinish`, and
  `start()` replaces whatever was running.
- **Steps and programs are referenced, not copied**, so declare them at namespace scope, as above.
  Passing a temporary program to `start()` is a compile error.
- For a step with an `until`, `lastExitWasTimeout()` tells a real arrival from a give-up, which is worth an
  event in the log. (A fixed-length step always exits on its timeout, so check which step it was.)

There are two ways for steps to act on the robot, and they mix freely:

- **Level-triggered**: the rest of your tick computes each output from the step, e.g. the claw
  outtakes whenever `sequence.is(Phase::outtake)`. This fits when your per-tick function already
  derives every output from state; 96671H's macros work this way.
- **`onEnter` commands**: each step commands its mechanism once as it begins
  (`.onEnter = [] { arm.setTarget(...); }`). The mechanism has to be in task mode to follow through,
  but then the same program also runs from autonomous with `runBlocking()`, where there's no per-tick
  function to compute outputs. The [complete example](#a-complete-example) does this.

## Autonomous

With a mechanism in task mode, autonomous can command it and move on, or wait for it:

```cpp
lift.setTarget(levels.levelPosition(2));                              // start it; don't wait
sapphirelib::waitUntil([] { return lift.position() >= 200.0; }, 1000); // wait for part of the way
const bool arrived = lift.moveTo(levels.levelPosition(3), 1500);       // go, and wait to settle
sequence.runBlocking(kScore, 3000);                                    // the driver's macro, whole
```

- `moveTo()` is `setTarget()` then `waitUntilSettled()`; both return false on timeout. A timeout of 0
  means no limit, not something to use in a match.
- `waitUntil(condition, timeoutMs, pollMs = 10)` blocks until the condition is true or the time is up,
  and always has a timeout. The condition is checked before the timeout, so one that comes true right
  at the deadline counts.
- `runBlocking()` starts a program and updates it until it finishes, giving up (and cancelling) after
  its timeout. Anything its steps wait on must keep running meanwhile, like a mechanism on its own task.
- `waitUntilSettled()` also works in manual mode, running `update()` itself while it waits, but only
  from the task that owns `update()`, and once it returns nothing drives the mechanism. If autonomous
  uses a mechanism, give it a task.

All of these are safe when PROS ends autonomous mid-wait: nothing holds a lock. The first
`master.update()` in opcontrol then reports `resumed()`, which is your cue to cancel sequences.

## Telemetry

With a [`telemetry::Logger`](TELEMETRY_FORMAT.md) running, a mechanism logs with two calls in
`initialize()`, before `startTask()`, since the step listener belongs to whichever task runs
`update()`:

```cpp
logger.pid("lift", lift.pid()); // every PID step, plus its gains whenever they change

// What the motors were actually sent: the "lift" channel's output is only the
// loop's share, before gravity feedforward and the final clamp.
sapphirelib::telemetry::Channel& act =
    logger.channel("lift.act", {"target", "pos", "volts", "law"});
lift.setStepListener([channel = &act](const PositionStep& step) {
    channel->record({step.target, step.position, step.volts, static_cast<double>(step.law)});
});
```

The listener runs inside every `update()` and reuses that update's sensor read; `record()` never
blocks or allocates, so it's safe on a control loop. For the moments a tuning session wants to find
again (a macro firing, a sequence giving up) log an event where it happens:

```cpp
if (sequence.active()) {
    // Only the dip step waits on something (its `until`); a fixed-length
    // step like the outtake always ends on its timeout, so that's no give-up.
    const bool waitingOnLift = sequence.is(Phase::dip);
    if (sequence.update(now) != SequenceUpdate::running && waitingOnLift &&
        sequence.lastExitWasTimeout()) {
        sapphirelib::telemetry::event("seq", "gave up waiting for the lift");
    }
}
```

Events are formatted on the calling task, so they're for moments, not per-tick data (use a channel
for that). [`examples/telemetry.cpp`](../examples/telemetry.cpp) sets up the logger itself.

## A complete example

A hypothetical robot with:

- an arm on two motors, with a rotation sensor on its pivot, held up against gravity by cosine
  feedforward and resting on a hard stop at the bottom, running on its own task;
- a clamp on a pneumatic piston;
- an intake roller that briefly reverses itself when it jams.

| Button | Does |
|---|---|
| L1 / L2 | Arm up / down one preset (hold to repeat) |
| R1 | Close / open the clamp |
| R2 (hold) | Intake in |
| Y (hold) | Intake out |
| A | Score, with the arm above STOW: dip the arm, open the clamp, then stow the arm |

This is [`examples/macros.cpp`](../examples/macros.cpp), which `make check-examples` compiles against
the current headers; if the two ever disagree, trust the file.

```cpp
#include <array>
#include <cstdint>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::Sequence;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
using sapphirelib::mechanism::Piston;
using sapphirelib::mechanism::PositionConfig;
using sapphirelib::mechanism::positionLawName;
using sapphirelib::mechanism::PositionMechanism;
using sapphirelib::mechanism::PresetLadder;
using sapphirelib::mechanism::readRotationDeg;
using sapphirelib::mechanism::Roller;

namespace {

// --- Ports ---
// Negate the sensor port if its reading goes down as the arm goes up.
constexpr std::int8_t kArmSensorPort = 4;
constexpr char kClampPort = 'A';

// --- Numbers to tune ---
// How far below its preset the arm dips to score, in degrees.
constexpr double kScoreDipDeg = 20.0;
// How long the clamp stays open before the arm stows.
constexpr std::uint32_t kScoreReleaseMs = 300;
constexpr double kIntakeVolts = 12.0;

// At namespace scope, never a local in opcontrol(): its button history has to
// survive opcontrol() being restarted, or a button held through a disable and
// re-enable reads as a fresh press (see input::Controller).
Controller master(pros::E_CONTROLLER_MASTER);

pros::Rotation armSensor(kArmSensorPort);

// Arm presets, in degrees on its rotation sensor, zeroed with the arm resting
// on its bottom stop: STOW, LOW, HIGH.
PresetLadder armLevels({{"ARM", {0.0, 60.0, 135.0}}});

const PositionConfig kArmConfig{
    .pid = {.gains = {.kP = 0.15, .kD = 0.005},
            .outputLimit = 12.0,
            // Preset changes step the target, which would otherwise kick the
            // output.
            .derivativeOnMeasurement = true,
            // arm.startTask(10) in initialize().
            .nominalDtS = 0.01},
    // The sensor reads 90 with the arm horizontal, where gravity pulls
    // hardest: 1.5V there, tapering to none with the arm hanging straight down.
    .gravity = {.cosineVolts = 1.5, .horizontalPosition = 90.0},
    // STOW sits on the bottom stop: drive down onto it, then rest at 0V
    // instead of pushing into it.
    .seat = {.enabled = true, .floor = 0.0, .seatVolts = 2.0, .restBand = 3.0},
    // Within 3 degrees for 100ms counts as settled.
    .tolerance = 3.0,
    .settleTimeMs = 100,
};

PositionMechanism arm({7, -8}, [] { return readRotationDeg(armSensor); }, kArmConfig);

// Extended = clamped shut.
Piston clamp(kClampPort);

Roller intake({11, -12}, {.enabled = true});

// Score: dip below the current preset, open the clamp, then stow. Each step
// commands its mechanism once, from onEnter, and the arm's own task does the
// rest. So the same program runs from opcontrol() (one update() per tick) and
// from autonomous() (runBlocking()).
enum class ScoreStep { dip, release, stow };

const std::array<Sequence<ScoreStep>::Step, 3> kScoreSteps{{
    {.id = ScoreStep::dip,
     .until = [] { return arm.settled(); },
     .timeoutMs = 800,
     .onEnter = [] { arm.setTarget(armLevels.levelPosition() - kScoreDipDeg); }},
    {.id = ScoreStep::release,
     .timeoutMs = kScoreReleaseMs,
     // onEnter isn't handed the tick's "now"; a later clock reading is harmless
     // here, since elapsed time never goes negative (util/timing.hpp).
     .onEnter = [] { clamp.set(false, sapphirelib::millis()); }},
    {.id = ScoreStep::stow,
     .until = [] { return arm.settled(); },
     .timeoutMs = 1000,
     .onEnter =
         [] {
             armLevels.setLevel(0);
             arm.setTarget(armLevels.levelPosition());
         }},
}};
// Referenced by the sequence, not copied, so it lives at namespace scope too.
const Sequence<ScoreStep>::Program kScore{.steps = kScoreSteps};

Sequence<ScoreStep> sequence;

bool wasJammed = false;

/// One driver-control tick, after master.update().
void updateMacros() {
    const std::uint32_t now = master.now();

    // After autonomous, a disable, or anything else that stopped this loop, a
    // half-finished sequence's timing is stale: drop it. (The arm needs
    // nothing here: its own task never stopped.)
    if (master.resumed()) sequence.cancel();

    // One chain, in priority order: a running sequence owns the arm and clamp,
    // then starting a score, then the manual buttons.
    if (sequence.active()) {
        sequence.update(now);
    } else if (master.pressed(Button::a) && armLevels.level() > 0) {
        sequence.start(kScore, now);
    } else {
        if (master.repeated(Button::l1, 400, 150)) armLevels.up();
        if (master.repeated(Button::l2, 400, 150)) armLevels.down();
        if (master.pressed(Button::r1)) clamp.toggle(now);
        // Setting the same target every tick is fine: only a new value restarts
        // settle timing.
        arm.setTarget(armLevels.levelPosition());
    }

    // The intake belongs to the driver whatever else is going on. spin() runs
    // every tick, 0V included, because jam detection only works while fed.
    double intakeVolts = 0.0;
    if (master.held(Button::r2)) {
        intakeVolts = kIntakeVolts;
    } else if (master.held(Button::y)) {
        intakeVolts = -kIntakeVolts;
    }
    intake.spin(intakeVolts, now);

    // One short buzz as each jam starts, not one per tick while it clears.
    const bool jammed = intake.jammed();
    if (jammed && !wasJammed) master.rumble(".");
    wasJammed = jammed;

    // Say what each line should show every tick; flushScreen() sends only what
    // changed, at most one write per 60ms.
    master.screen().setLine(0, "ARM %d %s", armLevels.level(), positionLawName(arm.law()));
    master.screen().setLine(1, "CLAMP %s", clamp.extended() ? "CLOSED" : "OPEN");
    master.screen().setLine(2, "%s", sequence.active() ? "SCORING" : jammed ? "INTAKE JAM" : "");
    master.flushScreen();
}

} // namespace

void initialize() {
    sapphirelib::initialize();

    // With the arm resting on its bottom stop: every preset is measured up
    // from here.
    armSensor.reset_position();
    // What the arm does whenever it brakes: sensor lost, stop(), and while
    // disabled.
    arm.motors().set_brake_mode_all(pros::v5::MotorBrake::hold);
    // The arm runs its own 10ms loop from here on (matching pid.nominalDtS),
    // so it keeps holding while autonomous blocks on drivetrain motions.
    arm.startTask(10);
}

void disabled() {
    // VEXos ignores motor commands while disabled; this just makes sure the
    // intake doesn't pick up where it left off when the robot is enabled again.
    intake.stop();
}

void autonomous() {
    clamp.set(true, sapphirelib::millis()); // grab the preload

    // Start the arm rising without waiting for it: its task keeps driving it
    // while autonomous moves on (to a drivetrain motion, say).
    armLevels.setLevel(1);
    arm.setTarget(armLevels.levelPosition());
    // ...drive to the goal here...

    // Block until it clears the goal's rim. Every wait has a timeout, so an
    // unplugged sensor costs a second rather than the rest of the match.
    sapphirelib::waitUntil([] { return arm.position() >= 45.0; }, 1000);

    // Go to a preset and wait for it to settle there, in one call. False if it
    // hadn't settled within 1.5s.
    armLevels.setLevel(2);
    arm.moveTo(armLevels.levelPosition(), 1500);

    // The same program the A button runs, run to completion.
    sequence.runBlocking(kScore, 3000);
}

void opcontrol() {
    while (true) {
        master.update(); // sample the controller once, at the top of the tick
        updateMacros();
        // Drive code goes here, reading the same sample: for example
        // drivetrain.arcade(master.axis(Axis::leftY), master.axis(Axis::rightX)).
        pros::delay(20);
    }
}
```
