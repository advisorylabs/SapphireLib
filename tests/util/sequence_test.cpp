// Host-side unit test for sapphirelib::Sequence — no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// util/sequence.hpp is header-only. runBlocking() reads time through
// util/clock.hpp, which this test defines itself as a fake clock whose
// delayMs() just advances the fake time.
//
// testMatchesTheOriginalPhaseMachine() is the proof that the driver macros'
// score and re-seat sequences behave identically on Sequence: it runs a
// verbatim copy of robot_macros.cpp's hand-written enterPhase()/advancePhase()
// machine (and the parts of update() around it) side by side with the same
// logic ported onto Sequence, over thousands of random driver traces, and
// requires the same phase, level, piston, claw, intake, and lift target on
// every tick.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/util/sequence_test.cpp -o sequence_test && ./sequence_test

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/sequence.hpp"
#include "sapphirelib/util/timing.hpp"

namespace {

std::uint32_t fakeNowMs = 1000;
std::vector<std::uint32_t> delays; // every delayMs() argument, in order

void resetClock(std::uint32_t nowMs) {
    fakeNowMs = nowMs;
    delays.clear();
}

} // namespace

namespace sapphirelib {

std::uint32_t millis() { return fakeNowMs; }
std::uint64_t micros() { return std::uint64_t{fakeNowMs} * 1000; }
void delayMs(std::uint32_t ms) {
    delays.push_back(ms);
    fakeNowMs += ms;
}

} // namespace sapphirelib

using sapphirelib::Sequence;
using sapphirelib::SequenceUpdate;

namespace {

enum class Step { a, b, c };
using Seq = Sequence<Step>;

bool always() { return true; }

// --- Programs are referenced, so temporaries must not compile ---------------

template <typename S>
concept CanStartTemporary = requires(S sequence) { sequence.start(typename S::Program{}, 0u); };

template <typename S>
concept CanRunTemporary = requires(S sequence) { sequence.runBlocking(typename S::Program{}); };

template <typename S>
concept CanStartNamed = requires(S sequence, const typename S::Program& program) {
    sequence.start(program, 0u);
    sequence.runBlocking(program);
};

static_assert(!CanStartTemporary<Seq>);
static_assert(!CanRunTemporary<Seq>);
static_assert(CanStartNamed<Seq>);

// --- Unit behavior ----------------------------------------------------------

void testQueriesWhileIdle() {
    Seq sequence;
    assert(!sequence.active());
    assert(!sequence.current().has_value());
    assert(!sequence.is(Step::a));
    assert(sequence.msInStep(1234) == 0);
    assert(!sequence.lastExitWasTimeout());
    assert(sequence.update(1234) == SequenceUpdate::idle); // harmless
}

void testOneTransitionPerUpdate() {
    // Every step's exit is already met, yet each update() moves one step.
    const std::array<Seq::Step, 3> steps{{
        {.id = Step::a, .until = always},
        {.id = Step::b, .until = always},
        {.id = Step::c, .until = always},
    }};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    sequence.start(program, 0);
    assert(sequence.active() && sequence.is(Step::a) && sequence.stepIndex() == 0);
    assert(sequence.update(0) == SequenceUpdate::advanced);
    assert(sequence.is(Step::b) && sequence.stepIndex() == 1);
    assert(sequence.update(0) == SequenceUpdate::advanced);
    assert(sequence.current() == Step::c && sequence.stepIndex() == 2);
    assert(sequence.update(0) == SequenceUpdate::finished);
    assert(!sequence.active());
    assert(sequence.update(0) == SequenceUpdate::idle);
}

void testStepTimerStartsAtTheTransition() {
    const std::array<Seq::Step, 2> steps{{
        {.id = Step::a, .timeoutMs = 100},
        {.id = Step::b, .timeoutMs = 50},
    }};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    sequence.start(program, 1000);
    assert(sequence.msInStep(1000) == 0);
    assert(sequence.update(1099) == SequenceUpdate::running);
    assert(sequence.msInStep(1099) == 99);
    assert(sequence.update(1100) == SequenceUpdate::advanced); // `>=`: exactly 100
    assert(sequence.msInStep(1100) == 0);
    assert(sequence.update(1149) == SequenceUpdate::running);
    assert(sequence.update(1150) == SequenceUpdate::finished);

    // An update that comes late starts the next step's clock at its own
    // `now`, not at the deadline the previous step had.
    sequence.start(program, 2000);
    assert(sequence.update(2130) == SequenceUpdate::advanced);
    assert(sequence.update(2179) == SequenceUpdate::running);
    assert(sequence.update(2180) == SequenceUpdate::finished);
}

void testUntilIsCheckedFirstAndNotOnTheEnteringTick() {
    int aChecks = 0;
    int bChecks = 0;
    bool aArrived = false;
    const std::array<Seq::Step, 2> steps{{
        {.id = Step::a,
         .until =
             [&] {
                 ++aChecks;
                 return aArrived;
             },
         .timeoutMs = 100},
        {.id = Step::b,
         .until =
             [&] {
                 ++bChecks;
                 return false;
             },
         .timeoutMs = 100},
    }};
    const Seq::Program program{.steps = steps};
    Seq sequence;

    sequence.start(program, 0);
    assert(aChecks == 0); // start() doesn't evaluate the step it enters
    assert(sequence.update(50) == SequenceUpdate::running);
    assert(aChecks == 1);

    // The condition comes true on the same tick the timeout runs out: it
    // counts as arrived, not timed out.
    aArrived = true;
    assert(sequence.update(100) == SequenceUpdate::advanced);
    assert(aChecks == 2);
    assert(!sequence.lastExitWasTimeout());
    assert(bChecks == 0); // b was entered on that update, so not checked yet

    assert(sequence.update(150) == SequenceUpdate::running);
    assert(bChecks == 1);
    // Still asked on the tick its timeout ends it — once per update.
    assert(sequence.update(200) == SequenceUpdate::finished);
    assert(bChecks == 2);
    assert(sequence.lastExitWasTimeout());
}

void testFixedDurationStepsExitByTimeout() {
    const std::array<Seq::Step, 1> steps{{{.id = Step::a, .timeoutMs = 10}}};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    sequence.start(program, 0);
    assert(sequence.update(10) == SequenceUpdate::finished);
    assert(sequence.lastExitWasTimeout());

    // A new start() clears it.
    sequence.start(program, 20);
    assert(!sequence.lastExitWasTimeout());
}

void testStepWithNoExitNeverEndsOnItsOwn() {
    const std::array<Seq::Step, 1> steps{{{.id = Step::a}}};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    sequence.start(program, 0);
    for (std::uint32_t now = 0; now < 100000; now += 20) {
        assert(sequence.update(now) == SequenceUpdate::running);
    }
    sequence.cancel();
    assert(!sequence.active());
}

void testOnEnterSeesTheStepItsEntering() {
    Seq sequence;
    std::vector<std::string> events;
    const std::array<Seq::Step, 2> steps{{
        {.id = Step::a,
         .timeoutMs = 10,
         .onEnter =
             [&] {
                 // Index and start time are already set when onEnter runs.
                 assert(sequence.is(Step::a) && sequence.stepIndex() == 0);
                 assert(sequence.msInStep(500) == 0);
                 events.push_back("enter a");
             }},
        {.id = Step::b,
         .timeoutMs = 10,
         .onEnter =
             [&] {
                 assert(sequence.is(Step::b) && sequence.stepIndex() == 1);
                 assert(sequence.msInStep(510) == 0);
                 events.push_back("enter b");
             }},
    }};
    const Seq::Program program{.steps = steps, .onFinish = [&] { events.push_back("finish"); }};

    sequence.start(program, 500);
    assert(events == std::vector<std::string>{"enter a"}); // during start() itself
    sequence.update(505);
    assert(events.size() == 1);
    sequence.update(510);
    assert((events == std::vector<std::string>{"enter a", "enter b"}));
    sequence.update(520);
    assert((events == std::vector<std::string>{"enter a", "enter b", "finish"}));
}

void testOnFinishRunsAfterGoingIdleAndMayChain() {
    Seq sequence;
    const std::array<Seq::Step, 1> secondSteps{{{.id = Step::b, .timeoutMs = 10}}};
    const Seq::Program second{.steps = secondSteps};
    bool wasIdleInOnFinish = false;
    const std::array<Seq::Step, 1> firstSteps{{{.id = Step::a, .timeoutMs = 10}}};
    const Seq::Program first{.steps = firstSteps, .onFinish = [&] {
                                 wasIdleInOnFinish = !sequence.active();
                                 sequence.start(second, 10);
                             }};

    sequence.start(first, 0);
    // The first program finished (that's what update() reports), and its
    // onFinish chained the second, which is now running.
    assert(sequence.update(10) == SequenceUpdate::finished);
    assert(wasIdleInOnFinish);
    assert(sequence.running(second) && sequence.is(Step::b));
    assert(sequence.update(19) == SequenceUpdate::running);
    assert(sequence.update(20) == SequenceUpdate::finished);
    assert(!sequence.active());
}

void testCancelSkipsOnFinish() {
    int finishes = 0;
    const std::array<Seq::Step, 1> steps{{{.id = Step::a, .timeoutMs = 10}}};
    const Seq::Program program{.steps = steps, .onFinish = [&] { ++finishes; }};
    Seq sequence;
    sequence.start(program, 0);
    sequence.cancel();
    assert(!sequence.active());
    assert(sequence.update(100) == SequenceUpdate::idle);
    assert(finishes == 0);
}

void testStartReplacesWithoutFinishing() {
    int firstFinishes = 0;
    const std::array<Seq::Step, 1> firstSteps{{{.id = Step::a, .timeoutMs = 10}}};
    const Seq::Program first{.steps = firstSteps, .onFinish = [&] { ++firstFinishes; }};
    const std::array<Seq::Step, 1> secondSteps{{{.id = Step::c, .timeoutMs = 30}}};
    const Seq::Program second{.steps = secondSteps};
    Seq sequence;
    sequence.start(first, 0);
    sequence.update(10); // first finishes normally
    assert(firstFinishes == 1 && sequence.lastExitWasTimeout());

    sequence.start(first, 20);
    sequence.start(second, 25); // replaces first mid-step
    assert(firstFinishes == 1);
    assert(sequence.running(second) && !sequence.running(first));
    assert(sequence.is(Step::c));
    assert(!sequence.lastExitWasTimeout()); // reset by start()
    assert(sequence.msInStep(25) == 0);     // timed from the new start
    assert(sequence.update(54) == SequenceUpdate::running);
    assert(sequence.update(55) == SequenceUpdate::finished);
}

void testEmptyProgramFinishesAtOnce() {
    int finishes = 0;
    const Seq::Program empty{.steps = {}, .onFinish = [&] { ++finishes; }};
    Seq sequence;
    sequence.start(empty, 0);
    assert(finishes == 1);
    assert(!sequence.active() && !sequence.current().has_value());
    assert(sequence.update(10) == SequenceUpdate::idle);
}

void testRunningComparesIdentity() {
    // Same steps, different Program objects: running() is about which one.
    const std::array<Seq::Step, 1> steps{{{.id = Step::a}}};
    const Seq::Program one{.steps = steps};
    const Seq::Program two{.steps = steps};
    Seq sequence;
    sequence.start(one, 0);
    assert(sequence.running(one));
    assert(!sequence.running(two));
    assert(sequence.is(Step::a));
}

void testCallbacksMayCancelOrBranch() {
    Seq sequence;
    const std::array<Seq::Step, 2> abortSteps{{
        {.id = Step::a, .until = always},
        {.id = Step::b, .onEnter = [&] { sequence.cancel(); }},
    }};
    const Seq::Program abort{.steps = abortSteps};
    sequence.start(abort, 0);
    // It did advance (b's onEnter ran), and b's onEnter then cancelled.
    assert(sequence.update(0) == SequenceUpdate::advanced);
    assert(!sequence.active());

    const std::array<Seq::Step, 1> otherSteps{{{.id = Step::c}}};
    const Seq::Program other{.steps = otherSteps};
    const std::array<Seq::Step, 2> branchSteps{{
        {.id = Step::a, .onEnter = [&] { sequence.start(other, 7); }},
        {.id = Step::b},
    }};
    const Seq::Program branch{.steps = branchSteps};
    sequence.start(branch, 0);
    assert(sequence.running(other) && sequence.is(Step::c));
}

void testTimeoutAcrossTheClockWrap() {
    const std::array<Seq::Step, 1> steps{{{.id = Step::a, .timeoutMs = 100}}};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    sequence.start(program, 0xFFFFFFF0u);
    assert(sequence.update(0x53u) == SequenceUpdate::running); // 99ms in
    assert(sequence.msInStep(0x53u) == 99);
    assert(sequence.update(0x54u) == SequenceUpdate::finished);
}

void testStaleNowDoesNotEndAStep() {
    // A `now` from before the step began counts as 0ms in, not ~49 days.
    const std::array<Seq::Step, 1> steps{{{.id = Step::a, .timeoutMs = 100}}};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    sequence.start(program, 1000);
    assert(sequence.update(990) == SequenceUpdate::running);
    assert(sequence.msInStep(990) == 0);
}

// --- runBlocking(), on the fake clock -----------------------------------------

void testRunBlockingFinishes() {
    const std::array<Seq::Step, 2> steps{{
        {.id = Step::a, .timeoutMs = 30},
        {.id = Step::b, .timeoutMs = 20},
    }};
    int finishes = 0;
    const Seq::Program program{.steps = steps, .onFinish = [&] { ++finishes; }};
    Seq sequence;
    resetClock(1000);
    assert(sequence.runBlocking(program));
    // a: polls at 1010, 1020, 1030 (exits); b from 1030: 1040, 1050 (exits).
    assert(fakeNowMs == 1050);
    assert(delays.size() == 5);
    for (std::uint32_t ms : delays) assert(ms == 10);
    assert(finishes == 1);
    assert(!sequence.active());
}

void testRunBlockingWaitsOnACondition() {
    // As if the lift, on its own task, arrives at t = 1075.
    const std::array<Seq::Step, 1> steps{{
        {.id = Step::a, .until = [] { return fakeNowMs >= 1075; }, .timeoutMs = 1000},
    }};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    resetClock(1000);
    assert(sequence.runBlocking(program, 0, 25));
    assert(fakeNowMs == 1075);
    assert(delays.size() == 3);
    assert(!sequence.lastExitWasTimeout());
}

void testRunBlockingTimesOutAndCancels() {
    int finishes = 0;
    const std::array<Seq::Step, 1> steps{{{.id = Step::a}}}; // never exits
    const Seq::Program program{.steps = steps, .onFinish = [&] { ++finishes; }};
    Seq sequence;
    resetClock(1000);
    assert(!sequence.runBlocking(program, 100, 10));
    assert(fakeNowMs == 1100); // `>=`: gives up at exactly the timeout
    assert(delays.size() == 10);
    assert(!sequence.active()); // cancelled, not left running
    assert(finishes == 0);
}

void testRunBlockingFinishingAtTheDeadlineCounts() {
    const std::array<Seq::Step, 1> steps{{{.id = Step::a, .timeoutMs = 100}}};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    resetClock(1000);
    assert(sequence.runBlocking(program, 100, 10)); // finishes on the timeout's tick
    assert(fakeNowMs == 1100);
}

void testRunBlockingReturnsFalseWhenACallbackCancelsOrReplaces() {
    Seq sequence;
    const std::array<Seq::Step, 2> abortSteps{{
        {.id = Step::a, .timeoutMs = 10},
        {.id = Step::b, .onEnter = [&] { sequence.cancel(); }},
    }};
    const Seq::Program abort{.steps = abortSteps};
    resetClock(1000);
    assert(!sequence.runBlocking(abort, 1000));
    assert(fakeNowMs == 1010); // returned as soon as it was cancelled

    const std::array<Seq::Step, 1> otherSteps{{{.id = Step::c}}};
    const Seq::Program other{.steps = otherSteps};
    const std::array<Seq::Step, 2> branchSteps{{
        {.id = Step::a, .timeoutMs = 10},
        {.id = Step::b, .onEnter = [&] { sequence.start(other, fakeNowMs); }},
    }};
    const Seq::Program branch{.steps = branchSteps};
    resetClock(1000);
    assert(!sequence.runBlocking(branch, 1000));
    assert(sequence.running(other)); // the program it branched to keeps going
}

void testRunBlockingEmptyProgram() {
    int finishes = 0;
    const Seq::Program empty{.steps = {}, .onFinish = [&] { ++finishes; }};
    Seq sequence;
    resetClock(1000);
    assert(sequence.runBlocking(empty, 100));
    assert(finishes == 1);
    assert(delays.empty());
}

void testRunBlockingZeroPollStillSleeps() {
    const std::array<Seq::Step, 1> steps{{{.id = Step::a, .timeoutMs = 3}}};
    const Seq::Program program{.steps = steps};
    Seq sequence;
    resetClock(1000);
    assert(sequence.runBlocking(program, 0, 0));
    assert(delays.size() == 3);
    for (std::uint32_t ms : delays) assert(ms == 1);
}

// --- Reference equivalence with robot_macros.cpp's phase machine -------------

/// One opcontrol tick's worth of inputs, the same for both machines: the
/// controller's buttons (sampled once), the lift's rotation sensor, and the
/// claw's distance sensor.
struct TickInput {
    std::uint32_t now = 0;
    bool intaking = false;         // R1 held
    bool deployedIntaking = false; // Y held
    bool scoreHeld = false;        // R2 held
    bool scorePressed = false;     // R2 new press
    bool upPressed = false;        // L1 new press
    bool downPressed = false;      // L2 new press
    bool modePressed = false;      // RIGHT new press
    double liftDeg = 0.0;          // NaN = sensor not answering
    bool pieceInClaw = false;
};

/// The phases the port's Sequence steps are named by (the original's
/// Phase::none is "no sequence running").
enum class Phase { scoreDescend, scoreOuttake, reseatLower, reseatRetract, reseatDeploy };

/// Everything one tick decided that the phase machine can affect.
struct TickOutput {
    std::optional<Phase> phase;
    int level = 0;
    std::size_t mode = 0;
    bool clawDeployed = false;
    bool deployedIntake = false;
    bool bottomOuttake = false;
    bool pistonExtended = false;
    std::int32_t clawMv = 0;
    std::int32_t intakeMv = 0;
    double liftTargetDeg = 0.0;
    bool restarted = false; // dropped stale state after a gap
    int arrivalChecks = 0;  // liftArrived() calls this tick

    bool operator==(const TickOutput& other) const {
        return phase == other.phase && level == other.level && mode == other.mode &&
               clawDeployed == other.clawDeployed && deployedIntake == other.deployedIntake &&
               bottomOuttake == other.bottomOuttake && pistonExtended == other.pistonExtended &&
               clawMv == other.clawMv && intakeMv == other.intakeMv &&
               std::bit_cast<std::uint64_t>(liftTargetDeg) ==
                   std::bit_cast<std::uint64_t>(other.liftTargetDeg) &&
               restarted == other.restarted && arrivalChecks == other.arrivalChecks;
    }
};

namespace original {

// Copied verbatim from src/robot_macros.cpp before the port (git show
// 73dc252:src/robot_macros.cpp): the constants and state the phase machine
// uses, enterPhase(), advancePhase(), the helpers they call, and update()
// from its first line down to the lift command. Only the device I/O is
// swapped for the test's inputs and outputs: the controller reads come from
// TickInput, the sensors from the tick's readings, the piston is a stand-in
// with the same is_extended/extend/retract, and the motor commands are
// recorded instead of sent (each swapped line is marked "test I/O"). Left
// out: the LEFT-button screen-view toggle and showStatus(), which only
// change what the controller screen shows (tests/input/ covers the screen),
// and driveLift()'s motor math (tests/mechanism/ covers the lift law).

TickInput tick;         // test I/O: this tick's inputs
TickOutput out;         // test I/O: what this tick commanded
std::uint32_t millis() { return tick.now; } // test I/O

struct Pneumatics { // test I/O: pros::adi::Pneumatics's piston state
    bool extended = false;
    bool is_extended() const { return extended; }
    void extend() { extended = true; }
    void retract() { extended = false; }
};

// --- Timing ---
// How long the claw piston gets to deploy before anything outtakes past it.
constexpr std::uint32_t kPistonDeployMs = 400;
// How long the claw outtakes when scoring above level 0.
constexpr std::uint32_t kHeightOuttakeMs = 500;
// The re-seat after releasing Y, once the lift is back down at level 0:
// piston retracted for this long, then redeployed for this long before the
// claw goes back to holding on its own.
constexpr std::uint32_t kReseatRetractMs = 250;
constexpr std::uint32_t kReseatDeployMs = 250;

// --- Speeds, in millivolts (12000 = full) ---
constexpr std::int32_t kIntakeMv = 12700;
constexpr std::int32_t kClawMv = 12700;  // intaking and outtaking
constexpr std::int32_t kClawHoldMv = 12700;

// How close the lift must get before a sequence waiting on it moves on (a
// score above level 0 waiting to outtake, or a re-seat waiting for level 0),
// and how long it waits for that before moving on anyway.
constexpr double kLiftToleranceDeg = 15.0;
constexpr std::uint32_t kLiftDescendTimeoutMs = 1000;

struct ScoringMode {
    const char* name;  // shown on the controller; 8 characters max
    // Lift target at each level, in degrees of the lift's rotation sensor up
    // from where it was zeroed. Entry 0 is level 0 (claw piston deployed,
    // lift down); each entry after it is one L1 notch, so the entry count
    // sets how many levels the mode has. Spacing needn't be even: a score
    // above level 0 dips halfway to the level below, however big that gap is.
    std::vector<double> levelDeg;
};

const std::array<ScoringMode, 3> kModes{{
    {"ALLIANCE", {0, 150, 300, 450, 600, 750, 900}},
    {"MEDIUM", {0, 180, 360, 540, 720, 900}},
    {"CENTER", {0, 225, 450, 675, 900}},
}};

// update() runs every opcontrol tick (20ms); a longer gap than this means
// opcontrol() stopped and restarted in between.
constexpr std::uint32_t kRestartGapMs = 100;

Pneumatics clawPiston; // test I/O: was pros::adi::Pneumatics clawPiston(kClawPistonPort, false)

enum class Phase { none, scoreDescend, scoreOuttake, reseatLower, reseatRetract, reseatDeploy };

struct State {
    std::size_t mode = 0;  // index into kModes
    // false = stowed: piston retracted, lift at level 0. Only R1 stows.
    bool clawDeployed = false;
    int level = 0;
    std::uint32_t deployedAtMs = 0;  // when the piston last extended
    bool bottomOuttake = false;  // R2 pressed at level 0 and still held
    bool deployedIntake = false;  // Y held (and not overridden by R1)
    Phase phase = Phase::none;
    std::uint32_t phaseStartMs = 0;
    bool showLiftMotors = false;  // controller lines 2-3 show lift motor readings

    std::array<std::string, 3> shownLines;  // what each controller line last showed
    std::uint32_t lastPrintMs = 0;
    std::uint32_t lastUpdateMs = 0;
};

State state;

const ScoringMode& currentMode() { return kModes[state.mode]; }

int maxLevel() { return static_cast<int>(currentMode().levelDeg.size()) - 1; }

void enterPhase(Phase phase, std::uint32_t now) {
    state.phase = phase;
    state.phaseStartMs = now;
}

void setPiston(bool extended, std::uint32_t now) {
    if (extended == clawPiston.is_extended()) return;
    if (extended) {
        clawPiston.extend();
        state.deployedAtMs = now;
    } else {
        clawPiston.retract();
    }
}

bool pieceInClaw() {
    return tick.pieceInClaw; // test I/O: was clawSensor.get_distance() <= kPieceDetectMm
}

/// NaN if the rotation sensor isn't answering.
double liftPositionDeg() {
    return tick.liftDeg; // test I/O: was liftSensor.get_position() / 100.0, or NaN on PROS_ERR
}

/// Halfway between `level` and the level below it.
double halfwayBelow(int level) {
    const std::vector<double>& levels = currentMode().levelDeg;
    return (levels[level] + levels[level - 1]) / 2.0;
}

double liftTargetDeg() {
    if (state.deployedIntake) return halfwayBelow(1);
    if (state.phase == Phase::scoreDescend || state.phase == Phase::scoreOuttake) {
        return halfwayBelow(state.level);
    }
    return currentMode().levelDeg[state.level];
}

/// Never true while the rotation sensor is unplugged (its position reads
/// NaN), so a sequence waiting on this falls through to its timeout.
bool liftArrived() { // test I/O: split over three lines to count the calls
    ++out.arrivalChecks; // test I/O
    return std::abs(liftPositionDeg() - liftTargetDeg()) <= kLiftToleranceDeg;
}

void advancePhase(std::uint32_t now) {
    const std::uint32_t elapsedMs = now - state.phaseStartMs;
    switch (state.phase) {
        case Phase::scoreDescend:
            if (liftArrived() || elapsedMs >= kLiftDescendTimeoutMs) {
                enterPhase(Phase::scoreOuttake, now);
            }
            break;
        case Phase::scoreOuttake:
            if (elapsedMs >= kHeightOuttakeMs) {
                state.phase = Phase::none;
                state.level = std::min(state.level + 1, maxLevel());
            }
            break;
        case Phase::reseatLower:
            if (liftArrived() || elapsedMs >= kLiftDescendTimeoutMs) {
                enterPhase(Phase::reseatRetract, now);
            }
            break;
        case Phase::reseatRetract:
            if (elapsedMs >= kReseatRetractMs) enterPhase(Phase::reseatDeploy, now);
            break;
        case Phase::reseatDeploy:
            if (elapsedMs >= kReseatDeployMs) state.phase = Phase::none;
            break;
        case Phase::none:
            break;
    }
}

void update() { // test I/O: was update(pros::Controller& controller)
    const std::uint32_t now = millis(); // test I/O: was pros::millis()
    // After autonomous, a disable, or a blocking routine run from opcontrol:
    // drop any half-finished sequence (its timing is stale), start the lift
    // loop fresh, and resend the controller text, which may have been lost.
    if (now - state.lastUpdateMs > kRestartGapMs) {
        state.phase = Phase::none;
        out.restarted = true; // test I/O: was liftPid.reset()
        for (std::string& line : state.shownLines) line.clear();
    }
    state.lastUpdateMs = now;

    // test I/O: the controller reads (get_digital / get_digital_new_press),
    // from the tick's one sample.
    const bool intaking = tick.intaking;                 // test I/O
    const bool deployedIntaking = tick.deployedIntaking; // test I/O
    const bool scoreHeld = tick.scoreHeld;               // test I/O
    const bool scorePressed = tick.scorePressed;         // test I/O
    const bool upPressed = tick.upPressed;               // test I/O
    const bool downPressed = tick.downPressed;           // test I/O
    const bool modePressed = tick.modePressed;           // test I/O

    if (modePressed) {
        state.mode = (state.mode + 1) % kModes.size();
        // The lift keeps its level in the new mode (moving to that mode's
        // height for it), capped if the new mode has fewer levels.
        state.level = std::min(state.level, maxLevel());
        // TODO: LED library's mode-change call goes here.
    }

    if (intaking) {
        // Resets to stowed from any state, cancelling whatever was in progress.
        state.phase = Phase::none;
        state.bottomOuttake = false;
        state.deployedIntake = false;
        state.clawDeployed = false;
        state.level = 0;
    } else if (deployedIntaking) {
        // The same reset, but with the claw deployed, and the lift up halfway
        // to level 1 (see liftTargetDeg()) until Y is released.
        state.phase = Phase::none;
        state.bottomOuttake = false;
        state.deployedIntake = true;
        state.clawDeployed = true;
        state.level = 0;
    } else if (state.deployedIntake) {
        // Y was just released: lower to level 0 and re-seat the piece before
        // the claw goes back to holding it on its own.
        state.deployedIntake = false;
        enterPhase(Phase::reseatLower, now);
    } else if (state.phase != Phase::none) {
        advancePhase(now);
    } else {
        if (upPressed) {
            if (!state.clawDeployed) {
                state.clawDeployed = true;
            } else if (state.level < maxLevel()) {
                ++state.level;
            }
        }
        // Never retracts the piston; only intaking does.
        if (downPressed && state.level > 0) --state.level;
        if (scorePressed) {
            if (state.level == 0) {
                state.clawDeployed = true;
                state.bottomOuttake = true;
            } else {
                enterPhase(Phase::scoreDescend, now);
            }
        }
    }
    if (!scoreHeld) state.bottomOuttake = false;

    const bool reseating = state.phase == Phase::reseatLower ||
                           state.phase == Phase::reseatRetract ||
                           state.phase == Phase::reseatDeploy;
    setPiston(state.clawDeployed && state.phase != Phase::reseatRetract, now);

    // Waits out the rest of the piston's deploy time, which is none at all if
    // it deployed long ago.
    const bool bottomOuttaking =
        state.bottomOuttake && now - state.deployedAtMs >= kPistonDeployMs;

    // Claw: outtaking beats intaking (R1, Y, or re-seating), which beats
    // holding a detected piece.
    std::int32_t clawMv = 0;
    if (bottomOuttaking || state.phase == Phase::scoreOuttake) {
        clawMv = -kClawMv;
    } else if (intaking || state.deployedIntake || reseating) {
        clawMv = kClawMv;
    } else if (pieceInClaw()) {
        clawMv = kClawHoldMv;
    }
    out.clawMv = clawMv; // test I/O: was claw.move_voltage(clawMv)

    // Intake: runs in only for R1 (Y intakes with the claw alone), and
    // reverses with the claw only for a level-0 score. Above level 0 the
    // piece leaves from up on the lift, so the intake stays off.
    std::int32_t intakeMv = 0;
    if (intaking) {
        intakeMv = kIntakeMv;
    } else if (bottomOuttaking) {
        intakeMv = -kIntakeMv;
    }
    out.intakeMv = intakeMv; // test I/O: was intake.move_voltage(intakeMv)

    out.liftTargetDeg = liftTargetDeg(); // test I/O: was driveLift(liftTargetDeg())
}

// --- End of the copy; the harness that drives it ---

void reset() {
    state = State{};
    clawPiston = Pneumatics{};
}

TickOutput run(const TickInput& input) {
    tick = input;
    out = TickOutput{};
    update();
    switch (state.phase) {
        case Phase::none: out.phase = std::nullopt; break;
        case Phase::scoreDescend: out.phase = ::Phase::scoreDescend; break;
        case Phase::scoreOuttake: out.phase = ::Phase::scoreOuttake; break;
        case Phase::reseatLower: out.phase = ::Phase::reseatLower; break;
        case Phase::reseatRetract: out.phase = ::Phase::reseatRetract; break;
        case Phase::reseatDeploy: out.phase = ::Phase::reseatDeploy; break;
    }
    out.level = state.level;
    out.mode = state.mode;
    out.clawDeployed = state.clawDeployed;
    out.deployedIntake = state.deployedIntake;
    out.bottomOuttake = state.bottomOuttake;
    out.pistonExtended = clawPiston.is_extended();
    return out;
}

} // namespace original

namespace ported {

// The same rules on the library's primitives, as the port writes them:
// Sequence for the phases, a GapDetector for the restart check (what
// input::Controller::resumed() reports), a TimedFlag-backed piston (what
// mechanism::Piston is), and a preset ladder for the modes and levels. The
// ladder and piston here are minimal stand-ins with the same arithmetic as
// mechanism::PresetLadder and mechanism::Piston, so this test depends only
// on util/.

using sapphirelib::GapDetector;
using sapphirelib::TimedFlag;

TickInput tick;
TickOutput out;

constexpr std::uint32_t kPistonDeployMs = 400;
constexpr std::uint32_t kHeightOuttakeMs = 500;
constexpr std::uint32_t kReseatRetractMs = 250;
constexpr std::uint32_t kReseatDeployMs = 250;
constexpr std::int32_t kIntakeMv = 12700;
constexpr std::int32_t kClawMv = 12700;
constexpr std::int32_t kClawHoldMv = 12700;
constexpr double kLiftToleranceDeg = 15.0;
constexpr std::uint32_t kLiftDescendTimeoutMs = 1000;

/// mechanism::PresetLadder's arithmetic: level changes clamp, table changes
/// wrap and cap the level.
class Ladder {
public:
    explicit Ladder(std::vector<std::vector<double>> tables) : tables_(std::move(tables)) {}

    void nextTable() {
        table_ = (table_ + 1) % tables_.size();
        level_ = std::min(level_, maxLevel());
    }
    std::size_t tableIndex() const { return table_; }
    int level() const { return level_; }
    int maxLevel() const { return static_cast<int>(tables_[table_].size()) - 1; }
    void setLevel(int level) { level_ = std::clamp(level, 0, maxLevel()); }
    bool up() {
        if (level_ >= maxLevel()) return false;
        ++level_;
        return true;
    }
    bool down() {
        if (level_ <= 0) return false;
        --level_;
        return true;
    }
    double levelPosition() const { return tables_[table_][level_]; }
    double levelPosition(int level) const {
        return tables_[table_][std::clamp(level, 0, maxLevel())];
    }
    double midpointBelow(int level) const {
        if (level <= 0) return levelPosition(0);
        return (levelPosition(level) + levelPosition(level - 1)) / 2.0;
    }

private:
    std::vector<std::vector<double>> tables_;
    std::size_t table_ = 0;
    int level_ = 0;
};

/// mechanism::Piston's bookkeeping: the commanded state and when it last
/// changed.
class Piston {
public:
    bool set(bool extended, std::uint32_t nowMs) { return state_.set(extended, nowMs); }
    bool extended() const { return state_.value(); }
    bool extendedFor(std::uint32_t ms, std::uint32_t nowMs) const {
        return state_.trueFor(ms, nowMs);
    }

private:
    TimedFlag state_{false};
};

Ladder makeLadder() {
    return Ladder({
        {0, 150, 300, 450, 600, 750, 900},
        {0, 180, 360, 540, 720, 900},
        {0, 225, 450, 675, 900},
    });
}

Ladder ladder = makeLadder();
Piston clawPiston;
GapDetector gap(100);

bool liftArrived();

// The programs exactly as the port declares them.
const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{
    {.id = Phase::scoreDescend, .until = liftArrived, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::scoreOuttake, .timeoutMs = kHeightOuttakeMs},
}};
const Sequence<Phase>::Program kScore{.steps = kScoreSteps, .onFinish = [] { ladder.up(); }};

const std::array<Sequence<Phase>::Step, 3> kReseatSteps{{
    {.id = Phase::reseatLower, .until = liftArrived, .timeoutMs = kLiftDescendTimeoutMs},
    {.id = Phase::reseatRetract, .timeoutMs = kReseatRetractMs},
    {.id = Phase::reseatDeploy, .timeoutMs = kReseatDeployMs},
}};
const Sequence<Phase>::Program kReseat{.steps = kReseatSteps};

Sequence<Phase> sequence;

struct State {
    bool clawDeployed = false;
    bool bottomOuttake = false;
    bool deployedIntake = false;
};

State state;

double liftTargetDeg() {
    if (state.deployedIntake) return ladder.midpointBelow(1);
    if (sequence.running(kScore)) return ladder.midpointBelow(ladder.level());
    return ladder.levelPosition();
}

/// What PositionMechanism::isNear() answers: false with no reading.
bool liftArrived() {
    ++out.arrivalChecks;
    return std::fabs(tick.liftDeg - liftTargetDeg()) <= kLiftToleranceDeg;
}

void update() {
    const std::uint32_t now = tick.now;
    if (gap.update(now)) { // controller.resumed()
        sequence.cancel();
        out.restarted = true; // lift.resetController()
    }

    const bool intaking = tick.intaking;
    const bool deployedIntaking = tick.deployedIntaking;
    const bool scoreHeld = tick.scoreHeld;
    const bool scorePressed = tick.scorePressed;

    if (tick.modePressed) ladder.nextTable();

    if (intaking) {
        sequence.cancel();
        state.bottomOuttake = false;
        state.deployedIntake = false;
        state.clawDeployed = false;
        ladder.setLevel(0);
    } else if (deployedIntaking) {
        sequence.cancel();
        state.bottomOuttake = false;
        state.deployedIntake = true;
        state.clawDeployed = true;
        ladder.setLevel(0);
    } else if (state.deployedIntake) {
        state.deployedIntake = false;
        sequence.start(kReseat, now);
    } else if (sequence.active()) {
        sequence.update(now);
    } else {
        if (tick.upPressed) {
            if (!state.clawDeployed) {
                state.clawDeployed = true;
            } else {
                ladder.up();
            }
        }
        if (tick.downPressed) ladder.down();
        if (scorePressed) {
            if (ladder.level() == 0) {
                state.clawDeployed = true;
                state.bottomOuttake = true;
            } else {
                sequence.start(kScore, now);
            }
        }
    }
    if (!scoreHeld) state.bottomOuttake = false;

    const bool reseating = sequence.running(kReseat);
    clawPiston.set(state.clawDeployed && !sequence.is(Phase::reseatRetract), now);

    const bool bottomOuttaking =
        state.bottomOuttake && clawPiston.extendedFor(kPistonDeployMs, now);

    std::int32_t clawMv = 0;
    if (bottomOuttaking || sequence.is(Phase::scoreOuttake)) {
        clawMv = -kClawMv;
    } else if (intaking || state.deployedIntake || reseating) {
        clawMv = kClawMv;
    } else if (tick.pieceInClaw) {
        clawMv = kClawHoldMv;
    }
    out.clawMv = clawMv;

    std::int32_t intakeMv = 0;
    if (intaking) {
        intakeMv = kIntakeMv;
    } else if (bottomOuttaking) {
        intakeMv = -kIntakeMv;
    }
    out.intakeMv = intakeMv;

    out.liftTargetDeg = liftTargetDeg();
}

void reset() {
    ladder = makeLadder();
    clawPiston = Piston{};
    gap = GapDetector(100);
    sequence = Sequence<Phase>{};
    state = State{};
}

TickOutput run(const TickInput& input) {
    tick = input;
    out = TickOutput{};
    update();
    out.phase = sequence.current();
    out.level = ladder.level();
    out.mode = ladder.tableIndex();
    out.clawDeployed = state.clawDeployed;
    out.deployedIntake = state.deployedIntake;
    out.bottomOuttake = state.bottomOuttake;
    out.pistonExtended = clawPiston.extended();
    return out;
}

} // namespace ported

/// A button held for random stretches: `pressChance` per tick of going down
/// while up, `releaseChance` of coming up while down.
struct RandomButton {
    double pressChance;
    double releaseChance;
    bool held = false;

    /// Advances one tick; true on a new press (down now, up last tick).
    bool step(std::mt19937& rng) {
        const bool wasHeld = held;
        const double roll = std::uniform_real_distribution<double>(0.0, 1.0)(rng);
        held = wasHeld ? roll >= releaseChance : roll < pressChance;
        return held && !wasHeld;
    }
};

const char* phaseName(const std::optional<Phase>& phase) {
    if (!phase) return "none";
    switch (*phase) {
        case Phase::scoreDescend: return "scoreDescend";
        case Phase::scoreOuttake: return "scoreOuttake";
        case Phase::reseatLower: return "reseatLower";
        case Phase::reseatRetract: return "reseatRetract";
        case Phase::reseatDeploy: return "reseatDeploy";
    }
    return "?";
}

void printOutput(const char* label, const TickOutput& o) {
    std::printf("  %s: phase=%s level=%d mode=%zu claw=%d deployedIntake=%d bottomOuttake=%d "
                "piston=%d clawMv=%d intakeMv=%d target=%.17g restarted=%d arrivalChecks=%d\n",
                label, phaseName(o.phase), o.level, o.mode, o.clawDeployed, o.deployedIntake,
                o.bottomOuttake, o.pistonExtended, static_cast<int>(o.clawMv),
                static_cast<int>(o.intakeMv), o.liftTargetDeg, o.restarted, o.arrivalChecks);
}

void testMatchesTheOriginalPhaseMachine() {
    std::mt19937 rng(96671);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    // How often each thing happened across all traces, so the comparison is
    // known to have covered it.
    int ticksIn[5] = {};
    int scoresFinished = 0;
    int reseatsFinished = 0;
    int arrivals = 0;
    int descendTimeouts = 0;
    int restarts = 0;
    int bottomOuttakes = 0;

    for (int trace = 0; trace < 10000; ++trace) {
        original::reset();
        ported::reset();

        RandomButton r1{.pressChance = 0.004, .releaseChance = 0.3};
        RandomButton y{.pressChance = 0.015, .releaseChance = 0.06};
        RandomButton r2{.pressChance = 0.06, .releaseChance = 0.25};
        RandomButton l1{.pressChance = 0.06, .releaseChance = 0.4};
        RandomButton l2{.pressChance = 0.02, .releaseChance = 0.4};
        RandomButton right{.pressChance = 0.01, .releaseChance = 0.5};
        bool piece = false;
        // The lift itself: working (tracking its target, arriving or not),
        // stuck somewhere, or unplugged, for stretches long enough that both
        // arriving and timing out happen.
        enum class Lift { working, stuck, unplugged } lift = Lift::working;
        double stuckDeg = 0.0;

        // Mostly a normal program clock; one trace in ten starts close enough
        // to the 32-bit wrap to run across it.
        const std::uint32_t startOffset = static_cast<std::uint32_t>(rng() % 20000);
        std::uint32_t now = unit(rng) < 0.1 ? 0xFFFFFFFFu - startOffset : startOffset / 4;
        double lastTarget = 0.0;
        const int ticks = 100 + static_cast<int>(rng() % 700);
        for (int t = 0; t < ticks; ++t) {
            // 20ms ticks with scheduler jitter, and now and then a gap
            // (autonomous, a disable, a blocking routine) — including right at
            // the 100ms restart threshold.
            const double gapRoll = unit(rng);
            now += gapRoll < 0.005   ? 100u + static_cast<std::uint32_t>(rng() % 2)
                   : gapRoll < 0.012 ? 100u + static_cast<std::uint32_t>(rng() % 2901)
                                     : 15u + static_cast<std::uint32_t>(rng() % 11);

            TickInput input;
            input.now = now;
            r1.step(rng);
            y.step(rng);
            input.intaking = r1.held;
            input.deployedIntaking = y.held;
            input.scorePressed = r2.step(rng);
            input.scoreHeld = r2.held;
            input.upPressed = l1.step(rng);
            input.downPressed = l2.step(rng);
            input.modePressed = right.step(rng);
            if (unit(rng) < 0.1) piece = !piece;
            input.pieceInClaw = piece;
            if (unit(rng) < 0.03) {
                const double liftRoll = unit(rng);
                lift = liftRoll < 0.6    ? Lift::working
                       : liftRoll < 0.85 ? Lift::stuck
                                         : Lift::unplugged;
                stuckDeg = 950.0 * unit(rng);
            }
            switch (lift) {
                case Lift::working: input.liftDeg = lastTarget + 50.0 * unit(rng) - 25.0; break;
                case Lift::stuck: input.liftDeg = stuckDeg; break;
                case Lift::unplugged:
                    input.liftDeg = std::numeric_limits<double>::quiet_NaN();
                    break;
            }

            const bool wasScoring = ported::sequence.running(ported::kScore);
            const bool wasReseating = ported::sequence.running(ported::kReseat);
            const bool wasDescending =
                ported::sequence.is(Phase::scoreDescend) || ported::sequence.is(Phase::reseatLower);

            const TickOutput expected = original::run(input);
            const TickOutput actual = ported::run(input);

            // The one intended difference: a GapDetector reports a restart on
            // its very first call, while the original only did once the clock
            // had passed 100ms. On the first tick there's nothing to drop, so
            // it changes no output — which the rest of the comparison checks.
            TickOutput comparable = actual;
            if (t == 0) comparable.restarted = expected.restarted;

            if (!(comparable == expected)) {
                std::printf("FAIL trace %d tick %d (now=%u):\n", trace, t, now);
                printOutput("original", expected);
                printOutput("ported  ", actual);
                assert(false);
            }

            lastTarget = expected.liftTargetDeg;
            if (expected.phase) ++ticksIn[static_cast<int>(*expected.phase)];
            if (wasScoring && !actual.phase && !input.intaking && !input.deployedIntaking &&
                !actual.restarted) {
                ++scoresFinished;
            }
            if (wasReseating && !actual.phase && !input.intaking && !input.deployedIntaking &&
                !actual.restarted) {
                ++reseatsFinished;
            }
            // Left a lift-waiting step this tick: by arriving, or by giving up.
            if (wasDescending && actual.phase &&
                (*actual.phase == Phase::scoreOuttake || *actual.phase == Phase::reseatRetract)) {
                ++(ported::sequence.lastExitWasTimeout() ? descendTimeouts : arrivals);
            }
            if (t > 0 && actual.restarted) ++restarts;
            if (actual.intakeMv < 0) ++bottomOuttakes;
        }
    }

    std::printf("  covered: %d/%d/%d/%d/%d ticks per phase, %d scores and %d re-seats finished, "
                "%d arrivals, %d descend timeouts, %d restarts, %d bottom outtake ticks\n",
                ticksIn[0], ticksIn[1], ticksIn[2], ticksIn[3], ticksIn[4], scoresFinished,
                reseatsFinished, arrivals, descendTimeouts, restarts, bottomOuttakes);
    for (int ticks : ticksIn) assert(ticks > 10000);
    assert(scoresFinished > 1000);
    assert(reseatsFinished > 1000);
    assert(arrivals > 1000);
    assert(descendTimeouts > 100);
    assert(restarts > 1000);
    assert(bottomOuttakes > 1000);
}

} // namespace

int main() {
    testQueriesWhileIdle();
    testOneTransitionPerUpdate();
    testStepTimerStartsAtTheTransition();
    testUntilIsCheckedFirstAndNotOnTheEnteringTick();
    testFixedDurationStepsExitByTimeout();
    testStepWithNoExitNeverEndsOnItsOwn();
    testOnEnterSeesTheStepItsEntering();
    testOnFinishRunsAfterGoingIdleAndMayChain();
    testCancelSkipsOnFinish();
    testStartReplacesWithoutFinishing();
    testEmptyProgramFinishesAtOnce();
    testRunningComparesIdentity();
    testCallbacksMayCancelOrBranch();
    testTimeoutAcrossTheClockWrap();
    testStaleNowDoesNotEndAStep();
    testRunBlockingFinishes();
    testRunBlockingWaitsOnACondition();
    testRunBlockingTimesOutAndCancels();
    testRunBlockingFinishingAtTheDeadlineCounts();
    testRunBlockingReturnsFalseWhenACallbackCancelsOrReplaces();
    testRunBlockingEmptyProgram();
    testRunBlockingZeroPollStillSleeps();
    testMatchesTheOriginalPhaseMachine();
    std::puts("sequence_test: all assertions passed");
    return 0;
}
