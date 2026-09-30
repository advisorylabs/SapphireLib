// Host-side unit test for sapphirelib::motion::ExitTracker and
// exitReasonName, no PROS/embedded dependencies, so it builds and runs with
// a normal desktop compiler.
//
// The drivetrain motions used to carry their settle/timeout bookkeeping
// inline; ExitTracker replaces it and must not change when any motion stops.
// testMatchesTheOldInlineLoop() runs a verbatim copy of that old loop against
// ExitTracker over thousands of random traces and requires the same exit
// tick, reason, and elapsed time on every one.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/exit_tracker_test.cpp src/sapphirelib/motion/exit_tracker.cpp -o exit_tracker_test && ./exit_tracker_test

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "sapphirelib/motion/exit_tracker.hpp"

using sapphirelib::motion::ExitReason;
using sapphirelib::motion::exitReasonName;
using sapphirelib::motion::ExitTracker;

namespace {

void testReasonNames() {
    assert(std::strcmp(exitReasonName(ExitReason::running), "running") == 0);
    assert(std::strcmp(exitReasonName(ExitReason::settled), "settled") == 0);
    assert(std::strcmp(exitReasonName(ExitReason::timedOut), "timeout") == 0);
    assert(std::strcmp(exitReasonName(ExitReason::aborted), "aborted") == 0);
}

void testSettlesAfterTheSettleTime() {
    ExitTracker tracker(/*settleTimeMs=*/100, /*timeoutMs=*/2000, /*startMs=*/1000);
    assert(tracker.elapsedMs() == 0); // before any update
    // Within threshold from the start: 10ms ticks accumulate 10, 20, ... 100.
    for (std::uint32_t now = 1010; now < 1100; now += 10) {
        assert(tracker.update(true, now) == ExitReason::running);
    }
    assert(tracker.update(true, 1100) == ExitReason::settled); // `>=`: exactly 100
    assert(tracker.elapsedMs() == 100);
}

void testLeavingTheThresholdResetsTheSettleTime() {
    ExitTracker tracker(100, 0, 0);
    assert(tracker.update(true, 50) == ExitReason::running);  // 50ms settled
    assert(tracker.update(false, 60) == ExitReason::running); // back to 0
    // Time within restarts counting from the tick that was outside: the
    // interval from that tick to the next one counts, as the old loop did.
    assert(tracker.update(true, 150) == ExitReason::running); // 90
    assert(tracker.update(true, 160) == ExitReason::settled); // 100
}

void testTimesOut() {
    ExitTracker tracker(100, 500, 1000);
    for (std::uint32_t now = 1010; now < 1500; now += 10) {
        assert(tracker.update(false, now) == ExitReason::running);
    }
    assert(tracker.update(false, 1500) == ExitReason::timedOut); // `>=`: exactly 500
    assert(tracker.elapsedMs() == 500);
}

void testZeroTimeoutNeverTimesOut() {
    ExitTracker tracker(100, 0, 0);
    for (std::uint32_t now = 10; now <= 600000; now += 10) {
        assert(tracker.update(false, now) == ExitReason::running);
    }
    assert(tracker.elapsedMs() == 600000);
}

void testSettledWinsOverTimeoutOnTheSameTick() {
    ExitTracker tracker(100, 100, 0);
    assert(tracker.update(true, 50) == ExitReason::running);
    assert(tracker.update(true, 100) == ExitReason::settled); // both are due at 100
}

void testZeroSettleTimeSettlesOnTheFirstTickWithin() {
    ExitTracker tracker(0, 1000, 0);
    assert(tracker.update(false, 10) == ExitReason::running);
    assert(tracker.update(true, 10) == ExitReason::settled); // even with no time passing
}

void testElapsedFollowsEveryUpdate() {
    ExitTracker tracker(100, 0, 5000);
    tracker.update(false, 5013);
    assert(tracker.elapsedMs() == 13);
    tracker.update(true, 5027);
    assert(tracker.elapsedMs() == 27);
}

void testAcrossTheClockWrap() {
    // Started 20ms before pros::millis() wraps.
    ExitTracker tracker(50, 200, 0xFFFFFFFFu - 19u);
    assert(tracker.update(true, 0xFFFFFFFFu) == ExitReason::running); // 19ms in
    assert(tracker.update(true, 30u) == ExitReason::settled);         // 31ms more
    assert(tracker.elapsedMs() == 50);

    ExitTracker timing(50, 200, 0xFFFFFFFFu - 19u);
    assert(timing.update(false, 179u) == ExitReason::running);
    assert(timing.update(false, 180u) == ExitReason::timedOut); // 200ms after start
}

void testStaleNowCountsAsNoTime() {
    // A "now" from before the previous tick counts as 0ms rather than the
    // ~49 days an unsigned subtraction would give (which would settle, and
    // time out, instantly).
    ExitTracker tracker(100, 1000, 1000);
    assert(tracker.update(true, 990) == ExitReason::running);
    assert(tracker.elapsedMs() == 0);
}

// --- Reference equivalence -------------------------------------------------

/// One motion's worth of inputs: what pros::millis() returned on each call
/// (the call before the loop, then one per tick) and each tick's error.
struct Trace {
    std::vector<std::uint32_t> clockReadings;
    std::vector<double> errors; // errors[k] is tick k+1's error
    double errorThreshold = 1.0;
    std::uint32_t settleTimeMs = 0;
    std::uint32_t timeoutMs = 0;
};

/// How a loop stopped: on which tick (1-based), why, and its elapsed time
/// then. ticks == 0 means it was still running when the trace ran out.
struct LoopExit {
    std::size_t ticks = 0;
    ExitReason reason = ExitReason::running;
    std::uint32_t elapsedMs = 0;
};

struct TraceEnded {};

namespace old_loop {

// Stand-ins for what the old loop called, fed from the trace.
const Trace* trace = nullptr;
std::size_t clockCalls = 0;
std::size_t errorReads = 0;

namespace pros {
std::uint32_t millis() {
    if (clockCalls >= trace->clockReadings.size()) throw TraceEnded{};
    return trace->clockReadings[clockCalls++];
}
void delay(std::uint32_t) {}
} // namespace pros

double readError() {
    if (errorReads >= trace->errors.size()) throw TraceEnded{};
    return trace->errors[errorReads++];
}

struct ExitConditions {
    double errorThreshold;
    std::uint32_t settleTimeMs;
    std::uint32_t timeoutMs;
};

constexpr std::uint32_t kLoopDelayMs = 10;

LoopExit run(const Trace& input) {
    trace = &input;
    clockCalls = 0;
    errorReads = 0;
    const ExitConditions exit{input.errorThreshold, input.settleTimeMs, input.timeoutMs};
    LoopExit result;
    try {
        // Copied verbatim from HolonomicDrivetrain::driveDistance() before
        // ExitTracker (git show 73dc252:src/sapphirelib/chassis/holonomic_drivetrain.cpp);
        // turnToHeading(), moveToPoint(), and moveToPose() carried the same
        // lines. Only the motor/PID work is replaced by readError(), and the
        // two `break`s record which one fired (marked "instrumentation").
        std::uint32_t settledForMs = 0;
        std::uint32_t lastTick = pros::millis();
        const std::uint32_t start = lastTick;

        while (true) {
            const double error = readError();

            const std::uint32_t now = pros::millis();
            if (std::fabs(error) <= exit.errorThreshold) {
                settledForMs += now - lastTick;
                if (settledForMs >= exit.settleTimeMs) {
                    result = {clockCalls - 1, ExitReason::settled, now - start}; // instrumentation
                    break;
                }
            } else {
                settledForMs = 0;
            }
            if (exit.timeoutMs > 0 && (now - start) >= exit.timeoutMs) {
                result = {clockCalls - 1, ExitReason::timedOut, now - start}; // instrumentation
                break;
            }

            lastTick = now;
            pros::delay(kLoopDelayMs);
        }
    } catch (const TraceEnded&) {
        result = LoopExit{};
    }
    return result;
}

} // namespace old_loop

/// The same motion with its bookkeeping on ExitTracker, the way the
/// drivetrains call it now.
LoopExit runWithTracker(const Trace& trace) {
    ExitTracker tracker(trace.settleTimeMs, trace.timeoutMs, trace.clockReadings[0]);
    for (std::size_t tick = 1; tick < trace.clockReadings.size(); ++tick) {
        const std::uint32_t now = trace.clockReadings[tick];
        const bool within = std::fabs(trace.errors[tick - 1]) <= trace.errorThreshold;
        const ExitReason reason = tracker.update(within, now);
        // Every tick, not just the last: elapsed is always now - start.
        assert(tracker.elapsedMs() == now - trace.clockReadings[0]);
        if (reason != ExitReason::running) return {tick, reason, tracker.elapsedMs()};
    }
    return {};
}

Trace randomTrace(std::mt19937& rng) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Trace trace;
    trace.errorThreshold = 0.25 + 2.75 * unit(rng);
    const std::uint32_t settleChoices[] = {0, 1, 10, 50, 100, 250, 500};
    trace.settleTimeMs =
        unit(rng) < 0.6 ? settleChoices[rng() % 7] : static_cast<std::uint32_t>(rng() % 600);
    trace.timeoutMs = unit(rng) < 0.3 ? 0u : static_cast<std::uint32_t>(rng() % 4000);

    // Mostly small start times, but a fifth start close enough to the 32-bit
    // wrap that the motion runs across it.
    std::uint32_t now = unit(rng) < 0.2 ? 0xFFFFFFFFu - static_cast<std::uint32_t>(rng() % 3000)
                                        : static_cast<std::uint32_t>(rng() % 100000);
    trace.clockReadings.push_back(now);

    const std::size_t ticks = 1 + rng() % 400;
    bool within = unit(rng) < 0.3;
    for (std::size_t i = 0; i < ticks; ++i) {
        // A 10ms delay plus the loop's own work, with the occasional tick
        // that lands in the same millisecond or gets preempted for a while.
        const double roll = unit(rng);
        const std::uint32_t dt = roll < 0.9    ? 10 + rng() % 2
                                 : roll < 0.95 ? rng() % 3
                                               : 12 + rng() % 50;
        now += dt;
        trace.clockReadings.push_back(now);

        // Runs of in and out of threshold, like a real approach and overshoot,
        // with the exact edge value sometimes (`<=` must count it as within).
        if (unit(rng) < 0.15) within = !within;
        const double sign = unit(rng) < 0.5 ? -1.0 : 1.0;
        double error;
        if (within) {
            error = unit(rng) < 0.05 ? sign * trace.errorThreshold
                                     : sign * trace.errorThreshold * unit(rng);
        } else {
            error = sign * (trace.errorThreshold + 1e-6 + 20.0 * unit(rng));
        }
        trace.errors.push_back(error);
    }
    return trace;
}

void testMatchesTheOldInlineLoop() {
    std::mt19937 rng(96671);
    int settled = 0;
    int timedOut = 0;
    int unfinished = 0;
    for (int i = 0; i < 20000; ++i) {
        const Trace trace = randomTrace(rng);
        const LoopExit expected = old_loop::run(trace);
        const LoopExit actual = runWithTracker(trace);
        if (actual.ticks != expected.ticks || actual.reason != expected.reason ||
            actual.elapsedMs != expected.elapsedMs) {
            std::printf("FAIL trace %d: old loop stopped at tick %zu (%s, %ums), "
                        "ExitTracker at tick %zu (%s, %ums)\n",
                        i, expected.ticks, exitReasonName(expected.reason), expected.elapsedMs,
                        actual.ticks, exitReasonName(actual.reason), actual.elapsedMs);
            assert(false);
        }
        if (expected.reason == ExitReason::settled) ++settled;
        if (expected.reason == ExitReason::timedOut) ++timedOut;
        if (expected.ticks == 0) ++unfinished;
    }
    // The traces must actually reach every outcome, or the comparison proves
    // little.
    assert(settled > 1000);
    assert(timedOut > 1000);
    assert(unfinished > 100);
}

} // namespace

int main() {
    testReasonNames();
    testSettlesAfterTheSettleTime();
    testLeavingTheThresholdResetsTheSettleTime();
    testTimesOut();
    testZeroTimeoutNeverTimesOut();
    testSettledWinsOverTimeoutOnTheSameTick();
    testZeroSettleTimeSettlesOnTheFirstTickWithin();
    testElapsedFollowsEveryUpdate();
    testAcrossTheClockWrap();
    testStaleNowCountsAsNoTime();
    testMatchesTheOldInlineLoop();
    std::puts("exit_tracker_test: all assertions passed");
    return 0;
}
