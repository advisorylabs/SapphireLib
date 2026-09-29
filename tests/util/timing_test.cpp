// Host-side unit test for sapphirelib's clock-free timing primitives
// (elapsedMs, Stopwatch, TimedFlag, GapDetector) — no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// util/timing.hpp is header-only, so there's no source file to link.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/util/timing_test.cpp -o timing_test && ./timing_test

#include <cassert>
#include <cstdint>
#include <cstdio>

#include "sapphirelib/util/timing.hpp"

using sapphirelib::elapsedMs;
using sapphirelib::GapDetector;
using sapphirelib::Stopwatch;
using sapphirelib::TimedFlag;

namespace {

// Everything here is constexpr, so it's usable in constant expressions too —
// which also proves none of it reads a clock behind the caller's back.
static_assert(elapsedMs(1000u, 1250u) == 250u);
static_assert(elapsedMs(0xFFFFFF00u, 0x10u) == 0x110u);
static_assert(elapsedMs(1250u, 1000u) == 0u);

void testElapsedMsNormal() {
    assert(elapsedMs(0u, 0u) == 0u);
    assert(elapsedMs(1000u, 1000u) == 0u);
    assert(elapsedMs(1000u, 1001u) == 1u);
    assert(elapsedMs(1000u, 61000u) == 60000u);
}

void testElapsedMsAcrossTheWrap() {
    // pros::millis() wraps from 0xFFFFFFFF to 0 after ~49.7 days; the
    // difference is still the real elapsed time across it.
    assert(elapsedMs(0xFFFFFF00u, 0x10u) == 0x110u);
    assert(elapsedMs(0xFFFFFFFFu, 0u) == 1u);
    assert(elapsedMs(0xFFFFFFF6u, 0xFFFFFFFFu) == 9u);
}

void testElapsedMsNowBeforeSinceIsZero() {
    // A bare unsigned subtraction would give ~4 billion here.
    assert(elapsedMs(1001u, 1000u) == 0u);
    assert(elapsedMs(0x10u, 0xFFFFFF00u) == 0u); // "since" just after a wrap
    // The largest forward difference still counts; one more is "negative".
    assert(elapsedMs(0u, 0x7FFFFFFFu) == 0x7FFFFFFFu);
    assert(elapsedMs(0u, 0x80000000u) == 0u);
}

void testStopwatchStoppedUntilRestart() {
    Stopwatch watch;
    assert(!watch.running());
    assert(watch.elapsedMs(5000u) == 0u);
    assert(!watch.hasElapsed(0u, 5000u)); // not even a 0ms wait while stopped
}

void testStopwatchMeasuresFromRestart() {
    Stopwatch watch;
    watch.restart(1000u);
    assert(watch.running());
    assert(watch.elapsedMs(1000u) == 0u);
    assert(watch.elapsedMs(1250u) == 250u);
    assert(watch.elapsedMs(900u) == 0u); // a stale "now" from before the restart

    // hasElapsed() is `>=`: true exactly at the boundary, not a tick later.
    assert(!watch.hasElapsed(250u, 1249u));
    assert(watch.hasElapsed(250u, 1250u));
    assert(watch.hasElapsed(0u, 1000u));

    watch.restart(2000u);
    assert(watch.elapsedMs(2100u) == 100u);

    watch.stop();
    assert(!watch.running());
    assert(watch.elapsedMs(9000u) == 0u);
    assert(!watch.hasElapsed(100u, 9000u));
}

void testStopwatchAcrossTheWrap() {
    Stopwatch watch;
    watch.restart(0xFFFFFFF0u);
    assert(watch.elapsedMs(0x10u) == 0x20u);
    assert(watch.hasElapsed(0x20u, 0x10u));
}

void testTimedFlagInitialState() {
    const TimedFlag off;
    assert(!off.value());
    assert(off.msInState(500u) == 500u); // held since 0 by default
    assert(off.falseFor(500u, 500u));
    assert(!off.trueFor(0u, 500u)); // wrong value, whatever the time

    const TimedFlag on(true, 1000u);
    assert(on.value());
    assert(on.msInState(1400u) == 400u);
    assert(on.trueFor(400u, 1400u));
    assert(!on.falseFor(0u, 1400u));
}

void testTimedFlagSetRecordsOnlyRealChanges() {
    TimedFlag flag;
    assert(flag.set(true, 1000u)); // changed
    assert(flag.value());

    // Setting the same value every tick must not restart the count — this is
    // what lets per-tick code call set() unconditionally.
    assert(!flag.set(true, 1100u));
    assert(!flag.set(true, 1200u));
    assert(flag.msInState(1400u) == 400u);

    assert(flag.set(false, 1500u));
    assert(!flag.value());
    assert(flag.msInState(1600u) == 100u);
    assert(!flag.set(false, 1700u));
    assert(flag.msInState(1800u) == 300u);
}

void testTimedFlagBoundariesAreInclusive() {
    TimedFlag piston;
    piston.set(true, 1000u);
    assert(!piston.trueFor(400u, 1399u));
    assert(piston.trueFor(400u, 1400u));
    assert(piston.trueFor(400u, 5000u));
    assert(!piston.falseFor(400u, 5000u));

    piston.set(false, 6000u);
    assert(!piston.trueFor(0u, 6000u));
    assert(!piston.falseFor(250u, 6249u));
    assert(piston.falseFor(250u, 6250u));
}

void testTimedFlagStaleNowCountsAsZero() {
    TimedFlag flag;
    flag.set(true, 1000u);
    assert(flag.msInState(999u) == 0u);
    assert(!flag.trueFor(1u, 999u));
    assert(flag.trueFor(0u, 999u)); // 0ms is always satisfied while true
}

void testGapDetectorFirstCallIsAGap() {
    GapDetector gap(100u);
    assert(gap.update(5000u));
    assert(!gap.update(5020u));
}

void testGapDetectorBoundaryIsExclusive() {
    // "More than maxGapMs": exactly maxGapMs apart is not a gap, one more is.
    GapDetector gap(100u);
    gap.update(1000u);
    assert(!gap.update(1100u));
    assert(gap.update(1201u));
    assert(!gap.update(1221u));
}

void testGapDetectorMeasuresFromThePreviousCall() {
    // Each call re-arms it, so steady 20ms ticks never gap however long they
    // run, and a single long stall gaps once.
    GapDetector gap(100u);
    gap.update(0u);
    for (std::uint32_t now = 20u; now <= 10000u; now += 20u) assert(!gap.update(now));
    assert(gap.update(13000u));
    assert(!gap.update(13020u));
}

void testGapDetectorAcrossTheWrap() {
    GapDetector gap(100u);
    gap.update(0xFFFFFFE0u);
    assert(!gap.update(0x40u)); // 0x60 = 96ms later
    assert(gap.update(0xB0u));  // 0x70 = 112ms later
}

void testGapDetectorStaleNowIsNotAGap() {
    GapDetector gap(100u);
    gap.update(5000u);
    assert(!gap.update(4000u)); // "before" the last call counts as 0ms
}

} // namespace

int main() {
    testElapsedMsNormal();
    testElapsedMsAcrossTheWrap();
    testElapsedMsNowBeforeSinceIsZero();
    testStopwatchStoppedUntilRestart();
    testStopwatchMeasuresFromRestart();
    testStopwatchAcrossTheWrap();
    testTimedFlagInitialState();
    testTimedFlagSetRecordsOnlyRealChanges();
    testTimedFlagBoundariesAreInclusive();
    testTimedFlagStaleNowCountsAsZero();
    testGapDetectorFirstCallIsAGap();
    testGapDetectorBoundaryIsExclusive();
    testGapDetectorMeasuresFromThePreviousCall();
    testGapDetectorAcrossTheWrap();
    testGapDetectorStaleNowIsNotAGap();
    std::puts("timing_test: all assertions passed");
    return 0;
}
