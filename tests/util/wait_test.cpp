// Host-side unit test for sapphirelib::waitUntil — no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// waitUntil() reads time only through util/clock.hpp, which this test defines
// itself as a fake clock: delayMs() just advances the fake time, so a
// "10 second" wait runs instantly and every poll lands on a known timestamp.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/util/wait_test.cpp src/sapphirelib/util/wait.cpp -o wait_test && ./wait_test

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/wait.hpp"

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

using sapphirelib::waitUntil;

namespace {

void testImmediatelyTrueDoesNotDelay() {
    resetClock(1000);
    int checks = 0;
    const bool done = waitUntil(
        [&] {
            ++checks;
            return true;
        },
        500);
    assert(done);
    assert(checks == 1);
    assert(delays.empty());
    assert(fakeNowMs == 1000);
}

void testReturnsOnThePollAfterTheConditionComesTrue() {
    resetClock(1000);
    int checks = 0;
    // True from t = 1095 on; polled every 10ms from 1000, so first seen at 1100.
    const bool done = waitUntil(
        [&] {
            ++checks;
            return fakeNowMs >= 1095;
        },
        1000, 10);
    assert(done);
    assert(fakeNowMs == 1100);
    assert(checks == 11); // at 1000, 1010, ..., 1100
    assert(delays.size() == 10);
}

void testConditionAtTheDeadlineWins() {
    // The condition comes true on exactly the pass where the timeout also
    // runs out. done() is checked first, so it counts as done.
    resetClock(1000);
    const bool done = waitUntil([] { return fakeNowMs >= 1100; }, 100, 10);
    assert(done);
    assert(fakeNowMs == 1100);
}

void testTimesOut() {
    resetClock(1000);
    int checks = 0;
    const bool done = waitUntil(
        [&] {
            ++checks;
            return false;
        },
        100, 10);
    assert(!done);
    assert(fakeNowMs == 1100); // `>=`: gives up at exactly the timeout
    assert(checks == 11);      // still checked once more at the deadline
}

void testTimeoutThatIsNotAMultipleOfThePoll() {
    // Polls at 0, 10, 20, 30ms: the first one at or past 25ms gives up.
    resetClock(1000);
    const bool done = waitUntil([] { return false; }, 25, 10);
    assert(!done);
    assert(fakeNowMs == 1030);
}

void testZeroTimeoutWaitsWithNoLimit() {
    resetClock(1000);
    const bool done = waitUntil([] { return fakeNowMs >= 1000 + 60000; }, 0, 10);
    assert(done);
    assert(fakeNowMs == 61000);
}

void testPollSpacing() {
    resetClock(1000);
    waitUntil([] { return fakeNowMs >= 1100; }, 0, 25);
    assert(delays.size() == 4); // 1000 -> 1025 -> 1050 -> 1075 -> 1100
    for (std::uint32_t ms : delays) assert(ms == 25);

    // The default poll is 10ms.
    resetClock(1000);
    waitUntil([] { return fakeNowMs >= 1030; }, 0);
    assert(delays.size() == 3);
    for (std::uint32_t ms : delays) assert(ms == 10);
}

void testZeroPollStillSleeps() {
    // A 0ms poll would only yield to same-priority tasks and starve the rest,
    // so it waits the scheduler's minimum of 1ms per pass instead.
    resetClock(1000);
    const bool done = waitUntil([] { return fakeNowMs >= 1005; }, 100, 0);
    assert(done);
    assert(delays.size() == 5);
    for (std::uint32_t ms : delays) assert(ms == 1);
}

void testTimeoutAcrossTheClockWrap() {
    // Started 50ms before pros::millis() wraps; a bare `now - start` would
    // still work here, but a `now < deadline` comparison would not.
    resetClock(0xFFFFFFFFu - 49u);
    const bool done = waitUntil([] { return false; }, 100, 10);
    assert(!done);
    assert(fakeNowMs == 50u); // 100ms later, on the far side of the wrap
}

} // namespace

int main() {
    testImmediatelyTrueDoesNotDelay();
    testReturnsOnThePollAfterTheConditionComesTrue();
    testConditionAtTheDeadlineWins();
    testTimesOut();
    testTimeoutThatIsNotAMultipleOfThePoll();
    testZeroTimeoutWaitsWithNoLimit();
    testPollSpacing();
    testZeroPollStillSleeps();
    testTimeoutAcrossTheClockWrap();
    std::puts("wait_test: all assertions passed");
    return 0;
}
