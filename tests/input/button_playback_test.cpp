// Host-side unit test for sapphirelib::input::ButtonPlayback, no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/input/button_playback_test.cpp src/sapphirelib/input/button_playback.cpp src/sapphirelib/input/button_tracker.cpp -o button_playback_test && ./button_playback_test

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "sapphirelib/input/button_playback.hpp"
#include "sapphirelib/input/button_tracker.hpp"

using sapphirelib::input::Button;
using sapphirelib::input::ButtonMask;
using sapphirelib::input::ButtonPlayback;
using sapphirelib::input::ButtonTracker;
using sapphirelib::input::maskOf;
using sapphirelib::input::TimedButtons;

namespace {

constexpr ButtonMask kR1 = maskOf(Button::r1);
constexpr ButtonMask kL1 = maskOf(Button::l1);

void testPlaysEachChangeAtItsTime() {
    ButtonPlayback playback({{0.0, 0}, {0.5, kR1}, {1.0, kR1 | kL1}, {1.5, 0}});
    assert(playback.advance(0.0) == 0);
    assert(playback.advance(0.49) == 0);
    assert(playback.advance(0.5) == kR1); // at its time, not after
    assert(playback.advance(0.99) == kR1);
    assert(playback.advance(1.01) == (kR1 | kL1));
    assert(!playback.finished());
    assert(playback.advance(2.0) == 0);
    assert(playback.finished());
    assert(playback.current() == 0);
}

void testAJumpedOverPressStillGetsATick() {
    // R1 held for 20ms, then nothing: a clock that jumps 50ms at a time lands past both
    ButtonPlayback playback({{0.0, 0}, {0.10, kR1}, {0.12, 0}, {0.30, kL1}});
    assert(playback.advance(0.05) == 0);
    assert(playback.advance(0.15) == kR1); // late, but pressed
    assert(playback.advance(0.15) == 0);   // and released on the next tick, even at the same time
    assert(playback.advance(0.20) == 0);
    assert(playback.advance(0.35) == kL1);
}

void testAPausedClockHolds() {
    ButtonPlayback playback({{0.0, kR1}, {1.0, 0}});
    assert(playback.advance(0.0) == kR1);
    for (int i = 0; i < 50; ++i) assert(playback.advance(0.4) == kR1); // the plan waited
    assert(playback.advance(0.3) == kR1);                            // backward: holds
    assert(playback.advance(1.0) == 0);
}

void testRepeatsAreCrossedAtOnce() {
    // the same buttons recorded again and again don't hold the playback back
    std::vector<TimedButtons> changes;
    for (int i = 0; i < 100; ++i) changes.push_back({i * 0.01, kR1});
    changes.push_back({1.0, 0});
    ButtonPlayback playback(std::move(changes));
    assert(playback.advance(0.5) == kR1);
    assert(playback.advance(1.0) == 0); // one tick, not 50
}

void testATrackerSeesTheDriversPresses() {
    // a driver's random presses, recorded every 20ms, played back on a clock with jitter: every
    // press the driver made is a press on playback, in order
    std::mt19937 rng(9667);
    std::uniform_int_distribution<int> bit(0, 11);
    std::uniform_real_distribution<double> jitter(-0.006, 0.012);
    std::vector<TimedButtons> recorded;
    ButtonTracker driver;
    std::vector<ButtonMask> driverPresses;
    ButtonMask held = 0;
    for (int tick = 0; tick < 2000; ++tick) {
        if (rng() % 4 == 0) held = static_cast<ButtonMask>(held ^ maskOf(static_cast<Button>(bit(rng))));
        recorded.push_back({tick * 0.02, held});
        driver.update(held, static_cast<std::uint32_t>(tick * 20));
        ButtonMask pressed = 0;
        for (int b = 0; b < 12; ++b) {
            if (driver.pressed(static_cast<Button>(b))) pressed |= maskOf(static_cast<Button>(b));
        }
        if (pressed) driverPresses.push_back(pressed);
    }

    ButtonPlayback playback(recorded);
    ButtonTracker replayed;
    std::vector<ButtonMask> replayedPresses;
    double clock = 0.0;
    std::uint32_t nowMs = 0;
    for (int tick = 0; tick < 4000 && !(playback.finished() && clock > 41.0); ++tick) {
        clock += 0.02 + jitter(rng);
        nowMs += 20;
        replayed.update(playback.advance(clock), nowMs);
        ButtonMask pressed = 0;
        for (int b = 0; b < 12; ++b) {
            if (replayed.pressed(static_cast<Button>(b))) pressed |= maskOf(static_cast<Button>(b));
        }
        if (pressed) replayedPresses.push_back(pressed);
    }
    if (replayedPresses != driverPresses) {
        std::printf("FAIL presses: driver made %zu, playback %zu\n", driverPresses.size(),
                    replayedPresses.size());
        assert(false);
    }
}

} // namespace

int main() {
    testPlaysEachChangeAtItsTime();
    testAJumpedOverPressStillGetsATick();
    testAPausedClockHolds();
    testRepeatsAreCrossedAtOnce();
    testATrackerSeesTheDriversPresses();
    std::puts("button_playback_test: all assertions passed");
    return 0;
}
