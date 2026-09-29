// Host-side unit test for sapphirelib::input::ButtonTracker — no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/input/button_tracker_test.cpp src/sapphirelib/input/button_tracker.cpp -o button_tracker_test && ./button_tracker_test

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "sapphirelib/input/button_tracker.hpp"

using sapphirelib::input::Button;
using sapphirelib::input::ButtonMask;
using sapphirelib::input::ButtonTracker;
using sapphirelib::input::kButtonCount;
using sapphirelib::input::maskOf;

namespace {

void expect(bool ok, const char* label) {
    if (!ok) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

void expectEq(std::uint32_t actual, std::uint32_t expected, const char* label) {
    if (actual != expected) {
        std::printf("FAIL %s: got %lu, expected %lu\n", label, static_cast<unsigned long>(actual),
                    static_cast<unsigned long>(expected));
        assert(false);
    }
}

Button buttonAt(std::size_t i) { return static_cast<Button>(i); }

// --- Reference: the PROS 4.2.2 kernel's new-press / new-release latch ---
//
// Copied from PROS 4.2.2 src/devices/controller.c. `pressed` stands in for
// the controller_get_digital() read, and the port mutex is dropped (the
// test is single-threaded). In PROS these arrays are kernel globals, one set
// per controller; here one KernelLatch is one controller.
struct KernelLatch {
    bool button_pressed[kButtonCount] = {false};
    bool button_released[kButtonCount] = {true, true, true, true, true, true,
                                          true, true, true, true, true, true};

    std::int32_t get_digital_new_press(std::size_t button_num, std::int32_t pressed) {
        if (!pressed) {
            button_pressed[button_num] = false;
        }
        if (pressed && !button_pressed[button_num]) {
            // button is currently pressed and was not detected as being pressed during
            // last check
            button_pressed[button_num] = true;
            return true;
        } else {
            return false; // button is not pressed or was already detected
        }
    }

    std::int32_t get_digital_new_release(std::size_t button_num, std::int32_t pressed) {
        if (pressed) {
            button_released[button_num] = false;
        }
        if (!pressed && !button_released[button_num]) {
            // button is currently not pressed and was detected as being pressed during
            // last check
            button_released[button_num] = true;
            return true;
        } else {
            return false; // button is pressed or was already detected
        }
    }
};

/// A random controller: each tick, each button flips with probability
/// `flipChance`, and ticks are `minStepMs`..`maxStepMs` apart.
struct RandomController {
    std::mt19937 rng;
    double flipChance;
    std::uint32_t minStepMs;
    std::uint32_t maxStepMs;
    ButtonMask held = 0;

    ButtonMask nextSample() {
        std::bernoulli_distribution flip(flipChance);
        for (std::size_t i = 0; i < kButtonCount; ++i) {
            if (flip(rng)) held = static_cast<ButtonMask>(held ^ (1u << i));
        }
        return held;
    }

    std::uint32_t nextStep() {
        return std::uniform_int_distribution<std::uint32_t>(minStepMs, maxStepMs)(rng);
    }
};

// The property the whole class rests on: read every button every tick, and
// the kernel's latch reports exactly pressed()/released() — including on the
// very first sample, where both start "not seen".
void testMatchesKernelLatchWhenEveryButtonIsReadEveryTick() {
    long ticksChecked = 0;
    for (std::uint32_t seed = 1; seed <= 200; ++seed) {
        std::mt19937 setup(seed);
        const double flipChance = std::uniform_real_distribution<double>(0.02, 0.6)(setup);
        RandomController controller{std::mt19937(seed * 7919u), flipChance, 5, 40};
        // Some runs start with buttons already down, so the first-sample case
        // is exercised with both states.
        controller.held = static_cast<ButtonMask>(setup() & 0x0FFFu);

        KernelLatch latch;
        ButtonTracker tracker;
        std::uint32_t now = setup();
        for (int tick = 0; tick < 2000; ++tick) {
            const ButtonMask sample = tick == 0 ? controller.held : controller.nextSample();
            tracker.update(sample, now);
            ButtonMask expectedPressed = 0;
            ButtonMask expectedReleased = 0;
            for (std::size_t i = 0; i < kButtonCount; ++i) {
                const std::int32_t value = (sample >> i) & 1u;
                const bool newPress = latch.get_digital_new_press(i, value) != 0;
                const bool newRelease = latch.get_digital_new_release(i, value) != 0;
                if (tracker.pressed(buttonAt(i)) != newPress ||
                    tracker.released(buttonAt(i)) != newRelease) {
                    std::printf("FAIL latch equivalence: seed %lu tick %d button %zu\n",
                                static_cast<unsigned long>(seed), tick, i);
                    assert(false);
                }
                if (newPress) {
                    expectedPressed = static_cast<ButtonMask>(expectedPressed | (1u << i));
                }
                if (newRelease) {
                    expectedReleased = static_cast<ButtonMask>(expectedReleased | (1u << i));
                }
            }
            expectEq(tracker.pressedMask(), expectedPressed, "pressedMask matches latch");
            expectEq(tracker.releasedMask(), expectedReleased, "releasedMask matches latch");
            expectEq(tracker.heldMask(), sample, "heldMask is the sample");
            now += controller.nextStep();
            ++ticksChecked;
        }
    }
    std::printf("  kernel latch equivalence: %ld ticks x 12 buttons matched\n", ticksChecked);
}

// --- Independent reference for the timing queries ---
//
// Written as the definitions read in the header, per press, rather than the
// sample-comparison arithmetic ButtonTracker uses, so the two can disagree
// if either is wrong.
struct ReferenceButton {
    bool held = false;
    bool previouslyHeld = false;
    std::uint32_t pressedAt = 0;
    std::uint32_t heldAtRelease = 0;
};

constexpr std::array<std::uint32_t, 5> kLongPressMs{0, 1, 20, 250, 500};

struct RepeatSpec {
    std::uint32_t delayMs;
    std::uint32_t periodMs;
};
constexpr std::array<RepeatSpec, 5> kRepeatSpecs{{{400, 150}, {0, 50}, {300, 0}, {25, 10}, {1, 1}}};

void testTimingQueriesMatchReferenceModel() {
    for (std::uint32_t seed = 1; seed <= 60; ++seed) {
        std::mt19937 setup(seed);
        const double flipChance = std::uniform_real_distribution<double>(0.01, 0.2)(setup);
        // Ticks from 1ms (many per repeat period) to 300ms (several repeat
        // periods per tick), so "at most one per update" is exercised too.
        const std::uint32_t maxStep = seed % 3 == 0 ? 300 : 40;
        RandomController controller{std::mt19937(seed * 104729u), flipChance, 1, maxStep};
        // Every third run starts just short of the 32-bit wrap of millis().
        std::uint32_t now = seed % 3 == 1 ? 0xFFFF0000u : setup() % 100000u;

        ButtonTracker tracker;
        std::array<ReferenceButton, kButtonCount> reference{};
        // Per button, per threshold: has this press already long-pressed?
        std::array<std::array<bool, kLongPressMs.size()>, kButtonCount> longFired{};
        // Per button, per spec: ms into the press when the next repeat is due.
        std::array<std::array<std::uint32_t, kRepeatSpecs.size()>, kButtonCount> nextRepeatAt{};

        for (int tick = 0; tick < 3000; ++tick) {
            const ButtonMask sample = controller.nextSample();
            tracker.update(sample, now);
            for (std::size_t i = 0; i < kButtonCount; ++i) {
                ReferenceButton& ref = reference[i];
                ref.previouslyHeld = ref.held;
                ref.held = ((sample >> i) & 1u) != 0;
                const bool justPressed = ref.held && !ref.previouslyHeld;
                const bool justReleased = !ref.held && ref.previouslyHeld;
                if (justPressed) {
                    ref.pressedAt = now;
                    longFired[i].fill(false);
                }
                if (justReleased) ref.heldAtRelease = now - ref.pressedAt;
                const std::uint32_t heldFor = ref.held ? now - ref.pressedAt : 0;
                const Button button = buttonAt(i);

                const std::uint32_t expectedHeldMs =
                    ref.held ? heldFor : (justReleased ? ref.heldAtRelease : 0);
                expectEq(tracker.heldMs(button), expectedHeldMs, "heldMs vs reference");

                for (std::size_t k = 0; k < kLongPressMs.size(); ++k) {
                    const std::uint32_t ms = kLongPressMs[k];
                    expect(tracker.heldFor(button, ms) == (ref.held && heldFor >= ms),
                           "heldFor vs reference");
                    bool fires = false;
                    if (ref.held && heldFor >= ms && !longFired[i][k]) {
                        longFired[i][k] = true;
                        fires = true;
                    }
                    expect(tracker.longPressed(button, ms) == fires, "longPressed vs reference");
                }

                for (std::size_t k = 0; k < kRepeatSpecs.size(); ++k) {
                    const RepeatSpec spec = kRepeatSpecs[k];
                    std::uint32_t& due = nextRepeatAt[i][k];
                    bool fires = false;
                    if (justPressed) {
                        fires = true;
                        due = spec.delayMs;
                    }
                    if (ref.held && spec.periodMs > 0 && heldFor >= due) {
                        // A repeat falling on the press tick merges with the press.
                        fires = true;
                        while (due <= heldFor) due += spec.periodMs;
                    }
                    if (tracker.repeated(button, spec.delayMs, spec.periodMs) != fires) {
                        std::printf("FAIL repeated vs reference: seed %lu tick %d button %zu "
                                    "spec %zu\n",
                                    static_cast<unsigned long>(seed), tick, i, k);
                        assert(false);
                    }
                }
            }
            // combo(a, b): the first update where both are held.
            for (std::size_t a = 0; a < kButtonCount; ++a) {
                for (std::size_t b = 0; b < kButtonCount; ++b) {
                    const bool bothNow = reference[a].held && reference[b].held;
                    const bool bothBefore =
                        reference[a].previouslyHeld && reference[b].previouslyHeld;
                    expect(tracker.combo(buttonAt(a), buttonAt(b)) == (bothNow && !bothBefore),
                           "combo vs reference");
                }
            }
            now += controller.nextStep();
        }
    }
}

void testPressedOnFirstSample() {
    // Like the kernel's flags, which also start "not seen": a button already
    // down at the first sample is a press.
    ButtonTracker tracker;
    tracker.update(maskOf(Button::a), 5000);
    expect(tracker.pressed(Button::a), "down at first sample is a press");
    expect(tracker.held(Button::a), "held at first sample");
    expect(!tracker.released(Button::a), "not released");
    expectEq(tracker.heldMs(Button::a), 0, "heldMs 0 on the press");
    expect(!tracker.pressed(Button::b), "untouched button not pressed");
    tracker.update(maskOf(Button::a), 5020);
    expect(!tracker.pressed(Button::a), "still held is not a second press");
    expectEq(tracker.heldMs(Button::a), 20, "heldMs counts from the press");
}

void testReleaseReportsHowLongItWasHeld() {
    ButtonTracker tracker;
    tracker.update(0, 1000);
    tracker.update(maskOf(Button::x), 1010);
    tracker.update(maskOf(Button::x), 1030);
    tracker.update(0, 1250);
    expect(tracker.released(Button::x), "released on the first sample it's up");
    expect(!tracker.held(Button::x), "not held once released");
    expectEq(tracker.heldMs(Button::x), 240, "heldMs on release = total hold");
    expect(!tracker.heldFor(Button::x, 0), "heldFor is false once released");
    tracker.update(0, 1270);
    expect(!tracker.released(Button::x), "released for one update only");
    expectEq(tracker.heldMs(Button::x), 0, "heldMs 0 after the release update");
}

void testHeldForBoundary() {
    ButtonTracker tracker;
    tracker.update(maskOf(Button::l1), 1000);
    tracker.update(maskOf(Button::l1), 1499);
    expect(!tracker.heldFor(Button::l1, 500), "499ms is not 500ms");
    tracker.update(maskOf(Button::l1), 1500);
    expect(tracker.heldFor(Button::l1, 500), "exactly 500ms counts (>=)");
    expect(tracker.heldFor(Button::l1, 0), "heldFor(0) while held");
}

void testLongPressedFiresExactlyOncePerPress() {
    ButtonTracker tracker;
    std::vector<std::uint32_t> fired;
    std::vector<std::uint32_t> firedZero;
    for (std::uint32_t t = 0; t <= 2000; t += 20) {
        // Held from 100ms to 1300ms, then again from 1500ms on.
        const bool down = (t >= 100 && t < 1300) || t >= 1500;
        tracker.update(down ? maskOf(Button::r1) : 0, t);
        if (tracker.longPressed(Button::r1, 500)) fired.push_back(t);
        if (tracker.longPressed(Button::r1, 0)) firedZero.push_back(t);
    }
    expect(fired == std::vector<std::uint32_t>{600, 2000}, "longPressed(500) once per press");
    expect(firedZero == std::vector<std::uint32_t>{100, 1500}, "longPressed(0) on the press");

    // Released before the threshold: never fires.
    ButtonTracker tap;
    tap.update(maskOf(Button::r1), 0);
    tap.update(maskOf(Button::r1), 480);
    expect(!tap.longPressed(Button::r1, 500), "short of the threshold");
    tap.update(0, 520);
    expect(!tap.longPressed(Button::r1, 500), "released before the threshold");
}

std::vector<std::uint32_t> repeatTimes(std::uint32_t stepMs, std::uint32_t releaseAt,
                                       std::uint32_t delayMs, std::uint32_t periodMs) {
    ButtonTracker tracker;
    std::vector<std::uint32_t> times;
    for (std::uint32_t t = 0; t <= 1200; t += stepMs) {
        tracker.update(t < releaseAt ? maskOf(Button::up) : 0, t);
        if (tracker.repeated(Button::up, delayMs, periodMs)) times.push_back(t);
    }
    return times;
}

void testRepeatedSchedule() {
    // The press, then delayMs after it, then every periodMs.
    expect(repeatTimes(10, 1000, 400, 150) == std::vector<std::uint32_t>{0, 400, 550, 700, 850},
           "press, delay, period");
    // A 30ms tick fires on the first tick at or past each due time.
    expect(repeatTimes(30, 1000, 400, 150) == std::vector<std::uint32_t>{0, 420, 570, 720, 870},
           "tick resolution");
    // A 250ms tick spans several 50ms due times but fires once per update.
    expect(repeatTimes(250, 1100, 400, 50) == std::vector<std::uint32_t>{0, 500, 750, 1000},
           "at most once per update");
    // periodMs = 0: the press only.
    expect(repeatTimes(10, 1000, 100, 0) == std::vector<std::uint32_t>{0}, "period 0 = press only");
    // delayMs = 0: the first repeat falls on the press, the next a period later.
    expect(repeatTimes(10, 350, 0, 100) == std::vector<std::uint32_t>{0, 100, 200, 300}, "delay 0");
}

void testComboEitherOrderOrTogether() {
    const ButtonMask b = maskOf(Button::b);
    const ButtonMask down = maskOf(Button::down);

    ButtonTracker bFirst;
    bFirst.update(b, 0);
    expect(!bFirst.combo(Button::b, Button::down), "one of two is not a combo");
    bFirst.update(b | down, 20);
    expect(bFirst.combo(Button::b, Button::down), "B then DOWN");
    expect(bFirst.combo(Button::down, Button::b), "argument order doesn't matter");
    bFirst.update(b | down, 40);
    expect(!bFirst.combo(Button::b, Button::down), "fires once, not while both stay held");

    ButtonTracker downFirst;
    downFirst.update(down, 0);
    downFirst.update(b | down, 20);
    expect(downFirst.combo(Button::b, Button::down), "DOWN then B");

    ButtonTracker together;
    together.update(0, 0);
    together.update(b | down, 20);
    expect(together.combo(Button::b, Button::down), "both in the same sample");

    // Releasing one and pressing it again re-arms the combo.
    together.update(b, 40);
    together.update(b | down, 60);
    expect(together.combo(Button::b, Button::down), "re-armed after a release");
}

void testMasks() {
    ButtonTracker tracker;
    tracker.update(maskOf(Button::a) | maskOf(Button::l2), 0);
    tracker.update(maskOf(Button::l2) | maskOf(Button::right), 20);
    expectEq(tracker.heldMask(), maskOf(Button::l2) | maskOf(Button::right), "heldMask");
    expectEq(tracker.pressedMask(), maskOf(Button::right), "pressedMask");
    expectEq(tracker.releasedMask(), maskOf(Button::a), "releasedMask");
}

void testBitsPastTheTwelfthAreIgnored() {
    ButtonTracker tracker;
    tracker.update(0xFFFF, 0);
    expectEq(tracker.heldMask(), 0x0FFF, "only twelve buttons");
    expectEq(tracker.pressedMask(), 0x0FFF, "only twelve presses");
    // A Button value outside the enum is never held, and never indexes the
    // per-button arrays.
    const auto bogus = static_cast<Button>(12);
    expect(!tracker.held(bogus) && !tracker.pressed(bogus), "out-of-enum button not held");
    expectEq(tracker.heldMs(bogus), 0, "out-of-enum heldMs");
    expect(!tracker.longPressed(bogus, 0) && !tracker.repeated(bogus, 0, 10),
           "out-of-enum long press / repeat");
    // Far outside: no bit at all, rather than a shift past the int's width.
    static_assert(maskOf(static_cast<Button>(12)) == 0 && maskOf(static_cast<Button>(200)) == 0);
    static_assert(maskOf(Button::a) == 0x0800);
    expect(!tracker.held(static_cast<Button>(200)), "far out-of-enum button not held");
}

void testHoldTimeAcrossMillisWrap() {
    ButtonTracker tracker;
    tracker.update(maskOf(Button::y), 0xFFFFFFF0u);
    tracker.update(maskOf(Button::y), 0x00000010u);
    expectEq(tracker.heldMs(Button::y), 0x20, "heldMs across the 32-bit wrap");
    expect(tracker.heldFor(Button::y, 0x20), "heldFor across the wrap");
    expect(tracker.longPressed(Button::y, 0x20), "longPressed across the wrap");
    expectEq(tracker.nowMs(), 0x10, "nowMs is the latest sample's time");
}

void testQueriesDoNotConsumeAnything() {
    // Unlike get_digital_new_press(), asking twice (or from two places)
    // gives the same answer, and not asking for a while doesn't make a press
    // arrive late.
    ButtonTracker tracker;
    tracker.update(0, 0);
    tracker.update(maskOf(Button::l1), 20);
    expect(tracker.pressed(Button::l1) && tracker.pressed(Button::l1), "asking twice");
    tracker.update(maskOf(Button::l1), 40);
    tracker.update(maskOf(Button::l1), 60);
    expect(!tracker.pressed(Button::l1), "an unasked press is not delivered late");
}

} // namespace

int main() {
    testMatchesKernelLatchWhenEveryButtonIsReadEveryTick();
    testTimingQueriesMatchReferenceModel();
    testPressedOnFirstSample();
    testReleaseReportsHowLongItWasHeld();
    testHeldForBoundary();
    testLongPressedFiresExactlyOncePerPress();
    testRepeatedSchedule();
    testComboEitherOrderOrTogether();
    testMasks();
    testBitsPastTheTwelfthAreIgnored();
    testHoldTimeAcrossMillisWrap();
    testQueriesDoNotConsumeAnything();
    std::puts("button_tracker_test: all assertions passed");
    return 0;
}
