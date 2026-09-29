/*
 * Host-side unit test for sapphirelib::mechanism::JamDetector — no
 * PROS/embedded dependencies, so it builds and runs with a normal desktop
 * compiler.
 *
 * (A block comment, not //, so the backslash-continued build line doesn't
 * trip -Wcomment.)
 *
 * Build & run:
 *   g++ -std=c++20 -Iinclude tests/mechanism/jam_detector_test.cpp \
 *       src/sapphirelib/mechanism/jam_detector.cpp \
 *       -o jam_detector_test && ./jam_detector_test
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <limits>
#include <random>

#include "sapphirelib/mechanism/jam_detector.hpp"

using sapphirelib::mechanism::JamConfig;
using sapphirelib::mechanism::JamDetector;

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                       \
            std::exit(1);                                                                          \
        }                                                                                          \
    } while (0)

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::uint32_t kTickMs = 10;

// The defaults, switched on: watch commands of 4V and up, stalled below 5 RPM
// for 250ms, then 6V the other way for 200ms.
constexpr JamConfig kEnabled{.enabled = true};

/// Feeds `ticks` ticks of the same command and velocity starting at `nowMs`,
/// checking every output is `expected`. Returns the time after the last tick.
std::uint32_t expectFor(JamDetector& jam, int ticks, double command, double rpm,
                        std::uint32_t nowMs, double expected) {
    for (int i = 0; i < ticks; ++i, nowMs += kTickMs) {
        const double out = jam.update(command, rpm, nowMs);
        if (out != expected) {
            std::printf("  at t=%lu: got %f, expected %f\n", static_cast<unsigned long>(nowMs), out,
                        expected);
        }
        CHECK(out == expected);
    }
    return nowMs;
}

void testDisabledPassesThrough() {
    JamDetector jam; // default config: off
    std::uint32_t now = 1000;
    now = expectFor(jam, 500, 12.0, 0.0, now, 12.0); // stalled for 5s: still just the command
    CHECK(!jam.reversing());
    CHECK(jam.update(-7.5, 0.0, now) == -7.5);
    CHECK(std::isnan(jam.update(kNaN, 0.0, now + kTickMs)));
}

void testNoJamWhileTurning() {
    JamDetector jam(kEnabled);
    expectFor(jam, 500, 12.0, 300.0, 1000, 12.0);
    expectFor(jam, 500, -12.0, -300.0, 6000, -12.0);
    CHECK(!jam.reversing());
}

void testSpinUpIsNotAJam() {
    // From rest to speed well inside stallMs.
    JamDetector jam(kEnabled);
    std::uint32_t now = 1000;
    for (int i = 0; i < 100; ++i, now += kTickMs) {
        const double rpm = std::min(300.0, i * 20.0); // 0 at first, moving by 10ms
        CHECK(jam.update(12.0, rpm, now) == 12.0);
    }
}

void testStallPulsesAfterStallMsThenResumes() {
    JamDetector jam(kEnabled);
    std::uint32_t now = 1000;
    // Stalled from t=1000: the command until 250ms have passed (t=1240 is
    // 240ms in)...
    now = expectFor(jam, 25, 12.0, 0.0, now, 12.0);
    CHECK(now == 1250);
    // ...and at exactly 250ms, the pulse: 6V the other way, for 200ms.
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
    CHECK(jam.reversing());
    now = expectFor(jam, 19, 12.0, 0.0, now + kTickMs, -6.0);
    CHECK(now == 1450);
    // Exactly 200ms after it started, back to the command...
    CHECK(jam.update(12.0, 0.0, now) == 12.0);
    CHECK(!jam.reversing());
    // ...and the stall timer starts over from there: still stuck, so another
    // pulse 250ms later.
    now = expectFor(jam, 24, 12.0, 0.0, now + kTickMs, 12.0); // t=1460..1690
    CHECK(now == 1700);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
}

void testPulseClearsJam() {
    JamDetector jam(kEnabled);
    std::uint32_t now = expectFor(jam, 25, 12.0, 0.0, 1000, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
    // Spinning backward freely during the pulse, then forward again after.
    now = expectFor(jam, 19, 12.0, -150.0, now + kTickMs, -6.0);
    expectFor(jam, 200, 12.0, 250.0, now, 12.0);
    CHECK(!jam.reversing());
}

void testNegativeCommandPulsesPositive() {
    JamDetector jam(kEnabled);
    std::uint32_t now = expectFor(jam, 25, -8.0, 0.0, 1000, -8.0);
    CHECK(jam.update(-8.0, 0.0, now) == 6.0);
}

void testWeakCommandIsNotWatched() {
    JamDetector jam(kEnabled);
    // Just under the threshold: a slow roller on purpose.
    expectFor(jam, 500, 3.99, 0.0, 1000, 3.99);
    expectFor(jam, 500, -3.99, 0.0, 6000, -3.99);
    // Exactly at it: watched.
    const std::uint32_t now = expectFor(jam, 25, 4.0, 0.0, 11000, 4.0);
    CHECK(jam.update(4.0, 0.0, now) == -6.0);
}

void testLowCommandRestartsStallTimer() {
    JamDetector jam(kEnabled);
    std::uint32_t now = expectFor(jam, 20, 12.0, 0.0, 1000, 12.0); // 200ms stalled
    now = expectFor(jam, 1, 0.0, 0.0, now, 0.0);                   // let go for a tick
    // A fresh 250ms from here, not the 50ms left before.
    now = expectFor(jam, 25, 12.0, 0.0, now, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
}

void testReleaseDuringPulseEndsIt() {
    JamDetector jam(kEnabled);
    std::uint32_t now = expectFor(jam, 25, 12.0, 0.0, 1000, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
    now += kTickMs;
    // The driver lets go mid-pulse: it stops reversing at once.
    CHECK(jam.update(0.0, 0.0, now) == 0.0);
    CHECK(!jam.reversing());
    // Pressing again starts a fresh stall timer.
    now = expectFor(jam, 25, 12.0, 0.0, now + kTickMs, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
}

void testDirectionChangeRestartsAndEndsPulse() {
    JamDetector jam(kEnabled);
    std::uint32_t now = expectFor(jam, 20, 12.0, 0.0, 1000, 12.0); // 200ms stalled intaking
    // Switched to outtaking: the roller has to turn around, so no pulse 50ms
    // later off the intake's stall time.
    now = expectFor(jam, 25, -12.0, 0.0, now, -12.0);
    CHECK(jam.update(-12.0, 0.0, now) == 6.0);
    now += kTickMs;
    // Switched back mid-pulse: the new command goes straight through.
    CHECK(jam.update(12.0, 0.0, now) == 12.0);
    CHECK(!jam.reversing());
}

void testMissingMotorNeverStalls() {
    // PROS reads an unplugged motor's velocity as infinity.
    for (double rpm : {kInf, -kInf, kNaN}) {
        JamDetector jam(kEnabled);
        expectFor(jam, 500, 12.0, rpm, 1000, 12.0);
        CHECK(!jam.reversing());
    }
}

void testGapRestartsTiming() {
    // 200ms stalled, then a 101ms gap with no calls: the old stall time no
    // longer counts, so no pulse on the first call after it.
    JamDetector jam(kEnabled);
    std::uint32_t now = expectFor(jam, 21, 12.0, 0.0, 1000, 12.0); // t=1000..1200
    now = 1200 + 101;
    now = expectFor(jam, 25, 12.0, 0.0, now, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);

    // A gap of exactly 100ms is still an ordinary (slow) tick.
    JamDetector steady(kEnabled);
    expectFor(steady, 21, 12.0, 0.0, 1000, 12.0);  // t=1000..1200
    CHECK(steady.update(12.0, 0.0, 1300) == -6.0); // 300ms stalled
}

void testResetForgetsEverything() {
    JamDetector jam(kEnabled);
    std::uint32_t now = expectFor(jam, 25, 12.0, 0.0, 1000, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
    jam.reset();
    CHECK(!jam.reversing());
    now = expectFor(jam, 25, 12.0, 0.0, now + kTickMs, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
}

void testNegativeReverseVoltsStillOpposesCommand() {
    JamDetector jam(JamConfig{.enabled = true, .reverseVolts = -6.0});
    std::uint32_t now = expectFor(jam, 25, 12.0, 0.0, 1000, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
}

void testAcrossClockWrap() {
    JamDetector jam(kEnabled);
    std::uint32_t now = 0xFFFFFF00u; // 256ms before millis() wraps
    now = expectFor(jam, 25, 12.0, 0.0, now, 12.0);
    CHECK(jam.update(12.0, 0.0, now) == -6.0);
    now = expectFor(jam, 19, 12.0, 0.0, now + kTickMs, -6.0);
    CHECK(jam.update(12.0, 0.0, now) == 12.0);
}

/// Random commands, speeds, and tick jitter (with the odd long gap), checking
/// the rules every output must follow.
void testRandomTracesKeepTheRules() {
    const JamConfig config{.enabled = true,
                           .minCommandVolts = 4.0,
                           .stallRpm = 5.0,
                           .stallMs = 120,
                           .reverseVolts = 5.0,
                           .reverseMs = 80};
    std::mt19937_64 rng(0x7a44ed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    long pulses = 0;
    for (int trace = 0; trace < 300; ++trace) {
        JamDetector jam(config);
        std::uint32_t now = static_cast<std::uint32_t>(unit(rng) * 4e9);
        double command = 0.0;
        // Recent calls, for "was it really slow and watched for stallMs?".
        struct Call {
            std::uint32_t nowMs;
            double command;
            bool slow;
        };
        std::deque<Call> history;
        bool wasReversing = false;
        std::uint32_t pulseStartMs = 0;
        for (int tick = 0; tick < 3000; ++tick) {
            now += unit(rng) < 0.01 ? 101 + static_cast<std::uint32_t>(unit(rng) * 500)
                                    : 5 + static_cast<std::uint32_t>(unit(rng) * 20);
            if (unit(rng) < 0.03) {
                const double options[] = {12.0, -12.0, 0.0, 3.0, 4.0, -6.0, 8.5};
                command = options[static_cast<int>(unit(rng) * 7.0) % 7];
            }
            const double rpm = unit(rng) < 0.7 ? unit(rng) * 4.0 : unit(rng) * 400.0 - 200.0;
            const double out = jam.update(command, rpm, now);

            if (history.size() > 0 && now - history.back().nowMs > 100) history.clear();
            history.push_back({now, command, std::fabs(rpm) < config.stallRpm});

            // The output is always the command or a pulse opposing it.
            const bool pulsing = out != command;
            CHECK(pulsing == jam.reversing());
            if (pulsing) {
                CHECK(std::fabs(command) >= config.minCommandVolts);
                CHECK(out == (command > 0.0 ? -config.reverseVolts : config.reverseVolts));
                if (!wasReversing) {
                    // A new pulse: every call over the last stallMs was slow,
                    // in this same direction, with no gap in between.
                    ++pulses;
                    pulseStartMs = now;
                    bool covered = false;
                    for (auto it = history.rbegin(); it != history.rend(); ++it) {
                        CHECK(it->slow);
                        CHECK((it->command > 0.0) == (command > 0.0));
                        CHECK(std::fabs(it->command) >= config.minCommandVolts);
                        if (now - it->nowMs >= config.stallMs) {
                            covered = true;
                            break;
                        }
                    }
                    CHECK(covered);
                } else {
                    // An ongoing pulse never outlasts reverseMs.
                    CHECK(now - pulseStartMs < config.reverseMs);
                }
            }
            wasReversing = pulsing;
        }
    }
    CHECK(pulses > 1000); // the traces really did exercise jams
    std::printf("  random traces: %ld pulses, all within the rules\n", pulses);
}

} // namespace

int main() {
    testDisabledPassesThrough();
    testNoJamWhileTurning();
    testSpinUpIsNotAJam();
    testStallPulsesAfterStallMsThenResumes();
    testPulseClearsJam();
    testNegativeCommandPulsesPositive();
    testWeakCommandIsNotWatched();
    testLowCommandRestartsStallTimer();
    testReleaseDuringPulseEndsIt();
    testDirectionChangeRestartsAndEndsPulse();
    testMissingMotorNeverStalls();
    testGapRestartsTiming();
    testResetForgetsEverything();
    testNegativeReverseVoltsStillOpposesCommand();
    testAcrossClockWrap();
    testRandomTracesKeepTheRules();
    std::puts("jam_detector_test: all assertions passed");
    return 0;
}
