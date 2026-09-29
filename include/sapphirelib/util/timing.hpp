/**
 * \file sapphirelib/util/timing.hpp
 *
 * Clock-free timing primitives for per-tick code: wrap-safe elapsed time, a
 * stopwatch, a bool that remembers when it last changed, and a detector for
 * gaps between calls. None of them read a clock — every call that needs
 * "now" takes it as an argument — so they're deterministic and host-tested
 * (tests/util/timing_test.cpp).
 *
 * The rule they exist to enforce: take "now" once per tick and hand that same
 * value to everything that tick. Comparing a tick-start "now" against a
 * timestamp some library took internally later in the tick is how
 * `now - stamp` underflows to ~49 days. Nothing here takes timestamps behind
 * your back, and elapsedMs() treats "now before stamp" as 0.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

namespace sapphirelib {

/// Milliseconds from `sinceMs` to `nowMs`, correct across the 32-bit wrap of
/// pros::millis(). If `nowMs` is *before* `sinceMs`, returns 0 instead of the
/// ~4 billion a bare unsigned subtraction would give.
constexpr std::uint32_t elapsedMs(std::uint32_t sinceMs, std::uint32_t nowMs) {
    const std::uint32_t difference = nowMs - sinceMs;
    // A true elapsed time of 2^31 ms (~24.8 days) can't happen within a
    // program run, so a difference that large can only be a negative one.
    return difference >= 0x80000000u ? 0u : difference;
}

/// Measures time since restart(). Stopped until the first restart().
class Stopwatch {
public:
    constexpr void restart(std::uint32_t nowMs) {
        startMs_ = nowMs;
        running_ = true;
    }

    constexpr void stop() { running_ = false; }

    constexpr bool running() const { return running_; }

    /// Time since restart(); 0 while stopped.
    constexpr std::uint32_t elapsedMs(std::uint32_t nowMs) const {
        return running_ ? sapphirelib::elapsedMs(startMs_, nowMs) : 0u;
    }

    /// Running, and at least `ms` since restart().
    constexpr bool hasElapsed(std::uint32_t ms, std::uint32_t nowMs) const {
        return running_ && elapsedMs(nowMs) >= ms;
    }

private:
    std::uint32_t startMs_ = 0;
    bool running_ = false;
};

/// A bool that remembers when it last changed. Examples: "has the piston been
/// out for 400ms", "has the lift been within tolerance for 200ms", "has the
/// sensor seen a piece for 50ms" (a debounce).
class TimedFlag {
public:
    /// Starts at `initial`, counted as having held it since `sinceMs`
    /// (0 = since the program started).
    constexpr explicit TimedFlag(bool initial = false, std::uint32_t sinceMs = 0)
        : value_(initial), changedAtMs_(sinceMs) {}

    /// Sets the value, recording the time only when it actually changes, so
    /// calling this every tick with the same value keeps counting from the
    /// real change. True if it changed.
    constexpr bool set(bool value, std::uint32_t nowMs) {
        if (value == value_) return false;
        value_ = value;
        changedAtMs_ = nowMs;
        return true;
    }

    constexpr bool value() const { return value_; }

    /// How long the current value has been held.
    constexpr std::uint32_t msInState(std::uint32_t nowMs) const {
        return elapsedMs(changedAtMs_, nowMs);
    }

    constexpr bool trueFor(std::uint32_t ms, std::uint32_t nowMs) const {
        return value_ && msInState(nowMs) >= ms;
    }

    constexpr bool falseFor(std::uint32_t ms, std::uint32_t nowMs) const {
        return !value_ && msInState(nowMs) >= ms;
    }

private:
    bool value_;
    std::uint32_t changedAtMs_;
};

/// Notices that a per-tick function stopped being called for a while (after
/// autonomous, a disable, or a blocking routine run from opcontrol()), so
/// state whose timing went stale can be dropped: a half-finished sequence,
/// a PID's derivative memory, what the controller screen was showing.
class GapDetector {
public:
    /// A gap is more than `maxGapMs` between consecutive update() calls.
    constexpr explicit GapDetector(std::uint32_t maxGapMs) : maxGapMs_(maxGapMs) {}

    /// Call once per tick. True on the first call ever, and on any call more
    /// than maxGapMs after the previous one.
    constexpr bool update(std::uint32_t nowMs) {
        const bool gap = !started_ || elapsedMs(lastMs_, nowMs) > maxGapMs_;
        started_ = true;
        lastMs_ = nowMs;
        return gap;
    }

private:
    std::uint32_t maxGapMs_;
    std::uint32_t lastMs_ = 0;
    bool started_ = false;
};

} // namespace sapphirelib
