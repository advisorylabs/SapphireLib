#pragma once

#include <cstdint>

// timing helpers for per-tick code. None of them read the clock: pass in "now", taken once per
// tick, and hand that same value to everything that tick. Mixing a tick's "now" with a timestamp
// taken later in the tick is how now - stamp underflows to ~49 days

namespace sapphirelib {

/**
 * @brief Get the time between two millis() readings, safe across the 32 bit wrap
 *
 * @param sinceMs the earlier reading, in milliseconds
 * @param nowMs the later reading, in milliseconds
 * @return std::uint32_t elapsed time, in milliseconds. 0 if nowMs is before sinceMs
 *
 * @b Example
 * @code {.cpp}
 * if (sapphirelib::elapsedMs(pressedAt, now) > 500) {
 *     // held for more than half a second
 * }
 * @endcode
 */
constexpr std::uint32_t elapsedMs(std::uint32_t sinceMs, std::uint32_t nowMs) {
    const std::uint32_t difference = nowMs - sinceMs;
    // a real elapsed time of 2^31 ms (~24.8 days) can't happen in one run, so a difference that
    // large can only be a negative one
    return difference >= 0x80000000u ? 0u : difference;
}

/**
 * @brief Measures the time since restart(). Stopped until the first restart()
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::Stopwatch outtake;
 * outtake.restart(now);
 * // later
 * if (outtake.hasElapsed(400, now)) intake.move_voltage(0);
 * @endcode
 */
class Stopwatch {
public:
    /**
     * @brief Start timing from now
     *
     * @param nowMs the current time, in milliseconds
     */
    constexpr void restart(std::uint32_t nowMs) {
        startMs_ = nowMs;
        running_ = true;
    }

    /**
     * @brief Stop the stopwatch
     */
    constexpr void stop() { running_ = false; }

    /**
     * @brief Whether the stopwatch is running
     */
    constexpr bool running() const { return running_; }

    /**
     * @brief Get the time since restart()
     *
     * @param nowMs the current time, in milliseconds
     * @return std::uint32_t elapsed time, in milliseconds. 0 while stopped
     */
    constexpr std::uint32_t elapsedMs(std::uint32_t nowMs) const {
        return running_ ? sapphirelib::elapsedMs(startMs_, nowMs) : 0u;
    }

    /**
     * @brief Whether the stopwatch is running and at least `ms` has passed since restart()
     *
     * @param ms how long, in milliseconds
     * @param nowMs the current time, in milliseconds
     */
    constexpr bool hasElapsed(std::uint32_t ms, std::uint32_t nowMs) const {
        return running_ && elapsedMs(nowMs) >= ms;
    }

private:
    std::uint32_t startMs_ = 0;
    bool running_ = false;
};

/**
 * @brief A bool that remembers when it last changed
 *
 * For "has the piston been out for 400ms", "has the lift been in tolerance for 200ms", or
 * debouncing a sensor
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::TimedFlag seesPiece;
 * seesPiece.set(distance.get() < 50, now);
 * // only react once the sensor has seen the piece for 50ms
 * if (seesPiece.trueFor(50, now)) claw.set(true, now);
 * @endcode
 */
class TimedFlag {
public:
    /**
     * @brief Construct a new TimedFlag
     *
     * @param initial the starting value. false by default
     * @param sinceMs when it took that value, in milliseconds. 0 (program start) by default
     */
    constexpr explicit TimedFlag(bool initial = false, std::uint32_t sinceMs = 0)
        : value_(initial), changedAtMs_(sinceMs) {}

    /**
     * @brief Set the value
     *
     * The time is only recorded when the value changes, so calling this every tick with the same
     * value keeps counting from the real change
     *
     * @param value the new value
     * @param nowMs the current time, in milliseconds
     * @return true the value changed
     */
    constexpr bool set(bool value, std::uint32_t nowMs) {
        if (value == value_) return false;
        value_ = value;
        changedAtMs_ = nowMs;
        return true;
    }

    /**
     * @brief Get the current value
     */
    constexpr bool value() const { return value_; }

    /**
     * @brief Get how long the current value has been held
     *
     * @param nowMs the current time, in milliseconds
     * @return std::uint32_t time since the last change, in milliseconds
     */
    constexpr std::uint32_t msInState(std::uint32_t nowMs) const {
        return elapsedMs(changedAtMs_, nowMs);
    }

    /**
     * @brief Whether the value is true and has been for at least `ms`
     */
    constexpr bool trueFor(std::uint32_t ms, std::uint32_t nowMs) const {
        return value_ && msInState(nowMs) >= ms;
    }

    /**
     * @brief Whether the value is false and has been for at least `ms`
     */
    constexpr bool falseFor(std::uint32_t ms, std::uint32_t nowMs) const {
        return !value_ && msInState(nowMs) >= ms;
    }

private:
    bool value_;
    std::uint32_t changedAtMs_;
};

/**
 * @brief Notices when a per-tick function stopped being called for a while
 *
 * After autonomous, a disable, or a blocking routine, state with stale timing (a half-finished
 * sequence, a PID's derivative) should be dropped
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::GapDetector resumed(250);
 * // in opcontrol's loop
 * if (resumed.update(now)) liftPID.reset();
 * @endcode
 */
class GapDetector {
public:
    /**
     * @brief Construct a new GapDetector
     *
     * @param maxGapMs the longest time between update() calls that isn't a gap, in milliseconds
     */
    constexpr explicit GapDetector(std::uint32_t maxGapMs) : maxGapMs_(maxGapMs) {}

    /**
     * @brief Record a tick. Call this once per tick
     *
     * @param nowMs the current time, in milliseconds
     * @return true this is the first call, or more than maxGapMs passed since the last one
     */
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
