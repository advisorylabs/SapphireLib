#pragma once

#include <cstdint>

namespace sapphirelib::motion {

/**
 * @brief Why a motion stopped
 */
enum class ExitReason : std::uint8_t {
    /** still going. Only ExitTracker::update() returns this */
    running,
    /** the error stayed within threshold for the settle time */
    settled,
    /** the timeout ran out first */
    timedOut,
    /** the motion couldn't start (an empty path, or no odometry) and commanded nothing */
    aborted,
    /**
     * the competition mode changed under it (autonomous ended, a disable) and it stopped where it
     * was. Only a motion on a task of its own (a MotionQueue's) gets to say so: one run from
     * autonomous() itself is deleted with that task. See competitionModeChanged()
     */
    interrupted,
};

/**
 * @brief Get a short name for an exit reason
 *
 * @param reason the exit reason
 * @return const char* "running", "settled", "timeout", "aborted", or "interrupted"
 */
const char* exitReasonName(ExitReason reason);

/** pros::competition::get_status()'s bits, here so competitionModeChanged() is pure */
inline constexpr std::uint8_t kCompetitionDisabled = 1 << 0;
inline constexpr std::uint8_t kCompetitionAutonomous = 1 << 1;
inline constexpr std::uint8_t kCompetitionConnected = 1 << 2;

/**
 * @brief Whether a motion started under one competition status should stop under another
 *
 * Under competition control (a field or switch connected as the motion started), once the mode
 * isn't the one it started in: autonomous ending, a disable, driver control starting. A motion on
 * a task of its own outlives autonomous(), and would otherwise drive on into driver control,
 * fighting the driver until it timed out. Off the field nothing interrupts a motion
 *
 * @param startStatus pros::competition::get_status() as the motion started
 * @param nowStatus pros::competition::get_status() now
 * @return true if the motion should stop
 */
constexpr bool competitionModeChanged(std::uint8_t startStatus, std::uint8_t nowStatus) {
    if ((startStatus & kCompetitionConnected) == 0) return false;
    constexpr std::uint8_t kMode = kCompetitionDisabled | kCompetitionAutonomous;
    return (startStatus & kMode) != (nowStatus & kMode);
}

/**
 * @brief What a blocking motion returns
 *
 * Ignoring it is fine, since every motion stops the chassis either way, but a routine can check
 * settled() to react to a motion that got stuck
 *
 * @b Example
 * @code {.cpp}
 * auto result = drivetrain().driveDistance(24);
 * if (!result.settled()) {
 *     printf("stopped %f inches short\n", result.finalError);
 * }
 * @endcode
 */
struct MotionResult {
    /** why the motion stopped */
    ExitReason reason = ExitReason::running;

    /**
     * the error on the last tick: inches for drive motions, degrees for turns, distance to the
     * target for pose motions
     */
    double finalError = 0.0;

    /** how long the motion ran, in milliseconds */
    std::uint32_t elapsedMs = 0;

    /**
     * @brief Whether the motion settled at its target
     */
    bool settled() const { return reason == ExitReason::settled; }
};

/**
 * @brief Settle and timeout tracking for a blocking loop
 *
 * Time within threshold adds up tick to tick, leaving the threshold resets it, settling is checked
 * before the timeout, and a timeout of 0 never times out
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::motion::ExitTracker exit(200, 2000, sapphirelib::millis());
 * while (exit.update(std::abs(error) < 1.0, sapphirelib::millis()) ==
 *        sapphirelib::motion::ExitReason::running) {
 *     // run the loop
 *     pros::delay(10);
 * }
 * @endcode
 */
class ExitTracker {
public:
    /**
     * @brief Construct a new ExitTracker
     *
     * @param settleTimeMs how long the error must stay within threshold, in milliseconds
     * @param timeoutMs longest time the loop can run, in milliseconds. 0 for no limit
     * @param startMs when the loop started, in milliseconds
     */
    ExitTracker(std::uint32_t settleTimeMs, std::uint32_t timeoutMs, std::uint32_t startMs);

    /**
     * @brief Update the tracker. Call it once per tick, after working out the error
     *
     * @param withinThreshold whether the error is within threshold this tick
     * @param nowMs the current time, in milliseconds
     * @return ExitReason running until the loop should stop
     */
    ExitReason update(bool withinThreshold, std::uint32_t nowMs);

    /**
     * @brief Get the time from the start to the latest update(), in milliseconds
     */
    std::uint32_t elapsedMs() const;

    /**
     * @brief Give the loop longer before it times out, for a motion whose work grew while it ran
     * (a trajectory carrying on into the next of a chain). Does nothing with no timeout
     *
     * @param extraMs how much longer, in milliseconds
     */
    void extendTimeout(std::uint32_t extraMs);

private:
    std::uint32_t settleTimeMs_;
    std::uint32_t timeoutMs_;
    std::uint32_t startMs_;
    std::uint32_t lastTickMs_;
    std::uint32_t settledForMs_ = 0;
    std::uint32_t elapsedMs_ = 0;
};

} // namespace sapphirelib::motion
