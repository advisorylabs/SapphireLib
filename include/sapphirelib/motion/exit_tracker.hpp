/**
 * \file sapphirelib/motion/exit_tracker.hpp
 *
 * The settle/timeout bookkeeping every blocking motion shares, and the result
 * a motion returns. Pure — the caller passes in "now" — so it's host-tested
 * (tests/motion/exit_tracker_test.cpp) and reusable by mechanisms or any
 * other loop that needs "stay within threshold for N ms, or give up after M".
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

namespace sapphirelib::motion {

/// Why a motion stopped.
enum class ExitReason : std::uint8_t {
    /// Still going (ExitTracker::update() only; a finished motion never
    /// reports this).
    running,
    /// The error stayed within threshold for the settle time.
    settled,
    /// The hard timeout ran out first.
    timedOut,
    /// The motion couldn't start at all — an empty path, or a pose motion
    /// with no odometry to read. It commanded nothing.
    aborted,
};

/// A short printable name: "running", "settled", "timeout", "aborted".
const char* exitReasonName(ExitReason reason);

/// What a blocking motion reports when it returns. Ignoring it is fine — every
/// motion still stops the chassis on exit either way — but an autonomous
/// routine can check settled() to react to a motion that got stuck.
struct MotionResult {
    ExitReason reason = ExitReason::running;

    /// The motion's error on its last tick, in its own units (inches for
    /// drive motions, degrees for turns; distance to the target for pose
    /// motions).
    double finalError = 0.0;

    /// How long the motion ran.
    std::uint32_t elapsedMs = 0;

    bool settled() const { return reason == ExitReason::settled; }
};

/// Settle/timeout bookkeeping for a blocking loop, with the exact semantics
/// the drivetrain motions have always had: time spent within threshold
/// accumulates tick to tick (measured between consecutive update() calls),
/// leaving the threshold resets it to zero, the settle check comes before the
/// timeout check, and a timeout of 0 never times out.
class ExitTracker {
public:
    ExitTracker(std::uint32_t settleTimeMs, std::uint32_t timeoutMs, std::uint32_t startMs);

    /// Call once per loop tick, after computing that tick's error. Returns
    /// ExitReason::running until the loop should stop.
    ExitReason update(bool withinThreshold, std::uint32_t nowMs);

    /// Milliseconds from the start to the latest update().
    std::uint32_t elapsedMs() const;

private:
    std::uint32_t settleTimeMs_;
    std::uint32_t timeoutMs_;
    std::uint32_t startMs_;
    std::uint32_t lastTickMs_;
    std::uint32_t settledForMs_ = 0;
    std::uint32_t elapsedMs_ = 0;
};

} // namespace sapphirelib::motion
