#include "sapphirelib/motion/exit_tracker.hpp"

#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::motion {

const char* exitReasonName(ExitReason reason) {
    switch (reason) {
        case ExitReason::running: return "running";
        case ExitReason::settled: return "settled";
        case ExitReason::timedOut: return "timeout";
        case ExitReason::aborted: return "aborted";
    }
    return "unknown"; // only reachable by casting an out-of-range value
}

ExitTracker::ExitTracker(std::uint32_t settleTimeMs, std::uint32_t timeoutMs, std::uint32_t startMs)
    : settleTimeMs_(settleTimeMs), timeoutMs_(timeoutMs), startMs_(startMs), lastTickMs_(startMs) {}

// This is the bookkeeping every drivetrain motion loop used to carry inline,
// in the same order, so moving a loop onto it changes no motion's behavior
// (tests/motion/exit_tracker_test.cpp checks that against a copy of the old
// loop). Differences are measured with sapphirelib::elapsedMs(), which is
// the same number as the old bare `now - then` whenever time moves forward —
// the only case pros::millis() produces — but counts a `nowMs` from before
// the previous tick as no time passing, rather than as ~49 days of settling.
ExitReason ExitTracker::update(bool withinThreshold, std::uint32_t nowMs) {
    ExitReason reason = ExitReason::running;
    if (withinThreshold) {
        settledForMs_ += sapphirelib::elapsedMs(lastTickMs_, nowMs);
        if (settledForMs_ >= settleTimeMs_) reason = ExitReason::settled;
    } else {
        settledForMs_ = 0;
    }
    // Settling is checked first, so a motion that settles on the same tick
    // its timeout runs out reports settled.
    if (reason == ExitReason::running && timeoutMs_ > 0 &&
        sapphirelib::elapsedMs(startMs_, nowMs) >= timeoutMs_) {
        reason = ExitReason::timedOut;
    }
    lastTickMs_ = nowMs;
    elapsedMs_ = sapphirelib::elapsedMs(startMs_, nowMs);
    return reason;
}

std::uint32_t ExitTracker::elapsedMs() const { return elapsedMs_; }

} // namespace sapphirelib::motion
