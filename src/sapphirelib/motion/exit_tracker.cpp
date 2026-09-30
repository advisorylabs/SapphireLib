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

// the same bookkeeping every motion loop used to carry inline, in the same order, so motions
// behave exactly as before. elapsedMs() counts a time before the last tick as no time passing
ExitReason ExitTracker::update(bool withinThreshold, std::uint32_t nowMs) {
    ExitReason reason = ExitReason::running;
    if (withinThreshold) {
        settledForMs_ += sapphirelib::elapsedMs(lastTickMs_, nowMs);
        if (settledForMs_ >= settleTimeMs_) reason = ExitReason::settled;
    } else {
        settledForMs_ = 0;
    }
    // settling is checked first, so settling on the same tick as the timeout counts as settled
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
