#include "sapphirelib/util/wait.hpp"

#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib {

bool waitUntil(const std::function<bool()>& done, std::uint32_t timeoutMs, std::uint32_t pollMs) {
    // A 0ms delay only yields to tasks of the same priority, so polling with
    // it would starve every lower-priority task (the GUI, telemetry's writer)
    // for the whole wait. 1ms is the finest wait the scheduler has anyway.
    const std::uint32_t delayPerPollMs = pollMs > 0 ? pollMs : 1;
    const std::uint32_t startMs = millis();
    while (true) {
        // The condition before the timeout, so one that comes true right at
        // the deadline still counts as done.
        if (done()) return true;
        if (timeoutMs > 0 && elapsedMs(startMs, millis()) >= timeoutMs) return false;
        delayMs(delayPerPollMs);
    }
}

} // namespace sapphirelib
