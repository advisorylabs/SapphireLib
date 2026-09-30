#include "sapphirelib/util/wait.hpp"

#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib {

bool waitUntil(const std::function<bool()>& done, std::uint32_t timeoutMs, std::uint32_t pollMs) {
    // a 0ms delay only yields to tasks of the same priority, which would starve the GUI and
    // the telemetry writer, so poll at least every 1ms
    const std::uint32_t delayPerPollMs = pollMs > 0 ? pollMs : 1;
    const std::uint32_t startMs = millis();
    while (true) {
        // check the condition before the timeout, so one that comes true right at the deadline
        // still counts
        if (done()) return true;
        if (timeoutMs > 0 && elapsedMs(startMs, millis()) >= timeoutMs) return false;
        delayMs(delayPerPollMs);
    }
}

} // namespace sapphirelib
