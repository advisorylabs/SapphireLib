#include "sapphirelib/telemetry/calibration_log.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

#include "pros/rtos.hpp"
#include "sapphirelib/telemetry/sd_card.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"

namespace sapphirelib::telemetry {

namespace {

// the longest the writer waits for the card for any one chunk. The Logger holds it a pass at a
// time, well under this
constexpr std::uint32_t kCardWaitMs = 2000;

constexpr const char* kNewFileHeader =
    "# SapphireLib calibration log: each calibration's results, added as it finishes.\n"
    "# Every number is exact: all the digits it needs to read back as the same double.\n"
    "# An '@' line starts each record: t is when it finished, in seconds since the program started.\n"
    "# Then a line per thing measured: kind,name=value,...  (was_* is the value it replaced)\n";

// whether this program run's "#" line is on the card yet
std::atomic<bool> runMarked{false};

void writeRecord(const std::string& kind, const std::string& body) {
    const bool firstThisRun = !runMarked.load(std::memory_order_acquire);
    const std::string text = (firstThisRun ? "\n# program started\n" : "") + body;
    for (const char* path : kCalibrationLogPaths) {
        if (appendCardFile(path, text, kNewFileHeader, kCardWaitMs)) {
            if (firstThisRun) runMarked.store(true, std::memory_order_release);
            SAPPHIRELIB_LOG_INFO("calib", "%s saved to %s", kind.c_str(), path);
            return;
        }
    }
    SAPPHIRELIB_LOG_ERROR("calib", "%s not saved: no SD card, or it stayed busy", kind.c_str());
}

} // namespace

void saveCalibration(const CalibrationRecord& record) {
    // stamped now, when it finished, not whenever the card comes free
    char stamp[48];
    std::snprintf(stamp, sizeof(stamp), "@ t=%.3f\n", sapphirelib::millis() / 1000.0);
    std::string body = stamp + record.text();
    // a task of its own: never the caller's to block on the card (the GUI's, or a competition
    // task, which PROS could delete holding SdCardLock). Lowest priority, as the Logger's writer
    pros::Task(
        [kind = record.kind(), body = std::move(body)] { writeRecord(kind, body); },
        TASK_PRIORITY_MIN + 1, TASK_STACK_DEPTH_DEFAULT, "calib log");
}

} // namespace sapphirelib::telemetry
