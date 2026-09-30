#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sapphirelib::diag {

/**
 * @brief Device types a SensorCheck can expect
 *
 * A check only asks the brain what's plugged into the port, so it works for devices SapphireLib
 * doesn't wrap too
 */
enum class DeviceKind { motor, imu, rotation, distance, optical };

/**
 * @brief A device you expect on a port
 */
struct SensorCheck {
    /** name shown in results, e.g. "Front left drive motor" */
    std::string label;

    /** the port. A negative (reversed) port is fine; anything outside +-1-21 fails */
    std::int8_t port;

    /** what should be plugged in */
    DeviceKind expected;
};

/**
 * @brief The result of one SensorCheck
 */
struct CheckResult {
    /** the check's name */
    std::string label;
    /** the check's port */
    std::int8_t port;
    /** whether the right device is plugged in */
    bool ok;

    /** empty when ok, otherwise what's plugged in instead, e.g. "nothing plugged in" */
    std::string detail;
};

/**
 * @brief Check what's plugged into a port right now
 *
 * Uses PROS's device registry, so the device doesn't need to be constructed first
 *
 * @param check the device to expect
 * @return CheckResult whether it's there, and what is if not
 *
 * @b Example
 * @code {.cpp}
 * auto result = sapphirelib::diag::runCheck({"IMU", 10, sapphirelib::diag::DeviceKind::imu});
 * if (!result.ok) printf("%s: %s\n", result.label.c_str(), result.detail.c_str());
 * @endcode
 */
CheckResult runCheck(const SensorCheck& check);

/**
 * @brief Run every check, in order
 */
std::vector<CheckResult> runChecks(const std::vector<SensorCheck>& checks);

} // namespace sapphirelib::diag
