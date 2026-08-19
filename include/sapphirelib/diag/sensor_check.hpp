/**
 * \file sapphirelib/diag/sensor_check.hpp
 *
 * Startup/live diagnostics: verifies that the sensors you tell it to expect
 * are actually the devices plugged into their configured ports, so a
 * mis-wired or unplugged sensor shows up as an on-screen warning instead of
 * a mysterious "the robot's not turning right" during a match.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sapphirelib::diag {

/// The device types SapphireLib knows how to check for. Extend as more
/// sensor wrappers are added (e.g. distance, vision).
enum class DeviceKind { motor, imu, rotation };

/// One sensor's expected configuration.
struct SensorCheck {
    /// Human-readable name shown in results, e.g. "Front-left drive motor".
    std::string label;

    /// Port as you'd pass it to the rest of SapphireLib — sign is allowed
    /// (negative = reversed motor) and ignored for the check, since a
    /// device is either plugged into a port or it isn't, regardless of
    /// which direction a motor on it spins.
    std::int8_t port;

    DeviceKind expected;
};

/// The result of running one SensorCheck.
struct CheckResult {
    std::string label;
    std::int8_t port;
    bool ok;

    /// Empty when ok — otherwise a short description of what's actually
    /// plugged in instead, e.g. "nothing plugged in" or "found a motor".
    std::string detail;
};

/// Reads what's actually plugged into `check.port` right now (via PROS's
/// device registry — doesn't require the device to have been constructed
/// first) and compares it against `check.expected`.
CheckResult runCheck(const SensorCheck& check);

/// Runs every check in order.
std::vector<CheckResult> runChecks(const std::vector<SensorCheck>& checks);

} // namespace sapphirelib::diag
