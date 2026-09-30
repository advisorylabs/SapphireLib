#include "sapphirelib/diag/sensor_check.hpp"

#include <cstdlib>

#include "pros/apix.h"

namespace sapphirelib::diag {

namespace {

// smart ports on a V5 brain
constexpr int kMaxPort = 21;

pros::c::v5_device_e_t expectedRegistryType(DeviceKind kind) {
    switch (kind) {
        case DeviceKind::motor: return pros::c::E_DEVICE_MOTOR;
        case DeviceKind::imu: return pros::c::E_DEVICE_IMU;
        case DeviceKind::rotation: return pros::c::E_DEVICE_ROTATION;
        case DeviceKind::distance: return pros::c::E_DEVICE_DISTANCE;
        case DeviceKind::optical: return pros::c::E_DEVICE_OPTICAL;
    }
    return pros::c::E_DEVICE_UNDEFINED;
}

std::string deviceTypeName(pros::c::v5_device_e_t type) {
    switch (type) {
        case pros::c::E_DEVICE_NONE: return "nothing plugged in";
        case pros::c::E_DEVICE_MOTOR: return "a motor";
        case pros::c::E_DEVICE_ROTATION: return "a rotation sensor";
        case pros::c::E_DEVICE_IMU: return "an IMU";
        case pros::c::E_DEVICE_DISTANCE: return "a distance sensor";
        case pros::c::E_DEVICE_RADIO: return "a radio";
        case pros::c::E_DEVICE_VISION: return "a vision sensor";
        case pros::c::E_DEVICE_ADI: return "an ADI expander";
        case pros::c::E_DEVICE_OPTICAL: return "an optical sensor";
        case pros::c::E_DEVICE_GPS: return "a GPS sensor";
        default: return "an unrecognized device";
    }
}

} // namespace

CheckResult runCheck(const SensorCheck& check) {
    // caught here, since the conversion below would turn port 0 into 255 and report a wiring
    // fault instead of a typo
    const int portNumber = std::abs(check.port);
    if (portNumber < 1 || portNumber > kMaxPort) {
        return CheckResult{check.label, check.port, false, "invalid port (must be 1-21)"};
    }

    // our ports are 1-21 with the sign for reversal; the registry is 0-20
    const std::uint8_t zeroIndexedPort = static_cast<std::uint8_t>(portNumber - 1);
    const pros::c::v5_device_e_t actual = pros::c::registry_get_plugged_type(zeroIndexedPort);
    const pros::c::v5_device_e_t expected = expectedRegistryType(check.expected);

    if (actual == expected) return CheckResult{check.label, check.port, true, ""};
    return CheckResult{check.label, check.port, false, "found " + deviceTypeName(actual)};
}

std::vector<CheckResult> runChecks(const std::vector<SensorCheck>& checks) {
    std::vector<CheckResult> results;
    results.reserve(checks.size());
    for (const auto& check : checks) results.push_back(runCheck(check));
    return results;
}

} // namespace sapphirelib::diag
