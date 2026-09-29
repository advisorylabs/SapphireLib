#include "sapphirelib/telemetry/motor_row.hpp"

#include <cmath>
#include <limits>

namespace sapphirelib::telemetry {

namespace {

/// pros/error.h's PROS_ERR, spelled out: this file includes no PROS header.
constexpr std::int32_t kProsErr = std::numeric_limits<std::int32_t>::max();

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

} // namespace

void motorRow(const MotorReadings& readings, double* values) {
    const bool voltageOk = readings.voltageMv != kProsErr;
    const bool currentOk = readings.currentMa != kProsErr;
    const bool temperatureOk = std::isfinite(readings.temperatureC);
    const bool velocityOk = std::isfinite(readings.velocityRpm);
    const bool efficiencyOk = std::isfinite(readings.efficiencyPct);
    const bool faultsOk = readings.faults != static_cast<std::uint32_t>(kProsErr);

    if (!voltageOk && !currentOk && !temperatureOk && !velocityOk && !efficiencyOk && !faultsOk) {
        for (std::size_t i = 0; i < kMotorColumnCount; ++i) values[i] = kNan;
        return;
    }
    values[0] = voltageOk ? readings.voltageMv / 1000.0 : kNan;
    values[1] = currentOk ? readings.currentMa / 1000.0 : kNan;
    values[2] = temperatureOk ? readings.temperatureC : kNan;
    values[3] = velocityOk ? readings.velocityRpm : kNan;
    values[4] = efficiencyOk ? readings.efficiencyPct : kNan;
    values[5] = faultsOk ? static_cast<double>(readings.faults) : kNan;
}

} // namespace sapphirelib::telemetry
