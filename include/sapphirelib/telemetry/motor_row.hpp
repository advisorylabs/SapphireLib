#pragma once

#include <cstddef>
#include <cstdint>

// what a motor channel (Logger::motor()) records. A V5 motor cuts its own power as it heats (to
// half at 55C, off by 70C) and reports it nowhere else, so this is the evidence for a mechanism
// that faded mid-match: temperature climbing, current capped, speed per volt falling

namespace sapphirelib::telemetry {

// a motor channel's columns after t_us: volts applied (V), current (A), temperature (C), velocity
// (RPM, output shaft), efficiency (%), and PROS's fault bits
inline constexpr const char* kMotorColumns[] = {"volts", "amps", "temp", "rpm", "eff", "faults"};
constexpr std::size_t kMotorColumnCount = 6;

// the faults column's bits, pros::motor_fault_e_t's values
constexpr std::uint32_t kMotorFaultOverTemp = 0x01;
constexpr std::uint32_t kMotorFaultDriverFault = 0x02;
constexpr std::uint32_t kMotorFaultOverCurrent = 0x04;
constexpr std::uint32_t kMotorFaultDriverOverCurrent = 0x08;

/**
 * @brief One motor's readings, exactly as PROS's C API returns them, error values included
 */
struct MotorReadings {
    std::int32_t voltageMv = 0; // motor_get_voltage()
    std::int32_t currentMa = 0; // motor_get_current_draw()
    double temperatureC = 0.0;  // motor_get_temperature()
    double velocityRpm = 0.0;   // motor_get_actual_velocity()
    double efficiencyPct = 0.0; // motor_get_efficiency()
    std::uint32_t faults = 0;   // motor_get_faults()
};

/**
 * @brief Turn a motor's readings into a telemetry row
 *
 * A single error reading becomes NaN. If every reading is an error (an unplugged motor), the whole
 * row is NaN, so the log shows a clean gap instead of plausible looking zeros
 *
 * @param readings the motor's readings
 * @param values where to write kMotorColumnCount values, in kMotorColumns order
 */
void motorRow(const MotorReadings& readings, double* values);

} // namespace sapphirelib::telemetry
