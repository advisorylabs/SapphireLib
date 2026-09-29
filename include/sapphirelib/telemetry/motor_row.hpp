/**
 * \file sapphirelib/telemetry/motor_row.hpp
 *
 * What a motor channel (telemetry::Logger::motor()) records: one V5 smart
 * motor's health — the volts it's applying, its current, temperature, speed,
 * efficiency and fault bits — turned from PROS's raw readings into telemetry
 * units. Pure: the readings come in as plain numbers, so the conversion (and
 * above all what an unplugged motor looks like in the log) is unit-tested on a
 * desktop compiler (tests/telemetry/motor_row_test.cpp).
 *
 * This is the data for diagnosing a robot that faded mid-match: a V5 motor
 * cuts its own power as it heats (to half at 55°C, off by 70°C) and reports
 * nothing up the command path, so the only evidence is here — temperature
 * climbing, current capped, speed per volt falling.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace sapphirelib::telemetry {

/// A motor channel's columns, after t_us: volts applied (V), current drawn
/// (A), temperature (°C), actual velocity (RPM, output shaft), efficiency (%),
/// and PROS's fault bits (see kMotorFault*).
inline constexpr const char* kMotorColumns[] = {"volts", "amps", "temp", "rpm", "eff", "faults"};
constexpr std::size_t kMotorColumnCount = 6;

/// The `faults` column's bits — pros::motor_fault_e_t's values.
constexpr std::uint32_t kMotorFaultOverTemp = 0x01;
constexpr std::uint32_t kMotorFaultDriverFault = 0x02;
constexpr std::uint32_t kMotorFaultOverCurrent = 0x04;
constexpr std::uint32_t kMotorFaultDriverOverCurrent = 0x08;

/// One motor's readings exactly as PROS's C API returns them — including its
/// error values: PROS_ERR (INT32_MAX) from the integer getters, PROS_ERR_F
/// (infinity) from the floating-point ones, and PROS_ERR converted to
/// uint32_t from motor_get_faults().
struct MotorReadings {
    std::int32_t voltageMv = 0; ///< motor_get_voltage()
    std::int32_t currentMa = 0; ///< motor_get_current_draw()
    double temperatureC = 0.0;  ///< motor_get_temperature()
    double velocityRpm = 0.0;   ///< motor_get_actual_velocity()
    double efficiencyPct = 0.0; ///< motor_get_efficiency()
    std::uint32_t faults = 0;   ///< motor_get_faults()
};

/// Fills `values[0..kMotorColumnCount)` from `readings`, in kMotorColumns'
/// order and units. Any single reading that is an error value becomes NaN.
///
/// If the motor answered nothing at all — every reading an error, which is
/// what an unplugged motor (or one on a dead cable) gives — the whole row is
/// NaN, so the log shows a clean gap an analyzer reads as "disconnected"
/// rather than a row of plausible-looking zeros.
void motorRow(const MotorReadings& readings, double* values);

} // namespace sapphirelib::telemetry
