// Host-side unit test for sapphirelib::telemetry::motorRow() — what one motor
// channel row records, and above all what an unplugged motor looks like in
// the log. No PROS dependency.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/motor_row_test.cpp src/sapphirelib/telemetry/motor_row.cpp -o motor_row_test && ./motor_row_test
// clang-format on

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "sapphirelib/telemetry/motor_row.hpp"

using sapphirelib::telemetry::kMotorColumnCount;
using sapphirelib::telemetry::kMotorColumns;
using sapphirelib::telemetry::kMotorFaultOverCurrent;
using sapphirelib::telemetry::kMotorFaultOverTemp;
using sapphirelib::telemetry::MotorReadings;
using sapphirelib::telemetry::motorRow;

namespace {

// pros/error.h's values, spelled out so this test needs no PROS header.
constexpr std::int32_t kProsErr = std::numeric_limits<std::int32_t>::max();
constexpr double kProsErrF = std::numeric_limits<double>::infinity();

void testColumns() {
    static_assert(sizeof(kMotorColumns) / sizeof(kMotorColumns[0]) == kMotorColumnCount);
    const char* expected[] = {"volts", "amps", "temp", "rpm", "eff", "faults"};
    for (std::size_t i = 0; i < kMotorColumnCount; ++i) {
        assert(std::strcmp(kMotorColumns[i], expected[i]) == 0);
    }
}

void testUnitsAndSigns() {
    // A reversed motor stalled against a load at -12V, hot and current-limited.
    double values[kMotorColumnCount];
    motorRow(MotorReadings{.voltageMv = -11980,
                           .currentMa = 2480,
                           .temperatureC = 55.0,
                           .velocityRpm = -1.5,
                           .efficiencyPct = 0.0,
                           .faults = kMotorFaultOverTemp | kMotorFaultOverCurrent},
             values);
    assert(values[0] == -11.98);
    assert(values[1] == 2.48);
    assert(values[2] == 55.0);
    assert(values[3] == -1.5);
    assert(values[4] == 0.0);
    assert(values[5] == 5.0);
}

void testUnpluggedMotorIsAWholeNanRow() {
    double values[kMotorColumnCount];
    motorRow(MotorReadings{.voltageMv = kProsErr,
                           .currentMa = kProsErr,
                           .temperatureC = kProsErrF,
                           .velocityRpm = kProsErrF,
                           .efficiencyPct = kProsErrF,
                           .faults = static_cast<std::uint32_t>(kProsErr)},
             values);
    for (const double value : values) assert(std::isnan(value));
}

void testOneFailedReadingIsOneNan() {
    // A single getter failing (a read racing a port reconfiguring, say) loses
    // that column only: the rest are still real readings.
    double values[kMotorColumnCount];
    motorRow(MotorReadings{.voltageMv = 6000,
                           .currentMa = 900,
                           .temperatureC = kProsErrF,
                           .velocityRpm = 150.0,
                           .efficiencyPct = 62.0,
                           .faults = 0},
             values);
    assert(values[0] == 6.0 && values[1] == 0.9);
    assert(std::isnan(values[2]));
    assert(values[3] == 150.0 && values[4] == 62.0 && values[5] == 0.0);
}

void testIdleMotorIsZeroesNotNan() {
    // Plugged in and doing nothing is a row of zeros (and room temperature) —
    // a very different thing from unplugged.
    double values[kMotorColumnCount];
    motorRow(MotorReadings{.temperatureC = 25.0}, values);
    assert(values[0] == 0.0 && values[1] == 0.0 && values[2] == 25.0);
    assert(values[3] == 0.0 && values[4] == 0.0 && values[5] == 0.0);
}

} // namespace

int main() {
    testColumns();
    testUnitsAndSigns();
    testUnpluggedMotorIsAWholeNanRow();
    testOneFailedReadingIsOneNan();
    testIdleMotorIsZeroesNotNan();
    std::printf("motor_row_test: all tests passed\n");
    return 0;
}
