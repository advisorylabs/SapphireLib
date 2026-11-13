// Host-side unit test for sapphirelib::telemetry's calibration log lines, no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/calibration_record_test.cpp src/sapphirelib/telemetry/calibration_record.cpp -o calibration_record_test && ./calibration_record_test
// clang-format on

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include "sapphirelib/telemetry/calibration_record.hpp"

using sapphirelib::telemetry::CalibrationRecord;
using sapphirelib::telemetry::formatExactNumber;

namespace {

std::string exact(double value) {
    char out[32];
    const std::size_t written = formatExactNumber(value, out, sizeof(out));
    assert(written == std::strlen(out));
    return out;
}

void testShortestThatReadsBack() {
    // a decimal with few digits prints as that decimal, not its 17-digit expansion
    assert(exact(0.1) == "0.1");
    assert(exact(2.75) == "2.75");
    assert(exact(-0.2457) == "-0.2457");
    assert(exact(48.125) == "48.125");
    // integers without a decimal point
    assert(exact(0.0) == "0");
    assert(exact(812.0) == "812");
    assert(exact(-3.0) == "-3");
    // one that needs all 17
    assert(exact(0.1 + 0.2) == "0.30000000000000004");
    assert(exact(2.5000000000000004) == "2.5000000000000004");
}

void testEveryValueReadsBackExactly() {
    const double values[] = {1.0 / 3.0,
                             2.0 / 3.0,
                             3.14159265358979323846,
                             -0.000123456789012345678,
                             1e-9,
                             123456789.123456789,
                             6.02214076e23,
                             std::nextafter(1.0, 2.0),
                             std::nextafter(0.1, 1.0),
                             std::numeric_limits<double>::min(),
                             std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::denorm_min()};
    for (const double value : values) {
        const std::string text = exact(value);
        assert(std::strtod(text.c_str(), nullptr) == value);
        // and no more than 17 significant digits, the most a double ever needs. Leading zeros
        // ("0.000123") aren't significant
        std::size_t digits = 0;
        bool significant = false;
        for (const char c : text) {
            if (c == 'e' || c == 'E') break;
            if (c < '0' || c > '9') continue;
            significant = significant || c != '0';
            if (significant) ++digits;
        }
        assert(digits >= 1 && digits <= 17);
    }
}

void testNotFinite() {
    assert(exact(std::numeric_limits<double>::infinity()) == "inf");
    assert(exact(-std::numeric_limits<double>::infinity()) == "-inf");
    const std::string nan = exact(std::numeric_limits<double>::quiet_NaN());
    assert(nan == "nan" || nan == "-nan");
}

void testTooSmall() {
    char out[4];
    std::strcpy(out, "xyz");
    assert(formatExactNumber(0.30000000000000004, out, sizeof(out)) == 0);
    assert(out[0] == '\0');
    assert(formatExactNumber(1.0, out, 0) == 0);
    assert(formatExactNumber(1.0, nullptr, 8) == 0);
}

void testRecordLines() {
    CalibrationRecord record("wheels");
    record.add("run", "forward").add("actual_in", 48.125);
    record.line("wheels.wheel").add("name", "Vertical").add("diameter_in", 0.1 + 0.2);
    record.line("wheels.wheel").add("name", std::string("Drive")).add("read_in", 47);
    assert(record.kind() == "wheels");
    assert(record.text() ==
           "wheels,run=forward,actual_in=48.125\n"
           "wheels.wheel,name=Vertical,diameter_in=0.30000000000000004\n"
           "wheels.wheel,name=Drive,read_in=47\n");
}

void testRecordKeepsLinesWhole() {
    // a comma or line break in a name or text would split the line, so they become spaces
    CalibrationRecord record("mounts,odd\nkind");
    record.add("error", "no two opposite sensors, saw\r\na wall").add("a,b", 1.0);
    assert(record.kind() == "mounts odd kind");
    assert(record.text() == "mounts odd kind,error=no two opposite sensors  saw  a wall,a b=1\n");
}

void testEmptyRecord() {
    const CalibrationRecord record("offsets");
    assert(record.text() == "offsets\n");
}

} // namespace

int main() {
    testShortestThatReadsBack();
    testEveryValueReadsBackExactly();
    testNotFinite();
    testTooSmall();
    testRecordLines();
    testRecordKeepsLinesWhole();
    testEmptyRecord();
    std::printf("calibration_record_test: all passed\n");
    return 0;
}
