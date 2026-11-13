#include "sapphirelib/telemetry/calibration_record.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace sapphirelib::telemetry {

namespace {

// a name or text value with anything that would split its line or its field made a space
void appendClean(std::string& out, std::string_view text) {
    for (const char c : text) out += (c == ',' || c == '\n' || c == '\r') ? ' ' : c;
}

} // namespace

std::size_t formatExactNumber(double value, char* out, std::size_t size) {
    if (out == nullptr || size == 0) return 0;
    // 15 digits print every double that came from a decimal of 15 or fewer as that decimal (0.1 as
    // "0.1"); 17 read back exactly whatever the double
    for (int digits = 15; digits <= 17; ++digits) {
        const int written = std::snprintf(out, size, "%.*g", digits, value);
        if (written < 0 || static_cast<std::size_t>(written) >= size) {
            out[0] = '\0';
            return 0;
        }
        if (digits == 17 || !std::isfinite(value) || std::strtod(out, nullptr) == value) {
            return static_cast<std::size_t>(written);
        }
    }
    return 0; // not reached: 17 digits always returns
}

CalibrationRecord::CalibrationRecord(std::string_view kind) {
    appendClean(kind_, kind);
    text_ = kind_;
}

CalibrationRecord& CalibrationRecord::line(std::string_view kind) {
    text_ += '\n';
    appendClean(text_, kind);
    return *this;
}

CalibrationRecord& CalibrationRecord::add(std::string_view name, double value) {
    char number[32];
    formatExactNumber(value, number, sizeof(number));
    text_ += ',';
    appendClean(text_, name);
    text_ += '=';
    text_ += number;
    return *this;
}

CalibrationRecord& CalibrationRecord::add(std::string_view name, std::string_view text) {
    text_ += ',';
    appendClean(text_, name);
    text_ += '=';
    appendClean(text_, text);
    return *this;
}

std::string CalibrationRecord::text() const { return text_ + '\n'; }

} // namespace sapphirelib::telemetry
