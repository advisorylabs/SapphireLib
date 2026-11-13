#pragma once

#include <cstddef>
#include <string>
#include <string_view>

namespace sapphirelib::telemetry {

/**
 * @brief Write a number with every digit it needs to read back as exactly the same double, and no
 * more: 0.1 as "0.1", but 2.5000000000000004 in full
 *
 * The shortest of 15, 16 or 17 significant digits that reads back exactly (17 always does).
 * Integers print without a decimal point, NaN and infinity as "nan" and "inf"
 *
 * @param value the number
 * @param out where to write, NUL-terminated
 * @param size bytes available. 32 always fits
 * @return std::size_t characters written, not counting the NUL. 0 if size was too small
 */
std::size_t formatExactNumber(double value, char* out, std::size_t size);

/**
 * @brief One calibration's results as lines of text, every number exact (formatExactNumber()),
 * for the SD card's calibration log (see saveCalibration())
 *
 * Each line is a kind and its values, comma separated: "offsets,vertical_offset_in=-0.2457,...".
 * line() starts another, for each of several things one calibration measured (a sensor, a wheel).
 * Commas and line breaks in a name or text value become spaces, so they can't split a line
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::telemetry::CalibrationRecord record("wheels");
 * record.add("actual_in", 48.125);
 * record.line("wheels.wheel").add("name", "Vertical").add("diameter_in", 2.7514);
 * sapphirelib::telemetry::saveCalibration(record);
 * @endcode
 */
class CalibrationRecord {
public:
    /**
     * @brief Start a record, with its first line
     *
     * @param kind what calibrated, e.g. "offsets"
     */
    explicit CalibrationRecord(std::string_view kind);

    /**
     * @brief Start another line
     *
     * @param kind what this line describes, e.g. "mounts.sensor"
     */
    CalibrationRecord& line(std::string_view kind);

    /**
     * @brief Add a number to the current line, exactly (formatExactNumber())
     */
    CalibrationRecord& add(std::string_view name, double value);

    /**
     * @brief Add a word or phrase to the current line
     */
    CalibrationRecord& add(std::string_view name, std::string_view text);

    /**
     * @brief The first line's kind, e.g. "offsets"
     */
    const std::string& kind() const { return kind_; }

    /**
     * @brief Every line, each ending in a line break
     */
    std::string text() const;

private:
    std::string kind_;
    std::string text_;
};

} // namespace sapphirelib::telemetry
