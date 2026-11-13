#pragma once

#include "sapphirelib/telemetry/calibration_record.hpp"

namespace sapphirelib::telemetry {

/** where saveCalibration() writes: the first whose folder exists ("sl", as the Logger's) */
inline constexpr const char* kCalibrationLogPaths[] = {"/usd/sl/CALIB.TXT", "/usd/CALIB.TXT"};

/**
 * @brief Add a calibration's results to the SD card's calibration log, CALIB.TXT, every number
 * exact
 *
 * Every calibration the library runs calls this as it finishes (the Odom page's Calibrate Offsets
 * and Calibrate Sensors, the PID page's Wheel Apply and Auto-Tune), so its results outlive the
 * screen showing them, at full precision, whether or not the Logger is recording. Each record
 * starts with an "@" line: when it finished, in seconds since the program started. Each program
 * run's first record starts with a "#" line saying so. The file only grows: delete it on a
 * computer to start over
 *
 * Returns at once: the write happens on a low priority task of its own, so it's safe from the GUI
 * task and from competition tasks alike. The terminal says where it went, or that there was no
 * card
 *
 * @param record the results
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::telemetry::CalibrationRecord record("offsets");
 * record.add("vertical_offset_in", offsetIn);
 * sapphirelib::telemetry::saveCalibration(record);
 * @endcode
 */
void saveCalibration(const CalibrationRecord& record);

} // namespace sapphirelib::telemetry
