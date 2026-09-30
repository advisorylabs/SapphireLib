#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

// log files are named "SL" + a six digit run number + ".CSV", always a legal 8.3 name, since the
// V5 has no clock to date files by. Each new file takes the next number after the highest there

namespace sapphirelib::telemetry {

// highest run number a name can carry ("SL999999.CSV")
constexpr std::uint32_t kMaxLogIndex = 999999;

/**
 * @brief Write a log file's name, like "SL000042.CSV" for 42
 *
 * @param out where to write, NUL-terminated
 * @param size bytes available. At least 13
 * @param index run number
 * @return false index is past kMaxLogIndex, or size is too small
 */
bool formatLogFileName(char* out, std::size_t size, std::uint32_t index);

/**
 * @brief Write a log file's full path, like "/usd/sl/SL000042.CSV"
 *
 * @param out where to write, NUL-terminated
 * @param size bytes available
 * @param directory the folder. A trailing '/' is fine
 * @param index run number
 * @return false index is out of range, or the path doesn't fit
 */
bool formatLogFilePath(char* out, std::size_t size, std::string_view directory,
                       std::uint32_t index);

/**
 * @brief Get the run number from a log file's name
 *
 * Case-insensitive, after any leading path
 *
 * @param name the file name
 * @return std::int32_t 42 for "SL000042.CSV", -1 for anything else (including near misses)
 */
std::int32_t parseLogFileIndex(std::string_view name);

/**
 * @brief Get the highest run number in a pros::usd::list_files() listing
 *
 * @param listing names separated by '\n'
 * @return std::int32_t the highest run number, or -1 if there are no log files
 */
std::int32_t highestLogFileIndex(std::string_view listing);

/**
 * @brief Get the path list_files() wants, which leaves off the "/usd" fopen() needs
 *
 * @param directory the folder, like "/usd/sl"
 * @return std::string_view "/sl" for "/usd/sl", "/" for "/usd"
 */
std::string_view listingPath(std::string_view directory);

} // namespace sapphirelib::telemetry
