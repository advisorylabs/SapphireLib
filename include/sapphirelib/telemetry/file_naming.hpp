/**
 * \file sapphirelib/telemetry/file_naming.hpp
 *
 * Log file names on the V5's SD card, which has no real-time clock to date
 * files by and no confirmed long-file-name support: "SL" + a six-digit run
 * index + ".CSV" — always a legal 8.3 name — and each new file takes the next
 * index after the highest already there. Pure string handling, unit-tested in
 * tests/telemetry/file_naming_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace sapphirelib::telemetry {

/// Highest run index a name can carry ("SL999999.CSV").
constexpr std::uint32_t kMaxLogIndex = 999999;

/// Writes "SL000042.CSV" for 42 into `out`, NUL-terminated. False if `index`
/// exceeds kMaxLogIndex or `size` is under 13.
bool formatLogFileName(char* out, std::size_t size, std::uint32_t index);

/// Writes the full path fopen() wants — "/usd/sl/SL000042.CSV" for directory
/// "/usd/sl" (a trailing '/' on the directory is fine) — into `out`,
/// NUL-terminated. False if `index` is out of range or the path doesn't fit;
/// PROS refuses paths of 128 characters or more anyway.
bool formatLogFilePath(char* out, std::size_t size, std::string_view directory,
                       std::uint32_t index);

/// 42 for "SL000042.CSV", matched case-insensitively (FAT short names can come
/// back upper-case, and a card touched on a PC may differ) after any leading
/// path. -1 for anything else, near-misses like "SL00042.CSV" or
/// "SL000042.TXT" included, so an unrelated file can't push the index.
std::int32_t parseLogFileIndex(std::string_view name);

/// The highest index among the names in a pros::usd::list_files() listing
/// (names separated by '\n'; '\r' tolerated), or -1 if it names no log file.
std::int32_t highestLogFileIndex(std::string_view listing);

/// The path list_files() wants, which omits the "/usd" prefix fopen() needs:
/// "/usd/sl" -> "/sl"; "/usd" and "/usd/" -> "/"; a trailing '/' is dropped
/// ("/usd/sl/" -> "/sl"). A directory without the prefix is returned as is.
std::string_view listingPath(std::string_view directory);

} // namespace sapphirelib::telemetry
