#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "sapphirelib/telemetry/record.hpp"

// encodes telemetry Records as lines of SLT v1 (docs/TELEMETRY_FORMAT.md). Every format function
// appends to a caller-owned buffer (out, with size bytes free) and returns how many bytes it
// appended, not counting a NUL. When the line doesn't fit it returns 0, and the caller doesn't
// advance its fill mark, so the writer can flush and retry without ever writing half a line

namespace sapphirelib::telemetry {

// major version on each file's first line ("#SLT,1"). Only bumped for a change an existing reader
// would misread; new row types, keys, and event tags don't bump it
constexpr int kFormatVersion = 1;

// upper bound on one row, '\n' included. The writer's staging buffer is always far larger
constexpr std::size_t kMaxLineBytes = 400;

/**
 * @brief What a file's header says about the run
 *
 * Strings are written as printable ASCII (anything else becomes a space), cut to 63 characters
 */
struct FileHeader {
    const char* library = "";   // "0.1.0"
    const char* kernel = "";    // "4.2.2"
    const char* build = "";     // "Sep 27 2026 14:02:11"
    const char* robotName = ""; // "96671H"
    const char* fileName = "";  // "SL000042.CSV"
    const char* directory = ""; // "/usd/sl", or "/usd" in the root fallback
    std::uint64_t openUs = 0;
};

/**
 * @brief Writer counters for an H row. See docs/TELEMETRY_FORMAT.md for each key
 */
struct WriterStats {
    std::uint32_t rows = 0;
    std::uint32_t bytes = 0;
    std::uint32_t writes = 0;
    std::uint32_t writeMaxUs = 0;
    std::uint32_t writeAvgUs = 0;
    std::uint32_t drops = 0;
    std::uint32_t unlogged = 0;
    std::uint32_t resyncs = 0;
    std::uint32_t breaks = 0;
    std::uint32_t faults = 0;
};

/**
 * @brief Make a name the format can carry unquoted
 *
 * Letters, digits, '_', '.', and '-' pass through; anything else becomes '_'. Allocates, so it's
 * for channel registration only
 *
 * @param name the name
 * @param maxLength longest result. 31 for channels and columns, 15 for event tags
 * @return std::string the cleaned name, "_" if empty
 */
std::string sanitizeName(const char* name, std::size_t maxLength = 31);

/**
 * @brief Format a number in fixed point
 *
 * Rounded half away from zero, trailing zeros trimmed, "-0" written as "0". NaN and infinity as
 * "nan", "inf", "-inf"; very large values in exponent form. The same on every compiler
 *
 * @param out where to write
 * @param size bytes free at out
 * @param value the number
 * @param decimals digits after the point, clamped to 0-6
 * @return std::size_t bytes written, 0 if it didn't fit
 */
std::size_t formatNumber(char* out, std::size_t size, double value, int decimals);

/**
 * @brief Format the header block: "#SLT,1" then the #meta lines
 */
std::size_t formatHeader(char* out, std::size_t size, const FileHeader& header);

/**
 * @brief Format a channel's #chan line
 */
std::size_t formatSchema(char* out, std::size_t size, const ChannelSchema& schema);

/**
 * @brief Format one Record as its S, G, C, R, or E row
 *
 * An event is formatted from its own payload only; use formatEventRecords() for one with
 * continuations. A continuation on its own formats as nothing
 */
std::size_t formatRecord(char* out, std::size_t size, const ChannelSchema& schema,
                         const Record& record);

/**
 * @brief Format the E row for an event that spans several Records
 *
 * @param parts the event Record, then its continuations in order
 * @param count how many parts
 */
std::size_t formatEventRecords(char* out, std::size_t size, const Record* const* parts,
                               std::size_t count);

/**
 * @brief Format an E row the writer logs itself (file open, SD faults, phase at open)
 */
std::size_t formatEvent(char* out, std::size_t size, std::uint64_t tUs, const char* tag,
                        const char* message);

/**
 * @brief Format a D row: rows lost from a channel so far
 */
std::size_t formatDrops(char* out, std::size_t size, std::uint64_t tUs, std::uint16_t channelId,
                        std::uint32_t droppedFull, std::uint32_t droppedContended);

/**
 * @brief Format an H row: writer health
 */
std::size_t formatHealth(char* out, std::size_t size, std::uint64_t tUs, const WriterStats& stats);

} // namespace sapphirelib::telemetry
