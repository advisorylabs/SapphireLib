/**
 * \file sapphirelib/telemetry/csv_format.hpp
 *
 * Encodes telemetry Records as lines of SLT v1, SapphireLib's on-disk
 * telemetry format (spec: docs/TELEMETRY_FORMAT.md). Pure — no PROS, no
 * allocation on the formatting path — and golden-tested byte for byte in
 * tests/telemetry/csv_format_test.cpp, so the robot and the off-robot app are
 * held to one spec.
 *
 * Every format* function appends to a caller-owned buffer (`out`, with `size`
 * bytes free) and returns how many bytes it appended — no NUL is written or
 * counted. When the line doesn't fit it returns 0: the caller must not advance
 * its fill mark, which is what lets the writer flush and retry without ever
 * emitting half a line. (The free space past the fill mark may have been used
 * as scratch by then; it holds nothing the caller counted.)
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "sapphirelib/telemetry/record.hpp"

namespace sapphirelib::telemetry {

/// Major version on each file's first line ("#SLT,1"). Bumped only for a
/// change an existing reader would misread. Additions — new row types, #meta
/// keys, H keys, event tags — don't bump it; readers must skip what they don't
/// recognize.
constexpr int kFormatVersion = 1;

/// Upper bound on one row, '\n' included: kMaxColumns values at their widest
/// plus the prefix, or the longest event. The writer's staging buffer is
/// always far larger than this, so a row that didn't fit always fits once the
/// buffer has been written out.
constexpr std::size_t kMaxLineBytes = 400;

/// What the header block says about the run. All strings are written as
/// printable ASCII (anything else becomes a space) and cut to 63 characters.
struct FileHeader {
    const char* library = "";   // "0.1.0"
    const char* kernel = "";    // "4.2.2"
    const char* build = "";     // "Sep 27 2026 14:02:11"
    const char* robotName = ""; // "96671H"
    const char* fileName = "";  // "SL000042.CSV"
    const char* directory = ""; // "/usd/sl", or "/usd" in the root fallback
    std::uint64_t openUs = 0;
};

/// Writer counters for an `H` row; key meanings are in the spec.
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

/// A name the format can carry unquoted: letters, digits, '_', '.', '-' pass
/// through, anything else becomes '_', cut to `maxLength`; an empty result
/// becomes "_". Channel and column names use 31, event tags 15. Allocates —
/// it's for channel registration; the formatters sanitize in place.
std::string sanitizeName(const char* name, std::size_t maxLength = 31);

/// `value` in fixed point with `decimals` places (clamped to 0-6), rounded
/// half away from zero, trailing zeros and a bare '.' trimmed, "-0" written as
/// "0". NaN/inf as "nan"/"inf"/"-inf"; |value| >= 1e12 as "d.dddddde+XX".
/// Deterministic across compilers and C libraries — no snprintf on the common
/// path.
std::size_t formatNumber(char* out, std::size_t size, double value, int decimals);

/// The whole header block: "#SLT,1" then the #meta lines.
std::size_t formatHeader(char* out, std::size_t size, const FileHeader& header);

/// A channel's "#chan" line.
std::size_t formatSchema(char* out, std::size_t size, const ChannelSchema& schema);

/// One Record of the channel `schema` describes, as its S/G/C/R/E row. An
/// event Record is formatted from its own payload only — use
/// formatEventRecords() for one with continuations. An eventContinued Record
/// on its own has no row and formats as nothing (returns 0).
std::size_t formatRecord(char* out, std::size_t size, const ChannelSchema& schema,
                         const Record& record);

/// The `E` row for an event that spans several Records: `parts[0]` is its
/// event Record, then its continuations in order (`count` in all).
std::size_t formatEventRecords(char* out, std::size_t size, const Record* const* parts,
                               std::size_t count);

/// An `E` row the writer emits itself (file open, SD faults, phase at open).
std::size_t formatEvent(char* out, std::size_t size, std::uint64_t tUs, const char* tag,
                        const char* message);

/// A `D` row: cumulative rows lost from channel `channelId`.
std::size_t formatDrops(char* out, std::size_t size, std::uint64_t tUs, std::uint16_t channelId,
                        std::uint32_t droppedFull, std::uint32_t droppedContended);

/// An `H` row.
std::size_t formatHealth(char* out, std::size_t size, std::uint64_t tUs, const WriterStats& stats);

} // namespace sapphirelib::telemetry
