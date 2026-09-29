/**
 * \file sapphirelib/telemetry/record.hpp
 *
 * Telemetry's data model: the fixed-size Record every producer hands to the
 * SD writer, and the schema that says how a channel's Records read. Pure data,
 * no PROS — the whole producer side of telemetry builds on a desktop compiler.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace sapphirelib::telemetry {

/// Most columns a channel can have: 13 floats after the 12-byte header make a
/// Record exactly 64 bytes.
constexpr std::size_t kMaxColumns = 13;

/// Bytes of payload a Record carries, as event text or packed doubles.
constexpr std::size_t kRecordTextBytes = kMaxColumns * sizeof(float);

/// Doubles that fit in the payload (see putWide()).
constexpr std::size_t kMaxWide = kRecordTextBytes / sizeof(double);

/// Most Records one event can span: its first, plus up to three
/// continuations. One Record only has room for about 35 characters of
/// message, and the drivetrains' motion events (a pose motion's target plus
/// its exit conditions) run past 100 — so an event longer than one Record
/// travels as several consecutive ones, committed together.
constexpr std::size_t kMaxEventRecords = 4;

/// Longest event tag, in characters. Longer tags are cut.
constexpr std::size_t kMaxEventTagChars = 15;

/// Longest event message, in characters: what's left of kMaxEventRecords
/// payloads after the longest tag and the two NULs that end tag and message.
/// Longer messages are cut.
constexpr std::size_t kMaxEventMessageChars =
    kMaxEventRecords * kRecordTextBytes - kMaxEventTagChars - 2;

/// Which row a Record becomes in the file — see docs/TELEMETRY_FORMAT.md.
enum class RecordKind : std::uint8_t {
    sample,         ///< `S` row: Record::count floats in Record::values.
    pidGains,       ///< `G` row: kP, kI, kD as packed doubles.
    pidConfig,      ///< `C` row: integralLimit, outputLimit, slewRate,
                    ///< derivativeOnMeasurement (0/1), nominalDtS as packed doubles.
    pidReset,       ///< `R` row: no payload.
    event,          ///< `E` row: "tag\0message\0" in Record::text, continued in
                    ///< the Record::count eventContinued Records that follow it.
    eventContinued, ///< The next kRecordTextBytes of the preceding event's text.
                    ///< Never a row of its own.
};

/// One row in flight, from the producer that recorded it to the writer task
/// that formats it.
///
/// Fixed-size and trivially copyable on purpose: producers run inside control
/// loops, and copying 64 bytes into a preallocated slot is the only kind of
/// work that's safe there — no allocation (PROS's malloc suspends the
/// scheduler), no formatting (snprintf of one float costs microseconds), no
/// locks (a lock held by a task the competition switch deletes is held
/// forever). Values are floats: 7 significant digits is past anything a V5
/// sensor resolves, and half the size doubles the history a ring holds.
struct Record {
    /// Microseconds since program start, stamped by Channel::commit() when the
    /// row was committed — never supplied by the caller.
    std::uint64_t tUs = 0;
    RecordKind kind = RecordKind::sample;
    /// Values in use, for a sample. For an event, how many eventContinued
    /// Records follow it (0 for an event that fits in one).
    std::uint8_t count = 0;
    /// PidStep::flags for a pid channel's sample; 0 otherwise.
    std::uint16_t flags = 0;
    union {
        float values[kMaxColumns];
        char text[kRecordTextBytes];
    };
};

static_assert(sizeof(Record) == 64, "a Record should stay one 64-byte slot");

/// Stores a double at full precision in the payload — for G/C rows, where a
/// gain like kD = 0.00015 shouldn't round through a float. memcpy because the
/// payload is only 4-byte aligned. `index` must be below kMaxWide.
inline void putWide(Record& record, std::size_t index, double value) {
    std::memcpy(record.text + index * sizeof(double), &value, sizeof(double));
}

inline double getWide(const Record& record, std::size_t index) {
    double value;
    std::memcpy(&value, record.text + index * sizeof(double), sizeof(double));
    return value;
}

/// What a channel carries — decides the columns a reader expects.
enum class ChannelKind : std::uint8_t {
    samples, ///< `S` rows with the channel's own columns.
    pid,     ///< One PID's steps: `S` rows with kPidColumns, plus G, C, R rows.
    events,  ///< `E` rows only.
};

/// Columns of every pid channel's `S` rows, after t_us. All but `flags` are
/// the PidStep fields of the same meaning; `flags` is PidStep::flags as an
/// integer, carried in Record::flags rather than as a float.
inline constexpr const char* kPidColumns[] = {"target", "meas",  "err", "p",  "i",
                                              "d",      "u_raw", "out", "dt", "flags"};

/// How many of kPidColumns travel in Record::values (all but `flags`).
constexpr std::size_t kPidValues = 9;

/// Everything a reader needs to interpret a channel's rows, written to the
/// file as its `#chan` line. Immutable once the channel exists, which is what
/// lets the writer task read it without a lock.
struct ChannelSchema {
    std::uint16_t id = 0;
    std::string name = {};
    ChannelKind kind = ChannelKind::samples;
    int decimals = 4;
    /// Column names after the implicit leading t_us: kPidColumns for a pid
    /// channel, none for events.
    std::vector<std::string> columns = {};
};

} // namespace sapphirelib::telemetry
