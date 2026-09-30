#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace sapphirelib::telemetry {

// most columns a channel can have. 13 floats after the 12 byte header make a 64 byte Record
constexpr std::size_t kMaxColumns = 13;

// bytes of payload a Record carries, as event text or packed doubles
constexpr std::size_t kRecordTextBytes = kMaxColumns * sizeof(float);

// doubles that fit in the payload, see putWide()
constexpr std::size_t kMaxWide = kRecordTextBytes / sizeof(double);

// most Records one event can span: its first plus three continuations. One Record only fits about
// 35 characters, and motion events run past 100
constexpr std::size_t kMaxEventRecords = 4;

// longest event tag, in characters. Longer tags are cut
constexpr std::size_t kMaxEventTagChars = 15;

// longest event message, in characters: the payload of kMaxEventRecords minus the longest tag and
// two NULs. Longer messages are cut
constexpr std::size_t kMaxEventMessageChars =
    kMaxEventRecords * kRecordTextBytes - kMaxEventTagChars - 2;

/**
 * @brief Which row a Record becomes in the file. See docs/TELEMETRY_FORMAT.md
 */
enum class RecordKind : std::uint8_t {
    sample,         // S row: Record::count floats in Record::values
    pidGains,       // G row: kP, kI, kD as packed doubles
    pidConfig,      // C row: integralLimit, outputLimit, slewRate,
                    // derivativeOnMeasurement (0/1), nominalDtS as packed doubles
    pidReset,       // R row: no payload
    event,          // E row: "tag\0message\0" in Record::text, continued in
                    // the Record::count eventContinued Records after it
    eventContinued, // the next kRecordTextBytes of the event before it. Never a row itself
};

/**
 * @brief One row on its way from the task that recorded it to the SD writer
 *
 * Fixed size and trivially copyable, since producers run inside control loops: copying 64 bytes
 * into a preallocated slot is the only safe work there (no allocating, formatting, or locking).
 * Values are floats; 7 digits is past anything a V5 sensor resolves
 */
struct Record {
    /** microseconds since program start, stamped by Channel::commit() */
    std::uint64_t tUs = 0;
    /** which row this becomes */
    RecordKind kind = RecordKind::sample;
    /** values in use for a sample. For an event, how many eventContinued Records follow */
    std::uint8_t count = 0;
    /** PidStep::flags for a pid channel's sample, 0 otherwise */
    std::uint16_t flags = 0;
    union {
        float values[kMaxColumns];
        char text[kRecordTextBytes];
    };
};

static_assert(sizeof(Record) == 64, "a Record should stay one 64-byte slot");

/**
 * @brief Store a double at full precision in a Record's payload, for gains and config
 *
 * @param record the record
 * @param index which double, below kMaxWide
 * @param value the value
 */
inline void putWide(Record& record, std::size_t index, double value) {
    std::memcpy(record.text + index * sizeof(double), &value, sizeof(double));
}

/**
 * @brief Read a double stored with putWide()
 */
inline double getWide(const Record& record, std::size_t index) {
    double value;
    std::memcpy(&value, record.text + index * sizeof(double), sizeof(double));
    return value;
}

/**
 * @brief What a channel carries
 */
enum class ChannelKind : std::uint8_t {
    samples, // S rows with the channel's own columns
    pid,     // one PID's steps: S rows with kPidColumns, plus G, C, and R rows
    events,  // E rows only
};

// columns of every pid channel's S rows, after t_us. flags is PidStep::flags as an integer
inline constexpr const char* kPidColumns[] = {"target", "meas",  "err", "p",  "i",
                                              "d",      "u_raw", "out", "dt", "flags"};

// how many of kPidColumns travel in Record::values (all but flags)
constexpr std::size_t kPidValues = 9;

/**
 * @brief How to read a channel's rows, written to the file as its #chan line
 *
 * Never changes once the channel exists, so the writer can read it without a lock
 */
struct ChannelSchema {
    std::uint16_t id = 0;
    std::string name = {};
    ChannelKind kind = ChannelKind::samples;
    int decimals = 4;
    /** column names after t_us: kPidColumns for a pid channel, none for events */
    std::vector<std::string> columns = {};
};

} // namespace sapphirelib::telemetry
