#include "sapphirelib/telemetry/csv_format.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace sapphirelib::telemetry {

namespace {

constexpr std::size_t kMaxNameChars = 31;
constexpr std::size_t kMaxMetaChars = 63;

/// Widest formatNumber() output: "-999999999999.999999" is 20 characters.
constexpr std::size_t kNumberBufferBytes = 32;

/// What a column with no value in its Record is written as.
constexpr float kMissing = std::numeric_limits<float>::quiet_NaN();

bool isNameChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
           c == '.' || c == '-';
}

std::size_t writeUnsigned(char* out, std::uint64_t value) {
    char reversed[20];
    std::size_t count = 0;
    do {
        reversed[count++] = static_cast<char>('0' + value % 10);
        value /= 10;
    } while (value != 0);
    for (std::size_t i = 0; i < count; ++i) out[i] = reversed[count - 1 - i];
    return count;
}

/// Trims a printf exponent to C99's form — sign plus at least two digits, no
/// extra leading zeros — since an old Windows C library prints "e+012" where
/// newlib and glibc print "e+12". The exponent is the only part C libraries
/// disagree on (all of them round the digits correctly), so this is what
/// makes the rare snprintf paths below deterministic too.
std::size_t normalizeExponent(char* text, std::size_t length) {
    char* e = static_cast<char*>(std::memchr(text, 'e', length));
    if (e == nullptr) return length;
    char* digits = e + 1;
    if (digits < text + length && (*digits == '+' || *digits == '-')) ++digits;
    char* first = digits;
    while (first + 2 < text + length && *first == '0') ++first; // keep two
    const std::size_t kept = static_cast<std::size_t>(text + length - first);
    std::memmove(digits, first, kept);
    return static_cast<std::size_t>(digits - text) + kept;
}

/// formatNumber()'s body, into a buffer known to be big enough.
std::size_t writeNumber(char* out, double value, int decimals) {
    if (std::isnan(value)) {
        std::memcpy(out, "nan", 3);
        return 3;
    }
    if (std::isinf(value)) {
        if (value > 0.0) {
            std::memcpy(out, "inf", 3);
            return 3;
        }
        std::memcpy(out, "-inf", 4);
        return 4;
    }

    const double magnitude = std::fabs(value);
    if (magnitude >= 1e12) {
        // Only garbage gets this big in a robot log; not worth a hand-rolled
        // exponent formatter.
        char buffer[kNumberBufferBytes];
        const int written = std::snprintf(buffer, sizeof(buffer), "%.6e", value);
        std::size_t length = written > 0 ? static_cast<std::size_t>(written) : 0;
        length = normalizeExponent(buffer, std::min(length, sizeof(buffer) - 1));
        std::memcpy(out, buffer, length);
        return length;
    }

    // Exact powers of ten, so the one multiply below is the only rounding
    // step before llround() — and IEEE multiplication rounds the same on the
    // brain's VFP as on a desktop's SSE, which is what makes the output
    // byte-identical on both. llround() is exact and rounds halves away from
    // zero. Below 1e12 with at most 6 decimals the product stays under 1e18,
    // well inside a long long.
    static constexpr double kScale[] = {1.0, 10.0, 100.0, 1e3, 1e4, 1e5, 1e6};
    static constexpr std::uint64_t kScaleInt[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    decimals = std::clamp(decimals, 0, 6);
    const auto scaled = static_cast<std::uint64_t>(std::llround(magnitude * kScale[decimals]));
    const std::uint64_t whole = scaled / kScaleInt[decimals];
    std::uint64_t fraction = scaled % kScaleInt[decimals];

    std::size_t length = 0;
    // Only a value that's still nonzero after rounding gets a sign, which is
    // how -0.0 and -0.00001 at 4 decimals both come out as "0".
    if (scaled != 0 && value < 0.0) out[length++] = '-';
    length += writeUnsigned(out + length, whole);
    if (fraction != 0) {
        char digits[6];
        for (int i = decimals - 1; i >= 0; --i) {
            digits[i] = static_cast<char>('0' + fraction % 10);
            fraction /= 10;
        }
        int end = decimals;
        while (end > 0 && digits[end - 1] == '0') --end; // trim trailing zeros
        out[length++] = '.';
        for (int i = 0; i < end; ++i) out[length++] = digits[i];
    }
    return length;
}

/// `%.9g` for G/C rows: full double precision for gains, exponent form
/// allowed. -0 is written as 0 and non-finite values spelled out, so no C
/// library's quirks ("-nan", "1.#INF") leak into the file.
std::size_t writeWide(char* out, double value) {
    if (std::isnan(value) || std::isinf(value) || value == 0.0) {
        return writeNumber(out, value == 0.0 ? 0.0 : value, 0);
    }
    char buffer[kNumberBufferBytes];
    const int written = std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    std::size_t length = written > 0 ? static_cast<std::size_t>(written) : 0;
    length = normalizeExponent(buffer, std::min(length, sizeof(buffer) - 1));
    std::memcpy(out, buffer, length);
    return length;
}

/// Appends into a fixed buffer without ever writing past it; finish() is 0 if
/// anything didn't fit. Every formatter below builds its line through one.
class LineWriter {
public:
    LineWriter(char* out, std::size_t size) : out_(out), size_(size) {}

    void put(char c) {
        if (length_ >= size_) {
            overflow_ = true;
            return;
        }
        out_[length_++] = c;
    }

    void put(const char* text) {
        while (*text != '\0') put(*text++);
    }

    void putChars(const char* chars, std::size_t count) {
        for (std::size_t i = 0; i < count; ++i) put(chars[i]);
    }

    void putUnsigned(std::uint64_t value) {
        char buffer[20];
        putChars(buffer, writeUnsigned(buffer, value));
    }

    void putNumber(double value, int decimals) {
        char buffer[kNumberBufferBytes];
        putChars(buffer, writeNumber(buffer, value, decimals));
    }

    void putWide(double value) {
        char buffer[kNumberBufferBytes];
        putChars(buffer, writeWide(buffer, value));
    }

    /// sanitizeName() without allocating.
    void putName(const char* name, std::size_t maxLength) {
        std::size_t count = 0;
        for (const char* c = name != nullptr ? name : ""; *c != '\0' && count < maxLength;
             ++c, ++count) {
            put(isNameChar(*c) ? *c : '_');
        }
        if (count == 0) put('_');
    }

    /// Free text (an E message, a #meta value): printable ASCII passes,
    /// anything else — CR and LF above all, which would split the line —
    /// becomes a space.
    void putText(const char* text, std::size_t maxLength) {
        std::size_t count = 0;
        for (const char* c = text != nullptr ? text : ""; *c != '\0' && count < maxLength;
             ++c, ++count) {
            put(*c >= 0x20 && *c <= 0x7e ? *c : ' ');
        }
    }

    std::size_t finish() const { return overflow_ ? 0 : length_; }

private:
    char* out_;
    std::size_t size_;
    std::size_t length_ = 0;
    bool overflow_ = false;
};

void putRowPrefix(LineWriter& line, char row, std::uint16_t id, std::uint64_t tUs) {
    line.put(row);
    line.put(',');
    line.putUnsigned(id);
    line.put(',');
    line.putUnsigned(tUs);
}

void putMeta(LineWriter& line, const char* key, const char* value) {
    line.put("#meta,");
    line.put(key);
    line.put(',');
    line.putText(value, kMaxMetaChars);
    line.put('\n');
}

const char* kindName(ChannelKind kind) {
    switch (kind) {
        case ChannelKind::pid: return "pid";
        case ChannelKind::events: return "events";
        default: return "samples";
    }
}

} // namespace

std::string sanitizeName(const char* name, std::size_t maxLength) {
    std::string clean;
    for (const char* c = name != nullptr ? name : ""; *c != '\0' && clean.size() < maxLength; ++c) {
        clean.push_back(isNameChar(*c) ? *c : '_');
    }
    if (clean.empty()) clean = "_";
    return clean;
}

std::size_t formatNumber(char* out, std::size_t size, double value, int decimals) {
    char buffer[kNumberBufferBytes];
    const std::size_t length = writeNumber(buffer, value, decimals);
    if (length > size) return 0;
    std::memcpy(out, buffer, length);
    return length;
}

std::size_t formatHeader(char* out, std::size_t size, const FileHeader& header) {
    LineWriter line(out, size);
    line.put("#SLT,");
    line.putUnsigned(kFormatVersion);
    line.put('\n');

    // "sapphirelib 0.1.0": which encoder wrote the file, for a reader that
    // ever needs to work around one version's quirk.
    line.put("#meta,writer,sapphirelib ");
    line.putText(header.library, kMaxMetaChars);
    line.put('\n');
    putMeta(line, "kernel", header.kernel);
    putMeta(line, "build", header.build);
    putMeta(line, "robot", header.robotName);
    putMeta(line, "file", header.fileName);
    putMeta(line, "dir", header.directory);

    line.put("#meta,open_us,");
    line.putUnsigned(header.openUs);
    line.put('\n');
    putMeta(line, "clock", "us_since_program_start");
    return line.finish();
}

std::size_t formatSchema(char* out, std::size_t size, const ChannelSchema& schema) {
    LineWriter line(out, size);
    line.put("#chan,");
    line.putUnsigned(schema.id);
    line.put(',');
    line.putName(schema.name.c_str(), kMaxNameChars);
    line.put(',');
    line.put(kindName(schema.kind));
    line.put(',');
    line.putUnsigned(static_cast<std::uint64_t>(std::clamp(schema.decimals, 0, 6)));

    // The spec fixes pid and events columns, so they come from here rather
    // than from whatever the schema happens to hold.
    if (schema.kind == ChannelKind::pid) {
        for (const char* column : kPidColumns) {
            line.put(',');
            line.put(column);
        }
    } else if (schema.kind == ChannelKind::samples) {
        const std::size_t columns = std::min(schema.columns.size(), kMaxColumns);
        for (std::size_t i = 0; i < columns; ++i) {
            line.put(',');
            line.putName(schema.columns[i].c_str(), kMaxNameChars);
        }
    }
    line.put('\n');
    return line.finish();
}

std::size_t formatRecord(char* out, std::size_t size, const ChannelSchema& schema,
                         const Record& record) {
    LineWriter line(out, size);
    const int decimals = std::clamp(schema.decimals, 0, 6);

    switch (record.kind) {
        case RecordKind::sample: {
            putRowPrefix(line, 'S', schema.id, record.tUs);
            if (schema.kind == ChannelKind::pid) {
                for (std::size_t i = 0; i < kPidValues; ++i) {
                    line.put(',');
                    line.putNumber(i < record.count ? record.values[i] : kMissing, decimals);
                }
                line.put(',');
                line.putUnsigned(record.flags);
            } else {
                // Exactly one value per declared column, so every S row of a
                // channel has the width its #chan line promised.
                const std::size_t columns = schema.kind == ChannelKind::samples
                                                ? std::min(schema.columns.size(), kMaxColumns)
                                                : 0;
                for (std::size_t i = 0; i < columns; ++i) {
                    line.put(',');
                    line.putNumber(i < record.count ? record.values[i] : kMissing, decimals);
                }
            }
            break;
        }
        case RecordKind::pidGains: {
            putRowPrefix(line, 'G', schema.id, record.tUs);
            for (std::size_t i = 0; i < 3; ++i) {
                line.put(',');
                line.putWide(getWide(record, i));
            }
            break;
        }
        case RecordKind::pidConfig: {
            putRowPrefix(line, 'C', schema.id, record.tUs);
            for (std::size_t i = 0; i < 5; ++i) {
                line.put(',');
                if (i == 3) {
                    line.put(getWide(record, i) != 0.0 ? '1' : '0'); // derivativeOnMeasurement
                } else {
                    line.putWide(getWide(record, i));
                }
            }
            break;
        }
        case RecordKind::pidReset: putRowPrefix(line, 'R', schema.id, record.tUs); break;
        case RecordKind::event: {
            const Record* parts[] = {&record};
            return formatEventRecords(out, size, parts, 1);
        }
        default: return 0; // eventContinued: part of the event before it, never a row
    }
    line.put('\n');
    return line.finish();
}

std::size_t formatEventRecords(char* out, std::size_t size, const Record* const* parts,
                               std::size_t count) {
    // Rejoin the payloads into the one "tag\0message\0" string
    // Channel::recordEvent() split up. The extra byte guarantees a final NUL
    // even if a payload somehow arrived without one.
    if (count == 0 || parts[0] == nullptr) return 0;
    char text[kMaxEventRecords * kRecordTextBytes + 1] = {};
    count = std::min(count, kMaxEventRecords);
    std::size_t length = 0;
    for (std::size_t i = 0; i < count && parts[i] != nullptr; ++i) {
        std::memcpy(text + length, parts[i]->text, kRecordTextBytes);
        length += kRecordTextBytes;
    }

    const char* tag = text;
    const std::size_t tagLength = std::strlen(tag);
    const char* message = tagLength < length ? text + tagLength + 1 : "";
    return formatEvent(out, size, parts[0]->tUs, tag, message);
}

std::size_t formatEvent(char* out, std::size_t size, std::uint64_t tUs, const char* tag,
                        const char* message) {
    LineWriter line(out, size);
    line.put("E,");
    line.putUnsigned(tUs);
    line.put(',');
    line.putName(tag, kMaxEventTagChars);
    line.put(',');
    line.putText(message, kMaxEventMessageChars);
    line.put('\n');
    return line.finish();
}

std::size_t formatDrops(char* out, std::size_t size, std::uint64_t tUs, std::uint16_t channelId,
                        std::uint32_t droppedFull, std::uint32_t droppedContended) {
    LineWriter line(out, size);
    line.put("D,");
    line.putUnsigned(tUs);
    line.put(',');
    line.putUnsigned(channelId);
    line.put(',');
    line.putUnsigned(droppedFull);
    line.put(',');
    line.putUnsigned(droppedContended);
    line.put('\n');
    return line.finish();
}

std::size_t formatHealth(char* out, std::size_t size, std::uint64_t tUs, const WriterStats& stats) {
    const struct {
        const char* key;
        std::uint32_t value;
    } fields[] = {
        {"rows", stats.rows},          {"bytes", stats.bytes},        {"writes", stats.writes},
        {"wmax_us", stats.writeMaxUs}, {"wavg_us", stats.writeAvgUs}, {"drops", stats.drops},
        {"unlogged", stats.unlogged},  {"resyncs", stats.resyncs},    {"breaks", stats.breaks},
        {"faults", stats.faults},
    };

    LineWriter line(out, size);
    line.put("H,");
    line.putUnsigned(tUs);
    for (const auto& field : fields) {
        line.put(',');
        line.put(field.key);
        line.put('=');
        line.putUnsigned(field.value);
    }
    line.put('\n');
    return line.finish();
}

} // namespace sapphirelib::telemetry
