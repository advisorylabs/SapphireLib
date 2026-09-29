#include "sapphirelib/telemetry/channel.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "sapphirelib/util/clock.hpp"

namespace sapphirelib::telemetry {

namespace {

std::size_t valueCountFor(const ChannelSchema& schema) {
    switch (schema.kind) {
        case ChannelKind::pid: return kPidValues;
        case ChannelKind::events: return 0;
        default: return std::min(schema.columns.size(), kMaxColumns);
    }
}

Record configRecord(const PID::Config& config) {
    Record record{};
    record.kind = RecordKind::pidConfig;
    putWide(record, 0, config.integralLimit);
    putWide(record, 1, config.outputLimit);
    putWide(record, 2, config.slewRate);
    putWide(record, 3, config.derivativeOnMeasurement ? 1.0 : 0.0);
    putWide(record, 4, config.nominalDtS);
    return record;
}

Record gainsRecord(const PIDGains& gains) {
    Record record{};
    record.kind = RecordKind::pidGains;
    putWide(record, 0, gains.kP);
    putWide(record, 1, gains.kI);
    putWide(record, 2, gains.kD);
    return record;
}

Record sampleRecord(const PidStep& step) {
    Record record{};
    record.kind = RecordKind::sample;
    record.count = static_cast<std::uint8_t>(kPidValues);
    record.flags = step.flags;
    // Same order as kPidColumns.
    const double values[kPidValues] = {step.target,    step.measurement, step.error,
                                       step.pTerm,     step.iTerm,       step.dTerm,
                                       step.rawOutput, step.output,      step.dtS};
    for (std::size_t i = 0; i < kPidValues; ++i) record.values[i] = static_cast<float>(values[i]);
    return record;
}

} // namespace

// --- Channel -----------------------------------------------------------------

Channel::Channel(ChannelSchema schema, std::size_t capacity,
                 const std::atomic<std::uint32_t>* fileEpoch)
    : schema_(std::move(schema)), valueCount_(valueCountFor(schema_)), ring_(capacity),
      fileEpoch_(fileEpoch) {
    // A samples channel can't carry more columns than a Record has values;
    // trimming the schema too keeps its #chan line honest about what S rows hold.
    if (schema_.kind == ChannelKind::samples && schema_.columns.size() > kMaxColumns) {
        schema_.columns.resize(kMaxColumns);
    }
}

const ChannelSchema& Channel::schema() const { return schema_; }

bool Channel::record(std::initializer_list<double> values) {
    return record(values.begin(), values.size());
}

bool Channel::record(const double* values, std::size_t count) {
    if (valueCount_ == 0) return false;
    Record record{};
    record.kind = RecordKind::sample;
    record.count = static_cast<std::uint8_t>(valueCount_);
    const std::size_t given = values == nullptr ? 0 : std::min(count, valueCount_);
    for (std::size_t i = 0; i < valueCount_; ++i) {
        record.values[i] =
            i < given ? static_cast<float>(values[i]) : std::numeric_limits<float>::quiet_NaN();
    }
    return commit(record);
}

bool Channel::recordEvent(const char* tag, const char* message) {
    // Laid out as one byte string across the Records' payloads —
    // "tag\0message\0" — then cut into Record-sized pieces. Sanitizing waits
    // for the writer: this runs on the caller's task and should stay a copy.
    char text[kMaxEventRecords * kRecordTextBytes] = {};
    std::size_t length = 0;
    for (const char* c = tag != nullptr ? tag : ""; *c != '\0' && length < kMaxEventTagChars; ++c) {
        text[length++] = *c;
    }
    text[length++] = '\0';
    std::size_t messageChars = 0;
    for (const char* c = message != nullptr ? message : "";
         *c != '\0' && messageChars < kMaxEventMessageChars; ++c, ++messageChars) {
        text[length++] = *c;
    }
    text[length++] = '\0';

    const std::size_t parts = (length + kRecordTextBytes - 1) / kRecordTextBytes;
    Record records[kMaxEventRecords] = {};
    for (std::size_t i = 0; i < parts; ++i) {
        records[i].kind = i == 0 ? RecordKind::event : RecordKind::eventContinued;
        records[i].count = i == 0 ? static_cast<std::uint8_t>(parts - 1) : 0;
        std::copy(text + i * kRecordTextBytes, text + (i + 1) * kRecordTextBytes, records[i].text);
    }
    return commitAll(records, parts);
}

bool Channel::commit(Record record) { return commitAll(&record, 1); }

bool Channel::commitAll(Record* records, std::size_t count) {
    std::uint32_t token;
    if (!gate_.tryEnter(token)) return false; // counted by the gate
    // Stamped inside the gate, so successive producers on one channel commit
    // in time order — the file promises t_us never decreases within a channel.
    const std::uint64_t now = sapphirelib::micros();
    for (std::size_t i = 0; i < count; ++i) records[i].tUs = now;
    const bool pushed = ring_.push(records, count);
    gate_.leave(token);
    return pushed;
}

std::uint32_t Channel::fileEpoch() const {
    return fileEpoch_ != nullptr ? fileEpoch_->load(std::memory_order_acquire) : 0;
}

const Record* Channel::front() { return ring_.front(); }

const Record* Channel::peek(std::size_t offset) { return ring_.peek(offset); }

void Channel::pop(std::size_t count) { ring_.pop(count); }

ProducerGate& Channel::gate() { return gate_; }

std::uint32_t Channel::droppedFull() const { return ring_.dropped(); }

std::uint32_t Channel::droppedContended() const { return gate_.contended(); }

std::uint32_t Channel::resyncs() const { return ring_.resyncs(); }

// --- PidProbe ----------------------------------------------------------------

PidProbe::PidProbe(Channel& channel, const PID& pid) : channel_(channel), pid_(&pid) {}

void PidProbe::onPidUpdate(const PID& pid, const PidStep& step) {
    if (&pid != pid_) {
        // A copy of our PID (copying a PID copies its observer pointer): its
        // steps would be indistinguishable from ours in the log, so drop them.
        foreignSteps_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    const std::uint32_t epoch = channel_.fileEpoch();
    if (epoch != announcedEpoch_) {
        // A new file opened since our last step: say C and G again, so this
        // file can be read without the previous one.
        announcedEpoch_ = epoch;
        configLogged_ = false;
        gainsLogged_ = false;
    }

    // Each is only marked logged once its commit actually went in — a dropped
    // C or G is retried on the next step rather than silently missing.
    if (!configLogged_) configLogged_ = channel_.commit(configRecord(pid.config()));
    const PIDGains& gains = pid.gains();
    if (!gainsLogged_ || gains.kP != loggedGains_.kP || gains.kI != loggedGains_.kI ||
        gains.kD != loggedGains_.kD) {
        const PIDGains snapshot = gains;
        if (channel_.commit(gainsRecord(snapshot))) {
            loggedGains_ = snapshot;
            gainsLogged_ = true;
        }
    }
    channel_.commit(sampleRecord(step));
}

void PidProbe::onPidReset(const PID& pid) {
    if (&pid != pid_) return;
    Record record{};
    record.kind = RecordKind::pidReset;
    channel_.commit(record);
}

std::uint32_t PidProbe::foreignSteps() const {
    return foreignSteps_.load(std::memory_order_relaxed);
}

} // namespace sapphirelib::telemetry
