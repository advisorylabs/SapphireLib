// Host-side unit test for sapphirelib::telemetry::Channel and PidProbe — no
// PROS/embedded dependencies. Channel stamps rows through the clock seam
// (sapphirelib::micros()), which this file defines as a fake clock it steps
// by hand.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/channel_test.cpp src/sapphirelib/telemetry/channel.cpp src/sapphirelib/telemetry/record_ring.cpp src/sapphirelib/control/pid.cpp -o channel_test && ./channel_test
// clang-format on

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/telemetry/channel.hpp"

namespace {
std::uint64_t fakeMicros = 0;
} // namespace

namespace sapphirelib {
std::uint64_t micros() { return fakeMicros; }
} // namespace sapphirelib

using sapphirelib::PID;
using sapphirelib::PIDGains;
using sapphirelib::PidStep;
using sapphirelib::telemetry::Channel;
using sapphirelib::telemetry::ChannelKind;
using sapphirelib::telemetry::ChannelSchema;
using sapphirelib::telemetry::getWide;
using sapphirelib::telemetry::kMaxEventMessageChars;
using sapphirelib::telemetry::kMaxEventTagChars;
using sapphirelib::telemetry::kPidColumns;
using sapphirelib::telemetry::kPidValues;
using sapphirelib::telemetry::kRecordTextBytes;
using sapphirelib::telemetry::PidProbe;
using sapphirelib::telemetry::Record;
using sapphirelib::telemetry::RecordKind;

namespace {

ChannelSchema samplesSchema(std::vector<std::string> columns) {
    return ChannelSchema{.id = 5,
                         .name = "test",
                         .kind = ChannelKind::samples,
                         .decimals = 4,
                         .columns = std::move(columns)};
}

ChannelSchema pidSchema() {
    return ChannelSchema{.id = 7,
                         .name = "lift",
                         .kind = ChannelKind::pid,
                         .decimals = 3,
                         .columns = {std::begin(kPidColumns), std::end(kPidColumns)}};
}

/// Pops and returns everything in the channel.
std::vector<Record> drain(Channel& channel) {
    std::vector<Record> records;
    while (const Record* record = channel.front()) {
        records.push_back(*record);
        channel.pop();
    }
    return records;
}

/// An event's tag and message, reassembled from its Records the way the
/// writer does it.
struct Event {
    std::string tag;
    std::string message;
};

Event joinEvent(const std::vector<Record>& records, std::size_t first) {
    std::string text;
    const std::size_t parts = 1 + records[first].count;
    for (std::size_t i = 0; i < parts; ++i) {
        const Record& part = records[first + i];
        assert(part.kind == (i == 0 ? RecordKind::event : RecordKind::eventContinued));
        assert(part.tUs == records[first].tUs); // one row, one timestamp
        text.append(part.text, kRecordTextBytes);
    }
    const std::size_t tagEnd = text.find('\0');
    const std::size_t messageEnd = text.find('\0', tagEnd + 1);
    return {text.substr(0, tagEnd), text.substr(tagEnd + 1, messageEnd - tagEnd - 1)};
}

void testRecordStampsTimeAndPads() {
    Channel channel(samplesSchema({"a", "b", "c"}), 16);
    fakeMicros = 1234;
    assert(channel.record({1.5, -2.25}));
    fakeMicros = 1300;
    assert(channel.record({1.0, 2.0, 3.0, 4.0, 5.0})); // extras ignored

    const std::vector<Record> records = drain(channel);
    assert(records.size() == 2);
    assert(records[0].tUs == 1234);
    assert(records[0].kind == RecordKind::sample);
    assert(records[0].count == 3);
    assert(records[0].values[0] == 1.5f && records[0].values[1] == -2.25f);
    assert(std::isnan(records[0].values[2])); // missing -> NaN
    assert(records[1].tUs == 1300 && records[1].count == 3);
    assert(records[1].values[2] == 3.0f);

    // Pointer form, including a null pointer (all NaN).
    assert(channel.record(nullptr, 3));
    const std::vector<Record> nulls = drain(channel);
    assert(nulls.size() == 1 && std::isnan(nulls[0].values[0]));
}

void testCommitOverwritesCallerTimestamp() {
    Channel channel(samplesSchema({"a"}), 8);
    Record record{};
    record.tUs = 999999; // a caller's own idea of "now" must never reach the file
    fakeMicros = 42;
    assert(channel.commit(record));
    assert(drain(channel)[0].tUs == 42);
}

void testEventsChannelHasNoSamples() {
    Channel channel(ChannelSchema{.id = 1, .name = "events", .kind = ChannelKind::events}, 8);
    assert(!channel.record({1.0}));
    assert(channel.front() == nullptr);
    assert(channel.droppedFull() == 0 && channel.droppedContended() == 0);
}

void testTooManyColumnsAreTrimmed() {
    std::vector<std::string> columns;
    for (int i = 0; i < 20; ++i) columns.push_back("c" + std::to_string(i));
    Channel channel(samplesSchema(columns), 8);
    assert(channel.schema().columns.size() == 13);
    double values[20] = {};
    assert(channel.record(values, 20));
    assert(drain(channel)[0].count == 13);
}

void testShortEventIsOneRecord() {
    Channel channel(ChannelSchema{.id = 1, .name = "events", .kind = ChannelKind::events}, 8);
    fakeMicros = 77;
    assert(channel.recordEvent("auton", "start,Turn Testing"));
    const std::vector<Record> records = drain(channel);
    assert(records.size() == 1);
    assert(records[0].count == 0);
    const Event event = joinEvent(records, 0);
    assert(event.tag == "auton" && event.message == "start,Turn Testing");

    // Null tag/message are logged as empty rather than crashing.
    assert(channel.recordEvent(nullptr, nullptr));
    const Event empty = joinEvent(drain(channel), 0);
    assert(empty.tag.empty() && empty.message.empty());
}

void testLongEventSpansRecords() {
    Channel channel(ChannelSchema{.id = 1, .name = "events", .kind = ChannelKind::events}, 16);
    // A drivetrain motion event: well past one Record's payload.
    const std::string message =
        "start,moveToPose,x=-123.456,y=-123.456,heading_deg=359.999,pos_threshold=1.000,"
        "heading_threshold=2.000,settle_ms=250,timeout_ms=4000";
    fakeMicros = 5000;
    assert(channel.recordEvent("motion", message.c_str()));
    assert(channel.recordEvent("mark", "driver")); // and a short one after it
    const std::vector<Record> records = drain(channel);
    // "motion\0" + 132 chars + "\0" = 140 bytes = 3 Records of 52.
    assert(records.size() == 4);
    assert(records[0].count == 2);
    const Event event = joinEvent(records, 0);
    assert(event.tag == "motion" && event.message == message);
    assert(joinEvent(records, 3).message == "driver");
}

void testEventTruncation() {
    Channel channel(ChannelSchema{.id = 1, .name = "events", .kind = ChannelKind::events}, 16);
    const std::string longTag(40, 't');
    const std::string longMessage(500, 'm');
    assert(channel.recordEvent(longTag.c_str(), longMessage.c_str()));
    const std::vector<Record> records = drain(channel);
    assert(records.size() == 4); // the most one event can span
    const Event event = joinEvent(records, 0);
    assert(event.tag == std::string(kMaxEventTagChars, 't'));
    assert(event.message == std::string(kMaxEventMessageChars, 'm'));
}

void testEventIsAllOrNothingWhenNearlyFull() {
    Channel channel(ChannelSchema{.id = 1, .name = "events", .kind = ChannelKind::events}, 8);
    for (int i = 0; i < 6; ++i) assert(channel.recordEvent("fill", "x"));
    const std::string message(100, 'a'); // needs 3 Records; 2 are free
    assert(!channel.recordEvent("motion", message.c_str()));
    assert(channel.droppedFull() == 1); // one row lost, not three
    int count = 0;
    while (channel.front() != nullptr) {
        assert(channel.front()->kind == RecordKind::event); // no stray continuation
        channel.pop();
        ++count;
    }
    assert(count == 6);
    assert(channel.recordEvent("motion", message.c_str()));
    assert(drain(channel).size() == 3);
}

void testContendedRecordIsDroppedNotWaited() {
    Channel channel(samplesSchema({"a"}), 8);
    std::uint32_t token = 0;
    assert(channel.gate().tryEnter(token)); // another task, mid-record
    assert(!channel.record({1.0}));
    assert(!channel.recordEvent("mark", "x"));
    assert(channel.droppedContended() == 2);
    channel.gate().leave(token);
    assert(channel.record({1.0}));
    assert(channel.droppedFull() == 0);
}

void testFileEpoch() {
    Channel lone(samplesSchema({"a"}), 8);
    assert(lone.fileEpoch() == 0);
    std::atomic<std::uint32_t> epoch{3};
    Channel owned(samplesSchema({"a"}), 8, &epoch);
    assert(owned.fileEpoch() == 3);
    epoch.store(4);
    assert(owned.fileEpoch() == 4);
}

// --- PidProbe ----------------------------------------------------------------

PID::Config liftConfig() {
    return PID::Config{.gains = {.kP = 0.2, .kI = 0.0, .kD = 0.01},
                       .integralLimit = 0.0,
                       .outputLimit = 12.7,
                       .slewRate = 0.0,
                       .derivativeOnMeasurement = true,
                       .nominalDtS = 0.02};
}

void testFirstUpdateLogsConfigGainsThenSample() {
    std::atomic<std::uint32_t> epoch{1};
    Channel channel(pidSchema(), 32, &epoch);
    PID pid(liftConfig());
    PidProbe probe(channel, pid);
    pid.setObserver(&probe);

    fakeMicros = 31540205;
    const double output = pid.update(150.0, 0.37);
    const std::vector<Record> records = drain(channel);
    assert(records.size() == 3);

    assert(records[0].kind == RecordKind::pidConfig);
    assert(getWide(records[0], 0) == 0.0);  // integralLimit
    assert(getWide(records[0], 1) == 12.7); // outputLimit
    assert(getWide(records[0], 2) == 0.0);  // slewRate
    assert(getWide(records[0], 3) == 1.0);  // derivativeOnMeasurement
    assert(getWide(records[0], 4) == 0.02); // nominalDtS

    assert(records[1].kind == RecordKind::pidGains);
    assert(getWide(records[1], 0) == 0.2); // full double precision, not float
    assert(getWide(records[1], 1) == 0.0);
    assert(getWide(records[1], 2) == 0.01);

    const Record& sample = records[2];
    const PidStep& step = pid.lastStep();
    assert(sample.kind == RecordKind::sample);
    assert(sample.count == kPidValues);
    // 29.9V asked of a 12.7V limit on the first step. (kI is 0, but the
    // anti-windup rollback still flags the error it declined to integrate.)
    assert(sample.flags == step.flags);
    assert(sample.flags == (PidStep::kFirstStep | PidStep::kSaturated | PidStep::kIntegralHeld));
    const double expected[kPidValues] = {step.target,    step.measurement, step.error,
                                         step.pTerm,     step.iTerm,       step.dTerm,
                                         step.rawOutput, step.output,      step.dtS};
    for (std::size_t i = 0; i < kPidValues; ++i) {
        assert(sample.values[i] == static_cast<float>(expected[i]));
    }
    assert(sample.values[7] == static_cast<float>(output));
    assert(sample.values[0] == 150.0f && sample.values[2] == static_cast<float>(150.0 - 0.37));
    for (const Record& record : records) assert(record.tUs == 31540205);
}

void testGainsLoggedOnlyWhenTheyChange() {
    std::atomic<std::uint32_t> epoch{1};
    Channel channel(pidSchema(), 32, &epoch);
    PID pid(liftConfig());
    PidProbe probe(channel, pid);
    pid.setObserver(&probe);

    pid.update(150.0, 0.0);
    drain(channel);
    pid.update(150.0, 1.0);
    std::vector<Record> records = drain(channel);
    assert(records.size() == 1 && records[0].kind == RecordKind::sample);
    assert((records[0].flags & PidStep::kFirstStep) == 0); // not the first step now
    assert(records[0].flags == pid.lastStep().flags);

    // Setting the same gains again isn't a change.
    pid.setGains(PIDGains{.kP = 0.2, .kI = 0.0, .kD = 0.01});
    pid.update(150.0, 2.0);
    assert(drain(channel).size() == 1);

    // PidTunerPage / Auto-Tune changing them from another task: noticed here.
    pid.setGains(PIDGains{.kP = 0.3, .kI = 0.05, .kD = 0.01});
    pid.update(150.0, 3.0);
    records = drain(channel);
    assert(records.size() == 2);
    assert(records[0].kind == RecordKind::pidGains);
    assert(getWide(records[0], 0) == 0.3 && getWide(records[0], 1) == 0.05);
    assert(records[1].kind == RecordKind::sample);
}

void testDroppedGainsAreRetried() {
    std::atomic<std::uint32_t> epoch{1};
    Channel channel(pidSchema(), 8, &epoch);
    PID pid(liftConfig());
    PidProbe probe(channel, pid);
    pid.setObserver(&probe);

    pid.update(150.0, 0.0);
    assert(drain(channel).size() == 3);

    // Fill the ring so the G row the next step wants can't go in.
    for (int i = 0; i < 8; ++i) assert(channel.record({0.0}));
    pid.setGains(PIDGains{.kP = 0.5});
    pid.update(150.0, 1.0);
    assert(channel.droppedFull() == 2); // G and S both refused
    drain(channel);

    // Room again: the probe must not believe the lost G was logged.
    pid.update(150.0, 2.0);
    const std::vector<Record> records = drain(channel);
    assert(records.size() == 2);
    assert(records[0].kind == RecordKind::pidGains && getWide(records[0], 0) == 0.5);
    assert(records[1].kind == RecordKind::sample);
}

void testNewFileReannouncesConfigAndGains() {
    std::atomic<std::uint32_t> epoch{0}; // no file open yet
    Channel channel(pidSchema(), 32, &epoch);
    PID pid(liftConfig());
    PidProbe probe(channel, pid);
    pid.setObserver(&probe);

    pid.update(150.0, 0.0);
    assert(drain(channel).size() == 3);
    pid.update(150.0, 1.0);
    assert(drain(channel).size() == 1);

    epoch.store(1); // the writer opened a file
    pid.update(150.0, 2.0);
    std::vector<Record> records = drain(channel);
    assert(records.size() == 3);
    assert(records[0].kind == RecordKind::pidConfig);
    assert(records[1].kind == RecordKind::pidGains);
    assert(records[2].kind == RecordKind::sample);

    pid.update(150.0, 3.0);
    assert(drain(channel).size() == 1);
}

void testResetLoggedOnlyWhenThereWasState() {
    std::atomic<std::uint32_t> epoch{1};
    Channel channel(pidSchema(), 32, &epoch);
    PID pid(liftConfig());
    PidProbe probe(channel, pid);
    pid.setObserver(&probe);

    pid.reset(); // nothing to clear yet
    assert(channel.front() == nullptr);

    pid.update(150.0, 0.0);
    drain(channel);
    fakeMicros = 16871020;
    pid.reset();
    pid.reset(); // a lift resting on its stop resets every tick: one R only
    std::vector<Record> records = drain(channel);
    assert(records.size() == 1);
    assert(records[0].kind == RecordKind::pidReset && records[0].tUs == 16871020);

    pid.update(-90.0, 0.0);
    records = drain(channel);
    assert(records.size() == 1 && (records[0].flags & PidStep::kFirstStep));
}

void testCopiedPidIsForeign() {
    std::atomic<std::uint32_t> epoch{1};
    Channel channel(pidSchema(), 32, &epoch);
    PID pid(liftConfig());
    PidProbe probe(channel, pid);
    pid.setObserver(&probe);

    PID copy = pid; // shares the observer pointer
    copy.update(150.0, 0.0);
    copy.update(150.0, 1.0);
    copy.reset();
    assert(channel.front() == nullptr);
    assert(probe.foreignSteps() == 2);

    pid.update(150.0, 0.0);
    assert(drain(channel).size() == 3);
}

} // namespace

int main() {
    testRecordStampsTimeAndPads();
    testCommitOverwritesCallerTimestamp();
    testEventsChannelHasNoSamples();
    testTooManyColumnsAreTrimmed();
    testShortEventIsOneRecord();
    testLongEventSpansRecords();
    testEventTruncation();
    testEventIsAllOrNothingWhenNearlyFull();
    testContendedRecordIsDroppedNotWaited();
    testFileEpoch();
    testFirstUpdateLogsConfigGainsThenSample();
    testGainsLoggedOnlyWhenTheyChange();
    testDroppedGainsAreRetried();
    testNewFileReannouncesConfigAndGains();
    testResetLoggedOnlyWhenThereWasState();
    testCopiedPidIsForeign();
    std::printf("channel_test: all tests passed\n");
    return 0;
}
