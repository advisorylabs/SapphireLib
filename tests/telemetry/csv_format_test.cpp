// Host-side golden test for sapphirelib::telemetry's SLT v1 encoder — no
// PROS/embedded dependencies. Every expected string here is byte-for-byte
// what the robot writes to the SD card, and what docs/TELEMETRY_FORMAT.md
// and tools/telemetry/slt_read.py promise; kGoldenFile below is also embedded
// in slt_read.py's --selftest, so the encoder and the reference parser are
// held to the same bytes.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/csv_format_test.cpp src/sapphirelib/telemetry/csv_format.cpp -o csv_format_test && ./csv_format_test
// clang-format on

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <string>
#include <vector>

#include "sapphirelib/telemetry/csv_format.hpp"

using sapphirelib::telemetry::ChannelKind;
using sapphirelib::telemetry::ChannelSchema;
using sapphirelib::telemetry::FileHeader;
using sapphirelib::telemetry::formatDrops;
using sapphirelib::telemetry::formatEvent;
using sapphirelib::telemetry::formatEventRecords;
using sapphirelib::telemetry::formatHeader;
using sapphirelib::telemetry::formatHealth;
using sapphirelib::telemetry::formatNumber;
using sapphirelib::telemetry::formatRecord;
using sapphirelib::telemetry::formatSchema;
using sapphirelib::telemetry::kMaxEventRecords;
using sapphirelib::telemetry::kMaxLineBytes;
using sapphirelib::telemetry::kPidColumns;
using sapphirelib::telemetry::kRecordTextBytes;
using sapphirelib::telemetry::putWide;
using sapphirelib::telemetry::Record;
using sapphirelib::telemetry::RecordKind;
using sapphirelib::telemetry::sanitizeName;
using sapphirelib::telemetry::WriterStats;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

std::string number(double value, int decimals) {
    char buffer[64];
    const std::size_t length = formatNumber(buffer, sizeof(buffer), value, decimals);
    assert(length > 0);
    return std::string(buffer, length);
}

void checkNumber(double value, int decimals, const char* expected) {
    const std::string actual = number(value, decimals);
    if (actual != expected) {
        std::printf("formatNumber(%.17g, %d): expected \"%s\", got \"%s\"\n", value, decimals,
                    expected, actual.c_str());
        assert(false);
    }
}

/// Runs `format` into a buffer of exactly the expected size (must succeed),
/// then one byte short (must return 0 — never half a line).
template <typename Format> void checkLine(Format format, const std::string& expected) {
    std::vector<char> buffer(expected.size() + 16, '#');
    const std::size_t length = format(buffer.data(), expected.size());
    const std::string actual(buffer.data(), length);
    if (actual != expected) {
        std::printf("expected: \"%s\"\n     got: \"%s\"\n", expected.c_str(), actual.c_str());
        assert(false);
    }
    assert(buffer[expected.size()] == '#'); // never wrote past `size`
    if (!expected.empty()) assert(format(buffer.data(), expected.size() - 1) == 0);
}

ChannelSchema schema(std::uint16_t id, const char* name, ChannelKind kind, int decimals,
                     std::vector<std::string> columns = {}) {
    return ChannelSchema{
        .id = id, .name = name, .kind = kind, .decimals = decimals, .columns = std::move(columns)};
}

ChannelSchema pidSchema(std::uint16_t id, const char* name, int decimals) {
    return schema(id, name, ChannelKind::pid, decimals,
                  {std::begin(kPidColumns), std::end(kPidColumns)});
}

Record sample(std::uint64_t tUs, std::initializer_list<float> values, std::uint16_t flags = 0) {
    Record record{};
    record.tUs = tUs;
    record.kind = RecordKind::sample;
    record.count = static_cast<std::uint8_t>(values.size());
    record.flags = flags;
    std::size_t i = 0;
    for (const float value : values) record.values[i++] = value;
    return record;
}

Record wide(std::uint64_t tUs, RecordKind kind, std::initializer_list<double> values) {
    Record record{};
    record.tUs = tUs;
    record.kind = kind;
    std::size_t i = 0;
    for (const double value : values) putWide(record, i++, value);
    return record;
}

Record reset(std::uint64_t tUs) {
    Record record{};
    record.tUs = tUs;
    record.kind = RecordKind::pidReset;
    return record;
}

/// An event's Records, laid out exactly as Channel::recordEvent() lays them
/// out ("tag\0message\0" cut into payload-sized pieces).
std::vector<Record> eventRecords(std::uint64_t tUs, const std::string& tag,
                                 const std::string& message) {
    std::string text = tag;
    text.push_back('\0');
    text += message;
    text.push_back('\0');
    const std::size_t parts = (text.size() + kRecordTextBytes - 1) / kRecordTextBytes;
    assert(parts <= kMaxEventRecords);
    text.resize(parts * kRecordTextBytes, '\0');
    std::vector<Record> records(parts);
    for (std::size_t i = 0; i < parts; ++i) {
        records[i].tUs = tUs;
        records[i].kind = i == 0 ? RecordKind::event : RecordKind::eventContinued;
        records[i].count = i == 0 ? static_cast<std::uint8_t>(parts - 1) : 0;
        std::memcpy(records[i].text, text.data() + i * kRecordTextBytes, kRecordTextBytes);
    }
    return records;
}

// --- formatNumber ------------------------------------------------------------

void testNumberGoldens() {
    checkNumber(0.0, 4, "0");
    checkNumber(-0.0, 4, "0");
    checkNumber(0.1f, 4, "0.1"); // a float's 0.1000000015 at 4 places
    checkNumber(-0.00004, 4, "0");
    checkNumber(-0.00005, 4, "-0.0001"); // halves round away from zero
    checkNumber(12.5, 0, "13");
    checkNumber(-12.5, 0, "-13");
    checkNumber(123456.789, 3, "123456.789");
    checkNumber(1.0, 4, "1");
    checkNumber(1.5, 4, "1.5");
    checkNumber(-2.25, 2, "-2.25");
    checkNumber(12.61f, 2, "12.61");
    checkNumber(31.3483f, 4, "31.3483");
    checkNumber(-0.0082f, 4, "-0.0082");
    checkNumber(0.01f, 4, "0.01");
    checkNumber(0.000001, 6, "0.000001");
    checkNumber(4294967295.0, 0, "4294967295");
    checkNumber(999999999999.4, 0, "999999999999");
    checkNumber(999999999999.5, 0, "1000000000000");
    checkNumber(kNaN, 4, "nan");
    checkNumber(-kNaN, 4, "nan");
    checkNumber(kInf, 4, "inf");
    checkNumber(-kInf, 4, "-inf");
    checkNumber(1e12, 4, "1.000000e+12");
    checkNumber(-2.5e15, 2, "-2.500000e+15");
    checkNumber(3.4028234663852886e38, 4, "3.402823e+38"); // FLT_MAX
    // Decimals are clamped to 0-6.
    checkNumber(1.23456789, 9, "1.234568");
    checkNumber(1.5, -3, "2");
}

void testNumberBufferTooSmall() {
    char buffer[8] = {'#', '#', '#', '#', '#', '#', '#', '#'};
    assert(formatNumber(buffer, 2, 123.0, 0) == 0);
    assert(buffer[0] == '#'); // nothing written at all
    assert(formatNumber(buffer, 3, 123.0, 0) == 3);
    assert(std::string(buffer, 3) == "123");
    assert(formatNumber(buffer, 0, 0.0, 0) == 0);
}

// --- sanitizeName --------------------------------------------------------------

void testSanitizeName() {
    assert(sanitizeName("lift.act") == "lift.act");
    assert(sanitizeName("char-fwd_2") == "char-fwd_2");
    assert(sanitizeName("my channel") == "my_channel");
    assert(sanitizeName("a,b\nc") == "a_b_c");
    assert(sanitizeName("") == "_");
    assert(sanitizeName(nullptr) == "_");
    assert(sanitizeName(std::string(40, 'x').c_str()) == std::string(31, 'x'));
    assert(sanitizeName("a_very_long_event_tag", 15) == "a_very_long_eve");
}

// --- Rows ----------------------------------------------------------------------

void testSchemaLines() {
    auto chan = [](const ChannelSchema& s) {
        return [s](char* out, std::size_t size) { return formatSchema(out, size, s); };
    };
    checkLine(chan(schema(0, "sys", ChannelKind::events, 0)), "#chan,0,sys,events,0\n");
    checkLine(chan(pidSchema(2, "drive", 4)),
              "#chan,2,drive,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags\n");
    checkLine(chan(schema(5, "odom", ChannelKind::samples, 4, {"x", "y", "heading"})),
              "#chan,5,odom,samples,4,x,y,heading\n");
    // pid columns are fixed by the spec, whatever the schema holds; names and
    // columns are sanitized; decimals clamped.
    checkLine(chan(schema(9, "bad name", ChannelKind::pid, 9)),
              "#chan,9,bad_name,pid,6,target,meas,err,p,i,d,u_raw,out,dt,flags\n");
    checkLine(chan(schema(10, "x", ChannelKind::samples, -1, {"fwd v", ""})),
              "#chan,10,x,samples,0,fwd_v,_\n");
    checkLine(chan(schema(11, "ev", ChannelKind::events, 4, {"ignored"})),
              "#chan,11,ev,events,4\n");
}

void testRecordRows() {
    const ChannelSchema odom = schema(5, "odom", ChannelKind::samples, 4, {"x", "y", "heading"});
    const ChannelSchema turn = pidSchema(3, "turn", 4);
    auto row = [](const ChannelSchema& s, const Record& r) {
        return [s, r](char* out, std::size_t size) { return formatRecord(out, size, s, r); };
    };

    checkLine(row(odom, sample(15013203, {0.0f, 0.0f, 0.12f})), "S,5,15013203,0,0,0.12\n");
    // Fewer values than columns: the rest are nan, so the row keeps its width.
    checkLine(row(odom, sample(1, {1.0f, 2.0f})), "S,5,1,1,2,nan\n");
    checkLine(row(odom, sample(2, {std::numeric_limits<float>::quiet_NaN(),
                                   std::numeric_limits<float>::infinity(), -0.0f})),
              "S,5,2,nan,inf,0\n");
    checkLine(row(turn, sample(15003514,
                               {90.0f, 0.0f, 90.0f, 31.5f, 0.0f, 0.0f, 31.5f, 12.0f, 0.01f}, 13)),
              "S,3,15003514,90,0,90,31.5,0,0,31.5,12,0.01,13\n");
    checkLine(row(turn, wide(15003512, RecordKind::pidGains, {0.35, 0.0, 0.0002})),
              "G,3,15003512,0.35,0,0.0002\n");
    // %.9g: gains keep full precision, exponent form where printf picks it.
    checkLine(row(turn, wide(7, RecordKind::pidGains, {1.5e-05, 123456789012.0, -0.0})),
              "G,3,7,1.5e-05,1.23456789e+11,0\n");
    checkLine(row(turn, wide(8, RecordKind::pidGains, {kNaN, kInf, 0.1})), "G,3,8,nan,inf,0.1\n");
    checkLine(row(turn, wide(15003511, RecordKind::pidConfig, {0.0, 12.0, 0.0, 0.0, 0.01})),
              "C,3,15003511,0,12,0,0,0.01\n");
    checkLine(row(turn, wide(9, RecordKind::pidConfig, {5.0, 12.7, 0.5, 1.0, 0.02})),
              "C,3,9,5,12.7,0.5,1,0.02\n");
    checkLine(row(turn, reset(16871020)), "R,3,16871020\n");
    checkLine(row(odom, eventRecords(15003390, "auton", "start,Turn Testing")[0]),
              "E,15003390,auton,start,Turn Testing\n");

    // A continuation on its own is never a row.
    Record stray{};
    stray.kind = RecordKind::eventContinued;
    char buffer[64];
    assert(formatRecord(buffer, sizeof(buffer), odom, stray) == 0);
}

void testEventRows() {
    auto event = [](std::uint64_t t, const char* tag, const char* message) {
        return [=](char* out, std::size_t size) { return formatEvent(out, size, t, tag, message); };
    };
    // Commas in the message are kept: it's the last field and runs to the end.
    checkLine(event(2104502, "phase", "disabled,comp=1,field=1"),
              "E,2104502,phase,disabled,comp=1,field=1\n");
    // CR, LF, tabs and other control bytes would break the line: spaces.
    checkLine(event(1, "mark", "a\r\nb\tc\x01"), "E,1,mark,a  b c \n");
    checkLine(event(2, "my tag!", ""), "E,2,my_tag_,\n");
    checkLine(event(3, "", "x"), "E,3,_,x\n");
    checkLine(event(4, nullptr, nullptr), "E,4,_,\n");
    checkLine(event(5, "sixteen_chars_ab", "m"), "E,5,sixteen_chars_a,m\n");

    // An event spanning Records reads back whole.
    const std::string message =
        "start,moveToPose,x=-123.456,y=-123.456,heading_deg=359.999,pos_threshold=1.000,"
        "heading_threshold=2.000,settle_ms=250,timeout_ms=4000";
    const std::vector<Record> records = eventRecords(15003400, "motion", message);
    assert(records.size() == 3);
    const Record* parts[] = {&records[0], &records[1], &records[2]};
    checkLine([&](char* out, std::size_t size) { return formatEventRecords(out, size, parts, 3); },
              "E,15003400,motion," + message + "\n");

    // The longest possible event still fits well inside kMaxLineBytes.
    const std::vector<Record> longest =
        eventRecords(18446744073709551615ull, std::string(15, 't'), std::string(191, 'm'));
    assert(longest.size() == kMaxEventRecords);
    const Record* longestParts[] = {&longest[0], &longest[1], &longest[2], &longest[3]};
    char line[kMaxLineBytes];
    const std::size_t length = formatEventRecords(line, sizeof(line), longestParts, 4);
    assert(length == 2 + 20 + 1 + 15 + 1 + 191 + 1);
}

void testDropsAndHealthRows() {
    checkLine([](char* out, std::size_t size) { return formatDrops(out, size, 61000044, 1, 0, 1); },
              "D,61000044,1,0,1\n");
    const WriterStats stats{.rows = 4,
                            .bytes = 1034,
                            .writes = 1,
                            .writeMaxUs = 21873,
                            .writeAvgUs = 21873,
                            .drops = 0,
                            .unlogged = 2,
                            .resyncs = 0,
                            .breaks = 0,
                            .faults = 0};
    checkLine([&](char* out, std::size_t size) { return formatHealth(out, size, 3104771, stats); },
              "H,3104771,rows=4,bytes=1034,writes=1,wmax_us=21873,wavg_us=21873,drops=0,"
              "unlogged=2,resyncs=0,breaks=0,faults=0\n");
}

void testHeader() {
    const FileHeader header{.library = "0.1.0",
                            .kernel = "4.2.2",
                            .build = "Sep 27 2026 14:02:11",
                            .robotName = "96671H",
                            .fileName = "SL000042.CSV",
                            .directory = "/usd/sl",
                            .openUs = 2104331};
    checkLine([&](char* out, std::size_t size) { return formatHeader(out, size, header); },
              "#SLT,1\n"
              "#meta,writer,sapphirelib 0.1.0\n"
              "#meta,kernel,4.2.2\n"
              "#meta,build,Sep 27 2026 14:02:11\n"
              "#meta,robot,96671H\n"
              "#meta,file,SL000042.CSV\n"
              "#meta,dir,/usd/sl\n"
              "#meta,open_us,2104331\n"
              "#meta,clock,us_since_program_start\n");

    // Nulls and newlines in the strings can't corrupt the block.
    const FileHeader messy{.library = nullptr, .robotName = "bad\nname", .openUs = 0};
    checkLine([&](char* out, std::size_t size) { return formatHeader(out, size, messy); },
              "#SLT,1\n"
              "#meta,writer,sapphirelib \n"
              "#meta,kernel,\n"
              "#meta,build,\n"
              "#meta,robot,bad name\n"
              "#meta,file,\n"
              "#meta,dir,\n"
              "#meta,open_us,0\n"
              "#meta,clock,us_since_program_start\n");
}

// --- A whole file ----------------------------------------------------------------

/// The same bytes are embedded in tools/telemetry/slt_read.py (SELFTEST_FILE),
/// which parses them in its --selftest: keep the two in step.
const char* const kGoldenFile =
    "#SLT,1\n"
    "#meta,writer,sapphirelib 0.1.0\n"
    "#meta,kernel,4.2.2\n"
    "#meta,build,Sep 27 2026 14:02:11\n"
    "#meta,robot,96671H\n"
    "#meta,file,SL000042.CSV\n"
    "#meta,dir,/usd/sl\n"
    "#meta,open_us,2104331\n"
    "#meta,clock,us_since_program_start\n"
    "#chan,0,sys,events,0\n"
    "#chan,1,events,events,0\n"
    "#chan,2,drive,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags\n"
    "#chan,3,turn,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags\n"
    "#chan,5,odom,samples,4,x,y,heading\n"
    "#chan,6,batt,samples,2,volts,pct\n"
    "#chan,7,lift,pid,3,target,meas,err,p,i,d,u_raw,out,dt,flags\n"
    "#chan,8,lift.act,samples,3,target,pos,volts,law\n"
    "E,2104502,file,open,SL000042.CSV\n"
    "E,2104502,phase,disabled,comp=1,field=1\n"
    "H,3104771,rows=4,bytes=1034,writes=1,wmax_us=21873,wavg_us=21873,drops=0,unlogged=2,"
    "resyncs=0,breaks=0,faults=0\n"
    "E,15003114,phase,autonomous,comp=1,field=1\n"
    "S,5,15003201,0,0,0\n"
    "S,6,15003201,12.61,87\n"
    "E,15003390,auton,start,Turn Testing\n"
    "E,15003400,motion,start,turnToHeading,target_deg=90.000,threshold=2.000,settle_ms=200,"
    "timeout_ms=3000\n"
    "C,3,15003511,0,12,0,0,0.01\n"
    "G,3,15003512,0.35,0,0.0002\n"
    "S,3,15003514,90,0,90,31.5,0,0,31.5,12,0.01,13\n"
    "S,5,15013203,0,0,0.12\n"
    "S,3,15013604,89.59,0,89.59,31.3565,0,-0.0082,31.3483,12,0.01,5\n"
    "S,3,15023690,88.63,0,88.63,31.0205,0,-0.0192,31.0013,12,0.01,5\n"
    "S,3,15612874,1.84,0,1.84,0.644,0,-0.0012,0.6428,0.6428,0.01,0\n"
    "E,15612900,motion,end,turnToHeading,reason=settled,error=0.412,ms=609\n"
    "E,16871020,motion,start,turnToHeading,target_deg=0.000,threshold=2.000,settle_ms=200,"
    "timeout_ms=3000\n"
    "R,3,16871021\n"
    "S,3,16871027,-89.97,0,-89.97,-31.4895,0,0,-31.4895,-12,0.01,13\n"
    "E,30001022,phase,opcontrol,comp=1,field=1\n"
    "C,7,31540205,0,12.7,0,1,0.02\n"
    "G,7,31540206,0.2,0,0.01\n"
    "S,7,31540210,150,0.37,149.63,29.926,0,0,29.926,12.7,0.02,13\n"
    "S,8,31540236,150,0.37,12,0\n"
    "S,5,31540301,23.4512,-11.0833,91.87\n"
    "S,7,31560198,150,3.12,146.88,29.376,0,-1.375,28.001,12.7,0.02,5\n"
    "S,8,31560221,150,3.12,12,0\n"
    "E,48220114,mark,driver\n"
    "D,61000044,1,0,1\n";

void testGoldenFile() {
    const ChannelSchema sys = schema(0, "sys", ChannelKind::events, 0);
    const ChannelSchema events = schema(1, "events", ChannelKind::events, 0);
    const ChannelSchema drive = pidSchema(2, "drive", 4);
    const ChannelSchema turn = pidSchema(3, "turn", 4);
    const ChannelSchema odom = schema(5, "odom", ChannelKind::samples, 4, {"x", "y", "heading"});
    const ChannelSchema batt = schema(6, "batt", ChannelKind::samples, 2, {"volts", "pct"});
    const ChannelSchema lift = pidSchema(7, "lift", 3);
    const ChannelSchema liftAct =
        schema(8, "lift.act", ChannelKind::samples, 3, {"target", "pos", "volts", "law"});

    std::vector<char> file(16384);
    std::size_t used = 0;
    auto append = [&](std::size_t length) {
        assert(length > 0);
        used += length;
    };
    char* const base = file.data();
    auto out = [&] { return base + used; };
    auto room = [&] { return file.size() - used; };
    auto row = [&](const ChannelSchema& s, const Record& r) {
        append(formatRecord(out(), room(), s, r));
    };
    auto ev = [&](const ChannelSchema& s, std::uint64_t t, const char* tag, const char* msg) {
        const std::vector<Record> records = eventRecords(t, tag, msg);
        const Record* parts[kMaxEventRecords] = {};
        for (std::size_t i = 0; i < records.size(); ++i) parts[i] = &records[i];
        if (records.size() == 1) {
            row(s, records[0]);
        } else {
            append(formatEventRecords(out(), room(), parts, records.size()));
        }
    };

    append(formatHeader(out(), room(),
                        FileHeader{.library = "0.1.0",
                                   .kernel = "4.2.2",
                                   .build = "Sep 27 2026 14:02:11",
                                   .robotName = "96671H",
                                   .fileName = "SL000042.CSV",
                                   .directory = "/usd/sl",
                                   .openUs = 2104331}));
    for (const ChannelSchema* s : {&sys, &events, &drive, &turn, &odom, &batt, &lift, &liftAct}) {
        append(formatSchema(out(), room(), *s));
    }
    append(formatEvent(out(), room(), 2104502, "file", "open,SL000042.CSV"));
    append(formatEvent(out(), room(), 2104502, "phase", "disabled,comp=1,field=1"));
    append(formatHealth(out(), room(), 3104771,
                        WriterStats{.rows = 4,
                                    .bytes = 1034,
                                    .writes = 1,
                                    .writeMaxUs = 21873,
                                    .writeAvgUs = 21873,
                                    .unlogged = 2}));
    ev(sys, 15003114, "phase", "autonomous,comp=1,field=1");
    row(odom, sample(15003201, {0.0f, 0.0f, 0.0f}));
    row(batt, sample(15003201, {12.61f, 87.0f}));
    ev(events, 15003390, "auton", "start,Turn Testing");
    ev(events, 15003400, "motion",
       "start,turnToHeading,target_deg=90.000,threshold=2.000,settle_ms=200,timeout_ms=3000");
    row(turn, wide(15003511, RecordKind::pidConfig, {0.0, 12.0, 0.0, 0.0, 0.01}));
    row(turn, wide(15003512, RecordKind::pidGains, {0.35, 0.0, 0.0002}));
    row(turn, sample(15003514, {90.0f, 0.0f, 90.0f, 31.5f, 0.0f, 0.0f, 31.5f, 12.0f, 0.01f}, 13));
    row(odom, sample(15013203, {0.0f, 0.0f, 0.12f}));
    row(turn, sample(15013604,
                     {89.59f, 0.0f, 89.59f, 31.3565f, 0.0f, -0.0082f, 31.3483f, 12.0f, 0.01f}, 5));
    row(turn, sample(15023690,
                     {88.63f, 0.0f, 88.63f, 31.0205f, 0.0f, -0.0192f, 31.0013f, 12.0f, 0.01f}, 5));
    row(turn,
        sample(15612874, {1.84f, 0.0f, 1.84f, 0.644f, 0.0f, -0.0012f, 0.6428f, 0.6428f, 0.01f}, 0));
    ev(events, 15612900, "motion", "end,turnToHeading,reason=settled,error=0.412,ms=609");
    ev(events, 16871020, "motion",
       "start,turnToHeading,target_deg=0.000,threshold=2.000,settle_ms=200,timeout_ms=3000");
    row(turn, reset(16871021));
    row(turn,
        sample(16871027, {-89.97f, 0.0f, -89.97f, -31.4895f, 0.0f, 0.0f, -31.4895f, -12.0f, 0.01f},
               13));
    ev(sys, 30001022, "phase", "opcontrol,comp=1,field=1");
    row(lift, wide(31540205, RecordKind::pidConfig, {0.0, 12.7, 0.0, 1.0, 0.02}));
    row(lift, wide(31540206, RecordKind::pidGains, {0.2, 0.0, 0.01}));
    row(lift,
        sample(31540210, {150.0f, 0.37f, 149.63f, 29.926f, 0.0f, 0.0f, 29.926f, 12.7f, 0.02f}, 13));
    row(liftAct, sample(31540236, {150.0f, 0.37f, 12.0f, 0.0f}));
    row(odom, sample(31540301, {23.4512f, -11.0833f, 91.87f}));
    row(lift, sample(31560198,
                     {150.0f, 3.12f, 146.88f, 29.376f, 0.0f, -1.375f, 28.001f, 12.7f, 0.02f}, 5));
    row(liftAct, sample(31560221, {150.0f, 3.12f, 12.0f, 0.0f}));
    ev(events, 48220114, "mark", "driver");
    append(formatDrops(out(), room(), 61000044, 1, 0, 1));

    const std::string actual(file.data(), used);
    if (actual != kGoldenFile) {
        std::printf("golden file mismatch:\n%s\n", actual.c_str());
        assert(false);
    }
}

} // namespace

int main() {
    testNumberGoldens();
    testNumberBufferTooSmall();
    testSanitizeName();
    testSchemaLines();
    testRecordRows();
    testEventRows();
    testDropsAndHealthRows();
    testHeader();
    testGoldenFile();
    std::printf("csv_format_test: all tests passed\n");
    return 0;
}
