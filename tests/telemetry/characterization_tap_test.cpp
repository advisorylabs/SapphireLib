// Host-side unit test for sapphirelib::telemetry::tapCharacterization() and
// tapMechanismCharacterization() — no PROS/embedded dependencies. The
// characterization runners read time and sleep only through util/clock.hpp,
// which this file defines as a fake clock that moves a simulated axis, so it
// drives the real runners (src/sapphirelib/tuning/characterization_runner.cpp).
// What's under test is that the tap logs exactly one row per
// CharacterizationSample the runner records, whatever order the runner calls
// measure(), actuate() and hold() in.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/characterization_tap_test.cpp src/sapphirelib/telemetry/characterization_tap.cpp src/sapphirelib/telemetry/channel.cpp src/sapphirelib/telemetry/record_ring.cpp src/sapphirelib/control/pid.cpp src/sapphirelib/tuning/characterization_runner.cpp -o characterization_tap_test && ./characterization_tap_test
// clang-format on

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "sapphirelib/telemetry/channel.hpp"
#include "sapphirelib/telemetry/characterization_tap.hpp"

namespace {
std::uint32_t fakeMs = 0;
} // namespace


using sapphirelib::telemetry::Channel;
using sapphirelib::telemetry::ChannelKind;
using sapphirelib::telemetry::ChannelSchema;
using sapphirelib::telemetry::Record;
using sapphirelib::telemetry::RecordKind;
using sapphirelib::telemetry::tapCharacterization;
using sapphirelib::telemetry::tapMechanismCharacterization;
using sapphirelib::tuning::CharacterizationConfig;
using sapphirelib::tuning::CharacterizationData;
using sapphirelib::tuning::CharacterizationRun;
using sapphirelib::tuning::CharacterizationSample;
using sapphirelib::tuning::MechanismCharacterizationConfig;

namespace {

// --- A simulated axis ---------------------------------------------------------

/// First-order motor + inertia: speed chases gain * volts with time constant
/// tauS, integrated in 1ms steps as the fake clock advances. Every call the
/// runner makes is logged, so a tapped and an untapped run can be compared
/// call for call.
struct Axis {
    double gain = 5.0; // steady-state units/s per volt
    double tauS = 0.08;
    double volts = 0.0;
    double speed = 0.0;
    double position = 0.0;
    std::vector<std::string> calls;

    bool held = false; // on the brake: see hold()

    void advance(std::uint32_t ms) {
        for (std::uint32_t i = 0; i < ms; ++i) {
            if (held) {
                speed = 0.0;
                ++fakeMs;
                continue;
            }
            speed += (gain * volts - speed) * (0.001 / tauS);
            position += speed * 0.001;
            ++fakeMs;
        }
    }

    double measure() {
        char text[64];
        std::snprintf(text, sizeof(text), "measure@%u=%.17g", fakeMs, position);
        calls.emplace_back(text);
        return position;
    }

    void actuate(double v) {
        char text[64];
        std::snprintf(text, sizeof(text), "actuate@%u(%.17g)", fakeMs, v);
        calls.emplace_back(text);
        volts = v;
        held = false;
    }

    void hold() {
        char text[64];
        std::snprintf(text, sizeof(text), "hold@%u", fakeMs);
        calls.emplace_back(text);
        held = true;
    }
};

Axis* axis = nullptr; // the axis the fake clock moves

} // namespace

// The clock seam (util/clock.hpp), faked: time only moves when the runner
// sleeps, and moving it moves the axis.
namespace sapphirelib {
std::uint32_t millis() { return fakeMs; }
std::uint64_t micros() { return static_cast<std::uint64_t>(fakeMs) * 1000; }
void delayMs(std::uint32_t ms) { axis->advance(ms); }
} // namespace sapphirelib

namespace {

// --- Helpers -------------------------------------------------------------------

struct Row {
    std::uint64_t tUs;
    float volts;
    float position;
};

Channel makeChannel() {
    // Big enough that no row of a whole run is ever dropped.
    return Channel(ChannelSchema{.id = 7,
                                 .name = "char.fwd",
                                 .kind = ChannelKind::samples,
                                 .decimals = 4,
                                 .columns = {"volts", "pos"}},
                   8192);
}

std::vector<Row> drain(Channel& channel) {
    std::vector<Row> rows;
    while (const Record* record = channel.front()) {
        assert(record->kind == RecordKind::sample);
        assert(record->count == 2);
        rows.push_back(Row{record->tUs, record->values[0], record->values[1]});
        channel.pop();
    }
    return rows;
}

CharacterizationConfig configFor(Axis& target, double maxTravel, std::uint32_t maxSegmentMs) {
    CharacterizationConfig config;
    config.actuate = [&target](double v) { target.actuate(v); };
    config.measure = [&target] { return target.measure(); };
    config.maxTravel = maxTravel;
    config.maxSegmentMs = maxSegmentMs;
    return config;
}

/// Runs the runner twice from the same start — once plain, once tapped
/// — and returns what the tapped run logged, after checking the tap changed
/// nothing the runner or the axis could see.
struct TappedRun {
    CharacterizationData data;
    std::vector<Row> rows;
};

TappedRun runBoth(double maxTravel, std::uint32_t maxSegmentMs) {
    Axis plain;
    axis = &plain;
    fakeMs = 1000;
    const CharacterizationData expected =
        sapphirelib::tuning::runCharacterization(configFor(plain, maxTravel, maxSegmentMs));

    Axis tapped;
    axis = &tapped;
    fakeMs = 1000;
    Channel channel = makeChannel();
    const CharacterizationData data = sapphirelib::tuning::runCharacterization(
        tapCharacterization(configFor(tapped, maxTravel, maxSegmentMs), channel));

    // The original callbacks saw exactly the same calls, same arguments, same
    // order, at the same (fake) times...
    assert(tapped.calls == plain.calls);
    // ...so the runner recorded exactly the same data.
    auto sameRuns = [](const std::vector<CharacterizationRun>& a,
                       const std::vector<CharacterizationRun>& b) {
        assert(a.size() == b.size());
        for (std::size_t r = 0; r < a.size(); ++r) {
            assert(a[r].size() == b[r].size());
            for (std::size_t i = 0; i < a[r].size(); ++i) {
                assert(a[r][i].timeMs == b[r][i].timeMs);
                assert(a[r][i].volts == b[r][i].volts);
                assert(a[r][i].position == b[r][i].position);
            }
        }
    };
    sameRuns(data.ramps, expected.ramps);
    sameRuns(data.steps, expected.steps);
    assert(channel.droppedFull() == 0 && channel.droppedContended() == 0);

    return TappedRun{data, drain(channel)};
}

std::vector<CharacterizationRun> segmentsInOrder(const CharacterizationData& data) {
    // runCharacterization()'s order: ramp forward, ramp back, step forward,
    // step back.
    std::vector<CharacterizationRun> runs = data.ramps;
    runs.insert(runs.end(), data.steps.begin(), data.steps.end());
    return runs;
}

bool sameAsSample(const Row& row, const CharacterizationSample& sample) {
    return row.volts == static_cast<float>(sample.volts) &&
           row.position == static_cast<float>(sample.position);
}

// --- Tests -----------------------------------------------------------------------

void testUnlimitedTravelLogsExactlyTheSamples() {
    // maxTravel 0 (a turn axis): every segment runs to its duration cap, so
    // the rows are the samples — no more, no fewer, in order.
    const TappedRun run = runBoth(0.0, 400);
    const std::vector<CharacterizationRun> segments = segmentsInOrder(run.data);

    std::size_t expected = 0;
    for (const CharacterizationRun& segment : segments) expected += segment.size();
    assert(expected > 100); // the run really did something
    assert(run.rows.size() == expected);

    std::size_t row = 0;
    for (const CharacterizationRun& segment : segments) {
        for (const CharacterizationSample& sample : segment) {
            assert(sameAsSample(run.rows[row], sample));
            ++row;
        }
    }
}

void testTravelLimitedSegmentsEndWithOneZeroVoltRow() {
    // A translation axis with a short travel limit: each segment is cut off
    // by it, and the runner's closing actuate(0) follows a measure() — the
    // one that found the axis out of range. That shows up as a single 0V row
    // at that position after the segment's samples (the documented extra).
    constexpr double kMaxTravel = 4.0;
    const TappedRun run = runBoth(kMaxTravel, 2500);
    const std::vector<CharacterizationRun> segments = segmentsInOrder(run.data);

    std::size_t row = 0;
    for (const CharacterizationRun& segment : segments) {
        assert(!segment.empty());
        // Cut short by travel, not by the duration cap.
        assert(segment.back().timeMs + 10 < 100 + 2500);
        for (const CharacterizationSample& sample : segment) {
            assert(row < run.rows.size());
            assert(sameAsSample(run.rows[row], sample));
            ++row;
        }
        // The extra row. The segment's origin is its first sample's position:
        // the fake axis only moves when the fake clock does.
        assert(row < run.rows.size());
        const Row& extra = run.rows[row];
        assert(extra.volts == 0.0f);
        assert(std::fabs(extra.position - static_cast<float>(segment.front().position)) >=
               static_cast<float>(kMaxTravel) - 1e-4f);
        ++row;
    }
    assert(row == run.rows.size()); // nothing else: waits between segments log nothing
}

void testRowsAreTimestampedWhenActuated() {
    const TappedRun run = runBoth(0.0, 300);
    for (std::size_t i = 1; i < run.rows.size(); ++i) {
        assert(run.rows[i].tUs >= run.rows[i - 1].tUs);
    }
    // Within a segment rows are one sample period apart; between segments
    // there's the stop-wait (at least one 100ms check).
    std::size_t gaps = 0;
    for (std::size_t i = 1; i < run.rows.size(); ++i) {
        const std::uint64_t dt = run.rows[i].tUs - run.rows[i - 1].tUs;
        assert(dt == 10000 || dt >= 100000);
        if (dt >= 100000) ++gaps;
    }
    assert(gaps == 3); // four segments
}

void testIncompleteConfigIsReturnedUnchanged() {
    Axis target;
    axis = &target;
    fakeMs = 0;
    Channel channel = makeChannel();

    CharacterizationConfig noMeasure = configFor(target, 0.0, 100);
    noMeasure.measure = nullptr;
    CharacterizationConfig tapped = tapCharacterization(noMeasure, channel);
    assert(!tapped.measure);
    tapped.actuate(3.0);
    assert(target.volts == 3.0); // still the original callback
    assert(channel.front() == nullptr);

    CharacterizationConfig noActuate = configFor(target, 0.0, 100);
    noActuate.actuate = nullptr;
    tapped = tapCharacterization(noActuate, channel);
    assert(!tapped.actuate);
    assert(tapped.measure() == target.position);
    assert(channel.front() == nullptr);

    // The rest of the config passes through untouched.
    CharacterizationConfig custom = configFor(target, 12.0, 1234);
    custom.stepVolts = 7.5;
    custom.samplePeriodMs = 20;
    tapped = tapCharacterization(custom, channel);
    assert(tapped.maxTravel == 12.0 && tapped.maxSegmentMs == 1234);
    assert(tapped.stepVolts == 7.5 && tapped.samplePeriodMs == 20);
}

void testActuateWithoutFreshMeasureLogsNothing() {
    Axis target;
    axis = &target;
    fakeMs = 0;
    Channel channel = makeChannel();
    const CharacterizationConfig tapped = tapCharacterization(configFor(target, 0.0, 100), channel);

    tapped.actuate(1.0); // no measure() yet
    assert(channel.front() == nullptr);

    target.position = 2.5;
    assert(tapped.measure() == 2.5);
    assert(tapped.measure() == 2.5); // polling twice still pairs with one actuate
    tapped.actuate(4.0);
    tapped.actuate(0.0); // no fresh measure since the last row
    const std::vector<Row> rows = drain(channel);
    assert(rows.size() == 1);
    assert(rows[0].volts == 4.0f && rows[0].position == 2.5f);
}

void testMechanismHeldSamplesAreLoggedAsNan() {
    // The mechanism runner holds the axis (rather than 0V) through each
    // pre-roll and after each segment. Held samples are recorded with NaN
    // volts, and the tap must log them the same way — plus one NaN row where
    // each limit-cut segment ended — and change nothing else.
    const auto configFor = [](Axis& target) {
        MechanismCharacterizationConfig config;
        config.axis.actuate = [&target](double v) { target.actuate(v); };
        config.axis.measure = [&target] { return target.measure(); };
        config.hold = [&target] { target.hold(); };
        config.lowerLimit = 0.5;
        config.upperLimit = 6.0;
        return config;
    };

    Axis plain;
    axis = &plain;
    fakeMs = 1000;
    const CharacterizationData expected =
        sapphirelib::tuning::runMechanismCharacterization(configFor(plain));

    Axis tapped;
    axis = &tapped;
    fakeMs = 1000;
    Channel channel = makeChannel();
    const CharacterizationData data = sapphirelib::tuning::runMechanismCharacterization(
        tapMechanismCharacterization(configFor(tapped), channel));
    assert(tapped.calls == plain.calls);
    assert(!data.aborted);

    const std::vector<Row> rows = drain(channel);
    const std::vector<CharacterizationRun> segments = segmentsInOrder(data);
    assert(segmentsInOrder(expected).size() == segments.size() && segments.size() == 4);
    std::size_t row = 0;
    for (const CharacterizationRun& segment : segments) {
        assert(!segment.empty());
        int heldSamples = 0;
        for (const CharacterizationSample& sample : segment) {
            assert(row < rows.size());
            if (std::isnan(sample.volts)) {
                ++heldSamples;
                assert(std::isnan(rows[row].volts));
                assert(rows[row].position == static_cast<float>(sample.position));
            } else {
                assert(sameAsSample(rows[row], sample));
            }
            ++row;
        }
        assert(heldSamples == 10); // 100ms pre-roll at 10ms
        // The limit-cut extra row: held, at the out-of-range position.
        assert(row < rows.size());
        assert(std::isnan(rows[row].volts));
        ++row;
    }
    assert(row == rows.size());
    // Up segments end at the upper limit, down segments at the lower one.
    assert(segments[0].back().position < 6.0 && rows.size() > 40);
}

} // namespace

int main() {
    testUnlimitedTravelLogsExactlyTheSamples();
    testTravelLimitedSegmentsEndWithOneZeroVoltRow();
    testRowsAreTimestampedWhenActuated();
    testIncompleteConfigIsReturnedUnchanged();
    testActuateWithoutFreshMeasureLogsNothing();
    testMechanismHeldSamplesAreLoggedAsNan();
    std::printf("characterization_tap_test: all tests passed\n");
    return 0;
}
