// Host-side unit test for sapphirelib::telemetry::tapCharacterization() — no
// PROS/embedded dependencies. The characterization runner itself sleeps on
// pros::delay(), so this file carries a copy of its loop
// (src/sapphirelib/tuning/characterization_runner.cpp) with the PROS calls
// swapped for a fake clock, and drives it against a simulated axis. Keep the
// copy in step with the real runner: what's under test is that the tap logs
// exactly one row per CharacterizationSample the runner records, whatever
// order the runner calls measure() and actuate() in.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/characterization_tap_test.cpp src/sapphirelib/telemetry/characterization_tap.cpp src/sapphirelib/telemetry/channel.cpp src/sapphirelib/telemetry/record_ring.cpp src/sapphirelib/control/pid.cpp -o characterization_tap_test && ./characterization_tap_test
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

namespace sapphirelib {
std::uint64_t micros() { return static_cast<std::uint64_t>(fakeMs) * 1000; }
} // namespace sapphirelib

using sapphirelib::telemetry::Channel;
using sapphirelib::telemetry::ChannelKind;
using sapphirelib::telemetry::ChannelSchema;
using sapphirelib::telemetry::Record;
using sapphirelib::telemetry::RecordKind;
using sapphirelib::telemetry::tapCharacterization;
using sapphirelib::tuning::CharacterizationConfig;
using sapphirelib::tuning::CharacterizationData;
using sapphirelib::tuning::CharacterizationRun;
using sapphirelib::tuning::CharacterizationSample;

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

    void advance(std::uint32_t ms) {
        for (std::uint32_t i = 0; i < ms; ++i) {
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
    }
};

Axis* axis = nullptr; // the axis the fake clock moves

void delayMs(std::uint32_t ms) { axis->advance(ms); }
std::uint32_t millis() { return fakeMs; }

// --- Copy of characterization_runner.cpp, PROS calls swapped for the fakes ----

// In its own namespace so the calls below name it explicitly — otherwise
// argument-dependent lookup also finds the real tuning::runCharacterization().
namespace runner_copy {

constexpr double kStoppedFraction = 0.002;
constexpr std::uint32_t kStoppedCheckMs = 100;

void waitUntilStopped(const CharacterizationConfig& config) {
    config.actuate(0.0);
    const double threshold =
        std::fmax(config.maxTravel > 0.0 ? config.maxTravel * kStoppedFraction : 0.0, 0.01);
    const std::uint32_t start = millis();
    double last = config.measure();
    while (millis() - start < config.settleTimeoutMs) {
        delayMs(kStoppedCheckMs);
        const double now = config.measure();
        if (std::fabs(now - last) < threshold) return;
        last = now;
    }
}

template <typename VoltsAt>
CharacterizationRun runSegment(const CharacterizationConfig& config, VoltsAt voltsAt) {
    CharacterizationRun run;
    const std::uint32_t startMs = millis();
    const double origin = config.measure();

    while (true) {
        const std::uint32_t elapsedMs = millis() - startMs;
        if (elapsedMs >= config.preRollMs + config.maxSegmentMs) break;

        const double position = config.measure();
        if (config.maxTravel > 0.0 && std::fabs(position - origin) >= config.maxTravel) break;

        const double volts =
            elapsedMs < config.preRollMs ? 0.0 : voltsAt((elapsedMs - config.preRollMs) / 1000.0);
        config.actuate(volts);
        run.push_back(
            CharacterizationSample{.timeMs = elapsedMs, .volts = volts, .position = position});
        delayMs(config.samplePeriodMs);
    }

    config.actuate(0.0);
    return run;
}

CharacterizationData runCharacterization(const CharacterizationConfig& config) {
    CharacterizationData data;
    const double rampRate = std::fabs(config.rampVoltsPerS);
    const double rampMax = std::fabs(config.rampMaxVolts);
    const double step = std::fabs(config.stepVolts);

    for (const double direction : {1.0, -1.0}) {
        waitUntilStopped(config);
        data.ramps.push_back(runSegment(
            config, [&](double t) { return direction * std::min(rampRate * t, rampMax); }));
    }
    for (const double direction : {1.0, -1.0}) {
        waitUntilStopped(config);
        data.steps.push_back(runSegment(config, [&](double) { return direction * step; }));
    }

    waitUntilStopped(config);
    return data;
}

} // namespace runner_copy

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

/// Runs the copied runner twice from the same start — once plain, once tapped
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
        runner_copy::runCharacterization(configFor(plain, maxTravel, maxSegmentMs));

    Axis tapped;
    axis = &tapped;
    fakeMs = 1000;
    Channel channel = makeChannel();
    const CharacterizationData data = runner_copy::runCharacterization(
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

} // namespace

int main() {
    testUnlimitedTravelLogsExactlyTheSamples();
    testTravelLimitedSegmentsEndWithOneZeroVoltRow();
    testRowsAreTimestampedWhenActuated();
    testIncompleteConfigIsReturnedUnchanged();
    testActuateWithoutFreshMeasureLogsNothing();
    std::printf("characterization_tap_test: all tests passed\n");
    return 0;
}
