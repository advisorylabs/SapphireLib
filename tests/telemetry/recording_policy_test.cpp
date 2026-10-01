// Host-side unit test for sapphirelib::telemetry's RecordingPolicy, when the logger records: the
// Home page's button outside matches, and automatically under competition control. No
// PROS/embedded dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/telemetry/recording_policy_test.cpp src/sapphirelib/telemetry/recording_policy.cpp -o recording_policy_test && ./recording_policy_test

#include <cassert>
#include <cstdio>
#include <string_view>

#include "sapphirelib/telemetry/recording_policy.hpp"

using namespace sapphirelib::telemetry;

namespace {

constexpr std::uint32_t kDelayMs = 5000;
constexpr RecordingRequest kNone = RecordingRequest::none;
constexpr RecordingRequest kStart = RecordingRequest::start;
constexpr RecordingRequest kStop = RecordingRequest::stop;

bool started(const RecordingChange& change, RecordingReason reason) {
    return change.start && !change.stop && change.reason == reason;
}

bool stopped(const RecordingChange& change, RecordingReason reason) {
    return change.stop && !change.start && change.reason == reason;
}

bool nothing(const RecordingChange& change) { return !change.start && !change.stop; }

void testButtonOnlyInThePits() {
    // 96671H: nothing until the button, and the button both ways
    RecordingPolicy policy(false, true, kDelayMs);
    assert(nothing(policy.update(false, kNone, 0)));
    assert(!policy.recording());
    assert(policy.reason() == RecordingReason::none);
    assert(started(policy.update(false, kStart, 10), RecordingReason::manual));
    assert(policy.recording() && policy.reason() == RecordingReason::manual);
    assert(nothing(policy.update(false, kStart, 20))); // already recording
    assert(stopped(policy.update(false, kStop, 30), RecordingReason::manual));
    assert(nothing(policy.update(false, kStop, 40))); // already stopped
    assert(!policy.recording());
}

void testRecordAtStart() {
    // the library default: one recording from startup, which the competition rules leave alone
    RecordingPolicy policy(true, true, kDelayMs);
    assert(started(policy.update(true, kNone, 0), RecordingReason::startup));
    assert(nothing(policy.update(false, kNone, 10)));
    assert(nothing(policy.update(false, kNone, 10 + 2 * kDelayMs)));
    assert(policy.recording() && policy.reason() == RecordingReason::startup);
}

void testMatchRecordsItself() {
    RecordingPolicy policy(false, true, kDelayMs);
    assert(nothing(policy.update(false, kNone, 0)));
    assert(started(policy.update(true, kNone, 100), RecordingReason::competition));
    assert(nothing(policy.update(true, kNone, 200)));
    // unplugged: still recording until the delay runs out
    assert(nothing(policy.update(false, kNone, 1000)));
    assert(nothing(policy.update(false, kNone, 1000 + kDelayMs - 1)));
    assert(stopped(policy.update(false, kNone, 1000 + kDelayMs), RecordingReason::competition));
    assert(!policy.recording());
}

void testTetherBlipKeepsOneFile() {
    RecordingPolicy policy(false, true, kDelayMs);
    assert(started(policy.update(true, kNone, 0), RecordingReason::competition));
    assert(nothing(policy.update(false, kNone, 1000)));
    assert(nothing(policy.update(true, kNone, 1500))); // back before the delay: same recording
    assert(nothing(policy.update(true, kNone, 1000 + 2 * kDelayMs)));
    assert(policy.recording() && policy.reason() == RecordingReason::competition);
}

void testConnectedAtStartup() {
    // plugged into the field before the program started: the first update is the connection
    RecordingPolicy policy(false, true, kDelayMs);
    assert(started(policy.update(true, kNone, 0), RecordingReason::competition));
}

void testCompetitionRecordingOff() {
    RecordingPolicy policy(false, false, kDelayMs);
    assert(nothing(policy.update(true, kNone, 0)));
    assert(!policy.recording());
}

void testManualRecordingSurvivesTheSwitch() {
    // started by hand in the pits, then plugged into a switch and out again: still the user's
    RecordingPolicy policy(false, true, kDelayMs);
    assert(started(policy.update(false, kStart, 0), RecordingReason::manual));
    assert(nothing(policy.update(true, kNone, 100)));
    assert(nothing(policy.update(false, kNone, 200)));
    assert(nothing(policy.update(false, kNone, 200 + 2 * kDelayMs)));
    assert(policy.recording() && policy.reason() == RecordingReason::manual);
}

void testStopDuringAMatchHolds() {
    // stopped by hand mid-match: not restarted until the next connection
    RecordingPolicy policy(false, true, kDelayMs);
    assert(started(policy.update(true, kNone, 0), RecordingReason::competition));
    assert(stopped(policy.update(true, kStop, 100), RecordingReason::competition));
    assert(nothing(policy.update(true, kNone, 200)));
    assert(nothing(policy.update(false, kNone, 300)));
    assert(nothing(policy.update(false, kNone, 300 + 2 * kDelayMs)));
    assert(started(policy.update(true, kNone, 20000), RecordingReason::competition));
}

void testRestartMidMatchIsManual() {
    // stopped and started again by hand during a match: the new one is the user's, so pulling the
    // tether doesn't end it
    RecordingPolicy policy(false, true, kDelayMs);
    assert(started(policy.update(true, kNone, 0), RecordingReason::competition));
    assert(stopped(policy.update(true, kStop, 100), RecordingReason::competition));
    assert(started(policy.update(true, kStart, 200), RecordingReason::manual));
    assert(nothing(policy.update(false, kNone, 300)));
    assert(nothing(policy.update(false, kNone, 300 + 2 * kDelayMs)));
    assert(policy.recording());
}

void testRequestOnTheConnectingTick() {
    // a stop tapped on the very tick the switch connects: the tap wins
    RecordingPolicy policy(false, true, kDelayMs);
    assert(nothing(policy.update(false, kNone, 0)));
    assert(started(policy.update(false, kStart, 10), RecordingReason::manual));
    assert(stopped(policy.update(true, kStop, 20), RecordingReason::manual));
    assert(!policy.recording());
}

void testClockWrap() {
    // millis() wraps after 49 days; the delay is unsigned arithmetic, so it still works
    RecordingPolicy policy(false, true, kDelayMs);
    const std::uint32_t nearWrap = 0xFFFFFFFFu - 1000;
    assert(started(policy.update(true, kNone, nearWrap), RecordingReason::competition));
    assert(nothing(policy.update(false, kNone, nearWrap + 10)));
    assert(nothing(policy.update(false, kNone, nearWrap + 10 + kDelayMs - 1)));
    assert(stopped(policy.update(false, kNone, nearWrap + 10 + kDelayMs),
                   RecordingReason::competition));
}

void testReasonNames() {
    assert(std::string_view(recordingReasonName(RecordingReason::manual)) == "manual");
    assert(std::string_view(recordingReasonName(RecordingReason::competition)) == "competition");
    assert(std::string_view(recordingReasonName(RecordingReason::startup)) == "startup");
    assert(std::string_view(recordingReasonName(RecordingReason::none)) == "none");
}

} // namespace

int main() {
    testButtonOnlyInThePits();
    testRecordAtStart();
    testMatchRecordsItself();
    testTetherBlipKeepsOneFile();
    testConnectedAtStartup();
    testCompetitionRecordingOff();
    testManualRecordingSurvivesTheSwitch();
    testStopDuringAMatchHolds();
    testRestartMidMatchIsManual();
    testRequestOnTheConnectingTick();
    testClockWrap();
    testReasonNames();
    std::printf("recording_policy_test: all tests passed\n");
    return 0;
}
