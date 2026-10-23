/**
 * \file teach.cpp
 *
 * Teach mode: recording a drive in driver control, saving and loading it,
 * and replaying it as the Taught Route auton. See robot/teach.hpp.
 *
 * Team 96671H: Hitmen
 */

#include "robot/teach.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robot/devices.hpp"
#include "robot/driver.hpp"
#include "robot/macros.hpp"
#include "robot/telemetry.hpp"
#include "sapphirelib/input/button_playback.hpp"
#include "sapphirelib/motion/motion_queue.hpp"
#include "sapphirelib/motion/recorded_trajectory.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/telemetry/sd_card.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"

namespace robot::teach {

using sapphirelib::input::Button;
using sapphirelib::input::ButtonPlayback;
using sapphirelib::input::Controller;
using sapphirelib::motion::RecordedSample;
using sapphirelib::motion::ReplayPlan;
using sapphirelib::motion::trajectoryFromRecording;
using sapphirelib::telemetry::cardInserted;
using sapphirelib::telemetry::readCardFile;
using sapphirelib::telemetry::writeCardFile;

namespace {

// Where the drive is kept: beside the logs, or in the card's root without the
// sl folder (the V5 can't make folders). Read back in the same order, like
// TUNE.CFG.
constexpr const char* kPaths[] = {"/usd/sl/TEACH.TXT", "/usd/TEACH.TXT"};

// The longest drive kept, a skills run with room to spare: recording stops
// there on its own.
constexpr std::uint32_t kMaxRecordingMs = 75000;

// A saved drive is about 40 bytes a tick; a file much bigger than the
// longest drive isn't one of ours.
constexpr std::size_t kMaxFileBytes = 64 * (kMaxRecordingMs / kDriverLoopMs + 1) + 256;

enum class State : std::uint8_t {
    empty,     // nothing taught
    recording, // update() is recording
    ready,     // a drive is kept, to save and to replay
};

// The card a chunk at a time, held for each (telemetry::readCardFile(),
// writeCardFile()), so the logger, which holds it for each of its writer's
// passes, never shares it and never waits long. The longest to wait for it
// for one chunk, in milliseconds: the logger's slowest writes take a couple
// hundred.
constexpr std::uint32_t kCardWaitMs = 2000;

// What became of the kept drive on the card. A save only counts once the file
// reads back exactly as written.
enum class Card : std::uint8_t { none, saving, saved, loaded, failed, mismatch };

// replayPercent before anything has worked it out, and when it can't replay
// (the terminal says why).
constexpr int kUnknown = -1;
constexpr int kCantReplay = -2;

std::atomic<State> state{State::empty};
std::atomic<Card> card{Card::none};
std::atomic<bool> toggleRequested{false};
std::atomic<std::uint32_t> keptMs{0};
std::atomic<int> replayPercent{kUnknown};

// The drive. Only update() writes it, and only while recording; the save
// task and the replay only read it while it's ready, and update() won't start
// a new one while the last is still saving. Atomics around it, no mutex:
// PROS deletes the competition tasks on every mode change, and a mutex held
// by one at that moment would stay locked.
std::vector<RecordedSample> drive;
std::uint32_t startMs = 0;
std::unique_ptr<pros::Task> saveTask;

// How fast the kept drive replays with the axis models installed now, for the
// Home page, saying why in the terminal if it can't.
void check() {
    const ReplayPlan plan = trajectoryFromRecording(drive, drivetrain().axisModels());
    if (plan.ok) {
        replayPercent.store(static_cast<int>(std::lround(plan.speedScale * 100.0)));
    } else {
        replayPercent.store(kCantReplay);
        SAPPHIRELIB_LOG_ERROR("teach", "the taught drive can't replay: %s", plan.error.c_str());
    }
}

// The drive to the card, on a task of its own so driving doesn't wait for it,
// then read back: it only counts as saved if the card has exactly what was
// written, so a write the card got wrong shows on the Home page, not at the
// next match's replay.
void save() {
    std::string text(sapphirelib::motion::kRecordingHeader);
    text.reserve(text.size() + 40 * drive.size());
    char row[96];
    for (const RecordedSample& sample : drive) {
        sapphirelib::motion::formatRecordedSample(row, sizeof(row), sample);
        text += row;
    }
    const std::uint64_t startUs = sapphirelib::micros();
    const char* where = nullptr;
    bool verified = false;
    if (cardInserted(kCardWaitMs)) {
        for (const char* path : kPaths) {
            if (!writeCardFile(path, text, kCardWaitMs)) continue;
            where = path;
            std::string back;
            verified = readCardFile(path, back, kMaxFileBytes, kCardWaitMs) && back == text;
            break;
        }
    }
    const double ms = (sapphirelib::micros() - startUs) / 1000.0;
    if (where != nullptr && verified) {
        card.store(Card::saved);
        SAPPHIRELIB_LOG_INFO("teach",
                             "saved %.1fs of driving to %s (%u bytes, read back in %.0fms)",
                             keptMs.load() / 1000.0, where, static_cast<unsigned>(text.size()), ms);
        sapphirelib::telemetry::event("teach", "saved,bytes=%u,ms=%.0f,verified=1",
                                      static_cast<unsigned>(text.size()), ms);
    } else if (where != nullptr) {
        card.store(Card::mismatch);
        SAPPHIRELIB_LOG_ERROR("teach",
                              "%s didn't read back as written: the card got the save wrong", where);
        sapphirelib::telemetry::event("teach", "saved,bytes=%u,ms=%.0f,verified=0",
                                      static_cast<unsigned>(text.size()), ms);
    } else {
        card.store(Card::failed);
        SAPPHIRELIB_LOG_ERROR("teach", "couldn't save the drive: is there an SD card?");
        sapphirelib::telemetry::event("teach", "save_failed");
    }
    check();
}

void begin(Controller& controller) {
    if (card.load() == Card::saving) {
        controller.rumble("...");
        SAPPHIRELIB_LOG_WARN("teach", "still saving the last drive: try again in a moment");
        return;
    }
    drive.clear();
    drive.reserve(kMaxRecordingMs / kDriverLoopMs + 1);
    startMs = controller.now();
    keptMs.store(0);
    replayPercent.store(kUnknown);
    card.store(Card::none);
    state.store(State::recording);
    controller.rumble("-");
    sapphirelib::telemetry::event("teach", "start");
    SAPPHIRELIB_LOG_INFO("teach", "recording");
}

void finish(Controller& controller) {
    const bool kept = drive.size() >= 2;
    state.store(kept ? State::ready : State::empty);
    controller.rumble("..");
    sapphirelib::telemetry::event("teach", "stop,ms=%u,samples=%u",
                                  static_cast<unsigned>(keptMs.load()),
                                  static_cast<unsigned>(drive.size()));
    SAPPHIRELIB_LOG_INFO("teach", "recorded %.1fs", keptMs.load() / 1000.0);
    if (!kept) return;
    card.store(Card::saving);
    saveTask = std::make_unique<pros::Task>([] { save(); }, TASK_PRIORITY_MIN + 1,
                                            TASK_STACK_DEPTH_DEFAULT, "Teach save");
}

void record(const Controller& controller) {
    const std::uint32_t elapsedMs = controller.now() - startMs;
    if (!drive.empty() && elapsedMs <= drive.back().tMs) return;
    // the pose motions drive by, so the replay follows the same frame
    const sapphirelib::odom::Pose pose = odometry().getPose();
    drive.push_back({.tMs = elapsedMs,
                     .xIn = pose.xIn,
                     .yIn = pose.yIn,
                     .headingDeg = pose.headingDeg,
                     .buttons = controller.buttons().heldMask()});
    keptMs.store(elapsedMs);
}

} // namespace

void load() {
    if (!cardInserted(kCardWaitMs)) return;
    std::string text;
    for (const char* path : kPaths) {
        if (!readCardFile(path, text, kMaxFileBytes, kCardWaitMs)) continue;
        std::string error;
        if (text.size() > kMaxFileBytes) {
            SAPPHIRELIB_LOG_ERROR("teach", "%s is too big to be a taught drive", path);
            return;
        }
        if (!sapphirelib::motion::parseRecording(text, drive, error) || drive.size() < 2) {
            drive.clear();
            SAPPHIRELIB_LOG_ERROR("teach", "%s: %s", path,
                                  error.empty() ? "too short to replay" : error.c_str());
            return;
        }
        keptMs.store(drive.back().tMs);
        card.store(Card::loaded);
        state.store(State::ready);
        SAPPHIRELIB_LOG_INFO("teach", "loaded %.1fs of driving from %s", keptMs.load() / 1000.0,
                             path);
        check();
        return;
    }
}

void update(Controller& controller) {
    // a restart (autonomous, a disable, a routine run from here) ends the drive
    if (state.load() == State::recording && controller.resumed()) finish(controller);

    const bool combo = controller.combo(Button::x, Button::a);
    if (toggleRequested.exchange(false) || combo) {
        if (pros::competition::is_connected()) {
            // only off the field, like X+UP: a match isn't the place to teach
            if (combo) controller.rumble("...");
        } else if (state.load() == State::recording) {
            finish(controller);
        } else {
            begin(controller);
        }
    }

    if (state.load() != State::recording) return;
    if (controller.now() - startMs >= kMaxRecordingMs) {
        finish(controller);
        return;
    }
    record(controller);
}

bool recording() { return state.load() == State::recording; }

void requestToggle() { toggleRequested.store(true); }

std::string statusText() {
    char text[64];
    const double seconds = keptMs.load() / 1000.0;
    switch (state.load()) {
        case State::recording:
            std::snprintf(text, sizeof(text), "Teach: RECORDING %.0fs (X+A stops)", seconds);
            return text;
        case State::empty: return "Teach: none yet (X+A in driver control)";
        case State::ready: break;
    }
    const char* where = "";
    switch (card.load()) {
        case Card::saving: where = ", saving"; break;
        case Card::saved: where = ", saved"; break;
        case Card::loaded: where = " from the card"; break;
        case Card::failed: where = ", NOT SAVED"; break;
        case Card::mismatch: where = ", SAVE BAD"; break;
        case Card::none: break;
    }
    const int percent = replayPercent.load();
    if (percent == kCantReplay) {
        std::snprintf(text, sizeof(text), "Teach: %.1fs%s, can't replay", seconds, where);
    } else if (percent >= 0) {
        std::snprintf(text, sizeof(text), "Teach: %.1fs%s, replays at %d%%", seconds, where,
                      percent);
    } else {
        std::snprintf(text, sizeof(text), "Teach: %.1fs%s", seconds, where);
    }
    return text;
}

void replayAuton() {
    if (state.load() != State::ready) {
        SAPPHIRELIB_LOG_ERROR("teach", "%s",
                              state.load() == State::recording
                                  ? "still recording: stop teaching (X+A) first"
                                  : "nothing taught yet: X+A in driver control records a drive");
        return;
    }
    // static: the queue's task reads the plan to its end, even past this call
    static ReplayPlan plan;
    const std::uint64_t startUs = sapphirelib::micros();
    plan = trajectoryFromRecording(drive, drivetrain().axisModels());
    if (!plan.ok) {
        SAPPHIRELIB_LOG_ERROR("teach", "can't replay the taught drive: %s", plan.error.c_str());
        return;
    }
    SAPPHIRELIB_LOG_INFO("teach",
                         "replaying %.1fs at %.0f%% of the driver's speed (planned in %.1fms)",
                         plan.trajectory.durationS(), plan.speedScale * 100.0,
                         (sapphirelib::micros() - startUs) / 1000.0);
    sapphirelib::telemetry::event("teach", "replay,ms=%u,scale=%.3f",
                                  static_cast<unsigned>(plan.trajectory.durationS() * 1000.0),
                                  plan.speedScale);

    // where the drive started: put the robot back on that spot
    odometry().setPose(plan.start);
    static sapphirelib::motion::MotionQueue queue;
    // until the replay's motion starts, the progress is the last trajectory's
    const std::uint32_t motion = drivetrain().trajectoryProgress().motion + 1;
    queue.enqueue([] { drivetrain().followTrajectory(plan.trajectory); });
    queue.run();

    // The driver's buttons into the driver's own macros, on the plan's clock:
    // the mechanisms do what the driver did where they did it, waiting with
    // the plan if the robot is held up. A Controller of its own, so the real
    // one's button history is untouched.
    static Controller playback(pros::E_CONTROLLER_MASTER,
                               {.resumeGapMs = 100, .screenIntervalMs = 60});
    ButtonPlayback buttons(plan.buttons);
    while (queue.isBusy()) {
        const auto progress = drivetrain().trajectoryProgress();
        playback.replay(buttons.advance(progress.motion == motion ? progress.timeS : 0.0));
        macros::update(playback);
        pros::delay(kDriverLoopMs);
    }
    macros::stop();
}

namespace {

// The SD Stress Test: rounds of the biggest drive teach mode saves, each
// written and read back, while the logger records. A file of its own, so the
// taught drive is left alone.
constexpr const char* kStressPaths[] = {"/usd/sl/SDTEST.TXT", "/usd/SDTEST.TXT"};
constexpr int kStressRounds = 10;

std::atomic<bool> stressRunning{false};

void stressTest() {
    sapphirelib::telemetry::Logger& log = logger();
    const bool wasRecording = log.recording();
    if (!wasRecording) {
        log.startRecording();
        pros::delay(1500); // the writer opens a file and settles into its passes
    }
    const sapphirelib::telemetry::LoggerStatus before = log.status();

    int verified = 0;
    double slowestMs = 0.0;
    double totalMs = 0.0;
    std::size_t bytes = 0;
    const char* where = "none";
    for (int round = 0; round < kStressRounds; ++round) {
        // a full-length drive's rows, different every round so a stale file can't pass
        std::string text(sapphirelib::motion::kRecordingHeader);
        char row[96];
        std::snprintf(row, sizeof(row), "# SD stress test, round %d\n", round);
        text += row;
        for (std::uint32_t tMs = 0; tMs <= kMaxRecordingMs; tMs += kDriverLoopMs) {
            const double t = tMs / 1000.0 + round;
            sapphirelib::motion::formatRecordedSample(
                row, sizeof(row),
                {.tMs = tMs,
                 .xIn = 60.0 * std::sin(0.3 * t),
                 .yIn = 60.0 * std::cos(0.2 * t),
                 .headingDeg = std::fmod(40.0 * t, 360.0),
                 .buttons = static_cast<std::uint16_t>((tMs / 100 + round) & 0xFFF)});
            text += row;
        }
        bytes = text.size();

        const std::uint64_t startUs = sapphirelib::micros();
        bool ok = false;
        for (const char* path : kStressPaths) {
            if (!writeCardFile(path, text, kCardWaitMs)) continue;
            std::string back;
            ok = readCardFile(path, back, text.size() + 1, kCardWaitMs) && back == text;
            where = path;
            break;
        }
        const double ms = (sapphirelib::micros() - startUs) / 1000.0;
        totalMs += ms;
        slowestMs = std::max(slowestMs, ms);
        if (ok) ++verified;
        SAPPHIRELIB_LOG_INFO("sdtest", "round %d: %s in %.0fms", round,
                             ok ? "read back as written" : "FAILED", ms);
    }

    const sapphirelib::telemetry::LoggerStatus after = log.status();
    const std::uint32_t dropped = after.droppedRows - before.droppedRows;
    const bool loggerFine = after.state == sapphirelib::telemetry::LoggerState::logging &&
                            after.fileIndex == before.fileIndex;
    SAPPHIRELIB_LOG_INFO("sdtest",
                         "%d/%d rounds of %u bytes read back as written (%s), %.0fms on average, "
                         "slowest %.0fms; the logger %s, %u rows dropped meanwhile, in SL%06ld.CSV",
                         verified, kStressRounds, static_cast<unsigned>(bytes), where,
                         totalMs / kStressRounds, slowestMs,
                         loggerFine ? "kept logging to the same file" : "FAULTED or reopened",
                         static_cast<unsigned>(dropped), static_cast<long>(after.fileIndex));
    sapphirelib::telemetry::event("sdtest",
                                  "rounds=%d,verified=%d,bytes=%u,avg_ms=%.0f,max_ms=%.0f,"
                                  "dropped=%u,logger=%s",
                                  kStressRounds, verified, static_cast<unsigned>(bytes),
                                  totalMs / kStressRounds, slowestMs,
                                  static_cast<unsigned>(dropped), loggerFine ? "ok" : "faulted");
    if (!wasRecording) {
        pros::delay(500); // so the summary event lands in this file
        log.stopRecording();
    }
}

} // namespace

void sdStressAuton() {
    if (stressRunning.exchange(true)) return;
    // on a task of its own, which a mode change never deletes holding the card
    static std::unique_ptr<pros::Task> task;
    task = std::make_unique<pros::Task>(
        [] {
            stressTest();
            stressRunning.store(false);
        },
        TASK_PRIORITY_MIN + 1, TASK_STACK_DEPTH_DEFAULT, "SD stress");
    while (stressRunning.load()) pros::delay(50);
}

} // namespace robot::teach
