/**
 * \file sapphirelib/util/sequence.hpp
 *
 * Non-blocking timed sequences for per-tick code. For example: "dip the
 * lift, outtake for 500ms, then go up a level", advanced one step at a time
 * from a driver-control loop instead of blocking it. The same programs can
 * also run to completion from autonomous with runBlocking().
 *
 * Steps are named by your own enum, and outputs usually stay level-triggered
 * in your code ("claw outtakes while sequence.is(Phase::scoreOuttake)"), so a
 * Sequence only answers "which step are we in" and decides when to move on.
 * Pure (runBlocking() goes through util/clock.hpp); see
 * tests/util/sequence_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>

#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib {

/// One step of a SequenceProgram.
///
/// A step exits on the first update() where `until()` is true, or where
/// `timeoutMs` has passed since the step was entered (0 = no limit),
/// whichever comes first. `until` is checked first. A fixed-duration step
/// is just a timeout with no `until`. A step with neither never exits on its
/// own (only cancel() or start() ends it).
template <typename StepId> struct SequenceStep {
    StepId id;

    /// Early exit condition. Must not start or cancel the sequence itself.
    std::function<bool()> until = nullptr;

    std::uint32_t timeoutMs = 0;

    /// Runs once, on the start()/update() that enters this step. May start or
    /// cancel the sequence (to branch or abort).
    std::function<void()> onEnter = nullptr;
};

/// An ordered list of steps plus what to do after the last one. The steps
/// are referenced, not copied, so declare them (and the program) at
/// namespace scope, e.g.:
///
///   const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{...}};
///   const Sequence<Phase>::Program kScore{.steps = kScoreSteps, .onFinish = ...};
template <typename StepId> struct SequenceProgram {
    std::span<const SequenceStep<StepId>> steps;

    /// Runs on the update() where the last step exits, after the sequence has
    /// gone idle, so it may start the next program. Not run on cancel().
    std::function<void()> onFinish = nullptr;
};

/// What one Sequence::update() did.
enum class SequenceUpdate : std::uint8_t {
    idle,     ///< nothing was running
    running,  ///< the current step continues
    advanced, ///< the current step exited and the next one began
    finished, ///< the last step exited, onFinish ran, and the sequence is idle
};

/// Runs at most one SequenceProgram at a time. Starting another replaces it.
///
/// update() makes **at most one transition per call**: a step entered on
/// this call gets its first chance to exit on the next one, and its timeout
/// counts from the `nowMs` it was entered with. Not thread-safe; drive it
/// from one task.
template <typename StepId> class Sequence {
public:
    using Step = SequenceStep<StepId>;
    using Program = SequenceProgram<StepId>;

    /// Starts `program` at its first step (running that step's onEnter),
    /// replacing whatever was running. The replaced program's onFinish does
    /// not run. An empty program finishes at once: onFinish runs and the
    /// sequence stays idle.
    void start(const Program& program, std::uint32_t nowMs) {
        program_ = &program;
        lastExitWasTimeout_ = false;
        if (program.steps.empty()) {
            finish();
            return;
        }
        enter(0, nowMs);
    }

    /// Programs are referenced, not copied, so a temporary would dangle.
    void start(const Program&& program, std::uint32_t nowMs) = delete;

    /// Stops without running onFinish.
    void cancel() { program_ = nullptr; }

    /// Checks the current step's exit and moves on if it's met. Call once per
    /// tick while active(); calling it while idle is harmless.
    SequenceUpdate update(std::uint32_t nowMs) {
        if (program_ == nullptr) return SequenceUpdate::idle;
        const Step& step = program_->steps[index_];
        // Two independent checks, `until` first — so a step whose condition
        // comes true on the same tick its timeout runs out counts as having
        // arrived, and there's no if/else ordering for one to hide the other.
        const bool conditionMet = step.until && step.until();
        const bool timedOut = !conditionMet && step.timeoutMs > 0 &&
                              sapphirelib::elapsedMs(stepStartMs_, nowMs) >= step.timeoutMs;
        if (!conditionMet && !timedOut) return SequenceUpdate::running;
        lastExitWasTimeout_ = timedOut;
        if (index_ + 1 < program_->steps.size()) {
            enter(index_ + 1, nowMs);
            return SequenceUpdate::advanced;
        }
        finish();
        return SequenceUpdate::finished;
    }

    bool active() const { return program_ != nullptr; }

    /// Whether `program` (this exact object) is the one running.
    bool running(const Program& program) const { return program_ == &program; }

    /// Active and in step `id`.
    bool is(StepId id) const { return program_ != nullptr && program_->steps[index_].id == id; }

    std::optional<StepId> current() const {
        if (program_ == nullptr) return std::nullopt;
        return program_->steps[index_].id;
    }

    /// Index of the current step in its program (meaningful only while active()).
    std::size_t stepIndex() const { return index_; }

    /// Time in the current step; 0 while idle.
    std::uint32_t msInStep(std::uint32_t nowMs) const {
        return program_ != nullptr ? sapphirelib::elapsedMs(stepStartMs_, nowMs) : 0u;
    }

    /// Whether the most recent step exit was its timeout rather than `until`.
    /// This is always true for fixed-duration steps. For telemetry, and for
    /// "did the lift actually arrive or did we give up".
    bool lastExitWasTimeout() const { return lastExitWasTimeout_; }

    /// Autonomous only: starts `program` and updates it every `pollMs` until it
    /// finishes. Gives up (cancels, returns false) after `timeoutMs` (0 = no
    /// limit). Also returns false if a callback cancels it or starts a
    /// different program. Anything its `until` conditions wait on must keep
    /// running meanwhile, e.g. a PositionMechanism on its own task. A `pollMs`
    /// of 0 polls every 1ms, as waitUntil() does.
    bool runBlocking(const Program& program, std::uint32_t timeoutMs = 0,
                     std::uint32_t pollMs = 10) {
        // A 0ms delay only yields to tasks of the same priority, so polling
        // with it would starve every lower-priority task (the GUI, telemetry's
        // writer) for the whole run.
        const std::uint32_t delayPerPollMs = pollMs > 0 ? pollMs : 1;
        const std::uint32_t startMs = millis();
        start(program, startMs);
        if (program.steps.empty()) return true;
        while (running(program)) {
            delayMs(delayPerPollMs);
            const std::uint32_t nowMs = millis();
            if (update(nowMs) == SequenceUpdate::finished) return true;
            if (timeoutMs > 0 && running(program) &&
                sapphirelib::elapsedMs(startMs, nowMs) >= timeoutMs) {
                cancel();
                return false;
            }
        }
        return false;
    }

    bool runBlocking(const Program&& program, std::uint32_t timeoutMs = 0,
                     std::uint32_t pollMs = 10) = delete;

private:
    void enter(std::size_t index, std::uint32_t nowMs) {
        // Index and start time first, so an onEnter that asks which step it's
        // in (or starts something else) sees the step it's entering.
        index_ = index;
        stepStartMs_ = nowMs;
        const Step& step = program_->steps[index];
        if (step.onEnter) step.onEnter();
    }

    void finish() {
        const Program* finished = program_;
        program_ = nullptr; // idle before onFinish, so it can chain a new start()
        if (finished->onFinish) finished->onFinish();
    }

    const Program* program_ = nullptr;
    std::size_t index_ = 0;
    std::uint32_t stepStartMs_ = 0;
    bool lastExitWasTimeout_ = false;
};

} // namespace sapphirelib
