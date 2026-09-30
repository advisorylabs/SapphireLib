#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>

#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib {

/**
 * @brief One step of a SequenceProgram
 *
 * A step ends on the first update() where until() is true or timeoutMs has passed, whichever comes
 * first (until is checked first). A fixed-length step is just a timeout with no until. A step with
 * neither only ends when the sequence is cancelled or restarted
 */
template <typename StepId> struct SequenceStep {
    /** the id this step is known by, usually a value of your own enum */
    StepId id;

    /** ends the step early when it returns true. Must not start or cancel the sequence */
    std::function<bool()> until = nullptr;

    /** longest time the step can last, in milliseconds. 0 for no limit */
    std::uint32_t timeoutMs = 0;

    /** runs once when the step is entered. May start or cancel the sequence, to branch or abort */
    std::function<void()> onEnter = nullptr;
};

/**
 * @brief An ordered list of steps, and what to do after the last one
 *
 * @note the steps are referenced, not copied, so declare them and the program at namespace scope
 *
 * @b Example
 * @code {.cpp}
 * const std::array<Sequence<Phase>::Step, 2> kScoreSteps{{
 *     {.id = Phase::descend, .until = liftArrived, .timeoutMs = 800},
 *     {.id = Phase::outtake, .timeoutMs = 500},
 * }};
 * const Sequence<Phase>::Program kScore{.steps = kScoreSteps};
 * @endcode
 */
template <typename StepId> struct SequenceProgram {
    /** the steps, in order */
    std::span<const SequenceStep<StepId>> steps;

    /**
     * runs after the last step ends, once the sequence is idle, so it can start the next program.
     * Not run on cancel()
     */
    std::function<void()> onFinish = nullptr;
};

/**
 * @brief What one Sequence::update() did
 */
enum class SequenceUpdate : std::uint8_t {
    idle,     // nothing was running
    running,  // the current step continues
    advanced, // the current step ended and the next one began
    finished, // the last step ended, onFinish ran, and the sequence is idle
};

/**
 * @brief Non-blocking timed sequence, for "dip the lift, outtake for 500ms, then go up a level"
 * from driver control without blocking it
 *
 * Runs one SequenceProgram at a time, one step per tick at most: a step entered on this update()
 * gets its first chance to end on the next one. The same programs can run to completion from
 * autonomous with runBlocking()
 *
 * @note not thread-safe, so update it from one task
 *
 * @b Example
 * @code {.cpp}
 * enum class Phase { descend, outtake };
 * sapphirelib::Sequence<Phase> sequence;
 *
 * void opcontrol() {
 *     while (true) {
 *         const std::uint32_t now = sapphirelib::millis();
 *         if (master.get_digital_new_press(DIGITAL_R2)) sequence.start(kScore, now);
 *         sequence.update(now);
 *         // outputs stay level-triggered in your code
 *         intake.move_voltage(sequence.is(Phase::outtake) ? -12000 : 0);
 *         pros::delay(20);
 *     }
 * }
 * @endcode
 */
template <typename StepId> class Sequence {
public:
    using Step = SequenceStep<StepId>;
    using Program = SequenceProgram<StepId>;

    /**
     * @brief Start a program at its first step, replacing whatever was running
     *
     * The replaced program's onFinish doesn't run. An empty program finishes right away
     *
     * @param program the program to run
     * @param nowMs the current time, in milliseconds
     */
    void start(const Program& program, std::uint32_t nowMs) {
        program_ = &program;
        lastExitWasTimeout_ = false;
        if (program.steps.empty()) {
            finish();
            return;
        }
        enter(0, nowMs);
    }

    // programs are referenced, not copied, so a temporary would dangle
    void start(const Program&& program, std::uint32_t nowMs) = delete;

    /**
     * @brief Stop the sequence without running onFinish
     */
    void cancel() { program_ = nullptr; }

    /**
     * @brief Check the current step, and move on if it's done. Call this once per tick
     *
     * @param nowMs the current time, in milliseconds
     * @return SequenceUpdate what happened this tick
     */
    SequenceUpdate update(std::uint32_t nowMs) {
        if (program_ == nullptr) return SequenceUpdate::idle;
        const Step& step = program_->steps[index_];
        // check until first, so a condition that comes true on the same tick as the timeout
        // counts as arriving
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

    /**
     * @brief Whether a program is running
     */
    bool active() const { return program_ != nullptr; }

    /**
     * @brief Whether this exact program is the one running
     */
    bool running(const Program& program) const { return program_ == &program; }

    /**
     * @brief Whether the sequence is running and in step `id`
     */
    bool is(StepId id) const { return program_ != nullptr && program_->steps[index_].id == id; }

    /**
     * @brief Get the current step's id
     *
     * @return std::optional<StepId> the id, or nullopt while idle
     */
    std::optional<StepId> current() const {
        if (program_ == nullptr) return std::nullopt;
        return program_->steps[index_].id;
    }

    /**
     * @brief Get the index of the current step in its program. Only meaningful while active()
     */
    std::size_t stepIndex() const { return index_; }

    /**
     * @brief Get the time spent in the current step
     *
     * @param nowMs the current time, in milliseconds
     * @return std::uint32_t time in the step, in milliseconds. 0 while idle
     */
    std::uint32_t msInStep(std::uint32_t nowMs) const {
        return program_ != nullptr ? sapphirelib::elapsedMs(stepStartMs_, nowMs) : 0u;
    }

    /**
     * @brief Whether the last step to end timed out, rather than meeting its until condition
     *
     * Always true for fixed-length steps. Useful for "did the lift get there, or did we give up"
     */
    bool lastExitWasTimeout() const { return lastExitWasTimeout_; }

    /**
     * @brief Run a program to completion. For autonomous only
     *
     * Anything the until conditions wait on must keep running meanwhile, e.g. a PositionMechanism
     * on its own task
     *
     * @param program the program to run
     * @param timeoutMs longest time to wait, in milliseconds. 0 for no limit
     * @param pollMs how often to update the sequence, in milliseconds. 10 by default
     * @return true the program finished
     * @return false it timed out, or a callback cancelled it or started a different program
     *
     * @b Example
     * @code {.cpp}
     * void autonomous() {
     *     // score, giving up after 3 seconds
     *     sequence.runBlocking(kScore, 3000);
     * }
     * @endcode
     */
    bool runBlocking(const Program& program, std::uint32_t timeoutMs = 0,
                     std::uint32_t pollMs = 10) {
        // a 0ms delay only yields to tasks of the same priority, which would starve the GUI and
        // the telemetry writer, so poll at least every 1ms
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
        // set the index and start time first, so onEnter sees the step it's entering
        index_ = index;
        stepStartMs_ = nowMs;
        const Step& step = program_->steps[index];
        if (step.onEnter) step.onEnter();
    }

    void finish() {
        const Program* finished = program_;
        program_ = nullptr; // idle before onFinish, so it can start a new program
        if (finished->onFinish) finished->onFinish();
    }

    const Program* program_ = nullptr;
    std::size_t index_ = 0;
    std::uint32_t stepStartMs_ = 0;
    bool lastExitWasTimeout_ = false;
};

} // namespace sapphirelib
