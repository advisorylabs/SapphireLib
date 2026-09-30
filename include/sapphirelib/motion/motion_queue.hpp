#pragma once

#include <deque>
#include <functional>
#include <memory>

#include "pros/rtos.hpp"

namespace sapphirelib::motion {

/**
 * @brief Runs blocking motions one after another on a background task
 *
 * Lets autonomous() queue a routine and keep going, e.g. to also run a mechanism
 *
 * @b Example
 * @code {.cpp}
 * // static, never a local in autonomous(): the task outlives the call
 * static sapphirelib::motion::MotionQueue queue;
 *
 * void autonomous() {
 *     queue.enqueue([] { drivetrain().driveDistance(24); });
 *     queue.enqueue([] { drivetrain().turnToHeading(90); });
 *     queue.run();
 *     // do other things while the chassis moves
 *     queue.waitUntilDone();
 * }
 * @endcode
 */
class MotionQueue {
public:
    MotionQueue() = default;

    /**
     * @brief Add a motion to run after everything already queued
     *
     * Safe while the queue is running, including from inside a queued motion
     *
     * @param motion the motion to run
     */
    void enqueue(std::function<void()> motion);

    /**
     * @brief Start running the queue on a background task. Returns right away
     *
     * Only the first call starts the task. The task runs for the rest of the program, so the
     * MotionQueue must be static
     */
    void run();

    /**
     * @brief Whether a motion is running or queued
     */
    bool isBusy() const;

    /**
     * @brief Block until the queue is empty and idle
     */
    void waitUntilDone() const;

    /**
     * @brief Drop every motion that hasn't started yet. The running motion keeps going
     */
    void clear();

private:
    mutable pros::Mutex mutex_;
    std::deque<std::function<void()>> pending_;
    bool busy_ = false;
    std::unique_ptr<pros::Task> task_;
};

} // namespace sapphirelib::motion
