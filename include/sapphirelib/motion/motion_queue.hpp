/**
 * \file sapphirelib/motion/motion_queue.hpp
 *
 * Sequential, non-blocking runner for a series of blocking motions.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <deque>
#include <functional>
#include <memory>

#include "pros/rtos.hpp"

namespace sapphirelib::motion {

/// Runs a sequence of blocking motions (driveDistance(), turnToHeading(),
/// moveToPoint(), moveToPose(), followPath(), or any other callable) one
/// after another on a background PROS task, so autonomous() can enqueue a
/// whole routine and keep running (e.g. to also manage a mechanism)
/// instead of blocking on each motion in turn.
///
/// Holds a pros::Mutex internally, which is non-copyable/non-movable, so —
/// like MotorGroup-based classes elsewhere in SapphireLib — MotionQueue can
/// only be constructed in place, never passed by value.
class MotionQueue {
public:
    MotionQueue() = default;

    /// Appends a motion to run after everything already queued finishes.
    /// Safe to call while the queue is running (including from within a
    /// queued motion, to enqueue more work).
    void enqueue(std::function<void()> motion);

    /// Starts processing the queue on a background task. Returns
    /// immediately — call waitUntilDone() to block until every enqueued
    /// motion (including ones enqueued after run() was called) has run.
    /// Call at most once per MotionQueue instance.
    void run();

    /// True while a motion is executing or more are queued behind it.
    bool isBusy() const;

    /// Blocks until the queue is empty and idle.
    void waitUntilDone() const;

    /// Drops every motion that hasn't started running yet. Does not
    /// interrupt a motion already in progress.
    void clear();

private:
    mutable pros::Mutex mutex_;
    std::deque<std::function<void()>> pending_;
    bool busy_ = false;
    std::unique_ptr<pros::Task> task_;
};

} // namespace sapphirelib::motion
