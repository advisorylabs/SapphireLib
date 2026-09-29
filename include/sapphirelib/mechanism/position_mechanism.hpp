/**
 * \file sapphirelib/mechanism/position_mechanism.hpp
 *
 * A closed-loop position mechanism: a lift, an arm, or anything else a motor
 * group drives to a position read off a sensor. It provides PID plus gravity
 * feedforward, optional seat-and-rest at a hard stop, a sensor-loss
 * fallback, manual override, and settle detection for autonomous. Run it
 * from your own per-tick function with update(), or give it a background
 * task with startTask() so it keeps holding while blocking autonomous
 * motions run. The control law itself is pure and host-tested (see
 * position_control.hpp).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>

#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/mechanism/position_control.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::mechanism {

/// Reads the mechanism's position, in the same units as its targets. NaN
/// means "no reading right now" (unplugged sensor) and triggers the
/// sensor-loss fallback. Called from whichever task runs update(), and from
/// position() on any task, so it must be safe from both (PROS device reads
/// are). To fall back to a second sensor instead of braking, return that
/// sensor's reading when the first gives NaN; no library change is needed.
using PositionSource = std::function<double()>;

/// A rotation sensor's position in degrees, or NaN when it isn't answering
/// (PROS_ERR). This is the usual PositionSource body:
///   PositionMechanism lift({-20, 19}, [] { return readRotationDeg(liftSensor); }, {...});
double readRotationDeg(const pros::Rotation& sensor);

/// How the mechanism is being driven.
enum class PositionMode : std::uint8_t {
    position, ///< closed loop on target(): setTarget(), holdPosition()
    voltage,  ///< open loop: setVolts()
    off,      ///< brake every update: stop(), and the state before any command
};

/// What one update() saw and did: for a screen readout, a telemetry row (a
/// superset of tuning::CharacterizationSample's time/volts/position), or a
/// test.
struct PositionStep {
    std::uint32_t timeMs = 0; ///< the nowMs update() was given
    PositionLaw law = PositionLaw::off;
    PositionMode mode = PositionMode::off;
    double target = 0.0;   ///< the last target set (whatever the mode)
    double position = 0.0; ///< NaN if the sensor didn't answer
    double volts = 0.0;    ///< what the motors were sent; 0 when braking, NaN under
                           ///< external control (whoever has them knows)
    bool atTarget = false; ///< position mode and within tolerance
    bool settled = false;  ///< see PositionMechanism::settled()
};

/// Two ways to run it:
///
/// - **Manual:** call update(now) once per tick from your own loop, as the
///   robot's driver macros do from opcontrol(). Set pid.nominalDtS to that
///   tick. Nothing drives the mechanism while your loop isn't running, and
///   the gap reset handles the restart.
/// - **Task:** startTask(periodMs) once from initialize(). Then every
///   setTarget() from autonomous or opcontrol is picked up by the
///   mechanism's own task, which keeps holding through blocking drivetrain
///   motions. update() is then ignored (with a one-time warning), since two
///   loops commanding one motor group fight.
///
/// Thread safety: commands (setTarget/setVolts/holdPosition/stop/
/// resetController/setGravity/beginExternalControl/endExternalControl) and
/// queries (target/mode/position/isNear/atTarget/settled/law/gravity/
/// externalControl) may be called from any task. They go through lock-free
/// atomics, never a mutex, so a competition task that PROS deletes mid-call
/// can't leave a lock held. Command from one task at a time. update(),
/// startTask(), setStepListener() and pid() changes belong to one owner.
///
/// Holds a pros::MotorGroup and (after startTask) a task capturing `this`,
/// so, like Odometry, construct it in place at namespace scope or as a
/// static, and never destroy it once its task is running. The constructor
/// only builds the PROS motor group (no device commands), so namespace scope
/// is safe. Set the brake mode (used on sensor loss and by stop()) yourself,
/// with motors().set_brake_mode_all() in initialize().
class PositionMechanism {
public:
    PositionMechanism(std::initializer_list<std::int8_t> motorPorts, PositionSource position,
                      PositionConfig config);

    PositionMechanism(const PositionMechanism&) = delete;
    PositionMechanism& operator=(const PositionMechanism&) = delete;

    // --- Commands (any task) ---

    /// Closed loop on `target`. Setting the same target again (every tick, say)
    /// doesn't restart settle timing; a new value does. A NaN or infinite
    /// target is ignored, since there's no way to drive toward it.
    void setTarget(double target);

    /// Manual override: open-loop volts (clamped to ±maxVolts), loop reset
    /// every update, until the next setTarget()/holdPosition()/stop(). NaN
    /// counts as 0V.
    void setVolts(double volts);

    /// Closed loop on wherever it is right now, for releasing a manual
    /// override without a jump. With no reading it stop()s instead, since
    /// there's nothing to hold against.
    void holdPosition();

    /// brake() every update (with the brake mode set on motors()).
    void stop();

    /// Clears the PID's memory before the next update. For a loop that
    /// restarted after its timing went stale (the gap reset also does this on
    /// its own; this is the explicit version).
    void resetController();

    double target() const;
    PositionMode mode() const;

    /// Replaces the gravity feedforward — e.g. with what Auto-Tune just
    /// measured (tuning::MechanismModel::gravityFeedforward()) — from any
    /// task, taking effect at the next update. Its four fields are separate
    /// atomics, so an update racing this call can mix old and new values for
    /// that one update.
    void setGravity(GravityFeedforward gravity);

    /// The gravity feedforward in effect: the config's until setGravity().
    GravityFeedforward gravity() const;

    // --- Lending the motors out (any task) ---

    /// Hands the motors to something else until endExternalControl() — a
    /// tuning run driving them directly (tuning::runMechanismCharacterization()
    /// through its start/finish hooks). Meanwhile update() and the task still
    /// read the sensor and report every step (law external, volts NaN), but
    /// never command the motors, and keep the PID reset. Commands are still
    /// accepted, and take effect once the motors come back, so a driver loop
    /// that sets a target every tick needn't know any of this is happening.
    /// Not counted: one end undoes any number of begins.
    void beginExternalControl();

    /// Takes the motors back. The next update starts the loop fresh, on
    /// whatever target (or mode) is current by then.
    void endExternalControl();

    /// True between beginExternalControl() and endExternalControl().
    bool externalControl() const;

    // --- Queries (any task) ---

    /// A fresh reading from the source (NaN if it isn't answering). Not cached.
    double position() const;

    /// Whether a fresh reading is within tolerance of `target`. False when
    /// there's no reading.
    bool isNear(double target) const;

    /// isNear(target()), in position mode only.
    bool atTarget() const;

    /// In position mode, within tolerance on update() calls for at least
    /// settleTimeMs, all since the target last changed. Never true for a
    /// target that no update() has worked on yet, so waiting on it right
    /// after setTarget() can't return early on the old target's result.
    bool settled() const;

    /// The law the latest update applied.
    PositionLaw law() const;

    // --- Running it ---

    /// Manual mode's per-tick call: reads the position, commands the motors,
    /// and returns what it did. `nowMs` is your tick's one "now". Ignored
    /// (returns a default step) once startTask() has run.
    PositionStep update(std::uint32_t nowMs);

    /// Starts a background task (default priority) running update every
    /// `periodMs` (at least 1) with pros::Task::delay_until. Warns if that
    /// doesn't match pid.nominalDtS. While the competition state is
    /// disabled, the task keeps running (so settle state and the step
    /// listener stay live) but brakes with the PID cleared (law off), so
    /// nothing winds up while VEXos ignores the motors, and the loop starts
    /// fresh at enable. False if already started. Call it from initialize(),
    /// after setStepListener().
    bool startTask(std::uint32_t periodMs = 10);

    bool taskRunning() const;

    /// Autonomous: blocks until settled() or `timeoutMs` (0 = no limit),
    /// polling every `pollMs` (0 = pid.nominalDtS). False on timeout.
    ///
    /// Without a task, it runs update() itself on each poll, so it works in
    /// manual mode too — from the task that owns update(), with nothing else
    /// calling it meanwhile. But then nothing drives the mechanism once it
    /// returns: the motors keep the last voltage sent until the next
    /// update(). To hold through the motions that follow, use startTask().
    ///
    /// Blocks, so never from a per-tick driver-control function.
    bool waitUntilSettled(std::uint32_t timeoutMs, std::uint32_t pollMs = 0);

    /// setTarget(target) then waitUntilSettled(timeoutMs).
    bool moveTo(double target, std::uint32_t timeoutMs);

    // --- Plumbing ---

    /// The motors: for brake mode (set it in initialize(); sensor loss and
    /// stop() use it), per-motor current, temperature, and so on.
    pros::MotorGroup& motors();

    /// The loop's PID. Hand it to PidTunerPage::addController() or to the
    /// telemetry logger's PID tracking. Its state belongs to whichever task
    /// runs update().
    PID& pid();

    /// The config as constructed — except gravity, which setGravity() can
    /// change afterwards; read gravity() for the live one.
    const PositionConfig& config() const;

    /// Called at the end of every update, on the task that ran it: the
    /// mechanism's own task after startTask(). Must be quick and non-blocking
    /// (a telemetry Channel::record() is). Set it before startTask().
    void setStepListener(std::function<void(const PositionStep&)> listener);

private:
    /// One update. `disabled` is the task's competition-disabled check;
    /// manual update() never passes it.
    PositionStep step(std::uint32_t nowMs, bool disabled);
    void bumpCommand();
    std::uint32_t nominalPeriodMs() const;

    pros::MotorGroup motors_;
    PositionSource source_;
    PositionConfig config_;
    PID pid_;

    // Shared with other tasks: lock-free atomics only (see the class comment).
    std::atomic<PositionMode> mode_{PositionMode::off};
    std::atomic<double> target_{0.0};
    std::atomic<double> volts_{0.0};
    /// Which command settle state belongs to: 31 bits, bumped whenever the
    /// target or mode changes.
    std::atomic<std::uint32_t> commandGen_{0};
    /// (gen << 1) | settled, written by step() in one store so settled() can
    /// never pair one command's flag with another command's generation.
    std::atomic<std::uint32_t> settledState_{0};
    std::atomic<PositionLaw> law_{PositionLaw::off};
    std::atomic<bool> resetRequested_{false};
    std::atomic<bool> taskRunning_{false};
    std::atomic<bool> external_{false};
    // The live gravity feedforward (see setGravity()).
    std::atomic<double> gravityConstantVolts_;
    std::atomic<double> gravityCosineVolts_;
    std::atomic<double> gravityHorizontalPosition_;
    std::atomic<double> gravityArmDegreesPerUnit_;

    // Owned by whichever task runs step().
    GapDetector gap_;
    /// Within tolerance, and since when — restarted for each new command.
    TimedFlag near_;
    std::uint32_t nearGen_ = 0;
    std::function<void(const PositionStep&)> listener_;
    std::unique_ptr<pros::Task> task_;
    bool warnedUpdateWithTask_ = false;
};

} // namespace sapphirelib::mechanism
