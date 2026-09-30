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

/**
 * @brief Reads a mechanism's position, in the same units as its targets
 *
 * Return NaN when there's no reading (an unplugged sensor) to trigger the sensor loss fallback. To
 * fall back to a second sensor instead, return its reading when the first gives NaN. Called from
 * several tasks, so it must be safe from any (PROS device reads are)
 */
using PositionSource = std::function<double()>;

/**
 * @brief Read a rotation sensor in degrees, or NaN when it isn't answering
 *
 * @param sensor the rotation sensor
 * @return double position, in degrees
 *
 * @b Example
 * @code {.cpp}
 * pros::Rotation liftSensor(5);
 * sapphirelib::mechanism::PositionMechanism lift(
 *     {-20, 19}, [] { return sapphirelib::mechanism::readRotationDeg(liftSensor); }, liftConfig);
 * @endcode
 */
double readRotationDeg(const pros::Rotation& sensor);

/**
 * @brief How a mechanism is being driven
 */
enum class PositionMode : std::uint8_t {
    position, // closed loop on target(): setTarget(), holdPosition()
    voltage,  // open loop: setVolts()
    off,      // brake every update: stop(), and before any command
};

/**
 * @brief What one update() saw and did, for a screen, telemetry, or a test
 */
struct PositionStep {
    std::uint32_t timeMs = 0; // the time update() was given
    PositionLaw law = PositionLaw::off;
    PositionMode mode = PositionMode::off;
    double target = 0.0;   // the last target set, whatever the mode
    double position = 0.0; // NaN if the sensor didn't answer
    double volts = 0.0;    // what the motors were sent. 0 when braking, NaN under
                           // external control
    bool atTarget = false; // in position mode and within tolerance
    bool settled = false;  // see PositionMechanism::settled()
};

/**
 * @brief A lift, arm, or anything else a motor group drives to a position read off a sensor
 *
 * PID plus gravity feedforward, optional seat and rest at a hard stop, a sensor loss fallback,
 * manual override, and settle detection. Run it one of two ways:
 *   - call update() once per tick from your own loop. Nothing drives it while your loop isn't
 *     running
 *   - call startTask() once from initialize(), and its own task keeps holding through blocking
 *     drivetrain motions. update() is then ignored
 *
 * Commands and queries are safe from any task (lock-free atomics, never a mutex), but command from
 * one task at a time. update(), startTask(), setStepListener(), and pid() changes belong to one
 * owner
 *
 * @note construct it at namespace scope or as a static, and never destroy it once its task is
 * running. Set the brake mode yourself in initialize(), with motors().set_brake_mode_all()
 *
 * @b Example
 * @code {.cpp}
 * pros::Rotation liftSensor(5);
 * sapphirelib::mechanism::PositionMechanism lift(
 *     {-20, 19}, [] { return sapphirelib::mechanism::readRotationDeg(liftSensor); }, liftConfig);
 *
 * void initialize() {
 *     lift.motors().set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
 *     lift.startTask(10);
 * }
 *
 * void autonomous() {
 *     lift.moveTo(450, 1500); // raise the lift, giving up after 1.5 seconds
 * }
 * @endcode
 */
class PositionMechanism {
public:
    /**
     * @brief Construct a new PositionMechanism. Makes no device calls
     *
     * @param motorPorts motor ports. Negative reverses a motor
     * @param position reads the position
     * @param config PID, feedforward, and settle settings
     */
    PositionMechanism(std::initializer_list<std::int8_t> motorPorts, PositionSource position,
                      PositionConfig config);

    PositionMechanism(const PositionMechanism&) = delete;
    PositionMechanism& operator=(const PositionMechanism&) = delete;

    // commands, from any task

    /**
     * @brief Move to a target with closed loop control
     *
     * Setting the same target again (every tick, say) doesn't restart settle timing. A target that
     * isn't finite is ignored
     *
     * @param target the target position
     */
    void setTarget(double target);

    /**
     * @brief Drive with open loop volts until the next setTarget(), holdPosition(), or stop()
     *
     * @param volts voltage, clamped to +-maxVolts. NaN counts as 0V
     *
     * @b Example
     * @code {.cpp}
     * // manual override while a button is held, then hold wherever it ends up
     * if (master.held(Button::up)) lift.setVolts(8.0);
     * else if (master.released(Button::up)) lift.holdPosition();
     * @endcode
     */
    void setVolts(double volts);

    /**
     * @brief Hold wherever the mechanism is now, to end a manual override without a jump
     *
     * With no reading, it stop()s instead
     */
    void holdPosition();

    /**
     * @brief Brake every update, with the brake mode set on motors()
     */
    void stop();

    /**
     * @brief Clear the PID's memory before the next update
     */
    void resetController();

    /**
     * @brief Get the last target set
     */
    double target() const;

    /**
     * @brief Get how the mechanism is being driven
     */
    PositionMode mode() const;

    /**
     * @brief Replace the gravity feedforward, e.g. with what Auto-Tune measured
     *
     * Takes effect at the next update. Its fields are separate atomics, so one update can mix old
     * and new values
     *
     * @param gravity the new feedforward
     */
    void setGravity(GravityFeedforward gravity);

    /**
     * @brief Get the gravity feedforward in effect
     */
    GravityFeedforward gravity() const;

    // lending the motors out, from any task

    /**
     * @brief Hand the motors to something else, like a tuning run, until endExternalControl()
     *
     * Meanwhile updates still read the sensor and report every step (law external, volts NaN) but
     * never command the motors. Commands are still accepted and take effect once the motors come
     * back, so a driver loop that sets a target every tick doesn't need to know. One end undoes
     * any number of begins
     */
    void beginExternalControl();

    /**
     * @brief Take the motors back. The next update starts the PID fresh
     */
    void endExternalControl();

    /**
     * @brief Whether something else has the motors
     */
    bool externalControl() const;

    // queries, from any task

    /**
     * @brief Get a fresh position reading. Not cached
     *
     * @return double position, or NaN if the sensor isn't answering
     */
    double position() const;

    /**
     * @brief Whether a fresh reading is within tolerance of a target. False with no reading
     *
     * @param target the target to check
     */
    bool isNear(double target) const;

    /**
     * @brief Whether it's in position mode and within tolerance of target()
     */
    bool atTarget() const;

    /**
     * @brief Whether it's in position mode and has stayed within tolerance for settleTimeMs since
     * the target last changed
     *
     * Never true for a target no update has worked on yet, so waiting on it right after
     * setTarget() can't return early on the old target
     */
    bool settled() const;

    /**
     * @brief Get the control law the latest update used
     */
    PositionLaw law() const;

    // running it

    /**
     * @brief Read the position and command the motors. Call once per tick in manual mode
     *
     * Ignored once startTask() has run
     *
     * @param nowMs the current time, in milliseconds
     * @return PositionStep what the update did
     *
     * @b Example
     * @code {.cpp}
     * void opcontrol() {
     *     while (true) {
     *         master.update();
     *         lift.setTarget(ladder.levelPosition());
     *         lift.update(master.now());
     *         pros::delay(20);
     *     }
     * }
     * @endcode
     */
    PositionStep update(std::uint32_t nowMs);

    /**
     * @brief Start a task that updates the mechanism every periodMs
     *
     * Warns if the period doesn't match pid.nominalDtS. While disabled, the task keeps running but
     * brakes with the PID cleared, so nothing winds up. Call it from initialize(), after
     * setStepListener()
     *
     * @param periodMs update period, in milliseconds. 10 by default
     * @return true the task started
     * @return false it was already started
     */
    bool startTask(std::uint32_t periodMs = 10);

    /**
     * @brief Whether the task is running
     */
    bool taskRunning() const;

    /**
     * @brief Wait until settled() or a timeout. For autonomous
     *
     * Without a task, this runs update() itself, but nothing drives the mechanism once it returns;
     * use startTask() to keep holding through the motions that follow
     *
     * @note blocks, so never call it from a per-tick driver control function
     *
     * @param timeoutMs longest time to wait, in milliseconds. 0 for no limit
     * @param pollMs how often to check, in milliseconds. 0 (the default) uses pid.nominalDtS
     * @return true it settled
     * @return false it timed out
     */
    bool waitUntilSettled(std::uint32_t timeoutMs, std::uint32_t pollMs = 0);

    /**
     * @brief setTarget(), then waitUntilSettled()
     *
     * @param target the target position
     * @param timeoutMs longest time to wait, in milliseconds. 0 for no limit
     * @return true it settled
     * @return false it timed out
     */
    bool moveTo(double target, std::uint32_t timeoutMs);

    // plumbing

    /**
     * @brief Get the motors, for brake mode, current, temperature, and so on
     */
    pros::MotorGroup& motors();

    /**
     * @brief Get the PID, e.g. for PidTunerPage::addController() or telemetry
     *
     * Its state belongs to whichever task runs update()
     */
    PID& pid();

    /**
     * @brief Get the config as constructed. Use gravity() for the live gravity feedforward
     */
    const PositionConfig& config() const;

    /**
     * @brief Set a function called at the end of every update, on the task that ran it
     *
     * @note must be quick and non-blocking (a telemetry Channel::record() is). Set it before
     * startTask()
     *
     * @param listener the function
     */
    void setStepListener(std::function<void(const PositionStep&)> listener);

private:
    // one update. disabled is the task's competition check; manual update() never passes it
    PositionStep step(std::uint32_t nowMs, bool disabled);
    void bumpCommand();
    std::uint32_t nominalPeriodMs() const;

    pros::MotorGroup motors_;
    PositionSource source_;
    PositionConfig config_;
    PID pid_;

    // shared with other tasks: lock-free atomics only
    std::atomic<PositionMode> mode_{PositionMode::off};
    std::atomic<double> target_{0.0};
    std::atomic<double> volts_{0.0};
    // which command the settle state belongs to: 31 bits, bumped when the target or mode changes
    std::atomic<std::uint32_t> commandGen_{0};
    // (gen << 1) | settled, in one store so settled() never pairs one command's flag with
    // another's generation
    std::atomic<std::uint32_t> settledState_{0};
    std::atomic<PositionLaw> law_{PositionLaw::off};
    std::atomic<bool> resetRequested_{false};
    std::atomic<bool> taskRunning_{false};
    std::atomic<bool> external_{false};
    // the live gravity feedforward, see setGravity()
    std::atomic<double> gravityConstantVolts_;
    std::atomic<double> gravityCosineVolts_;
    std::atomic<double> gravityHorizontalPosition_;
    std::atomic<double> gravityArmDegreesPerUnit_;

    // owned by whichever task runs step()
    GapDetector gap_;
    // within tolerance, and since when. Restarted for each new command
    TimedFlag near_;
    std::uint32_t nearGen_ = 0;
    std::function<void(const PositionStep&)> listener_;
    std::unique_ptr<pros::Task> task_;
    bool warnedUpdateWithTask_ = false;
};

} // namespace sapphirelib::mechanism
