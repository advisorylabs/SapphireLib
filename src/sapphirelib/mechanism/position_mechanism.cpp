#include "sapphirelib/mechanism/position_mechanism.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "pros/error.h"
#include "pros/misc.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"
#include "sapphirelib/util/wait.hpp"

namespace sapphirelib::mechanism {

// Commands cross tasks through these with no mutex (see the class comment). If
// one of them ever needs a lock on this toolchain, that's a hidden mutex on
// the control path, which PROS deleting a competition task mid-call could
// orphan — so make it a build error instead.
static_assert(std::atomic<double>::is_always_lock_free,
              "PositionMechanism shares its target across tasks without a mutex");
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
static_assert(std::atomic<PositionMode>::is_always_lock_free);
static_assert(std::atomic<PositionLaw>::is_always_lock_free);
static_assert(std::atomic<bool>::is_always_lock_free);

namespace {

/// commandGen_ keeps to 31 bits so settledState_ can hold (gen << 1) | settled
/// without dropping any of it.
constexpr std::uint32_t kGenMask = 0x7FFFFFFFu;

} // namespace

double readRotationDeg(const pros::Rotation& sensor) {
    const std::int32_t centidegrees = sensor.get_position();
    if (centidegrees == PROS_ERR) return std::numeric_limits<double>::quiet_NaN();
    return centidegrees / 100.0;
}

PositionMechanism::PositionMechanism(std::initializer_list<std::int8_t> motorPorts,
                                     PositionSource position, PositionConfig config)
    : motors_(motorPorts), source_(std::move(position)), config_(config), pid_(config.pid),
      gravityConstantVolts_(config.gravity.constantVolts),
      gravityCosineVolts_(config.gravity.cosineVolts),
      gravityHorizontalPosition_(config.gravity.horizontalPosition),
      gravityArmDegreesPerUnit_(config.gravity.armDegreesPerUnit), gap_(config.resetAfterGapMs) {
    // An empty std::function would throw when called. Reading "no sensor"
    // instead leaves the mechanism braking, which is visible (law() reports
    // "no sensor") rather than fatal.
    if (!source_) source_ = [] { return std::numeric_limits<double>::quiet_NaN(); };
}

void PositionMechanism::setTarget(double target) {
    if (!std::isfinite(target)) return;
    // Target first, then mode, then the generation: step() loads them in the
    // opposite order, so a step that sees the new generation is guaranteed to
    // see this target and mode too, and settle state can't be credited to a
    // command it wasn't computed for.
    const double previousTarget = target_.exchange(target);
    const PositionMode previousMode = mode_.exchange(PositionMode::position);
    if (previousTarget != target || previousMode != PositionMode::position) bumpCommand();
}

void PositionMechanism::setVolts(double volts) {
    // A NaN would survive the clamp in step(), and converting it to
    // millivolts is undefined behavior.
    volts_.store(std::isnan(volts) ? 0.0 : volts);
    // Only the mode change matters to settle state; new volts in voltage mode
    // are the same command as far as settled() is concerned (never settled).
    if (mode_.exchange(PositionMode::voltage) != PositionMode::voltage) bumpCommand();
}

void PositionMechanism::holdPosition() {
    const double position = source_();
    if (!std::isfinite(position)) {
        stop();
        return;
    }
    setTarget(position);
}

void PositionMechanism::stop() {
    if (mode_.exchange(PositionMode::off) != PositionMode::off) bumpCommand();
}

void PositionMechanism::resetController() { resetRequested_.store(true); }

double PositionMechanism::target() const { return target_.load(); }

void PositionMechanism::setGravity(GravityFeedforward gravity) {
    gravityConstantVolts_.store(gravity.constantVolts);
    gravityCosineVolts_.store(gravity.cosineVolts);
    gravityHorizontalPosition_.store(gravity.horizontalPosition);
    gravityArmDegreesPerUnit_.store(gravity.armDegreesPerUnit);
}

GravityFeedforward PositionMechanism::gravity() const {
    return GravityFeedforward{.constantVolts = gravityConstantVolts_.load(),
                              .cosineVolts = gravityCosineVolts_.load(),
                              .horizontalPosition = gravityHorizontalPosition_.load(),
                              .armDegreesPerUnit = gravityArmDegreesPerUnit_.load()};
}

void PositionMechanism::beginExternalControl() { external_.store(true); }

void PositionMechanism::endExternalControl() {
    // Cleared by the reset the external steps already do, but asked for
    // anyway: a step that loaded external_ just before this store has one
    // more update() to run on stale memory otherwise.
    resetRequested_.store(true);
    external_.store(false);
}

bool PositionMechanism::externalControl() const { return external_.load(); }

PositionMode PositionMechanism::mode() const { return mode_.load(); }

double PositionMechanism::position() const { return source_(); }

bool PositionMechanism::isNear(double target) const {
    // NaN (no reading) compares false, so "no sensor" is never "near".
    return std::fabs(position() - target) <= config_.tolerance;
}

bool PositionMechanism::atTarget() const {
    return mode() == PositionMode::position && isNear(target());
}

bool PositionMechanism::settled() const {
    const std::uint32_t state = settledState_.load();
    return (state & 1u) != 0 && (state >> 1) == commandGen_.load();
}

PositionLaw PositionMechanism::law() const { return law_.load(); }

PositionStep PositionMechanism::update(std::uint32_t nowMs) {
    if (taskRunning_.load()) {
        // Two loops commanding one motor group fight. Warn once rather than
        // every tick (or asserting, which would take the robot down
        // mid-match).
        if (!warnedUpdateWithTask_) {
            warnedUpdateWithTask_ = true;
            SAPPHIRELIB_LOG_WARN("mechanism",
                                 "update() ignored: startTask() is already running this mechanism");
        }
        return PositionStep{};
    }
    return step(nowMs, /*disabled=*/false);
}

bool PositionMechanism::startTask(std::uint32_t periodMs) {
    // Claimed before the task exists, in one step, so a second call can't
    // start a second loop, and update() is ignored from here on rather than
    // interleaving with the task's first step.
    if (taskRunning_.exchange(true)) return false;
    // A 0ms period would be a busy loop starving every lower-priority task.
    periodMs = std::max<std::uint32_t>(periodMs, 1);
    const double periodS = periodMs / 1000.0;
    if (std::fabs(periodS - config_.pid.nominalDtS) > 0.0005) {
        // kI and kD are per second, scaled by nominalDtS — which is only right
        // if the loop really runs that often.
        SAPPHIRELIB_LOG_WARN("mechanism",
                             "startTask(%lu) runs every %.3fs but pid.nominalDtS is %.3fs; set "
                             "nominalDtS = %.3f so kI and kD mean what they say",
                             static_cast<unsigned long>(periodMs), periodS, config_.pid.nominalDtS,
                             periodS);
    }
    task_ = std::make_unique<pros::Task>(
        [this, periodMs] {
            // delay_until keeps the period steady however long a step takes,
            // so nominalDtS stays true.
            std::uint32_t wake = pros::millis();
            while (true) {
                step(sapphirelib::millis(), pros::competition::is_disabled() != 0);
                pros::Task::delay_until(&wake, periodMs);
            }
        },
        "PositionMechanism");
    // Never deleted: deleting a task in the middle of a PROS device call can
    // orphan that port's kernel mutex. And since only this task touches the
    // motors from now on, an autonomous or opcontrol task that PROS deletes
    // was only ever touching atomics.
    return true;
}

bool PositionMechanism::taskRunning() const { return taskRunning_.load(); }

bool PositionMechanism::waitUntilSettled(std::uint32_t timeoutMs, std::uint32_t pollMs) {
    return waitUntil(
        [this] {
            // Manual mode: nothing else is running the loop, so run it here.
            if (!taskRunning_.load()) step(sapphirelib::millis(), /*disabled=*/false);
            return settled();
        },
        timeoutMs, pollMs > 0 ? pollMs : nominalPeriodMs());
}

bool PositionMechanism::moveTo(double target, std::uint32_t timeoutMs) {
    setTarget(target);
    return waitUntilSettled(timeoutMs);
}

pros::MotorGroup& PositionMechanism::motors() { return motors_; }

PID& PositionMechanism::pid() { return pid_; }

const PositionConfig& PositionMechanism::config() const { return config_; }

void PositionMechanism::setStepListener(std::function<void(const PositionStep&)> listener) {
    listener_ = std::move(listener);
}

PositionStep PositionMechanism::step(std::uint32_t nowMs, bool disabled) {
    // Updates far enough apart that the PID's memory (last reading, integral)
    // is from before the gap: start it fresh. PID::reset() is idempotent, so
    // this and an explicit resetController() can both fire on one step.
    const bool gap = gap_.update(nowMs) && config_.resetAfterGapMs > 0;
    if (resetRequested_.exchange(false) || gap) pid_.reset();

    // Generation first, then mode and target (the reverse of setTarget()'s
    // stores; see there).
    const std::uint32_t gen = commandGen_.load();
    const PositionMode mode = mode_.load();
    const double target = target_.load();
    const double position = source_(); // the one sensor read of this update

    PositionCommand command;
    // Whoever has the motors (see beginExternalControl()) is commanding
    // them; this update only reads and reports. It wins over disabled too:
    // braking would fight a run that is itself responsible for stopping.
    const bool external = external_.load();
    if (external) {
        pid_.reset();
        command = {.law = PositionLaw::external, .volts = 0.0, .brake = false};
    } else if (disabled) {
        // VEXos ignores motor commands while the robot is disabled. Running
        // the loop anyway would integrate an error it can't act on and lurch
        // the mechanism at enable, so rest with the PID cleared instead (one
        // reset, not one per tick, since nothing updates it in between) and
        // start fresh on the first enabled step.
        pid_.reset();
        command = {.law = PositionLaw::off, .volts = 0.0, .brake = true};
    } else {
        switch (mode) {
            case PositionMode::position: {
                // The config with the live gravity (setGravity()); copied per
                // update rather than written into config_, which other tasks
                // may be reading through config().
                PositionConfig live = config_;
                live.gravity = gravity();
                command = computePositionCommand(pid_, live, target, position);
                break;
            }
            case PositionMode::voltage:
                // Reset every update, so handing back to closed loop starts
                // without stale derivative or integral.
                pid_.reset();
                command = {.law = PositionLaw::manual,
                           .volts = std::clamp(volts_.load(), -config_.maxVolts, config_.maxVolts),
                           .brake = false};
                break;
            case PositionMode::off:
                pid_.reset();
                command = {.law = PositionLaw::off, .volts = 0.0, .brake = true};
                break;
        }
    }
    // Brake mode, never move_voltage(0), on sensor loss and when off: the
    // user's brake mode (hold, usually) is what keeps a lift up.
    if (external) {
        // Not ours to command.
    } else if (command.brake) {
        motors_.brake();
    } else {
        motors_.move_voltage(command.millivolts());
    }

    const bool near = mode == PositionMode::position &&
                      std::fabs(position - target) <= config_.tolerance; // NaN -> false
    if (gen != nearGen_) {
        // A new command: settle timing starts over, whatever the old one's was.
        near_ = TimedFlag(false, nowMs);
        nearGen_ = gen;
    }
    near_.set(near, nowMs);
    const bool settled = near && near_.trueFor(config_.settleTimeMs, nowMs);
    settledState_.store((gen << 1) | (settled ? 1u : 0u));
    law_.store(command.law);

    const PositionStep result{.timeMs = nowMs,
                              .law = command.law,
                              .mode = mode,
                              .target = target,
                              .position = position,
                              .volts = external        ? std::numeric_limits<double>::quiet_NaN()
                                       : command.brake ? 0.0
                                                       : command.volts,
                              .atTarget = near,
                              .settled = settled};
    if (listener_) listener_(result);
    return result;
}

void PositionMechanism::bumpCommand() {
    // A CAS loop rather than load-then-store, so even two tasks commanding at
    // once (which the class comment says not to do) can't lose a bump and let
    // an old settle flag count for the new command.
    std::uint32_t gen = commandGen_.load();
    while (!commandGen_.compare_exchange_weak(gen, (gen + 1) & kGenMask)) {
    }
}

std::uint32_t PositionMechanism::nominalPeriodMs() const {
    // Clamped so a zero, negative, NaN, or absurd nominalDtS can't make the
    // poll a busy loop (or lround() meaningless).
    const double ms = config_.pid.nominalDtS * 1000.0;
    if (!(ms >= 1.0)) return 1;
    return static_cast<std::uint32_t>(std::lround(std::min(ms, 1000.0)));
}

} // namespace sapphirelib::mechanism
