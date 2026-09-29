/**
 * \file sapphirelib/control/pid.hpp
 *
 * Generic PID controller: proportional-integral-derivative control with an
 * integral windup guard, optional derivative-on-measurement, and optional
 * output slew-rate limiting. Framework-agnostic — no PROS dependency, so it
 * can be unit-tested on a desktop compiler (see tests/control/pid_test.cpp).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

namespace sapphirelib {

/// Proportional, integral, and derivative gains for a PID controller.
///
/// These are *continuous-time* gains, the standard convention: kI is
/// "output units per (error unit x second)" and kD is "output units per
/// (error unit / second)". PID::update() scales the integral and derivative
/// terms by its timestep accordingly, so a set of gains stays valid if the
/// loop period changes, and gains designed from a measured model (see
/// sapphirelib::tuning::designPositionGains()) can be used directly without
/// a per-tick conversion.
struct PIDGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
};

class PID;

/// Everything one PID::update() worked out, not just what it returned — so a
/// log (see telemetry::Logger::pid()) or a readout can show *why* the
/// controller commanded what it did. Tuning needs the individual terms:
/// "overshoots" is a kP/kD problem, "never quite arrives" an integral or
/// friction one, and only the terms tell those apart.
///
/// `target` and `measurement` are exactly what the caller passed. Some loops
/// fold the error into target and pass a measurement of 0 — the drivetrains'
/// turn loops, and the pose motions' distance loops — so read `error`, not
/// target or measurement, as the controller's view of how far off it was.
struct PidStep {
    /// The output hit Config::outputLimit and was clamped.
    static constexpr std::uint8_t kSaturated = 1u << 0;
    /// Config::slewRate limited this step's change in output.
    static constexpr std::uint8_t kSlewLimited = 1u << 1;
    /// Conditional-integration anti-windup rolled back this step's
    /// integration (see Config::outputLimit).
    static constexpr std::uint8_t kIntegralHeld = 1u << 2;
    /// First update() since construction or reset(): no derivative yet, and
    /// the start of a new response for anything splitting a log into steps.
    static constexpr std::uint8_t kFirstStep = 1u << 3;
    /// The dtS passed in was non-positive or implausibly large, so
    /// Config::nominalDtS was used instead.
    static constexpr std::uint8_t kDtFallback = 1u << 4;

    double target = 0.0;
    double measurement = 0.0;
    double error = 0.0;

    /// kP·error, kI·integral, kD·derivative — each already in output units.
    double pTerm = 0.0;
    double iTerm = 0.0;
    double dTerm = 0.0;

    /// The output after anti-windup but before slew limiting and clamping.
    double rawOutput = 0.0;
    /// What update() returned.
    double output = 0.0;
    /// The timestep actually used, in seconds.
    double dtS = 0.0;
    std::uint8_t flags = 0;
};

/// Receives every step of the PID it's attached to — see PID::setObserver().
/// An interface rather than a std::function so an attached observer costs one
/// indirect call on the loop's own task and PID stays allocation-free.
class PidObserver {
public:
    virtual ~PidObserver() = default;

    /// Called at the end of every update(), on whichever task called it, with
    /// the step that update() just computed. Must not block or allocate: it
    /// runs inside someone's control loop.
    virtual void onPidUpdate(const PID& pid, const PidStep& step) = 0;

    /// Called from reset() — but only when the PID had state to clear, so a
    /// loop that resets every tick while idle (a lift resting on its hard
    /// stop, say) doesn't report a reset every tick.
    virtual void onPidReset(const PID& /*pid*/) {}
};

/// A single-axis PID controller. One instance drives one control loop (e.g.
/// drive distance, or heading); construct a fresh one per loop.
class PID {
public:
    struct Config {
        PIDGains gains;

        /// Clamps the accumulated integral (in error units x seconds) to
        /// +-integralLimit before it's multiplied by kI. 0 disables the
        /// clamp. This is the explicit integral windup guard — without it,
        /// a controller stuck away from its target accumulates unbounded
        /// integral and overshoots badly once it finally gets close.
        ///
        /// Note that update() *also* applies automatic anti-windup whenever
        /// outputLimit is set (see below), so leaving this at 0 is a
        /// reasonable default; set it when you want a tighter bound on the
        /// integral term than "it alone may not saturate the output".
        double integralLimit = 0.0;

        /// Clamps the final output to +-outputLimit. 0 disables the clamp.
        ///
        /// When set, this also enables automatic anti-windup: on any tick
        /// where the unclamped output is saturated and this tick's error
        /// would drive it further into saturation, the integration for that
        /// tick is rolled back instead of accumulating charge the output
        /// can't express (conditional integration). This is what keeps an
        /// auto-tuned kI safe on a plant that spends real time at full
        /// output, without needing integralLimit hand-picked per loop.
        double outputLimit = 0.0;

        /// Limits how much the output can change between consecutive
        /// update() calls — per call, not per second, so the same number
        /// ramps twice as fast in a 10ms loop as in a 20ms one. 0 disables
        /// the limit.
        double slewRate = 0.0;

        /// When true, the derivative term is computed from the change in
        /// measurement instead of the change in error, avoiding "derivative
        /// kick" when the target changes abruptly.
        ///
        /// Only meaningful for call sites that pass a real measurement.
        /// Loops that fold the error into `target` and pass a constant
        /// `measurement` of 0 (as the drivetrains' turn loops do) must
        /// leave this false, since the measurement never changes there and
        /// the derivative term would be identically zero.
        bool derivativeOnMeasurement = false;

        /// Timestep, in seconds, assumed by the two-argument update()
        /// overload — i.e. how often the loop calling it ticks. The
        /// drivetrains run their control loops on a fixed 10ms delay, which
        /// is where this default comes from.
        ///
        /// A fixed nominal timestep is deliberate rather than measuring the
        /// real elapsed time every tick: on a jittery RTOS loop, dividing
        /// the derivative term by a measured dt amplifies scheduling jitter
        /// straight into the output. Loops that genuinely run at a variable
        /// rate should call the three-argument update() instead.
        double nominalDtS = 0.01;
    };

    explicit PID(Config config);

    /// Computes one control step, assuming Config::nominalDtS elapsed since
    /// the previous call. `target` and `measurement` must be in the same
    /// units; the returned output is clamped/rate-limited per Config.
    double update(double target, double measurement);

    /// Computes one control step over an explicit timestep `dtS`, in
    /// seconds — for loops that don't run at a fixed rate. A non-positive
    /// or implausibly large `dtS` falls back to Config::nominalDtS rather
    /// than producing a divide-by-zero or a derivative spike.
    double update(double target, double measurement, double dtS);

    /// Clears integral, previous error/measurement, and previous output
    /// state. Call before reusing a PID instance for a new motion.
    void reset();

    void setGains(PIDGains gains);
    const PIDGains& gains() const;

    /// The config this PID was constructed with, gains as last set by
    /// setGains(). For telemetry and readouts; there's no setter because
    /// nothing but the gains is meant to change on a live controller.
    const Config& config() const;

    /// What the most recent update() computed (all zeros before the first).
    /// Left alone by reset(), so a readout can still show the last step of a
    /// motion that has finished. Read it from the task that calls update(),
    /// or accept a torn read.
    const PidStep& lastStep() const;

    /// Attaches `observer` (nullptr detaches) to receive every step and every
    /// state-clearing reset — how telemetry::Logger::pid() records a
    /// controller without the PID knowing anything about SD cards, tasks, or
    /// PROS.
    ///
    /// Not synchronized: attach before any task starts calling update() on
    /// this PID (in initialize(), before the loop that runs it), not while it
    /// is live. The observer must outlive the attachment. Copying a PID copies
    /// this pointer, so an observer that cares should check which PID it was
    /// handed (telemetry::PidProbe does).
    void setObserver(PidObserver* observer);
    PidObserver* observer() const;

private:
    Config config_;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    double prevMeasurement_ = 0.0;
    double prevOutput_ = 0.0;
    bool hasPrev_ = false;
    PidStep lastStep_;
    PidObserver* observer_ = nullptr;
};

} // namespace sapphirelib
