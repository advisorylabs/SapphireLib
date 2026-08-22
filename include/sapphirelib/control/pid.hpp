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

namespace sapphirelib {

/// Proportional, integral, and derivative gains for a PID controller.
///
/// These are *continuous-time* gains, the standard convention: kI is
/// "output units per (error unit x second)" and kD is "output units per
/// (error unit / second)". PID::update() scales the integral and derivative
/// terms by its timestep accordingly, so a set of gains stays valid if the
/// loop period changes, and gains produced by textbook tuning rules (see
/// sapphirelib::tuning::gainsFromUltimate()) can be used directly without
/// a per-tick conversion.
struct PIDGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
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
        /// update() calls. 0 disables the limit.
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

private:
    Config config_;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    double prevMeasurement_ = 0.0;
    double prevOutput_ = 0.0;
    bool hasPrev_ = false;
};

} // namespace sapphirelib
