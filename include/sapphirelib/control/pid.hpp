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

        /// Clamps the accumulated integral to +-integralLimit before it's
        /// multiplied by kI. 0 disables the clamp. This is the integral
        /// windup guard — without it, a controller stuck away from its
        /// target accumulates unbounded integral and overshoots badly once
        /// it finally gets close.
        double integralLimit = 0.0;

        /// Clamps the final output to +-outputLimit. 0 disables the clamp.
        double outputLimit = 0.0;

        /// Limits how much the output can change between consecutive
        /// update() calls. 0 disables the limit.
        double slewRate = 0.0;

        /// When true, the derivative term is computed from the change in
        /// measurement instead of the change in error, avoiding "derivative
        /// kick" when the target changes abruptly.
        bool derivativeOnMeasurement = false;
    };

    explicit PID(Config config);

    /// Computes one control step. `target` and `measurement` must be in the
    /// same units; the returned output is clamped/rate-limited per Config.
    double update(double target, double measurement);

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
