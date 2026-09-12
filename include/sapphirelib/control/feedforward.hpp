/**
 * \file sapphirelib/control/feedforward.hpp
 *
 * A measured model of one drive axis: how many volts it takes to overcome
 * friction, hold a speed, and accelerate. Produced by
 * tuning::fitFeedforward() from a characterization run, and used both to
 * turn a wanted velocity straight into a voltage (feedforward) and to
 * design PID gains from physics instead of a rule of thumb
 * (tuning::designPositionGains()). No PROS dependency — see
 * tests/control/feedforward_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib {

/// V = kS·sign(v) + kV·v + kA·a, for one axis (forward, strafe, or turn).
///
/// Units follow whatever the axis was measured in: for a translation axis
/// measured with odometry, v is inches/second and a is inches/second²; for
/// a turn axis measured with the IMU, degrees/second and degrees/second².
/// The voltage is the axis command before wheel mixing — the same number a
/// PID on that axis outputs.
struct MotorFeedforward {
    /// Volts needed just to break static friction and start moving.
    double kS = 0.0;

    /// Volts per unit/second of steady speed — mostly the motors' back-EMF.
    double kV = 0.0;

    /// Volts per unit/second² of acceleration — the chassis's inertia.
    double kA = 0.0;

    /// True once kV and kA are both positive — the least a model needs to
    /// say anything about how the axis moves. A default-constructed
    /// (unmeasured) model is not valid.
    bool valid() const;

    /// Volts that would produce `velocity` while accelerating at
    /// `acceleration`. The friction term takes the sign of the velocity, or
    /// of the acceleration when starting from rest (so a motion profile's
    /// very first tick already pushes through static friction). Unclamped —
    /// the caller decides what to do past 12V.
    double volts(double velocity, double acceleration = 0.0) const;

    /// Steady speed reachable with `availableVolts` applied: (V - kS) / kV.
    /// 0 for an invalid model or if `availableVolts` can't overcome kS.
    double maxVelocity(double availableVolts = 12.0) const;
};

} // namespace sapphirelib
