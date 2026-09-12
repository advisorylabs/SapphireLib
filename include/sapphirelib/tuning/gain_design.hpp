/**
 * \file sapphirelib/tuning/gain_design.hpp
 *
 * Pole placement: turns a measured axis model (MotorFeedforward) and a
 * description of how a motion should behave (ResponseSpec) into PID gains.
 * Pure math, no PROS dependency — see tests/tuning/gain_design_test.cpp.
 *
 * The idea: with feedforward cancelling friction, one axis under PD control
 * obeys
 *
 *     kA·x'' + (kV + kD)·x' + kP·x = kP·target
 *
 * which is a spring and damper. Picking how fast it should respond (natural
 * frequency ω) and how much it may overshoot (damping ratio ζ) fixes both
 * gains outright:
 *
 *     kP = kA·ω²          kD = 2·ζ·ω·kA − kV
 *
 * The one thing that formula doesn't know about is delay. Every V5 control
 * loop acts on sensor readings a few tens of milliseconds old, and a design
 * that ignores that and asks for a very fast response oscillates on the
 * robot however clean it looks on paper. designPositionGains() checks the
 * phase margin that delay leaves and backs ω off until it's safe.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/control/pid.hpp"

namespace sapphirelib::tuning {

/// How one closed-loop motion should behave. Different uses of the same axis
/// want different specs — an autonomous turn wants to snap onto its heading,
/// driver heading hold wants to feel soft — and they can all be designed from
/// one measured model.
struct ResponseSpec {
    /// Time to settle within 2% of a step's size, in seconds.
    double settleTimeS = 0.6;

    /// 1.0 is the fastest response that never overshoots. Below 1 trades
    /// some overshoot for speed; above 1 approaches even more gently.
    /// Clamped to [0.1, 5].
    double dampingRatio = 1.0;

    /// Least acceptable phase margin once measured delay is accounted for.
    /// 45-60° is the usual range; lower responds faster but rings more and
    /// tolerates less change in the robot (a heavier game piece, a low
    /// battery) before it oscillates.
    double minPhaseMarginDeg = 50.0;
};

/// Result of designPositionGains().
struct GainDesign {
    /// False only for an invalid model or a nonsensical spec; a design that
    /// had to be slowed for delay is still ok (see limitedByDelay).
    bool ok = false;

    /// kP and kD; kI is always 0 — feedforward's kS term already removes the
    /// friction that usually leaves the steady-state error an integrator
    /// exists to clean up.
    PIDGains gains;

    /// The natural frequency actually used, in rad/s.
    double naturalFrequency = 0.0;

    /// What settleTimeS works out to with that frequency — longer than
    /// requested if limitedByDelay.
    double settleTimeS = 0.0;

    /// Phase margin of the final design with the given delay, in degrees.
    double phaseMarginDeg = 0.0;

    /// True if the requested settle time wasn't achievable within
    /// minPhaseMarginDeg, so the response was slowed until it was.
    bool limitedByDelay = false;
};

/// 2% settling time of a unit-natural-frequency spring-damper with damping
/// ratio `dampingRatio` (clamped to [0.1, 5]). Divide by a settle time to get
/// the natural frequency that achieves it. For example, critical damping
/// (ζ = 1) settles in about 5.83/ω.
double normalizedSettleTime(double dampingRatio);

/// Phase margin, in degrees, of `gains` (kP, kD; kI ignored) controlling an
/// axis with `model` through `delayS` seconds of pure delay. Returns 180 if
/// the loop gain never crosses 1 (kP = 0), and can go negative — an
/// unstable design.
double phaseMarginDeg(const MotorFeedforward& model, PIDGains gains, double delayS);

/// Pole-placement PD gains for position control of one axis — see the file
/// comment for the math. `delayS` is the axis's measured response delay
/// (tuning::estimateResponseDelayS()); 0 designs as if there were none.
///
/// If kV alone already provides more damping than the spec asks for, kD
/// comes out negative; it is clamped to 0 instead, which leaves the response
/// somewhat more damped than requested rather than feeding speed back as
/// positive feedback.
GainDesign designPositionGains(const MotorFeedforward& model, ResponseSpec spec,
                               double delayS = 0.0);

} // namespace sapphirelib::tuning
