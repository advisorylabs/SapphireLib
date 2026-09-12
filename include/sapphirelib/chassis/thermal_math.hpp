/**
 * \file sapphirelib/chassis/thermal_math.hpp
 *
 * A V5 smart motor that gets hot quietly stops delivering what you asked
 * for: its firmware derates available power in steps as temperature climbs,
 * and nothing in the command path reports that back. On a holonomic chassis
 * that's worse than just "slower" — the four corners' contributions are
 * supposed to cancel in every axis but the one you're driving, so a single
 * derated corner breaks the cancellation and the chassis picks up motion it
 * was never asked for.
 *
 * This is the pure math behind noticing that and handing the correction to
 * the Asterisk center wheels. No PROS dependency — see
 * tests/chassis/thermal_math_test.cpp. HolonomicDrivetrain wires it up (see
 * AsteriskConfig::thermalCompensation).
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::chassis {

/// One value per corner wheel, in HolonomicDrivetrain's port order.
struct CornerValues {
    double frontLeft = 0.0;
    double frontRight = 0.0;
    double backLeft = 0.0;
    double backRight = 0.0;
};

/// What the center wheels should add to what they're already commanded.
struct CenterCorrection {
    /// Added to both center wheels — replaces missing forward/back thrust,
    /// or cancels forward/back motion the corners are producing by accident.
    double commonVolts = 0.0;

    /// Added to the left center wheel and subtracted from the right —
    /// same differential-yaw convention as AsteriskConfig::turnContribution.
    double differentialVolts = 0.0;
};

/// Estimated fraction of its rated output a V5 smart motor still delivers at
/// `tempC` — 1.0 when cool, 0.0 once thermal shutdown takes over.
///
/// The V5 derates in discrete steps (published as roughly 50% at 55C, 25% at
/// 60C, 12.5% at 65C, off at 70C), but this returns a curve that ramps
/// across a couple of degrees around each step rather than jumping. That's
/// deliberate: the reported temperature dithers, and a step function would
/// have anything driven off this value slamming back and forth every time a
/// reading crossed a threshold. Between steps the two agree to within a few
/// percent, which is well inside the accuracy this estimate can claim
/// anyway.
///
/// A non-finite reading — which is what PROS hands back for a motor that
/// isn't answering (PROS_ERR_F is infinity) — reads as 1.0, so a
/// disconnected cable never masquerades as a hot motor. Diagnosing that one
/// is diag::SensorCheck's job.
double thermalPowerFraction(double tempC);

/// What the center wheels have to add so the chassis actually does what the
/// corner mix asked for, given that each corner only delivers
/// `survivingFraction` of the voltage it was handed.
///
/// Each corner falls short by `commandedVolts * (1 - survivingFraction)`.
/// Summing those shortfalls with the same sign patterns
/// HolonomicDrivetrain's mixer uses to build the corner commands
/// ([+ + + +] for forward, [+ - + -] for yaw) projects them back onto the
/// axes the center wheels can actually push on, and halving spreads the job
/// across the two of them.
///
/// That one expression covers two cases that look unrelated on the robot:
///
///   - Corners derating *together* while driving forward. The shortfalls
///     reinforce, and the result is the center wheels driving harder to hold
///     the chassis's speed up.
///
///   - A *single* corner derating while strafing. The four corners'
///     forward components are meant to cancel exactly; one weak corner
///     breaks that, and the chassis creeps forward or back — and twists —
///     across a strafe that was supposed to be pure sideways. Here the
///     shortfalls don't cancel, and the result is the center wheels holding
///     the chassis straight. They can't add sideways thrust — nothing
///     mounted fore/aft can — so a strafe on hot corners is still slower,
///     but it stops wandering off its line.
///
/// `gain` scales how much of the shortfall is attempted (0 disables),
/// `maxVolts` caps each component, and the result is then faded out by
/// `centerPowerFraction` so center wheels that are themselves heating up get
/// asked for less rather than more.
CenterCorrection centerThermalCorrection(CornerValues commandedVolts,
                                         CornerValues survivingFraction,
                                         double centerPowerFraction, double gain,
                                         double maxVolts);

} // namespace sapphirelib::chassis
