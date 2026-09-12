/**
 * \file sapphirelib/control/heading_hold.hpp
 *
 * Turning the turn stick from a rate command into a heading command. Instead
 * of "how hard to spin right now", the stick steers a heading the chassis is
 * then closed-loop held on — so letting go stops the chassis pointed
 * somewhere definite, and anything that knocks it off that heading (a
 * collision, an uneven strafe, one wheel biting harder than another) gets
 * corrected without the driver doing anything.
 *
 * This is the pure part: advancing the held heading. Closing the loop on it
 * is the drivetrain's job — see
 * HolonomicDrivetrain::holonomicFieldCentricHeadingHold(). No PROS
 * dependency; see tests/control/heading_hold_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib {

/// Shapes how the turn stick steers the held heading. Defaults are a
/// reasonable starting point for a VRC chassis; all three interact, so read
/// slewDegPerSec's note on which one to reach for first.
struct HeadingHoldConfig {
    /// How fast a fully deflected turn stick sweeps the held heading, in
    /// degrees per second. This is how fast the driver can *ask* to turn —
    /// not necessarily how fast the chassis actually turns, which is capped
    /// by maxLeadDeg and by how hard the heading-hold PID chases.
    ///
    /// If turning feels sluggish, raise maxLeadDeg or the heading-hold PID's kP
    /// before raising this: past the point where the chassis can keep up,
    /// more slew just pins the held heading against the lead clamp and
    /// changes nothing.
    double slewDegPerSec = 180.0;

    /// Stick magnitude, normalized, below which the stick counts as centered.
    ///
    /// Not optional in practice. A V5 stick at rest still reports a count or
    /// two, curveJoystick() passes that straight through, and a held heading
    /// integrates it — so without a deadband the chassis would very slowly
    /// rotate all match with nobody touching the controls. Input above the
    /// deadband is rescaled to span the full range, so there's no jump as
    /// the stick crosses out of it.
    double deadband = 0.05;

    /// How far, in degrees, the held heading is allowed to get ahead of
    /// where the chassis is actually pointing.
    ///
    /// This is what keeps a rate-steered heading honest. Without it, holding
    /// the stick sweeps the target as fast as slewDegPerSec says while the
    /// chassis lags behind by however much error its heading-hold PID needs to keep
    /// moving — and that debt is real: release the stick and the chassis
    /// keeps rotating until it has paid off every degree. Capping the lead
    /// bounds how far past the driver's intent it can coast, at the cost of
    /// making this (not slewDegPerSec) the real limit on sustained turn rate.
    ///
    /// 0 or less disables the cap, which is only sensible if slewDegPerSec is
    /// already slow enough that the chassis never falls behind.
    double maxLeadDeg = 20.0;
};

/// Advances a held heading by one tick of turn-stick input.
///
/// `heldHeadingDeg` is the previous held heading and `currentHeadingDeg`
/// where the chassis actually points (both in degrees, same convention as
/// pros::Imu::get_heading()); `turnInput` is the normalized [-1, 1] stick,
/// positive turning the same way a positive heading reading grows, and
/// `dtS` is the seconds since the last call. A non-positive or implausibly
/// large `dtS` advances nothing rather than dumping a lump of rotation in —
/// same reasoning as PID::update()'s guard on its own timestep.
///
/// The result stays within `config.maxLeadDeg` of `currentHeadingDeg`, so it
/// tracks the chassis rather than running off on its own; it is not wrapped
/// to any particular range, since the only thing that ever reads it is a
/// wrapDegrees180() difference against a live heading.
double advanceHeldHeadingDeg(double heldHeadingDeg, double currentHeadingDeg, double turnInput,
                             double dtS, HeadingHoldConfig config);

} // namespace sapphirelib
