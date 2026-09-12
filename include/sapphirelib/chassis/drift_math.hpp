/**
 * \file sapphirelib/chassis/drift_math.hpp
 *
 * The measurement behind AsteriskConfig::driftCorrectionKP — how much of the
 * forward/back motion a vertical tracking wheel sees during a strafe is
 * actually drift — factored out as pure math with no PROS dependency, so it
 * can be unit-tested on a desktop compiler (see
 * tests/chassis/drift_math_test.cpp).
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::chassis {

/// Forward/back displacement over one control tick that nobody asked for —
/// the error HolonomicDrivetrain's center wheels correct while strafing.
///
/// `verticalWheelDeltaIn` is the vertical tracking wheel's signed travel this
/// tick; `verticalOffsetIn` its perpendicular offset from the tracking center
/// (OdometryConfig::verticalOffsetIn, positive = right of center);
/// `rotationDeltaDeg` the chassis's unwrapped heading change this tick
/// (clockwise-positive); `strafeTravelIn` the sideways travel the corner
/// drive encoders report this tick (positive = right); `throttleVolts` /
/// `strafeVolts` the forward and sideways components of this tick's corner
/// command.
///
/// Two things the raw wheel reading includes are removed:
///   - The wheel's rotation arc (offset * radians turned, same correction as
///     computeOdometryDelta()), so turning while strafing isn't drift.
///   - The forward travel the command asked for, taken as the strafe travel
///     actually achieved scaled by the command's forward-per-sideways ratio,
///     so an intentional diagonal isn't drift either.
/// Deliberately *not* removed: forward travel the corner encoders report on
/// their own. Corners spinning unevenly under the same command (one derated
/// or dragging motor, uneven weight on the rollers) is the most common
/// source of strafe drift, and the encoders see it as real forward motion —
/// subtracting it would blind the correction to exactly what it's for.
///
/// Returns 0 when `strafeVolts` is 0 (no strafe, so no commanded direction to
/// measure drift against).
double strafeDriftIn(double verticalWheelDeltaIn, double verticalOffsetIn, double rotationDeltaDeg,
                     double strafeTravelIn, double throttleVolts, double strafeVolts);

} // namespace sapphirelib::chassis
