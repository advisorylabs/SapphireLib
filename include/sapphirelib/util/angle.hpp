/**
 * \file sapphirelib/util/angle.hpp
 *
 * Small angle-wrapping helper shared by every heading-based control loop
 * (tank and holonomic turnToHeading(), heading-drift correction, ...).
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib {

/// Wraps a degree value to (-180, 180] — the shortest signed error to 0.
/// Use this before feeding a heading error into a PID so a target/measurement
/// pair straddling the 0/360 seam (e.g. target 5, measurement 355) doesn't
/// produce a ~350 degree error instead of the true ~10 degree one.
double wrapDegrees180(double degrees);

} // namespace sapphirelib
