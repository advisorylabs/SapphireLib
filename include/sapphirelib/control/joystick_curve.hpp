/**
 * \file sapphirelib/control/joystick_curve.hpp
 *
 * Stick shaping for driver control: a cubic curve that blends a linear
 * response with a cubic one, so small stick deflections near center give
 * finer control while full deflection still reaches +-1, and a deadband that
 * ignores the count or two a stick reads at rest.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib {

/// Applies a cubic joystick curve to a normalized input in [-1, 1].
/// `curve` in [0, 1]: 0 is linear (no curve), 1 is a pure cubic response.
/// `curve` is clamped to [0, 1]. The output preserves sign and always maps
/// -1 -> -1, 0 -> 0, 1 -> 1, and is monotonic over curve in [0, 1].
double curveJoystick(double input, double curve);

/// Zeroes a normalized input inside +-deadband and rescales the rest so the
/// output still spans [-1, 1] with no jump at the deadband's edge:
/// sign(x)·(|x| − deadband)/(1 − deadband). A V5 stick at rest reads a
/// count or two, which is enough to creep a mechanism or a held heading.
/// `deadband` <= 0 returns `input` unchanged; >= 1 returns 0. Otherwise an
/// input beyond +-1 comes out as +-1.
///
/// Apply it before curveJoystick(), so the curve starts from a clean zero.
double applyDeadband(double input, double deadband);

} // namespace sapphirelib
