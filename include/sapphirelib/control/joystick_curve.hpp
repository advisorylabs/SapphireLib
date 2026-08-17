/**
 * \file sapphirelib/control/joystick_curve.hpp
 *
 * Cubic joystick curve for driver control: blends a linear response with a
 * cubic one so small stick deflections near center give finer control while
 * full deflection still reaches +-1.
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

} // namespace sapphirelib
