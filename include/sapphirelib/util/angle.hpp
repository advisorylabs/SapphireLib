#pragma once

namespace sapphirelib {

/**
 * @brief Wrap an angle to (-180, 180]
 *
 * Use this on a heading error before passing it to a PID, so a target of 5 and a measurement of
 * 355 gives an error of 10 degrees instead of -350
 *
 * @param degrees the angle to wrap, in degrees
 * @return double the equivalent angle in (-180, 180]
 *
 * @b Example
 * @code {.cpp}
 * // shortest signed error from 355 to 5 degrees
 * double error = sapphirelib::wrapDegrees180(5 - 355); // 10
 * @endcode
 */
double wrapDegrees180(double degrees);

} // namespace sapphirelib
