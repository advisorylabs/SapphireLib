#pragma once

namespace sapphirelib {

/**
 * @brief Apply a cubic curve to a joystick input
 *
 * Blends a linear and a cubic response, so small movements give finer control while full stick
 * still reaches 1. Keeps the sign, and always maps -1 to -1, 0 to 0, and 1 to 1
 *
 * @param input stick value, -1 to 1
 * @param curve how curved the response is, 0 (linear) to 1 (pure cubic). Clamped to that range
 * @return double the curved input
 *
 * @b Example
 * @code {.cpp}
 * double throttle = sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_Y) / 127.0, 0.5);
 * @endcode
 */
double curveJoystick(double input, double curve);

/**
 * @brief Apply a deadband to a joystick input
 *
 * Inputs inside the deadband become 0, and the rest is rescaled so the output still spans -1 to 1
 * with no jump at the edge. Apply it before curveJoystick()
 *
 * @param input stick value, -1 to 1. Values past +-1 come out as +-1
 * @param deadband the deadband, 0 to 1. 0 or less returns input unchanged, 1 or more returns 0
 * @return double the input with the deadband applied
 *
 * @b Example
 * @code {.cpp}
 * double turn = sapphirelib::applyDeadband(master.get_analog(ANALOG_RIGHT_X) / 127.0, 0.05);
 * turn = sapphirelib::curveJoystick(turn, 0.5);
 * @endcode
 */
double applyDeadband(double input, double deadband);

} // namespace sapphirelib
