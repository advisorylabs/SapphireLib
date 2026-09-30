#include "sapphirelib/control/joystick_curve.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib {

double curveJoystick(double input, double curve) {
    curve = std::clamp(curve, 0.0, 1.0);
    return curve * input * input * input + (1.0 - curve) * input;
}

double applyDeadband(double input, double deadband) {
    if (deadband <= 0.0) return input;
    if (deadband >= 1.0) return 0.0;
    const double magnitude = std::fabs(input);
    if (magnitude <= deadband) return 0.0;
    // measured from the band's edge, so the output rises from 0 there instead of jumping
    return std::copysign((std::min(magnitude, 1.0) - deadband) / (1.0 - deadband), input);
}

} // namespace sapphirelib
