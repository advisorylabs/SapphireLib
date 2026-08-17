#include "sapphirelib/control/joystick_curve.hpp"

#include <algorithm>

namespace sapphirelib {

double curveJoystick(double input, double curve) {
    curve = std::clamp(curve, 0.0, 1.0);
    return curve * input * input * input + (1.0 - curve) * input;
}

} // namespace sapphirelib
