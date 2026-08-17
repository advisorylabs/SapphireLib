#include "sapphirelib/util/angle.hpp"

#include <cmath>

namespace sapphirelib {

double wrapDegrees180(double degrees) {
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped <= -180.0) wrapped += 360.0;
    if (wrapped > 180.0) wrapped -= 360.0;
    return wrapped;
}

} // namespace sapphirelib
