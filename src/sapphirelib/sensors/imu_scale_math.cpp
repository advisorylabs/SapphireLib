#include "sapphirelib/sensors/imu_scale_math.hpp"

#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::sensors {

double rawHeadingDeltaDeg(double lastRawHeadingDeg, double rawHeadingDeg) {
    return wrapDegrees180(rawHeadingDeg - lastRawHeadingDeg);
}

double wrapDegrees360(double degrees) {
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0) wrapped += 360.0;
    return wrapped;
}

double fieldHeadingDeg(double cumulativeDeg, double headingOffsetDeg) {
    return wrapDegrees360(cumulativeDeg + headingOffsetDeg);
}

double headingOffsetFor(double targetHeadingDeg, double cumulativeDeg) {
    return wrapDegrees180(targetHeadingDeg - cumulativeDeg);
}

double calibrateHeadingScale(double actualTurns, double measuredTurns) { return actualTurns / measuredTurns; }

} // namespace sapphirelib::sensors
