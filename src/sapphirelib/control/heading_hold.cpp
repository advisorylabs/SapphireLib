#include "sapphirelib/control/heading_hold.hpp"

#include <algorithm>
#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib {

namespace {

// same limit as PID::update(). Anything longer is a scheduling hiccup, not control time
constexpr double kMaxPlausibleDtS = 0.5;

// a deadband this wide leaves no usable stick travel, so clamp it instead of dividing by zero
constexpr double kMaxDeadband = 0.9;

// remove the deadband and rescale the rest to -1 to 1, so there's no jump at the edge
double applyDeadband(double input, double deadband) {
    deadband = std::clamp(deadband, 0.0, kMaxDeadband);

    const double magnitude = std::fabs(input);
    if (magnitude <= deadband) return 0.0;

    return std::copysign((magnitude - deadband) / (1.0 - deadband), input);
}

} // namespace

double advanceHeldHeadingDeg(double heldHeadingDeg, double currentHeadingDeg, double turnInput,
                             double dtS, HeadingHoldConfig config) {
    if (dtS > 0.0 && dtS <= kMaxPlausibleDtS) {
        const double rate = applyDeadband(turnInput, config.deadband) * config.slewDegPerSec;
        heldHeadingDeg += rate * dtS;
    }

    // keep the held heading within maxLeadDeg of the chassis, every tick, so a shove can't build
    // up lead either. wrapDegrees180() handles the 0/360 seam (5 vs 355 is 10 degrees of lead)
    if (config.maxLeadDeg > 0.0) {
        const double leadDeg = wrapDegrees180(heldHeadingDeg - currentHeadingDeg);
        heldHeadingDeg =
            currentHeadingDeg + std::clamp(leadDeg, -config.maxLeadDeg, config.maxLeadDeg);
    }

    return heldHeadingDeg;
}

} // namespace sapphirelib
