#include "sapphirelib/odom/odometry_math.hpp"

#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::odom {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

} // namespace

PoseDelta computeOdometryDelta(double lastHeadingDeg, double headingDeg, double verticalDeltaIn,
                                double horizontalDeltaIn, double verticalOffsetIn,
                                double horizontalOffsetIn) {
    const double dThetaDeg = wrapDegrees180(headingDeg - lastHeadingDeg);
    const double dThetaRad = dThetaDeg * kDegToRad;

    // remove each wheel's arc from turning, leaving its real movement
    const double localForwardIn = verticalDeltaIn - verticalOffsetIn * dThetaRad;
    const double localLateralIn = horizontalDeltaIn - horizontalOffsetIn * dThetaRad;

    // rotate the local movement onto the field using the average heading over the update
    const double avgHeadingRad = (lastHeadingDeg + dThetaDeg / 2.0) * kDegToRad;
    const double cosHeading = std::cos(avgHeadingRad);
    const double sinHeading = std::sin(avgHeadingRad);

    return PoseDelta{
        .dxIn = localForwardIn * sinHeading + localLateralIn * cosHeading,
        .dyIn = localForwardIn * cosHeading - localLateralIn * sinHeading,
    };
}

double calibrateTrackingWheelOffsetIn(double wheelDistanceIn, double rotatedRadians) {
    return wheelDistanceIn / rotatedRadians;
}

} // namespace sapphirelib::odom
