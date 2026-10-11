#include "sapphirelib/chassis/orbit_math.hpp"

#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// a pivot ahead is the same orbit with front and back swapped: both translation axes mirror, and
// rotation doesn't
double direction(bool pivotAhead) { return pivotAhead ? -1.0 : 1.0; }

} // namespace

OrbitPivot pivotBehind(odom::Pose pose, double radiusIn, bool pivotAhead) {
    // heading 0 faces +y and 90 faces +x (see odom::Pose)
    const double headingRad = pose.headingDeg * kDegToRad;
    const double backIn = direction(pivotAhead) * radiusIn;
    return OrbitPivot{.xIn = pose.xIn - backIn * std::sin(headingRad),
                      .yIn = pose.yIn - backIn * std::cos(headingRad)};
}

OrbitFrame measureOrbit(OrbitPivot pivot, odom::Pose pose, bool pivotAhead, double minRadiusIn) {
    double dxIn = pose.xIn - pivot.xIn;
    double dyIn = pose.yIn - pivot.yIn;
    double radiusIn = std::hypot(dxIn, dyIn);
    if (radiusIn < minRadiusIn) {
        // push the pivot back along the line to the chassis; right on top of it, there's no line,
        // so it goes straight behind
        pivot = radiusIn > 1e-9 ? OrbitPivot{.xIn = pose.xIn - dxIn / radiusIn * minRadiusIn,
                                             .yIn = pose.yIn - dyIn / radiusIn * minRadiusIn}
                                : pivotBehind(pose, minRadiusIn, pivotAhead);
        dxIn = pose.xIn - pivot.xIn;
        dyIn = pose.yIn - pivot.yIn;
        radiusIn = minRadiusIn;
    }

    // the heading pointing straight away from the pivot, in the pose's convention
    const double awayDeg = std::atan2(dxIn, dyIn) * kRadToDeg;
    const double facingDeg = pivotAhead ? awayDeg + 180.0 : awayDeg;
    return OrbitFrame{.pivot = pivot,
                      .radiusIn = radiusIn,
                      .headingErrorDeg = wrapDegrees180(facingDeg - pose.headingDeg)};
}

OrbitTranslation orbitTranslation(double radial, double around, double headingErrorDeg,
                                  bool pivotAhead) {
    // the orbit's frame is the chassis's (with front and back swapped for a pivot ahead), turned
    // clockwise by the heading error
    const double errorRad = headingErrorDeg * kDegToRad;
    const double cosError = std::cos(errorRad);
    const double sinError = std::sin(errorRad);
    return OrbitTranslation{
        .forward = direction(pivotAhead) * (radial * cosError - around * sinError),
        .strafe = direction(pivotAhead) * (around * cosError + radial * sinError),
    };
}

} // namespace sapphirelib::chassis
