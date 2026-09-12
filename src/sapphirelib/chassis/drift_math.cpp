#include "sapphirelib/chassis/drift_math.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

} // namespace

double strafeDriftIn(double verticalWheelDeltaIn, double verticalOffsetIn, double rotationDeltaDeg,
                     double strafeTravelIn, double throttleVolts, double strafeVolts) {
    if (strafeVolts == 0.0) return 0.0;

    const double translationForwardIn =
        verticalWheelDeltaIn - verticalOffsetIn * rotationDeltaDeg * kDegToRad;
    const double commandedForwardIn = strafeTravelIn * (throttleVolts / strafeVolts);
    return translationForwardIn - commandedForwardIn;
}

} // namespace sapphirelib::chassis
