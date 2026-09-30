#include "sapphirelib/chassis/thermal_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace sapphirelib::chassis {

namespace {

struct DeratingPoint {
    double tempC;
    double fraction;
};

// the V5's derating steps (50% at 55C, 25% at 60C, 12.5% at 65C, off at 70C), each widened
// into a ~2C ramp so the curve is continuous. Must stay sorted by tempC
constexpr DeratingPoint kDeratingCurve[] = {
    {54.0, 1.0},
    {56.0, 0.5},
    {59.0, 0.5},
    {61.0, 0.25},
    {64.0, 0.25},
    {66.0, 0.125},
    {69.0, 0.125},
    {71.0, 0.0},
};

constexpr std::size_t kDeratingPointCount = sizeof(kDeratingCurve) / sizeof(kDeratingCurve[0]);

// what the corners miss is split between the two center wheels
constexpr double kCenterWheelCount = 2.0;

} // namespace

double thermalPowerFraction(double tempC) {
    // an unplugged motor reads infinity, and NaN would fall through every comparison below and
    // read as fully shut down
    if (!std::isfinite(tempC)) return 1.0;

    if (tempC <= kDeratingCurve[0].tempC) return kDeratingCurve[0].fraction;

    for (std::size_t i = 1; i < kDeratingPointCount; ++i) {
        const DeratingPoint& high = kDeratingCurve[i];
        if (tempC > high.tempC) continue;

        const DeratingPoint& low = kDeratingCurve[i - 1];
        const double span = high.tempC - low.tempC;
        const double t = (tempC - low.tempC) / span;
        return low.fraction + t * (high.fraction - low.fraction);
    }

    return kDeratingCurve[kDeratingPointCount - 1].fraction;
}

CenterCorrection centerThermalCorrection(CornerValues commandedVolts,
                                         CornerValues survivingFraction,
                                         double centerPowerFraction, double gain,
                                         double maxVolts) {
    if (gain <= 0.0 || maxVolts <= 0.0) return CenterCorrection{};

    // volts each corner was asked for and isn't producing
    const double lostFrontLeft = commandedVolts.frontLeft * (1.0 - survivingFraction.frontLeft);
    const double lostFrontRight = commandedVolts.frontRight * (1.0 - survivingFraction.frontRight);
    const double lostBackLeft = commandedVolts.backLeft * (1.0 - survivingFraction.backLeft);
    const double lostBackRight = commandedVolts.backRight * (1.0 - survivingFraction.backRight);

    // the mixer's forward and yaw patterns, so these are in the same units the center wheels
    // already carry. No strafe pattern: the center wheels can't push sideways
    const double forwardLost = lostFrontLeft + lostFrontRight + lostBackLeft + lostBackRight;
    const double yawLost = lostFrontLeft - lostFrontRight + lostBackLeft - lostBackRight;

    // cap before fading, so maxVolts means the same thing whatever the center wheels' temperature
    const double scale = gain / kCenterWheelCount;
    const double headroom = std::clamp(centerPowerFraction, 0.0, 1.0);

    return CenterCorrection{
        std::clamp(forwardLost * scale, -maxVolts, maxVolts) * headroom,
        std::clamp(yawLost * scale, -maxVolts, maxVolts) * headroom,
    };
}

} // namespace sapphirelib::chassis
