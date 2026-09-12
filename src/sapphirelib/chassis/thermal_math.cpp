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

// The V5's published derating steps (50% at 55C, 25% at 60C, 12.5% at 65C,
// shutdown at 70C), each widened into a ~2C ramp so the curve is continuous
// — see thermalPowerFraction()'s comment for why that matters more here than
// matching the steps exactly. Must stay sorted by tempC.
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

// Whatever the corners fail to deliver is split between the two center
// wheels.
constexpr double kCenterWheelCount = 2.0;

} // namespace

double thermalPowerFraction(double tempC) {
    // Covers PROS_ERR_F (infinity) from a motor that isn't answering, and
    // NaN, which would otherwise fall through every comparison below and
    // come back as the last point's fraction — i.e. "fully shut down".
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

    // Volts' worth of thrust each corner was asked for and isn't producing.
    const double lostFrontLeft = commandedVolts.frontLeft * (1.0 - survivingFraction.frontLeft);
    const double lostFrontRight = commandedVolts.frontRight * (1.0 - survivingFraction.frontRight);
    const double lostBackLeft = commandedVolts.backLeft * (1.0 - survivingFraction.backLeft);
    const double lostBackRight = commandedVolts.backRight * (1.0 - survivingFraction.backRight);

    // Same sign patterns the mixer uses to recover throttle and turn from a
    // set of corner voltages, so these read out in the same units as the
    // throttle/turn terms the center wheels already carry. Strafe has no
    // pattern here on purpose: the center wheels face fore/aft and can't
    // push sideways, so a sideways shortfall isn't theirs to fix.
    const double forwardLost = lostFrontLeft + lostFrontRight + lostBackLeft + lostBackRight;
    const double yawLost = lostFrontLeft - lostFrontRight + lostBackLeft - lostBackRight;

    // Capped before the fade, so maxVolts stays a clean statement about what
    // a healthy center wheel will be asked for, and a derating one is scaled
    // down from that rather than measured against a different ceiling.
    const double scale = gain / kCenterWheelCount;
    const double headroom = std::clamp(centerPowerFraction, 0.0, 1.0);

    return CenterCorrection{
        std::clamp(forwardLost * scale, -maxVolts, maxVolts) * headroom,
        std::clamp(yawLost * scale, -maxVolts, maxVolts) * headroom,
    };
}

} // namespace sapphirelib::chassis
