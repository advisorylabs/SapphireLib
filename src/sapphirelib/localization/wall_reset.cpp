#include "sapphirelib/localization/wall_reset.hpp"

#include <cmath>
#include <limits>

namespace sapphirelib::localization {

namespace {

constexpr double kPi = 3.14159265358979323846;
// a beam this close to parallel with a pair of walls never reaches them
constexpr double kParallel = 1e-9;

// how far along a beam, from position p moving at dir per inch, until it reaches the wall at lo
// or hi it's heading for: infinity if it runs parallel to them
double toWallIn(double p, double dir, double lo, double hi) {
    if (dir > kParallel) return (hi - p) / dir;
    if (dir < -kParallel) return (lo - p) / dir;
    return std::numeric_limits<double>::infinity();
}

} // namespace

WallFix wallFix(const FieldMap& map, double xIn, double yIn, double headingDeg,
                const DistanceSensorMount& mount, double readingIn, const WallResetConfig& config) {
    if (!std::isfinite(readingIn) || readingIn < 0.0) return WallFix{.why = "no reading"};

    const SensorRay ray = sensorRay(xIn, yIn, headingDeg, mount);
    if (!map.contains(ray.xIn, ray.yIn)) return WallFix{.why = "sensor outside the field"};

    // the wall the beam reaches first, from where the robot thinks it is
    const double toXWallIn = toWallIn(ray.xIn, ray.dirX, map.minXIn(), map.maxXIn());
    const double toYWallIn = toWallIn(ray.yIn, ray.dirY, map.minYIn(), map.maxYIn());
    const bool xWall = toXWallIn < toYWallIn;
    const double toWall = xWall ? toXWallIn : toYWallIn;
    if (!std::isfinite(toWall)) return WallFix{.why = "no wall ahead"};

    // off square: the beam's share across the wall is the cosine of the angle from its normal
    const double across = std::fabs(xWall ? ray.dirX : ray.dirY);
    if (across < std::cos(config.maxAngleDeg * kPi / 180.0)) return WallFix{.why = "too steep"};

    // where along the wall the beam lands, against the corners at either end
    const double along = xWall ? ray.yIn + toWall * ray.dirY : ray.xIn + toWall * ray.dirX;
    const double lo = xWall ? map.minYIn() : map.minXIn();
    const double hi = xWall ? map.maxYIn() : map.maxXIn();
    if (along < lo + config.cornerMarginIn || along > hi - config.cornerMarginIn) {
        return WallFix{.why = "near a corner"};
    }

    // the sensor sits the reading back from the wall along the beam, and the robot moves with it
    const double dir = xWall ? ray.dirX : ray.dirY;
    const double wall = xWall ? (dir > 0.0 ? map.maxXIn() : map.minXIn())
                              : (dir > 0.0 ? map.maxYIn() : map.minYIn());
    const double sensorNow = xWall ? ray.xIn : ray.yIn;
    const double shiftIn = (wall - readingIn * dir) - sensorNow;
    if (config.maxJumpIn > 0.0 && std::fabs(shiftIn) > config.maxJumpIn) {
        return WallFix{.why = "too far from odometry"};
    }

    return WallFix{.ok = true,
                   .axis = xWall ? WallAxis::x : WallAxis::y,
                   .valueIn = (xWall ? xIn : yIn) + shiftIn,
                   .shiftIn = shiftIn};
}

} // namespace sapphirelib::localization
