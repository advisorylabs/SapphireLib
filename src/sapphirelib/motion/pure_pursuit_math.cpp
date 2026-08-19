#include "sapphirelib/motion/pure_pursuit_math.hpp"

#include <cmath>

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

struct Vec2 {
    double x;
    double y;
};

Vec2 sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

} // namespace

LocalOffset toLocalFrame(double dxIn, double dyIn, double headingDeg) {
    const double headingRad = headingDeg * kDegToRad;
    return LocalOffset{
        .forwardIn = dxIn * std::sin(headingRad) + dyIn * std::cos(headingRad),
        .lateralIn = dxIn * std::cos(headingRad) - dyIn * std::sin(headingRad),
    };
}

LookaheadResult findLookaheadPoint(double xIn, double yIn, const Path& path, double lookaheadIn,
                                    std::size_t fromIndex) {
    const std::vector<Waypoint>& points = path.waypoints();
    const Vec2 center{xIn, yIn};

    bool found = false;
    Waypoint bestPoint{};
    std::size_t bestIndex = fromIndex;

    const std::size_t startIndex = fromIndex < points.size() ? fromIndex : points.size() - 1;
    for (std::size_t i = startIndex; i + 1 < points.size(); ++i) {
        const Vec2 p1{points[i].xIn, points[i].yIn};
        const Vec2 p2{points[i + 1].xIn, points[i + 1].yIn};
        const Vec2 d = sub(p2, p1);
        const Vec2 f = sub(p1, center);

        const double a = dot(d, d);
        if (a < 1e-9) continue; // degenerate (duplicate) waypoint pair

        const double b = 2.0 * dot(f, d);
        const double c = dot(f, f) - lookaheadIn * lookaheadIn;
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) continue;

        // Only the further ("exit") root, t2, can ever be a valid forward
        // pursuit target — t1 ("entry") is always the same or an earlier
        // point along the segment, so on a segment where the chassis is
        // already past the entry point (or the segment ends before exiting
        // the circle, so t2 falls outside [0,1]), t1 would send the target
        // backward along the path instead of ahead of the chassis. If t2
        // isn't on this segment, keep scanning later segments for the
        // actual exit point instead of settling for t1.
        const double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);
        if (t2 < 0.0 || t2 > 1.0) continue;

        // Keep scanning later segments even after finding a hit — a later
        // segment's intersection is further along the path and should win.
        found = true;
        bestPoint = Waypoint{p1.x + d.x * t2, p1.y + d.y * t2};
        bestIndex = i;
    }

    if (!found) {
        // The whole remaining path is within the lookahead radius (or
        // there's only one point left) — converge on the final waypoint.
        return LookaheadResult{points.back(), points.size() - 1};
    }
    return LookaheadResult{bestPoint, bestIndex};
}

} // namespace sapphirelib::motion
