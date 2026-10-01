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

        // only the exit root, t2, can be a forward target; t1 would point back along the path
        const double t2 = discriminant < 0.0 ? -1.0 : (-b + std::sqrt(discriminant)) / (2.0 * a);
        if (t2 < 0.0 || t2 > 1.0) {
            // before the first hit, this segment lies inside the circle: keep looking. After one,
            // the path has left the circle, and a later segment that comes back within reach (a
            // closed lap's last side, passing beside its first) is for later in the path
            if (found) break;
            continue;
        }

        // keep scanning after a hit, since the next segment's intersection is further along
        found = true;
        bestPoint = Waypoint{p1.x + d.x * t2, p1.y + d.y * t2};
        bestIndex = i;
    }

    if (!found) {
        // the whole rest of the path is within the lookahead, so head for the last waypoint
        return LookaheadResult{points.back(), points.size() - 1};
    }
    return LookaheadResult{bestPoint, bestIndex};
}

bool reachedFinalApproach(double distanceToFinalIn, double finalApproachIn,
                          std::size_t segmentIndex, std::size_t waypointCount) {
    // the last segment runs from waypoint count - 2 to count - 1; findLookaheadPoint() reports
    // count - 1 once the whole rest of the path is within the lookahead. Written as a sum so a
    // one-waypoint path (no segments at all) counts as already there
    const bool onLastSegment = segmentIndex + 2 >= waypointCount;
    return onLastSegment && distanceToFinalIn <= finalApproachIn;
}

} // namespace sapphirelib::motion
