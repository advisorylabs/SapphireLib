#include "sapphirelib/localization/field_map.hpp"

#include <cmath>
#include <limits>

namespace sapphirelib::localization {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// below this the ray and segment count as parallel, and the segment is skipped
constexpr double kParallelEpsilon = 1e-12;

} // namespace

FieldMap::FieldMap(double minXIn, double minYIn, double maxXIn, double maxYIn)
    : minXIn_(minXIn), minYIn_(minYIn), maxXIn_(maxXIn), maxYIn_(maxYIn) {
    // the perimeter, always the first four segments
    segments_.push_back({minXIn, minYIn, maxXIn, minYIn});
    segments_.push_back({maxXIn, minYIn, maxXIn, maxYIn});
    segments_.push_back({maxXIn, maxYIn, minXIn, maxYIn});
    segments_.push_back({minXIn, maxYIn, minXIn, minYIn});
}

FieldMap FieldMap::centered(double sizeIn) {
    const double half = sizeIn / 2.0;
    return FieldMap(-half, -half, half, half);
}

void FieldMap::addSegment(Segment segment) { segments_.push_back(segment); }

void FieldMap::addBox(double minXIn, double minYIn, double maxXIn, double maxYIn) {
    segments_.push_back({minXIn, minYIn, maxXIn, minYIn});
    segments_.push_back({maxXIn, minYIn, maxXIn, maxYIn});
    segments_.push_back({maxXIn, maxYIn, minXIn, maxYIn});
    segments_.push_back({minXIn, maxYIn, minXIn, minYIn});
}

double FieldMap::castRayIn(double xIn, double yIn, double dirX, double dirY) const {
    double nearest = std::numeric_limits<double>::infinity();
    for (const Segment& s : segments_) {
        // solve start + t * dir = p1 + u * (p2 - p1) with 2D cross products
        const double ex = s.x2In - s.x1In;
        const double ey = s.y2In - s.y1In;
        const double denom = dirX * ey - dirY * ex;
        if (std::fabs(denom) < kParallelEpsilon) continue;

        const double wx = s.x1In - xIn;
        const double wy = s.y1In - yIn;
        const double t = (wx * ey - wy * ex) / denom;
        const double u = (wx * dirY - wy * dirX) / denom;
        if (t >= 0.0 && u >= 0.0 && u <= 1.0 && t < nearest) nearest = t;
    }
    return nearest;
}

double FieldMap::castRayAtHeadingIn(double xIn, double yIn, double headingDeg) const {
    const double headingRad = headingDeg * kDegToRad;
    return castRayIn(xIn, yIn, std::sin(headingRad), std::cos(headingRad));
}

bool FieldMap::contains(double xIn, double yIn) const {
    return xIn >= minXIn_ && xIn <= maxXIn_ && yIn >= minYIn_ && yIn <= maxYIn_;
}

const std::vector<Segment>& FieldMap::segments() const { return segments_; }

double FieldMap::minXIn() const { return minXIn_; }

double FieldMap::minYIn() const { return minYIn_; }

double FieldMap::maxXIn() const { return maxXIn_; }

double FieldMap::maxYIn() const { return maxYIn_; }

} // namespace sapphirelib::localization
