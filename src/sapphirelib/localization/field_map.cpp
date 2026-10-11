#include "sapphirelib/localization/field_map.hpp"

#include <algorithm>
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

void ParallelRayCaster::aim(const FieldMap& map, double dirX, double dirY) {
    dirX_ = dirX;
    dirY_ = dirY;
    minXIn_ = map.minXIn();
    minYIn_ = map.minYIn();
    maxXIn_ = map.maxXIn();
    maxYIn_ = map.maxYIn();
    // castRayIn() skips a wall whose cross product with the ray is below kParallelEpsilon; so does
    // the shortcut, with the same products
    const bool reachesX = std::fabs(dirX * (maxYIn_ - minYIn_)) >= kParallelEpsilon;
    const bool reachesY = std::fabs(dirY * (maxXIn_ - minXIn_)) >= kParallelEpsilon;
    wallXIn_ = dirX > 0.0 ? maxXIn_ : minXIn_;
    wallYIn_ = dirY > 0.0 ? maxYIn_ : minYIn_;
    inverseDirX_ = reachesX ? 1.0 / dirX : 0.0;
    inverseDirY_ = reachesY ? 1.0 / dirY : 0.0;

    edges_.clear();
    perimeterEdges_ = 0;
    const std::vector<Segment>& segments = map.segments();
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const Segment& s = segments[i];
        const double ex = s.x2In - s.x1In;
        const double ey = s.y2In - s.y1In;
        const double denom = dirX * ey - dirY * ex;
        // parallel to every ray, so never hit, as in castRayIn()
        if (std::fabs(denom) < kParallelEpsilon) continue;
        // castRayIn()'s numerators, (p1 - start) x e and (p1 - start) x dir, with the p1 parts
        // worked out here: the start's parts are the only ones left per ray
        edges_.push_back(Edge{.ex = ex,
                              .ey = ey,
                              .inverseDenominator = 1.0 / denom,
                              .crossT = s.x1In * ey - s.y1In * ex,
                              .crossU = s.x1In * dirY - s.y1In * dirX});
        // the walls are always the first four segments
        if (i < 4) ++perimeterEdges_;
    }
}

double ParallelRayCaster::castIn(double xIn, double yIn) const {
    double nearest = std::numeric_limits<double>::infinity();
    std::size_t first = 0;
    if (xIn >= minXIn_ && xIn <= maxXIn_ && yIn >= minYIn_ && yIn <= maxYIn_) {
        // inside the walls: out through the nearer of the two ahead
        if (inverseDirX_ != 0.0) nearest = (wallXIn_ - xIn) * inverseDirX_;
        if (inverseDirY_ != 0.0) nearest = std::min(nearest, (wallYIn_ - yIn) * inverseDirY_);
        first = perimeterEdges_;
    }
    const double startCrossDir = xIn * dirY_ - yIn * dirX_;
    for (std::size_t i = first; i < edges_.size(); ++i) {
        const Edge& e = edges_[i];
        const double t = (e.crossT - (xIn * e.ey - yIn * e.ex)) * e.inverseDenominator;
        const double u = (e.crossU - startCrossDir) * e.inverseDenominator;
        if (t >= 0.0 && u >= 0.0 && u <= 1.0 && t < nearest) nearest = t;
    }
    return nearest;
}

} // namespace sapphirelib::localization
