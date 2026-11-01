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

// the walls are always the first four segments (see the constructor)
constexpr std::size_t kPerimeterSegments = 4;

// a cone wider than this either side is taken as this, so its slope stays finite
constexpr double kMaxConeHalfAngleDeg = 80.0;

// A face turned more than 60 degrees from the beam is left out of the cone: it sends too little of
// the beam back to be what the sensor reads. The sides of a loader seen from beside it run along
// the beam, and the sensor read past them to the wall (96671H's SL000034), while it read the face
// toward it 13 degrees off center (SL000028). This is cos(60 degrees)
constexpr double kMinConeFacing = 0.5;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// a cone's slope: inside it, |across| <= slope * along. 0 for a cone that sees nothing
double coneSlope(double halfAngleDeg) {
    if (!(halfAngleDeg > 0.0)) return 0.0;
    return std::tan(std::min(halfAngleDeg, kMaxConeHalfAngleDeg) * kDegToRad);
}

// The squared distance from a cone's tip to the nearest point of a segment inside the cone, with
// the segment's ends (a1, c1) and (a2, c2) along the cone and across it from the tip. Infinity
// when no part of it is inside. The cone is two half-planes, slope * along - across >= 0 and
// slope * along + across >= 0, which make |across| <= slope * along, and so along >= 0 too.
// Each is linear along the segment, so clipping to it keeps one stretch of it; what's left of
// the segment is nearest the tip where the whole line is, held to that stretch
double nearestInConeSquared(double a1, double c1, double a2, double c2, double slope,
                            double inverseLengthSquared) {
    double t0 = 0.0;
    double t1 = 1.0;
    for (const double side : {1.0, -1.0}) {
        const double f1 = slope * a1 - side * c1;
        const double f2 = slope * a2 - side * c2;
        if (f1 < 0.0 && f2 < 0.0) return kInfinity;
        if (f1 < 0.0) {
            t0 = std::max(t0, f1 / (f1 - f2));
        } else if (f2 < 0.0) {
            t1 = std::min(t1, f1 / (f1 - f2));
        }
    }
    if (t0 > t1) return kInfinity;
    const double da = a2 - a1;
    const double dc = c2 - c1;
    const double t = std::clamp(-(a1 * da + c1 * dc) * inverseLengthSquared, t0, t1);
    const double a = a1 + t * da;
    const double c = c1 + t * dc;
    return a * a + c * c;
}

// 1 / a segment's length squared, or 0 for a point
double inverseLengthSquared(const Segment& s) {
    const double dx = s.x2In - s.x1In;
    const double dy = s.y2In - s.y1In;
    const double squared = dx * dx + dy * dy;
    return squared > 0.0 ? 1.0 / squared : 0.0;
}

// whether a segment faces a beam pointing (dirX, dirY) closely enough to be seen in its cone (see
// kMinConeFacing): the cosine between the beam and the segment's normal, which is the sine
// between the beam and the segment. A point faces no way, and isn't seen
bool facesBeam(const Segment& s, double dirX, double dirY) {
    const double cross = dirX * (s.y2In - s.y1In) - dirY * (s.x2In - s.x1In);
    return std::fabs(cross) * std::sqrt(inverseLengthSquared(s)) >= kMinConeFacing;
}

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

double FieldMap::castConeIn(double xIn, double yIn, double dirX, double dirY,
                            double halfAngleDeg) const {
    const double slope = coneSlope(halfAngleDeg);
    if (!(slope > 0.0)) return kInfinity;
    double nearestSquared = kInfinity;
    for (std::size_t i = kPerimeterSegments; i < segments_.size(); ++i) {
        const Segment& s = segments_[i];
        if (!facesBeam(s, dirX, dirY)) continue;
        // each end along the cone, and across it (to the cone's left)
        const double x1 = s.x1In - xIn;
        const double y1 = s.y1In - yIn;
        const double x2 = s.x2In - xIn;
        const double y2 = s.y2In - yIn;
        nearestSquared = std::min(
            nearestSquared,
            nearestInConeSquared(x1 * dirX + y1 * dirY, y1 * dirX - x1 * dirY, x2 * dirX + y2 * dirY,
                                 y2 * dirX - x2 * dirY, slope, inverseLengthSquared(s)));
    }
    return std::sqrt(nearestSquared);
}

double FieldMap::castBeamIn(double xIn, double yIn, double dirX, double dirY,
                            double halfAngleDeg) const {
    return std::min(castRayIn(xIn, yIn, dirX, dirY),
                    castConeIn(xIn, yIn, dirX, dirY, halfAngleDeg));
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

void ConeCaster::aim(const FieldMap& map, double dirX, double dirY, double halfAngleDeg) {
    dirX_ = dirX;
    dirY_ = dirY;
    slope_ = coneSlope(halfAngleDeg);
    slopeNorm_ = std::sqrt(1.0 + slope_ * slope_);
    edges_.clear();
    if (!(slope_ > 0.0)) return;
    const std::vector<Segment>& segments = map.segments();
    for (std::size_t i = kPerimeterSegments; i < segments.size(); ++i) {
        const Segment& s = segments[i];
        if (!facesBeam(s, dirX, dirY)) continue;
        // castConeIn()'s along and across, from the origin: the tip's are the only parts left per
        // cone
        const double middleX = 0.5 * (s.x1In + s.x2In);
        const double middleY = 0.5 * (s.y1In + s.y2In);
        edges_.push_back(Edge{.along1 = s.x1In * dirX + s.y1In * dirY,
                              .across1 = s.y1In * dirX - s.x1In * dirY,
                              .along2 = s.x2In * dirX + s.y2In * dirY,
                              .across2 = s.y2In * dirX - s.x2In * dirY,
                              .alongMiddle = middleX * dirX + middleY * dirY,
                              .acrossMiddle = middleY * dirX - middleX * dirY,
                              .halfLength = 0.5 * std::hypot(s.x2In - s.x1In, s.y2In - s.y1In),
                              .inverseLengthSquared = inverseLengthSquared(s)});
    }
}

double ConeCaster::castIn(double xIn, double yIn) const {
    const double tipAlong = xIn * dirX_ + yIn * dirY_;
    const double tipAcross = yIn * dirX_ - xIn * dirY_;
    double nearestSquared = kInfinity;
    for (const Edge& e : edges_) {
        // the segment lies within halfLength of its middle: passed over if that circle is wholly
        // outside the cone, |across| - slope * along being how far outside it a point is, in
        // units of 1 / slopeNorm_
        const double middleAlong = e.alongMiddle - tipAlong;
        const double middleAcross = e.acrossMiddle - tipAcross;
        if (std::fabs(middleAcross) - slope_ * middleAlong > e.halfLength * slopeNorm_) continue;
        nearestSquared = std::min(
            nearestSquared, nearestInConeSquared(e.along1 - tipAlong, e.across1 - tipAcross,
                                                 e.along2 - tipAlong, e.across2 - tipAcross, slope_,
                                                 e.inverseLengthSquared));
    }
    return std::sqrt(nearestSquared);
}

} // namespace sapphirelib::localization
