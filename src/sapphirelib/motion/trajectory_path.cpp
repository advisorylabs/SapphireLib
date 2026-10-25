#include "sapphirelib/motion/trajectory_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

// waypoints closer than this are the same spot
constexpr double kMinChordIn = 1e-6;

// step of the polyline that measures arc length, in inches of chord. Fine enough that the
// polyline's length is the curve's to a few parts in a million
constexpr double kArcStepIn = 0.05;

// one natural cubic spline through a run of waypoints, parameterized by chord length
struct Piece {
    std::vector<double> t;  // knots: cumulative chord length
    std::vector<double> x;  // waypoint x
    std::vector<double> y;  // waypoint y
    std::vector<double> mx; // second derivatives at the knots
    std::vector<double> my;
    std::size_t firstWaypoint; // global index of the piece's first waypoint
};

struct CurvePoint {
    double x, y, dx, dy, ddx, ddy;
};

// spline second derivatives by the Thomas algorithm: zero at the end, and at the start too
// (natural) unless startSlope clamps the first derivative there
std::vector<double> secondDerivatives(const std::vector<double>& t,
                                      const std::vector<double>& values,
                                      std::optional<double> startSlope = std::nullopt) {
    const std::size_t n = t.size() - 1;
    std::vector<double> m(n + 1, 0.0);
    // the unknowns are m[first] to m[n - 1]
    const std::size_t first = startSlope ? 0 : 1;
    if (n < first + 1) return m;
    std::vector<double> diag(n + 1, 0.0);
    std::vector<double> rhs(n + 1, 0.0);
    if (startSlope) {
        // the clamped end's row: 2h*m[0] + h*m[1] = 6((v1 - v0)/h - slope)
        const double h1 = t[1] - t[0];
        diag[0] = 2.0 * h1;
        rhs[0] = 6.0 * ((values[1] - values[0]) / h1 - *startSlope);
    }
    for (std::size_t i = 1; i < n; ++i) {
        const double h0 = t[i] - t[i - 1];
        const double h1 = t[i + 1] - t[i];
        diag[i] = 2.0 * (h0 + h1);
        rhs[i] = 6.0 * ((values[i + 1] - values[i]) / h1 - (values[i] - values[i - 1]) / h0);
    }
    // forward elimination: row i is h0*m[i-1] + diag[i]*m[i] + h1*m[i+1] = rhs[i]
    for (std::size_t i = first + 1; i < n; ++i) {
        const double h0 = t[i] - t[i - 1];
        const double factor = h0 / diag[i - 1];
        diag[i] -= factor * h0;
        rhs[i] -= factor * rhs[i - 1];
    }
    for (std::size_t i = n - 1;; --i) {
        const double h1 = t[i + 1] - t[i];
        m[i] = (rhs[i] - h1 * m[i + 1]) / diag[i];
        if (i == first) break;
    }
    return m;
}

// the spline and its first two derivatives (by chord length) at t, on segment i
CurvePoint evaluate(const Piece& piece, std::size_t i, double t) {
    const double h = piece.t[i + 1] - piece.t[i];
    const double u = t - piece.t[i];
    const double w = piece.t[i + 1] - t;
    const auto axis = [&](const std::vector<double>& v, const std::vector<double>& m, double& f,
                          double& df, double& ddf) {
        const double a = v[i] / h - m[i] * h / 6.0;
        const double b = v[i + 1] / h - m[i + 1] * h / 6.0;
        f = m[i] * w * w * w / (6.0 * h) + m[i + 1] * u * u * u / (6.0 * h) + a * w + b * u;
        df = -m[i] * w * w / (2.0 * h) + m[i + 1] * u * u / (2.0 * h) - a + b;
        ddf = m[i] * w / h + m[i + 1] * u / h;
    };
    CurvePoint p{};
    axis(piece.x, piece.mx, p.x, p.dx, p.ddx);
    axis(piece.y, piece.my, p.y, p.dy, p.ddy);
    return p;
}

// a heading keyframe: where along the path, and the heading there, unwrapped
struct Keyframe {
    double sIn;
    double headingDeg;
};

// smootherstep easing between keyframes: heading, and its first two derivatives by arc length
void headingAt(const std::vector<Keyframe>& keys, double sIn, PathSample& sample) {
    sample.headingRateDegPerIn = 0.0;
    sample.headingAccelDegPerIn2 = 0.0;
    if (sIn >= keys.back().sIn) {
        sample.headingDeg = keys.back().headingDeg;
        return;
    }
    std::size_t k = 0;
    while (k + 1 < keys.size() && keys[k + 1].sIn <= sIn) ++k;
    const Keyframe& a = keys[k];
    const Keyframe& b = keys[k + 1];
    const double span = b.sIn - a.sIn;
    const double delta = b.headingDeg - a.headingDeg;
    const double u = (sIn - a.sIn) / span;
    const double u2 = u * u;
    const double u3 = u2 * u;
    sample.headingDeg = a.headingDeg + delta * (6.0 * u2 * u3 - 15.0 * u2 * u2 + 10.0 * u3);
    sample.headingRateDegPerIn = delta * (30.0 * u2 * u2 - 60.0 * u3 + 30.0 * u2) / span;
    sample.headingAccelDegPerIn2 = delta * (120.0 * u3 - 180.0 * u2 + 60.0 * u) / (span * span);
}

SampledPath failure(const char* format, std::size_t a = 0, std::size_t b = 0) {
    char text[128];
    std::snprintf(text, sizeof(text), format, static_cast<unsigned>(a), static_cast<unsigned>(b));
    SampledPath result;
    result.error = text;
    return result;
}

} // namespace

SampledPath samplePath(const std::vector<TrajectorySection>& sections, const PathEnds& ends) {
    // flatten the chain: every waypoint, the section each segment belongs to (segment j runs from
    // waypoint j to j + 1), and where the robot stops
    std::vector<TrajectoryWaypoint> waypoints;
    std::vector<std::size_t> segmentSection;
    std::vector<bool> stopAt;
    // the waypoint each section after the first starts from: the previous one's last
    std::vector<std::size_t> sectionStartWaypoint;
    for (std::size_t k = 0; k < sections.size(); ++k) {
        if (sections[k].waypoints.empty()) return failure("section %u has no waypoints", k);
        sectionStartWaypoint.push_back(waypoints.empty() ? 0 : waypoints.size() - 1);
        for (const TrajectoryWaypoint& waypoint : sections[k].waypoints) {
            if (!waypoints.empty()) segmentSection.push_back(k);
            waypoints.push_back(waypoint);
            stopAt.push_back(false);
        }
        if (sections[k].stopAtEnd) stopAt.back() = true;
    }
    if (waypoints.size() < 2) return failure("a path needs at least 2 waypoints");
    if (!waypoints.front().headingDeg) {
        return failure("the first waypoint needs a heading (where the robot starts)");
    }
    for (std::size_t j = 0; j + 1 < waypoints.size(); ++j) {
        const double chord = std::hypot(waypoints[j + 1].xIn - waypoints[j].xIn,
                                        waypoints[j + 1].yIn - waypoints[j].yIn);
        if (chord < kMinChordIn) {
            return failure("waypoints %u and %u are at the same spot", j, j + 1);
        }
    }
    stopAt.front() = true;
    stopAt.back() = true;

    // one spline per run between stops
    std::vector<Piece> pieces;
    std::size_t start = 0;
    for (std::size_t j = 1; j < waypoints.size(); ++j) {
        if (!stopAt[j]) continue;
        Piece piece;
        piece.firstWaypoint = start;
        double t = 0.0;
        for (std::size_t i = start; i <= j; ++i) {
            if (i > start) {
                t += std::hypot(waypoints[i].xIn - waypoints[i - 1].xIn,
                                waypoints[i].yIn - waypoints[i - 1].yIn);
            }
            piece.t.push_back(t);
            piece.x.push_back(waypoints[i].xIn);
            piece.y.push_back(waypoints[i].yIn);
        }
        // already moving: the first piece leaves along the direction of travel, a unit vector
        // since the spline is parameterized by chord length
        std::optional<double> slopeX;
        std::optional<double> slopeY;
        if (start == 0 && ends.startTangentDeg) {
            slopeX = std::sin(*ends.startTangentDeg / kRadToDeg);
            slopeY = std::cos(*ends.startTangentDeg / kRadToDeg);
        }
        piece.mx = secondDerivatives(piece.t, piece.x, slopeX);
        piece.my = secondDerivatives(piece.t, piece.y, slopeY);
        pieces.push_back(std::move(piece));
        start = j;
    }

    // arc length: a fine polyline over each segment, which also gives each waypoint's distance
    // along the path for the heading keyframes
    struct ArcTable {
        std::vector<double> t;
        std::vector<double> s;
        std::vector<std::size_t> segment; // local segment of each fine step's end
    };
    std::vector<ArcTable> tables(pieces.size());
    std::vector<double> waypointS(waypoints.size(), 0.0);
    double pathS = 0.0;
    for (std::size_t p = 0; p < pieces.size(); ++p) {
        const Piece& piece = pieces[p];
        ArcTable& table = tables[p];
        table.t.push_back(0.0);
        table.s.push_back(pathS);
        table.segment.push_back(0);
        CurvePoint last = evaluate(piece, 0, 0.0);
        for (std::size_t i = 0; i + 1 < piece.t.size(); ++i) {
            const double h = piece.t[i + 1] - piece.t[i];
            const std::size_t steps =
                std::max<std::size_t>(8, static_cast<std::size_t>(std::ceil(h / kArcStepIn)));
            for (std::size_t k = 1; k <= steps; ++k) {
                const double t = k == steps ? piece.t[i + 1] : piece.t[i] + h * k / steps;
                const CurvePoint point = evaluate(piece, i, t);
                pathS += std::hypot(point.x - last.x, point.y - last.y);
                last = point;
                table.t.push_back(t);
                table.s.push_back(pathS);
                table.segment.push_back(i);
            }
            waypointS[piece.firstWaypoint + i + 1] = pathS;
        }
    }

    std::vector<Keyframe> keys;
    for (std::size_t j = 0; j < waypoints.size(); ++j) {
        if (!waypoints[j].headingDeg) continue;
        const double heading = *waypoints[j].headingDeg;
        // unwrapped, so each turn goes the shortest way from the last keyframe
        keys.push_back({waypointS[j], keys.empty()
                                          ? heading
                                          : keys.back().headingDeg +
                                                wrapDegrees180(heading - keys.back().headingDeg)});
    }

    SampledPath result;
    result.lengthIn = pathS;
    for (std::size_t waypoint : sectionStartWaypoint) {
        result.sectionStartIn.push_back(waypointS[waypoint]);
    }
    result.waypointIn = std::move(waypointS);
    for (std::size_t p = 0; p < pieces.size(); ++p) {
        const Piece& piece = pieces[p];
        const ArcTable& table = tables[p];
        const double s0 = table.s.front();
        const double length = table.s.back() - s0;
        // less a hair, so rounding in the length can't add a sample to an exact multiple
        const std::size_t count = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::ceil(length / kTrajectorySampleSpacingIn - 1e-9)));
        std::size_t fine = 1;
        for (std::size_t k = 0; k <= count; ++k) {
            const double sIn = k == count ? table.s.back() : s0 + length * k / count;
            while (fine + 1 < table.s.size() && table.s[fine] < sIn) ++fine;
            // the fine step [fine - 1, fine] holds sIn; interpolate the curve parameter across it
            const double ds = table.s[fine] - table.s[fine - 1];
            const double f = ds > 0.0 ? std::clamp((sIn - table.s[fine - 1]) / ds, 0.0, 1.0) : 0.0;
            const double t = table.t[fine - 1] + f * (table.t[fine] - table.t[fine - 1]);
            const std::size_t segment = table.segment[fine];
            const CurvePoint point = evaluate(piece, segment, t);
            const double speed = std::hypot(point.dx, point.dy);

            PathSample sample;
            sample.sIn = sIn;
            sample.xIn = point.x;
            sample.yIn = point.y;
            sample.tangentDeg = std::atan2(point.dx, point.dy) * kRadToDeg;
            // a spline that stalls (a cusp from a wild point layout) has no curvature to speak of
            sample.curvatureRadPerIn =
                speed > 1e-9
                    ? (point.dy * point.ddx - point.dx * point.ddy) / (speed * speed * speed)
                    : 0.0;
            headingAt(keys, sIn, sample);
            sample.section = segmentSection[piece.firstWaypoint + segment];
            const bool movingStart = p == 0 && ends.startTangentDeg.has_value();
            const bool movingEnd = p + 1 == pieces.size() && ends.movingAtEnd;
            sample.stop = (k == 0 && !movingStart) || (k == count && !movingEnd);
            result.samples.push_back(sample);
        }
    }
    result.ok = true;
    return result;
}

} // namespace sapphirelib::motion
