// Host-side unit test for sapphirelib::motion::samplePath(), no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/trajectory_path_test.cpp \
//       src/sapphirelib/motion/trajectory_path.cpp src/sapphirelib/util/angle.cpp \
//       -o trajectory_path_test && ./trajectory_path_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sapphirelib/motion/trajectory_path.hpp"

using sapphirelib::motion::kTrajectorySampleSpacingIn;
using sapphirelib::motion::PathSample;
using sapphirelib::motion::SampledPath;
using sapphirelib::motion::samplePath;
using sapphirelib::motion::TrajectorySection;
using sapphirelib::motion::TrajectoryWaypoint;

namespace {

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.9f, expected %.9f (+-%g)\n", label, actual, expected,
                    tolerance);
        assert(false);
    }
}

// one section through the given waypoints
std::vector<TrajectorySection> single(std::vector<TrajectoryWaypoint> waypoints) {
    return {TrajectorySection{.waypoints = std::move(waypoints)}};
}

// distance from a point to the polyline through the samples
double distanceToSamples(const SampledPath& path, double x, double y) {
    double best = 1e9;
    for (std::size_t k = 0; k + 1 < path.samples.size(); ++k) {
        const PathSample& a = path.samples[k];
        const PathSample& b = path.samples[k + 1];
        const double dx = b.xIn - a.xIn;
        const double dy = b.yIn - a.yIn;
        const double lengthSq = dx * dx + dy * dy;
        double f = lengthSq > 0.0 ? ((x - a.xIn) * dx + (y - a.yIn) * dy) / lengthSq : 0.0;
        f = std::fmax(0.0, std::fmin(1.0, f));
        best = std::fmin(best, std::hypot(a.xIn + f * dx - x, a.yIn + f * dy - y));
    }
    return best;
}

void testStraightLine() {
    // 3-4-5: 50 inches toward the right and up.
    const SampledPath path =
        samplePath(single({{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 30, .yIn = 40}}));
    assert(path.ok && path.error.empty());
    expectNear(path.lengthIn, 50.0, 1e-9, "straight length");
    assert(path.samples.size() == 101); // 100 steps of exactly 0.5in
    const double tangent = std::atan2(30.0, 40.0) * 180.0 / 3.14159265358979323846;
    for (std::size_t k = 0; k < path.samples.size(); ++k) {
        const PathSample& s = path.samples[k];
        expectNear(s.sIn, 0.5 * k, 1e-9, "straight s");
        expectNear(s.xIn, 0.6 * s.sIn, 1e-9, "straight x");
        expectNear(s.yIn, 0.8 * s.sIn, 1e-9, "straight y");
        expectNear(s.tangentDeg, tangent, 1e-9, "straight tangent");
        expectNear(s.curvatureRadPerIn, 0.0, 1e-12, "straight curvature");
        expectNear(s.headingDeg, 0.0, 0.0, "heading held");
        assert(s.section == 0);
        assert(s.stop == (k == 0 || k + 1 == path.samples.size()));
    }
}

void testPassesThroughWaypointsWithSmoothCurvature() {
    const std::vector<TrajectoryWaypoint> waypoints{{.xIn = 0, .yIn = 0, .headingDeg = 0},
                                                    {.xIn = 10, .yIn = 24},
                                                    {.xIn = 0, .yIn = 48},
                                                    {.xIn = 20, .yIn = 70},
                                                    {.xIn = 40, .yIn = 72}};
    const SampledPath path = samplePath(single(waypoints));
    assert(path.ok);
    // to within the sag of a 0.5in chord on these curves
    for (const TrajectoryWaypoint& w : waypoints) {
        expectNear(distanceToSamples(path, w.xIn, w.yIn), 0.0, 0.01, "passes through waypoint");
    }
    expectNear(path.samples.front().xIn, 0.0, 1e-12, "starts at the first waypoint");
    expectNear(path.samples.back().xIn, 40.0, 1e-9, "ends at the last waypoint: x");
    expectNear(path.samples.back().yIn, 72.0, 1e-9, "ends at the last waypoint: y");

    // natural ends: no curvature where the robot starts and stops
    expectNear(path.samples.front().curvatureRadPerIn, 0.0, 1e-9, "natural start");
    expectNear(path.samples.back().curvatureRadPerIn, 0.0, 1e-9, "natural end");

    // samples evenly spaced along the path, never more than the spacing apart
    for (std::size_t k = 1; k < path.samples.size(); ++k) {
        const PathSample& a = path.samples[k - 1];
        const PathSample& b = path.samples[k];
        const double ds = b.sIn - a.sIn;
        assert(ds > 0.0 && ds <= kTrajectorySampleSpacingIn + 1e-12);
        expectNear(ds, path.samples[1].sIn - path.samples[0].sIn, 1e-9, "even spacing");
        // the chord between neighbors is the arc, near enough on a gentle curve
        expectNear(std::hypot(b.xIn - a.xIn, b.yIn - a.yIn), ds, 1e-3, "chord matches arc");
    }

    // C2: curvature has no jump at the interior waypoints. The change across the sample nearest
    // each one is no bigger than the steady change of the samples around it; a spline that's
    // only C1 there (Catmull-Rom) jumps by many times that
    const auto kappa = [&](std::size_t k) { return path.samples[k].curvatureRadPerIn; };
    for (std::size_t w = 1; w + 1 < waypoints.size(); ++w) {
        std::size_t nearest = 0;
        for (std::size_t k = 0; k < path.samples.size(); ++k) {
            const auto d = [&](std::size_t i) {
                return std::hypot(path.samples[i].xIn - waypoints[w].xIn,
                                  path.samples[i].yIn - waypoints[w].yIn);
            };
            if (d(k) < d(nearest)) nearest = k;
        }
        const std::size_t k = nearest;
        const double across =
            std::fmax(std::fabs(kappa(k + 1) - kappa(k)), std::fabs(kappa(k) - kappa(k - 1)));
        const double around = std::fmax(std::fabs(kappa(k - 2) - kappa(k - 3)),
                                        std::fabs(kappa(k + 3) - kappa(k + 2)));
        if (across > 1.5 * around + 1e-4) {
            std::printf("FAIL curvature jumps %.5f at waypoint %u (%.5f around it)\n", across,
                        static_cast<unsigned>(w), around);
            assert(false);
        }
    }

    // the length agrees with a much finer measurement of the same samples' curve
    double polyline = 0.0;
    for (std::size_t k = 1; k < path.samples.size(); ++k) {
        polyline += std::hypot(path.samples[k].xIn - path.samples[k - 1].xIn,
                               path.samples[k].yIn - path.samples[k - 1].yIn);
    }
    expectNear(polyline, path.lengthIn, 1e-3 * path.lengthIn, "arc length");
}

void testCurvatureSignAndTangent() {
    // a quarter turn to the right (clockwise): positive curvature, tangent swinging 0 -> 90
    const SampledPath path = samplePath(single(
        {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 7, .yIn = 17}, {.xIn = 24, .yIn = 24}}));
    assert(path.ok);
    const PathSample& middle = path.samples[path.samples.size() / 2];
    assert(middle.curvatureRadPerIn > 0.0);
    assert(path.samples.front().tangentDeg < middle.tangentDeg);
    assert(middle.tangentDeg < path.samples.back().tangentDeg);

    // mirrored: a turn to the left is negative
    const SampledPath left = samplePath(single(
        {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = -7, .yIn = 17}, {.xIn = -24, .yIn = 24}}));
    expectNear(left.samples[left.samples.size() / 2].curvatureRadPerIn, -middle.curvatureRadPerIn,
               1e-12, "mirrored curvature");
}

void testStopSplitsTheSpline() {
    // up 24in, stop, then right 24in: a sharp corner is allowed where the robot is at rest
    const std::vector<TrajectorySection> sections{
        {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 24}},
         .stopAtEnd = true},
        {.waypoints = {{.xIn = 24, .yIn = 24}}},
    };
    const SampledPath path = samplePath(sections);
    assert(path.ok);
    expectNear(path.lengthIn, 48.0, 1e-9, "two straight pieces");
    std::size_t corner = 0;
    for (std::size_t k = 1; k < path.samples.size(); ++k) {
        if (path.samples[k].sIn == path.samples[k - 1].sIn) corner = k;
    }
    assert(corner > 0); // two samples at the stop, one per direction
    const PathSample& in = path.samples[corner - 1];
    const PathSample& out = path.samples[corner];
    assert(in.stop && out.stop);
    expectNear(in.xIn, 0.0, 1e-9, "stop x");
    expectNear(in.yIn, 24.0, 1e-9, "stop y");
    expectNear(in.tangentDeg, 0.0, 1e-9, "arrives heading up");
    expectNear(out.tangentDeg, 90.0, 1e-9, "leaves heading right");
    assert(in.section == 0 && out.section == 1);
    for (std::size_t k = 0; k < path.samples.size(); ++k) {
        const bool end = k == 0 || k + 1 == path.samples.size();
        assert(path.samples[k].stop == (end || k == corner - 1 || k == corner));
    }

    // without the stop the same points make one smooth curve through the corner
    const std::vector<TrajectorySection> through{
        {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 24}}},
        {.waypoints = {{.xIn = 24, .yIn = 24}}},
    };
    const SampledPath smooth = samplePath(through);
    assert(smooth.ok);
    for (std::size_t k = 1; k < smooth.samples.size(); ++k) {
        assert(smooth.samples[k].sIn > smooth.samples[k - 1].sIn);
        // a tight bend, but no 90 degree jump
        assert(std::fabs(smooth.samples[k].tangentDeg - smooth.samples[k - 1].tangentDeg) < 10.0);
        assert(smooth.samples[k].stop == (k + 1 == smooth.samples.size()));
    }
}

void testSectionsLabelTheirSamples() {
    const std::vector<TrajectorySection> sections{
        {.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 20}}},
        {.waypoints = {{.xIn = 0, .yIn = 30}, {.xIn = 0, .yIn = 40}}},
        {.waypoints = {{.xIn = 0, .yIn = 60}}},
    };
    const SampledPath path = samplePath(sections);
    assert(path.ok);
    for (const PathSample& s : path.samples) {
        const std::size_t expected = s.yIn <= 20.0 + 1e-9 ? 0 : s.yIn <= 40.0 + 1e-9 ? 1 : 2;
        assert(s.section == expected);
    }
    // each section starts at the previous one's last waypoint
    assert(path.sectionStartIn.size() == 3);
    expectNear(path.sectionStartIn[0], 0.0, 0.0, "first section starts at 0");
    expectNear(path.sectionStartIn[1], 20.0, 1e-9, "second section start");
    expectNear(path.sectionStartIn[2], 40.0, 1e-9, "third section start");
    // and how far along each waypoint is, the junctions included, in the chain's order
    const double expectedIn[] = {0.0, 20.0, 30.0, 40.0, 60.0};
    assert(path.waypointIn.size() == 5);
    for (std::size_t j = 0; j < 5; ++j) {
        expectNear(path.waypointIn[j], expectedIn[j], 1e-9, "waypoint distance");
    }
}

void testHeadingKeyframes() {
    // turn from 0 to 90 over the first 40 inches, then hold
    const SampledPath path = samplePath(single({{.xIn = 0, .yIn = 0, .headingDeg = 0},
                                                {.xIn = 0, .yIn = 20},
                                                {.xIn = 0, .yIn = 40, .headingDeg = 90},
                                                {.xIn = 0, .yIn = 60}}));
    assert(path.ok);
    double last = -1.0;
    for (const PathSample& s : path.samples) {
        if (s.sIn <= 40.0 + 1e-9) {
            assert(s.headingDeg >= last); // turns one way only
            last = s.headingDeg;
            // smootherstep at u = s / 40
            const double u = s.sIn / 40.0;
            expectNear(s.headingDeg,
                       90.0 * (6 * std::pow(u, 5) - 15 * std::pow(u, 4) + 10 * std::pow(u, 3)),
                       1e-6, "smootherstep heading");
        } else {
            expectNear(s.headingDeg, 90.0, 1e-12, "held after the last keyframe");
            expectNear(s.headingRateDegPerIn, 0.0, 0.0, "no turn after the last keyframe");
        }
    }
    // no turn rate or turn acceleration at the keyframes themselves
    expectNear(path.samples.front().headingRateDegPerIn, 0.0, 1e-12, "rate at the start");
    expectNear(path.samples.front().headingAccelDegPerIn2, 0.0, 1e-12, "accel at the start");
    const PathSample& at40 = path.samples[80];
    expectNear(at40.sIn, 40.0, 1e-9, "sample at the keyframe");
    expectNear(at40.headingDeg, 90.0, 1e-9, "keyframe reached");
    expectNear(at40.headingRateDegPerIn, 0.0, 1e-9, "rate at the keyframe");
    // the rate peaks halfway: 90 degrees * 30/16 / 40in
    expectNear(path.samples[40].headingRateDegPerIn, 90.0 * 1.875 / 40.0, 1e-6, "peak rate");
    // the derivatives agree with the heading's own central differences, to within a central
    // difference's error (h^2/6 times the third derivative: about 0.004 here)
    for (std::size_t k = 1; k + 1 < 80; ++k) {
        const double ds = path.samples[k + 1].sIn - path.samples[k - 1].sIn;
        expectNear(path.samples[k].headingRateDegPerIn,
                   (path.samples[k + 1].headingDeg - path.samples[k - 1].headingDeg) / ds, 0.005,
                   "rate matches the heading");
        expectNear(
            path.samples[k].headingAccelDegPerIn2,
            (path.samples[k + 1].headingRateDegPerIn - path.samples[k - 1].headingRateDegPerIn) /
                ds,
            0.005, "accel matches the rate");
    }
}

void testHeadingTurnsTheShortWay() {
    // 350 to 10 is 20 degrees clockwise through 0, not 340 counterclockwise
    const SampledPath path = samplePath(single({{.xIn = 0, .yIn = 0, .headingDeg = 350},
                                                {.xIn = 0, .yIn = 20, .headingDeg = 10},
                                                {.xIn = 0, .yIn = 40, .headingDeg = -30}}));
    assert(path.ok);
    expectNear(path.samples[40].headingDeg, 370.0, 1e-9, "350 + 20, continuous");
    expectNear(path.samples.back().headingDeg, 330.0, 1e-9, "then 40 back");
    for (std::size_t k = 1; k <= 40; ++k) {
        assert(path.samples[k].headingDeg >= path.samples[k - 1].headingDeg);
    }
}

void testRejections() {
    SampledPath path = samplePath({});
    assert(!path.ok && path.error == "a path needs at least 2 waypoints");
    path = samplePath(single({{.xIn = 0, .yIn = 0, .headingDeg = 0}}));
    assert(!path.ok && path.error == "a path needs at least 2 waypoints");
    path = samplePath(single({{.xIn = 0, .yIn = 0}, {.xIn = 0, .yIn = 10}}));
    assert(!path.ok && path.error == "the first waypoint needs a heading (where the robot starts)");
    path = samplePath(single(
        {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 10}, {.xIn = 0, .yIn = 10}}));
    assert(!path.ok && path.error == "waypoints 1 and 2 are at the same spot");
    path = samplePath({{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 9}}},
                       {.waypoints = {}}});
    assert(!path.ok && path.error == "section 1 has no waypoints");
    assert(path.samples.empty());
}

void testMovingEnds() {
    // already moving east at the start: the spline leaves (0, 0) heading east even though the
    // next point is straight north, and neither end is a stop
    const std::vector<TrajectorySection> sections =
        single({{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 40}});
    const SampledPath free = samplePath(sections);
    const SampledPath moving =
        samplePath(sections, {.startTangentDeg = 90.0, .movingAtEnd = true});
    assert(free.ok && moving.ok);
    expectNear(free.samples.front().tangentDeg, 0.0, 1e-9, "free start: straight at the point");
    expectNear(moving.samples.front().tangentDeg, 90.0, 1e-6, "clamped start: east");
    assert(free.samples.front().stop && free.samples.back().stop);
    assert(!moving.samples.front().stop && !moving.samples.back().stop);
    // it still ends at the waypoint, bending round to it
    expectNear(moving.samples.back().xIn, 0.0, 1e-9, "ends at the waypoint x");
    expectNear(moving.samples.back().yIn, 40.0, 1e-9, "ends at the waypoint y");
    assert(moving.lengthIn > free.lengthIn);
    double east = 0.0;
    for (const PathSample& s : moving.samples) east = std::fmax(east, s.xIn);
    assert(east > 1.0);

    // clamped to the way it's already going, a straight line stays exactly straight
    const SampledPath along = samplePath(sections, {.startTangentDeg = 0.0});
    for (const PathSample& s : along.samples) {
        expectNear(s.xIn, 0.0, 1e-9, "clamped along the line: straight");
        expectNear(s.curvatureRadPerIn, 0.0, 1e-9, "clamped along the line: no curvature");
    }

    // with a stop in the middle, only the first piece is clamped, and the stop stays a stop
    const SampledPath split = samplePath(
        {{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0}, {.xIn = 0, .yIn = 20}},
          .stopAtEnd = true},
         {.waypoints = {{.xIn = 20, .yIn = 20}, {.xIn = 20, .yIn = 40}}}},
        {.startTangentDeg = 45.0, .movingAtEnd = true});
    assert(split.ok);
    expectNear(split.samples.front().tangentDeg, 45.0, 1e-6, "split: clamped first piece");
    int stops = 0;
    for (const PathSample& s : split.samples) stops += s.stop ? 1 : 0;
    assert(stops == 2); // the middle stop's two samples, neither end
    // the second piece is natural: no curvature at its far end
    expectNear(split.samples.back().curvatureRadPerIn, 0.0, 1e-6, "natural end");

    // a clamped start on a single segment (two waypoints) bends to meet the direction
    const SampledPath one = samplePath(single({{.xIn = 0, .yIn = 0, .headingDeg = 0},
                                               {.xIn = 10, .yIn = 10}}),
                                       {.startTangentDeg = 0.0});
    assert(one.ok);
    expectNear(one.samples.front().tangentDeg, 0.0, 1e-6, "one segment: clamped");
    assert(one.samples.back().tangentDeg > 45.0); // curving round toward +x
}

} // namespace

int main() {
    testStraightLine();
    testPassesThroughWaypointsWithSmoothCurvature();
    testCurvatureSignAndTangent();
    testStopSplitsTheSpline();
    testSectionsLabelTheirSamples();
    testHeadingKeyframes();
    testHeadingTurnsTheShortWay();
    testRejections();
    testMovingEnds();
    std::puts("trajectory_path_test: all assertions passed");
    return 0;
}
