// Host-side unit test for sapphirelib::motion::toLocalFrame,
// findLookaheadPoint and reachedFinalApproach, no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/pure_pursuit_math_test.cpp \
//       src/sapphirelib/motion/pure_pursuit_math.cpp src/sapphirelib/motion/path.cpp \
//       -o pure_pursuit_math_test && ./pure_pursuit_math_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <vector>

#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/util/random.hpp"

using sapphirelib::Rng;
using sapphirelib::motion::findLookaheadPoint;
using sapphirelib::motion::Path;
using sapphirelib::motion::reachedFinalApproach;
using sapphirelib::motion::toLocalFrame;
using sapphirelib::motion::Waypoint;

namespace {

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-6) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
        assert(false);
    }
}

void expectTrue(bool condition, const char* label) {
    if (!condition) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

void testToLocalFrameAtHeadingZero() {
    // Facing "north" (heading 0): a point straight ahead is pure forward.
    const auto local = toLocalFrame(/*dxIn=*/0.0, /*dyIn=*/10.0, /*headingDeg=*/0.0);
    expectNear(local.forwardIn, 10.0, "heading0: forward");
    expectNear(local.lateralIn, 0.0, "heading0: lateral");
}

void testToLocalFrameAtHeadingEast() {
    // Facing "east" (heading 90): a point due east of the chassis is pure
    // forward; a point due north is pure lateral (to the chassis's left,
    // i.e. negative lateral, since +lateral is defined as "to the right").
    const auto forward = toLocalFrame(/*dxIn=*/10.0, /*dyIn=*/0.0, /*headingDeg=*/90.0);
    expectNear(forward.forwardIn, 10.0, "heading90: forward from +x");
    expectNear(forward.lateralIn, 0.0, "heading90: lateral from +x");

    const auto left = toLocalFrame(/*dxIn=*/0.0, /*dyIn=*/10.0, /*headingDeg=*/90.0);
    expectNear(left.forwardIn, 0.0, "heading90: forward from +y");
    expectNear(left.lateralIn, -10.0, "heading90: lateral from +y");
}

void testToLocalFrameRoundTrip() {
    // Rotating into the local frame and back out (same formula both
    // directions, the rotation matrix here is its own inverse) should
    // recover the original field-frame displacement.
    const double dxIn = 7.0;
    const double dyIn = -3.0;
    const double headingDeg = 37.0;
    const auto local = toLocalFrame(dxIn, dyIn, headingDeg);
    const auto field = toLocalFrame(local.forwardIn, local.lateralIn, headingDeg);
    expectNear(field.forwardIn, dxIn, "round trip: dx");
    expectNear(field.lateralIn, dyIn, "round trip: dy");
}

void testLookaheadFindsIntersectionOnCurrentSegment() {
    // Chassis at the origin, path running straight up +y. A 5in lookahead
    // circle should hit the segment at y=5.
    const Path path({Waypoint{0.0, 0.0}, Waypoint{0.0, 20.0}});
    const auto result = findLookaheadPoint(/*xIn=*/0.0, /*yIn=*/0.0, path, /*lookaheadIn=*/5.0,
                                            /*fromIndex=*/0);
    expectNear(result.point.xIn, 0.0, "lookahead on segment: x");
    expectNear(result.point.yIn, 5.0, "lookahead on segment: y");
}

void testLookaheadSkipsAheadAcrossSegments() {
    // Chassis already 8in up the path, one segment behind it (fromIndex=1)
    // the lookahead point should be found on segment 1 (10 -> 20), not
    // regress to segment 0.
    const Path path({Waypoint{0.0, 0.0}, Waypoint{0.0, 10.0}, Waypoint{0.0, 20.0}});
    const auto result = findLookaheadPoint(/*xIn=*/0.0, /*yIn=*/8.0, path, /*lookaheadIn=*/4.0,
                                            /*fromIndex=*/1);
    expectNear(result.point.xIn, 0.0, "lookahead across segments: x");
    expectNear(result.point.yIn, 12.0, "lookahead across segments: y");
}

void testLookaheadFallsBackToFinalWaypointNearEnd() {
    // Chassis within lookahead radius of the whole remaining path, no
    // circle intersection exists, so pursuit should converge on the last
    // waypoint instead.
    const Path path({Waypoint{0.0, 0.0}, Waypoint{0.0, 10.0}});
    const auto result = findLookaheadPoint(/*xIn=*/0.0, /*yIn=*/9.0, path, /*lookaheadIn=*/5.0,
                                            /*fromIndex=*/0);
    expectNear(result.point.xIn, 0.0, "lookahead fallback: x");
    expectNear(result.point.yIn, 10.0, "lookahead fallback: y");
}

void testFinalApproachWaitsForTheLastSegment() {
    // four waypoints, three segments: the last is segment 2
    expectTrue(!reachedFinalApproach(0.0, 6.0, 0, 4), "at the end, but on the first segment");
    expectTrue(!reachedFinalApproach(0.0, 6.0, 1, 4), "at the end, but on the second segment");
    expectTrue(reachedFinalApproach(5.9, 6.0, 2, 4), "last segment, inside the approach");
    expectTrue(!reachedFinalApproach(6.1, 6.0, 2, 4), "last segment, not close enough yet");
    // findLookaheadPoint() reports count - 1 once the rest of the path is inside the lookahead
    expectTrue(reachedFinalApproach(3.0, 6.0, 3, 4), "past the last segment");
    // one segment: nothing to wait for, as before
    expectTrue(reachedFinalApproach(3.0, 6.0, 0, 2), "single segment");
    // one waypoint: no segments at all
    expectTrue(reachedFinalApproach(3.0, 6.0, 0, 1), "single waypoint");
}

// followPath()'s pursuit loop, on a point robot that moves 0.4in (40in/s at 10ms) toward the
// lookahead point each tick until `finished` hands over to the final approach
struct PursuitRun {
    int ticks = 0;
    double xIn = 0.0;
    double yIn = 0.0;
    // how close the robot came to each waypoint on the way
    std::vector<double> closestToWaypointIn;
};

using FinalApproachRule =
    std::function<bool(double distToFinalIn, std::size_t segmentIndex, std::size_t count)>;

PursuitRun pursue(const Path& path, double startX, double startY, double lookaheadIn,
                  const FinalApproachRule& finished) {
    const std::vector<Waypoint>& points = path.waypoints();
    PursuitRun run{.xIn = startX, .yIn = startY};
    run.closestToWaypointIn.assign(points.size(), 1e9);
    std::size_t segmentIndex = 0;
    const double stepIn = 0.4;
    while (run.ticks < 20000) {
        for (std::size_t i = 0; i < points.size(); ++i) {
            run.closestToWaypointIn[i] =
                std::min(run.closestToWaypointIn[i],
                         std::hypot(points[i].xIn - run.xIn, points[i].yIn - run.yIn));
        }
        const double distToFinalIn =
            std::hypot(points.back().xIn - run.xIn, points.back().yIn - run.yIn);
        if (finished(distToFinalIn, segmentIndex, points.size())) break;
        const auto lookahead =
            findLookaheadPoint(run.xIn, run.yIn, path, lookaheadIn, segmentIndex);
        segmentIndex = lookahead.segmentIndex;
        const double dx = lookahead.point.xIn - run.xIn;
        const double dy = lookahead.point.yIn - run.yIn;
        const double d = std::hypot(dx, dy);
        if (d > 1e-9) {
            run.xIn += dx / d * std::min(stepIn, d);
            run.yIn += dy / d * std::min(stepIn, d);
        }
        ++run.ticks;
    }
    return run;
}

// the rule followPath() used before closed paths were handled, kept to check the new one against
bool oldFinalApproach(double distToFinalIn, std::size_t /*segmentIndex*/, std::size_t /*count*/) {
    return distToFinalIn <= 6.0;
}

bool newFinalApproach(double distToFinalIn, std::size_t segmentIndex, std::size_t count) {
    return reachedFinalApproach(distToFinalIn, 6.0, segmentIndex, count);
}

void testOpenPathsBehaveExactlyAsBefore() {
    // 500 random open paths, each heading generally downfield so it never comes back near its own
    // end: the new rule has to hand over on the same tick, from the same spot, as the old one
    Rng rng(7);
    for (int trial = 0; trial < 500; ++trial) {
        std::vector<Waypoint> points{{0.0, 0.0}};
        const int count = 2 + static_cast<int>(rng.uniform() * 6);
        for (int i = 1; i < count; ++i) {
            points.push_back({points.back().xIn + (rng.uniform() - 0.5) * 40.0,
                              points.back().yIn + 12.0 + rng.uniform() * 30.0});
        }
        const Path path(points);
        const double lookaheadIn = 6.0 + rng.uniform() * 10.0;
        const PursuitRun before = pursue(path, 0.0, 0.0, lookaheadIn, oldFinalApproach);
        const PursuitRun after = pursue(path, 0.0, 0.0, lookaheadIn, newFinalApproach);
        if (before.ticks != after.ticks || before.xIn != after.xIn || before.yIn != after.yIn) {
            std::printf("FAIL open path %d: %d ticks to (%.3f, %.3f), was %d to (%.3f, %.3f)\n",
                        trial, after.ticks, after.xIn, after.yIn, before.ticks, before.xIn,
                        before.yIn);
            assert(false);
        }
    }
}

void testClosedPathDrivesTheWholeLap() {
    // a 60in square lap starting and ending at (-30, -30)
    const Path lap({{-30, -30}, {-30, 30}, {30, 30}, {30, -30}, {-30, -30}});
    const PursuitRun before = pursue(lap, -30, -30, 10.0, oldFinalApproach);
    expectTrue(before.ticks == 0, "the old rule finishes a lap before moving");

    const PursuitRun after = pursue(lap, -30, -30, 10.0, newFinalApproach);
    // 240in at 0.4in a tick is 600 ticks; cutting the corners shortens it a little
    std::printf("  closed lap: %d ticks, handed over %.2fin from the start\n", after.ticks,
                std::hypot(after.xIn + 30, after.yIn + 30));
    expectTrue(after.ticks > 500 && after.ticks < 700, "the lap takes about 600 ticks");
    for (std::size_t i = 1; i + 1 < lap.waypoints().size(); ++i) {
        expectTrue(after.closestToWaypointIn[i] < 10.0, "passes every corner");
    }
    expectTrue(std::hypot(after.xIn + 30, after.yIn + 30) <= 6.0, "hands over near the start");

    // and a lap that starts partway along a side works the same way
    const Path fromMiddle({{-30, 0}, {-30, 30}, {30, 30}, {30, -30}, {-30, -30}, {-30, 0}});
    const PursuitRun middle = pursue(fromMiddle, -30, 0, 10.0, newFinalApproach);
    expectTrue(middle.ticks > 500, "a lap from mid-side runs too");
}

} // namespace

int main() {
    testToLocalFrameAtHeadingZero();
    testToLocalFrameAtHeadingEast();
    testToLocalFrameRoundTrip();
    testLookaheadFindsIntersectionOnCurrentSegment();
    testLookaheadSkipsAheadAcrossSegments();
    testLookaheadFallsBackToFinalWaypointNearEnd();
    testFinalApproachWaitsForTheLastSegment();
    testOpenPathsBehaveExactlyAsBefore();
    testClosedPathDrivesTheWholeLap();
    std::puts("pure_pursuit_math_test: all assertions passed");
    return 0;
}
