// Host-side unit test for sapphirelib::motion::toLocalFrame and
// findLookaheadPoint — no PROS/embedded dependencies, so it builds and runs
// with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/pure_pursuit_math_test.cpp \
//       src/sapphirelib/motion/pure_pursuit_math.cpp src/sapphirelib/motion/path.cpp \
//       -o pure_pursuit_math_test && ./pure_pursuit_math_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/motion/pure_pursuit_math.hpp"

using sapphirelib::motion::findLookaheadPoint;
using sapphirelib::motion::Path;
using sapphirelib::motion::toLocalFrame;
using sapphirelib::motion::Waypoint;

namespace {

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-6) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
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
    // directions — the rotation matrix here is its own inverse) should
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
    // — the lookahead point should be found on segment 1 (10 -> 20), not
    // regress to segment 0.
    const Path path({Waypoint{0.0, 0.0}, Waypoint{0.0, 10.0}, Waypoint{0.0, 20.0}});
    const auto result = findLookaheadPoint(/*xIn=*/0.0, /*yIn=*/8.0, path, /*lookaheadIn=*/4.0,
                                            /*fromIndex=*/1);
    expectNear(result.point.xIn, 0.0, "lookahead across segments: x");
    expectNear(result.point.yIn, 12.0, "lookahead across segments: y");
}

void testLookaheadFallsBackToFinalWaypointNearEnd() {
    // Chassis within lookahead radius of the whole remaining path — no
    // circle intersection exists, so pursuit should converge on the last
    // waypoint instead.
    const Path path({Waypoint{0.0, 0.0}, Waypoint{0.0, 10.0}});
    const auto result = findLookaheadPoint(/*xIn=*/0.0, /*yIn=*/9.0, path, /*lookaheadIn=*/5.0,
                                            /*fromIndex=*/0);
    expectNear(result.point.xIn, 0.0, "lookahead fallback: x");
    expectNear(result.point.yIn, 10.0, "lookahead fallback: y");
}

} // namespace

int main() {
    testToLocalFrameAtHeadingZero();
    testToLocalFrameAtHeadingEast();
    testToLocalFrameRoundTrip();
    testLookaheadFindsIntersectionOnCurrentSegment();
    testLookaheadSkipsAheadAcrossSegments();
    testLookaheadFallsBackToFinalWaypointNearEnd();
    std::puts("pure_pursuit_math_test: all assertions passed");
    return 0;
}
