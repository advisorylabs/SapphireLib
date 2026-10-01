// Host-side unit test for sapphirelib::localization::FieldMap, no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/localization/field_map_test.cpp src/sapphirelib/localization/field_map.cpp -o field_map_test && ./field_map_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/localization/field_map.hpp"

using sapphirelib::localization::FieldMap;
using sapphirelib::localization::kVrcFieldSizeIn;
using sapphirelib::localization::ParallelRayCaster;

namespace {

void expectNear(double actual, double expected, const char* label) {
    if (!(std::fabs(actual - expected) < 1e-9)) {
        std::printf("FAIL %s: got %.9f, expected %.9f\n", label, actual, expected);
        assert(false);
    }
}

void testCenteredFieldWalls() {
    const FieldMap map = FieldMap::centered();
    const double half = kVrcFieldSizeIn / 2.0;
    expectNear(map.minXIn(), -half, "min x");
    expectNear(map.maxYIn(), half, "max y");
    assert(map.segments().size() == 4);

    // headings are clockwise from +y: 0 is +y, 90 is +x
    expectNear(map.castRayAtHeadingIn(0, 0, 0), half, "center, facing +y");
    expectNear(map.castRayAtHeadingIn(0, 0, 90), half, "center, facing +x");
    expectNear(map.castRayAtHeadingIn(0, 0, 180), half, "center, facing -y");
    expectNear(map.castRayAtHeadingIn(0, 0, 270), half, "center, facing -x");
    expectNear(map.castRayAtHeadingIn(10, -20, 0), half + 20, "off center, facing +y");
    expectNear(map.castRayAtHeadingIn(10, -20, 90), half - 10, "off center, facing +x");
}

void testDiagonalRay() {
    const FieldMap map(0, 0, 100, 100);
    // 45 degrees from the middle hits the corner
    expectNear(map.castRayAtHeadingIn(50, 50, 45), 50 * std::sqrt(2.0), "diagonal to corner");
    // 30 degrees off +y from (50, 50): hits the top wall 50in up, at 50 / cos(30)
    expectNear(map.castRayAtHeadingIn(50, 50, 30), 50 / std::cos(30 * 3.14159265358979323846 / 180),
               "30 degrees to the top wall");
}

void testBoxBlocksTheRay() {
    FieldMap map = FieldMap::centered(140);
    map.addBox(-5, 20, 5, 30);
    assert(map.segments().size() == 8);
    expectNear(map.castRayAtHeadingIn(0, 0, 0), 20, "box in front");
    expectNear(map.castRayAtHeadingIn(0, 0, 180), 70, "wall behind, box doesn't matter");
    // just past the box's edge, the wall
    expectNear(map.castRayAtHeadingIn(6, 0, 0), 70, "beside the box");
    // grazing the box's corner still counts as a hit
    expectNear(map.castRayAtHeadingIn(5, 0, 0), 20, "box corner");
}

void testRayFromOutsidePointingAway() {
    const FieldMap map(0, 0, 10, 10);
    assert(std::isinf(map.castRayAtHeadingIn(-5, 5, 270)));
    // from outside, pointing in, the near wall is hit first
    expectNear(map.castRayAtHeadingIn(-5, 5, 90), 5, "outside pointing in");
}

void testContains() {
    const FieldMap map = FieldMap::centered(100);
    assert(map.contains(0, 0));
    assert(map.contains(50, -50));
    assert(!map.contains(50.01, 0));
    assert(!map.contains(0, -60));
}

void testCastRayMatchesHeadingVersion() {
    const FieldMap map = FieldMap::centered();
    for (double heading = 0; heading < 360; heading += 17) {
        const double rad = heading * 3.14159265358979323846 / 180;
        expectNear(map.castRayIn(3, -7, std::sin(rad), std::cos(rad)),
                   map.castRayAtHeadingIn(3, -7, heading), "direction vs heading");
    }
}

} // namespace

void testParallelRayCasterMatchesCastRay() {
    // starts inside and outside the walls, every direction including exactly along the axes, and
    // a map with boxes in it (one straddling a wall). A ray through the very end of a segment is
    // a hit or a miss by rounding, in either version, so the grid stays off every segment's ends
    FieldMap map = FieldMap::centered();
    map.addBox(-5, 20, 5, 30);
    map.addBox(60, -10, 80, 10);
    map.addSegment({-40, -40, -20, -50});
    ParallelRayCaster caster;
    double worst = 0.0;
    int hits = 0;
    for (int heading = 0; heading < 360; heading += 15) {
        const double rad = heading * 3.14159265358979323846 / 180.0;
        // headings 0 and 90 also exactly along the axes, as sin and cos don't give
        for (int exact = 0; exact < 2; ++exact) {
            double dirX = std::sin(rad);
            double dirY = std::cos(rad);
            if (exact == 1) {
                if (heading % 90 != 0) continue;
                dirX = heading == 90 ? 1.0 : heading == 270 ? -1.0 : 0.0;
                dirY = heading == 0 ? 1.0 : heading == 180 ? -1.0 : 0.0;
            }
            caster.aim(map, dirX, dirY);
            for (double x = -90.3; x < 90.0; x += 7.7) {
                for (double y = -90.13; y < 90.0; y += 9.1) {
                    const double expected = map.castRayIn(x, y, dirX, dirY);
                    const double got = caster.castIn(x, y);
                    if (std::isinf(expected)) {
                        if (!std::isinf(got)) {
                            std::printf("FAIL caster at (%.1f, %.1f) heading %d: got %.9f, "
                                        "expected nothing\n",
                                        x, y, heading, got);
                            assert(false);
                        }
                        continue;
                    }
                    ++hits;
                    worst = std::max(worst, std::fabs(got - expected));
                }
            }
        }
    }
    std::printf("  parallel caster within %.1e of castRayIn() over %d hits\n", worst, hits);
    assert(worst < 1e-9);
}

int main() {
    testCenteredFieldWalls();
    testDiagonalRay();
    testBoxBlocksTheRay();
    testRayFromOutsidePointingAway();
    testContains();
    testCastRayMatchesHeadingVersion();
    testParallelRayCasterMatchesCastRay();
    std::printf("field_map_test: all tests passed\n");
    return 0;
}
