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
#include <limits>

#include "sapphirelib/localization/field_map.hpp"

using sapphirelib::localization::ConeCaster;
using sapphirelib::localization::FieldMap;
using sapphirelib::localization::kVrcFieldSizeIn;
using sapphirelib::localization::ParallelRayCaster;
using sapphirelib::localization::Segment;

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

namespace {

// a red loader as 96671H's map has it: 3.74in deep against the left wall, 4.02in wide, 11.44in
// from the near wall, as its face and two sides
FieldMap withLoader() {
    FieldMap map = FieldMap::centered();
    map.addSegment({-66.51, -60.82, -66.51, -56.80});
    map.addSegment({-70.25, -60.82, -66.51, -60.82});
    map.addSegment({-70.25, -56.80, -66.51, -56.80});
    return map;
}

void testConeSeesWhatTheRayMisses() {
    // SL000028's start: the front sensor 8.92in from the loader's face, its center ray 2.04in clear
    // of the face's near corner. The ray reads the wall behind; a 15 degree cone reaches the corner
    // (13 degrees off center), a 10 degree one doesn't
    const FieldMap map = withLoader();
    const double x = -57.59;
    const double y = -54.76;
    expectNear(map.castRayIn(x, y, -1, 0), 12.66, "the ray, past the loader");
    expectNear(map.castConeIn(x, y, -1, 0, 15), std::hypot(8.92, 2.04), "a 15 degree cone");
    assert(std::isinf(map.castConeIn(x, y, -1, 0, 10)));
    expectNear(map.castBeamIn(x, y, -1, 0, 15), std::hypot(8.92, 2.04), "the beam, 15 degrees");
    expectNear(map.castBeamIn(x, y, -1, 0, 10), 12.66, "the beam, 10 degrees: the wall");
    expectNear(map.castBeamIn(x, y, -1, 0, 0), 12.66, "the beam, no cone: the ray");
}

void testConeLeavesOutFacesAlongTheBeam() {
    // SL000034's press: the face 19 degrees off center, out of the cone, and a side running along
    // the beam inside it, which the sensor read past
    const FieldMap map = withLoader();
    assert(std::isinf(map.castConeIn(-57.85, -53.73, -1, 0, 15)));
    // looked at from 45 degrees, both its face and its side face the beam enough to be seen
    const double rad = 45 * 3.14159265358979323846 / 180;
    assert(std::isfinite(map.castConeIn(-60.0, -52.0, -std::sin(rad), -std::cos(rad), 15)));
}

void testConeIgnoresTheWalls() {
    // the walls are the center ray's alone, however much of the cone they fill
    const FieldMap map = FieldMap::centered();
    for (double heading = 0; heading < 360; heading += 30) {
        const double rad = heading * 3.14159265358979323846 / 180;
        assert(std::isinf(map.castConeIn(40, -60, std::sin(rad), std::cos(rad), 30)));
        expectNear(map.castBeamIn(40, -60, std::sin(rad), std::cos(rad), 30),
                   map.castRayIn(40, -60, std::sin(rad), std::cos(rad)), "a beam of walls alone");
    }
}

void testConeIsTheNearestPointInside() {
    // a segment square across the beam, 10in out: its foot when the cone takes it in, else the
    // end of it nearest the foot that's inside the cone (where the cone's edge crosses it)
    FieldMap map = FieldMap::centered(1000);
    map.addSegment({10, -3, 10, 20});
    expectNear(map.castConeIn(0, 0, 1, 0, 20), 10, "the foot, inside");
    FieldMap offCenter = FieldMap::centered(1000);
    offCenter.addSegment({10, 2, 10, 20});
    expectNear(offCenter.castConeIn(0, 0, 1, 0, 20), std::hypot(10, 2), "the near end, inside");
    // a 15 degree cone is 2.68in across at 10in, short of a segment starting 3in over
    FieldMap clear = FieldMap::centered(1000);
    clear.addSegment({10, 3, 10, 20});
    assert(std::isinf(clear.castConeIn(0, 0, 1, 0, 15)));
    // half inside: the cone's edge crosses a slanted segment, whose nearest point is its end at
    // (10, 1)
    FieldMap slanted = FieldMap::centered(1000);
    slanted.addSegment({10, 1, 12, 5});
    expectNear(slanted.castConeIn(0, 0, 1, 0, 15), std::hypot(10, 1), "slanted, nearest end");
    // behind the tip, nothing
    assert(std::isinf(map.castConeIn(20, 0, 1, 0, 20)));
}

void testConeAgreesWithManyRays() {
    // the nearest hit among rays every 0.005 degrees across the cone, which can only be as near
    // or further, against castConeIn(): the same up to how far apart the rays land. The rays see
    // only the segments facing the beam, which is all the cone sees, and the walls are far off
    const Segment segments[] = {{-5, 20, 5, 20},    {5, 20, 5, 30},     {5, 30, -5, 30},
                                {-5, 30, -5, 20},   {30, -12, 34, -12}, {34, -12, 34, -4},
                                {34, -4, 30, -4},   {30, -4, 30, -12},  {-40, -40, -20, -50},
                                {-30, 10, -18, 14}};
    FieldMap map = FieldMap::centered(10000);
    for (const Segment& s : segments) map.addSegment(s);
    const double pi = 3.14159265358979323846;
    const double halfDeg = 15.0;
    double worst = 0.0;
    int hits = 0;
    for (int heading = 7; heading < 360; heading += 29) {
        const double rad = heading * pi / 180;
        const double dirX = std::sin(rad);
        const double dirY = std::cos(rad);
        FieldMap facing = FieldMap::centered(10000);
        for (const Segment& s : segments) {
            const double ex = s.x2In - s.x1In;
            const double ey = s.y2In - s.y1In;
            if (std::fabs(dirX * ey - dirY * ex) / std::hypot(ex, ey) >= 0.5) facing.addSegment(s);
        }
        for (double x = -45.3; x < 45.0; x += 11.9) {
            for (double y = -45.1; y < 45.0; y += 13.3) {
                const double cone = map.castConeIn(x, y, dirX, dirY, halfDeg);
                double rays = std::numeric_limits<double>::infinity();
                for (double offset = -halfDeg; offset <= halfDeg + 1e-9; offset += 0.005) {
                    const double r = (heading + offset) * pi / 180;
                    const double hit = facing.castRayIn(x, y, std::sin(r), std::cos(r));
                    if (hit < 1000.0) rays = std::min(rays, hit);
                }
                if (std::isinf(cone)) {
                    // a sliver of a segment right at the cone's edge can slip past the cone's
                    // rounding but not the rays'; none here
                    assert(std::isinf(rays));
                    continue;
                }
                assert(cone <= rays + 1e-9);
                if (std::isinf(rays)) continue;
                ++hits;
                worst = std::max(worst, rays - cone);
            }
        }
    }
    std::printf("  cone within %.1e of the nearest of many rays over %d cones\n", worst, hits);
    assert(hits > 20);
    // rays 0.005 degrees apart land at most about 0.03in apart on a face 130in off and turned 60
    // degrees from them
    assert(worst < 0.05);
}

} // namespace

void testConeCasterMatchesCastCone() {
    // ConeCaster against castConeIn(), as the parallel caster against castRayIn(): starts inside
    // and outside the walls, every direction, cones narrow and wide
    FieldMap map = withLoader();
    map.addBox(-5, 20, 5, 30);
    map.addBox(60, -10, 80, 10);
    map.addSegment({-40, -40, -20, -50});
    ConeCaster cone;
    double worst = 0.0;
    int hits = 0;
    for (const double halfDeg : {5.0, 15.0, 40.0}) {
        for (int heading = 0; heading < 360; heading += 15) {
            const double rad = heading * 3.14159265358979323846 / 180.0;
            const double dirX = std::sin(rad);
            const double dirY = std::cos(rad);
            cone.aim(map, dirX, dirY, halfDeg);
            for (double x = -90.3; x < 90.0; x += 7.7) {
                for (double y = -90.13; y < 90.0; y += 9.1) {
                    const double expected = map.castConeIn(x, y, dirX, dirY, halfDeg);
                    const double got = cone.castIn(x, y);
                    if (std::isinf(expected) || std::isinf(got)) {
                        if (std::isinf(expected) != std::isinf(got)) {
                            std::printf("FAIL cone at (%.1f, %.1f) heading %d, %.0f deg: got %.9f, "
                                        "expected %.9f\n",
                                        x, y, heading, halfDeg, got, expected);
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
    std::printf("  cone caster within %.1e of castConeIn() over %d hits\n", worst, hits);
    assert(hits > 1000);
    assert(worst < 1e-9);
    // no cone, nothing
    cone.aim(map, 1, 0, 0);
    assert(std::isinf(cone.castIn(-60, -58)));
}

int main() {
    testCenteredFieldWalls();
    testDiagonalRay();
    testBoxBlocksTheRay();
    testRayFromOutsidePointingAway();
    testContains();
    testCastRayMatchesHeadingVersion();
    testParallelRayCasterMatchesCastRay();
    testConeSeesWhatTheRayMisses();
    testConeLeavesOutFacesAlongTheBeam();
    testConeIgnoresTheWalls();
    testConeIsTheNearestPointInside();
    testConeAgreesWithManyRays();
    testConeCasterMatchesCastCone();
    std::printf("field_map_test: all tests passed\n");
    return 0;
}
