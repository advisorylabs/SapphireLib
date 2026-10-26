// Host-side unit test for sapphirelib::localization::wallFix, no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/localization/wall_reset_test.cpp \
//       src/sapphirelib/localization/wall_reset.cpp src/sapphirelib/localization/field_map.cpp \
//       src/sapphirelib/localization/sensor_model.cpp -o wall_reset_test && ./wall_reset_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "sapphirelib/localization/wall_reset.hpp"

using sapphirelib::localization::DistanceSensorMount;
using sapphirelib::localization::FieldMap;
using sapphirelib::localization::WallAxis;
using sapphirelib::localization::wallFix;
using sapphirelib::localization::WallResetConfig;

namespace {

constexpr double kPi = 3.14159265358979323846;

const FieldMap kField = FieldMap::centered(); // walls at +-70.25
const DistanceSensorMount kFront{.forwardIn = 7.0, .rightIn = 0.0, .facingDeg = 0.0};
const DistanceSensorMount kBack{.forwardIn = -7.0, .rightIn = 0.0, .facingDeg = 180.0};
const DistanceSensorMount kLeft{.forwardIn = 0.0, .rightIn = -6.0, .facingDeg = 270.0};

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-9) {
        std::printf("FAIL %s: got %f, expected %f\n", label, actual, expected);
        assert(false);
    }
}

void expectSkipped(const sapphirelib::localization::WallFix& fix, const char* why) {
    if (fix.ok || std::strcmp(fix.why, why) != 0) {
        std::printf("FAIL: expected skipped (%s), got ok=%d why=%s\n", why, fix.ok, fix.why);
        assert(false);
    }
}

void testBackSensorSetsY() {
    // at (0, -50) facing +y, the back sensor's face is at y = -57, 13.25in from the near wall. It
    // reads 15.25: the robot is 2in further up the field than odometry thinks
    const auto fix = wallFix(kField, 0, -50, 0, kBack, 15.25, {});
    assert(fix.ok && fix.axis == WallAxis::y);
    expectNear(fix.valueIn, -48.0, "y from the back sensor");
    expectNear(fix.shiftIn, 2.0, "shift");
}

void testAgreeingReadingMovesNothing() {
    const auto fix = wallFix(kField, 0, -50, 0, kBack, 13.25, {});
    assert(fix.ok);
    expectNear(fix.shiftIn, 0.0, "no shift");
}

void testSideSensorSetsX() {
    // facing +y, the left sensor looks at the left wall from x = -36 - 6 = -42: 28.25in. It reads
    // 27.25, so the robot is 1in nearer the left wall
    const auto fix = wallFix(kField, -36, 0, 0, kLeft, 27.25, {});
    assert(fix.ok && fix.axis == WallAxis::x);
    expectNear(fix.valueIn, -37.0, "x from the left sensor");
}

void testFacingRightTheFrontSensorSetsX() {
    // heading 90 faces +x: the front sensor's beam goes into the right wall
    const auto fix = wallFix(kField, 30, 10, 90, kFront, 30.25, {});
    assert(fix.ok && fix.axis == WallAxis::x);
    expectNear(fix.valueIn, 33.0, "x from the front sensor facing right");
}

void testAngledBeamUsesItsShareAcrossTheWall() {
    // turned 20 degrees: the front sensor's face is at (7 sin 20, 7 cos 20) from the robot, and a
    // reading r along the beam is r cos 20 across the far wall
    const double h = 20.0 * kPi / 180.0;
    const double reading = 40.0;
    const auto fix = wallFix(kField, 0, 0, 20, kFront, reading, {});
    assert(fix.ok && fix.axis == WallAxis::y);
    expectNear(fix.valueIn, 70.25 - reading * std::cos(h) - 7.0 * std::cos(h), "angled");
}

void testTooSteepIsSkipped() {
    expectSkipped(wallFix(kField, 0, 0, 40, kFront, 60, {}), "too steep");
    // allowed when the limit is raised
    assert(wallFix(kField, 0, 0, 40, kFront, 60, {.maxAngleDeg = 45}).ok);
}

void testNearACornerIsSkipped() {
    // the front sensor at x = 60 lands 10.25in from the right wall's corner
    const WallResetConfig override{.cornerMarginIn = 18};
    expectSkipped(wallFix(kField, 60, 0, 0, kFront, 63.25, override), "near a corner");
    assert(wallFix(kField, 60, 0, 0, kFront, 63.25, {}).ok);
    assert(wallFix(kField, 40, 0, 0, kFront, 63.25, override).ok);
}

void testTooFarFromOdometryIsSkipped() {
    // something 20in in front of the wall: the reading says the robot is 20in further up
    const WallResetConfig limited{.maxJumpIn = 8};
    expectSkipped(wallFix(kField, 0, 0, 0, kFront, 43.25, limited), "too far from odometry");
    const auto unlimited = wallFix(kField, 0, 0, 0, kFront, 43.25, {});
    assert(unlimited.ok);
    expectNear(unlimited.shiftIn, 20.0, "no limit");
}

void testBadInputsAreSkipped() {
    expectSkipped(wallFix(kField, 0, 0, 0, kFront, NAN, {}), "no reading");
    expectSkipped(wallFix(kField, 0, 0, 0, kFront, -1, {}), "no reading");
    expectSkipped(wallFix(kField, 0, 80, 0, kFront, 5, {}), "sensor outside the field");
}

} // namespace

int main() {
    testBackSensorSetsY();
    testAgreeingReadingMovesNothing();
    testSideSensorSetsX();
    testFacingRightTheFrontSensorSetsX();
    testAngledBeamUsesItsShareAcrossTheWall();
    testTooSteepIsSkipped();
    testNearACornerIsSkipped();
    testTooFarFromOdometryIsSkipped();
    testBadInputsAreSkipped();
    std::puts("wall_reset_test: all assertions passed");
    return 0;
}
