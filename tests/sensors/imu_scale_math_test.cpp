// Host-side unit test for sapphirelib::sensors::rawHeadingDeltaDeg,
// wrapDegrees360, fieldHeadingDeg, headingOffsetFor, and
// calibrateHeadingScale — no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/sensors/imu_scale_math_test.cpp \
//       src/sapphirelib/sensors/imu_scale_math.cpp src/sapphirelib/util/angle.cpp \
//       -o imu_scale_math_test && ./imu_scale_math_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/sensors/imu_scale_math.hpp"

using sapphirelib::sensors::calibrateHeadingScale;
using sapphirelib::sensors::fieldHeadingDeg;
using sapphirelib::sensors::headingOffsetFor;
using sapphirelib::sensors::rawHeadingDeltaDeg;
using sapphirelib::sensors::wrapDegrees360;

namespace {

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-9) {
        std::printf("FAIL %s: got %f, expected %f\n", label, actual, expected);
        assert(false);
    }
}

void testRawHeadingDeltaWithinRange() {
    expectNear(rawHeadingDeltaDeg(0.0, 10.0), 10.0, "0->10");
    expectNear(rawHeadingDeltaDeg(10.0, 0.0), -10.0, "10->0");
}

void testRawHeadingDeltaAcrossSeam() {
    // 355 -> 5 is a +10 degree turn, not -350.
    expectNear(rawHeadingDeltaDeg(355.0, 5.0), 10.0, "355->5");
    // 5 -> 355 is a -10 degree turn, not +350.
    expectNear(rawHeadingDeltaDeg(5.0, 355.0), -10.0, "5->355");
}

void testCumulativeTrackingAccumulatesPastOneRevolution() {
    // Simulate spinning through 355 -> 5 -> 15, accumulating deltas exactly
    // like Imu::updateCumulative() does — should total +20, not wrap.
    double cumulative = 0.0;
    cumulative += rawHeadingDeltaDeg(355.0, 5.0);
    cumulative += rawHeadingDeltaDeg(5.0, 15.0);
    expectNear(cumulative, 20.0, "cumulative across seam");
}

void testWrapDegrees360() {
    expectNear(wrapDegrees360(0.0), 0.0, "0");
    expectNear(wrapDegrees360(359.0), 359.0, "359");
    expectNear(wrapDegrees360(360.0), 0.0, "360");
    expectNear(wrapDegrees360(1260.0), 180.0, "1260 (3.5 turns)");
    expectNear(wrapDegrees360(-10.0), 350.0, "-10");
    expectNear(wrapDegrees360(-370.0), 350.0, "-370");
}

/// Distance between two headings on the circle, so 359.9999999 and 0 count
/// as equal the way a heading reading means them.
double headingDistance(double a, double b) {
    const double d = std::fabs(std::fmod(a - b, 360.0));
    return std::fmin(d, 360.0 - d);
}

void expectHeading(double actual, double expected, const char* label) {
    if (headingDistance(actual, expected) >= 1e-9 || actual < 0.0 || actual > 360.0) {
        std::printf("FAIL %s: got %.12f, expected %.12f (mod 360)\n", label, actual, expected);
        assert(false);
    }
}

void testFieldHeadingWithoutOffsetIsWrappedRotation() {
    // Offset 0 is what every Imu starts with: getHeadingDeg() must read
    // exactly what it did before offsets existed.
    for (double cumulative = -1000.0; cumulative <= 1000.0; cumulative += 7.25) {
        expectNear(fieldHeadingDeg(cumulative, 0.0), wrapDegrees360(cumulative),
                   "offset 0 == wrapDegrees360");
    }
}

void testOffsetRoundTrip() {
    // setHeadingDeg(target) stores headingOffsetFor(target, cumulative); the
    // very next read must give the target back — for any rotation history
    // (negative, several turns) and any target, seam values included.
    const double cumulatives[] = {0.0, 12.5, -12.5, 359.999, 360.0, 725.0, -1080.3, 12345.678};
    const double targets[] = {0.0, 90.0, 180.0, 270.0, 359.999, 360.0, -90.0, 450.0, 1e-9};
    for (const double cumulative : cumulatives) {
        for (const double target : targets) {
            const double offset = headingOffsetFor(target, cumulative);
            if (!(offset > -180.0 && offset <= 180.0)) {
                std::printf("FAIL offset range: %f for target %f\n", offset, target);
                assert(false);
            }
            expectHeading(fieldHeadingDeg(cumulative, offset), target, "round trip");
        }
    }
}

void testRotationAfterOffsetKeepsTracking() {
    // Re-frame to 270 at some arbitrary rotation, then turn: the heading has
    // to move by exactly what the robot turned, through the 0/360 seam.
    const double cumulativeAtSet = 37.0;
    const double offset = headingOffsetFor(270.0, cumulativeAtSet);
    expectHeading(fieldHeadingDeg(cumulativeAtSet + 45.0, offset), 315.0, "270 + 45");
    expectHeading(fieldHeadingDeg(cumulativeAtSet + 100.0, offset), 10.0, "270 + 100 wraps");
    expectHeading(fieldHeadingDeg(cumulativeAtSet - 270.0, offset), 0.0, "270 - 270");
    // Re-framing again replaces the offset rather than stacking on it.
    const double second = headingOffsetFor(0.0, cumulativeAtSet + 100.0);
    expectHeading(fieldHeadingDeg(cumulativeAtSet + 100.0, second), 0.0, "second re-frame");
}

void testCalibrateHeadingScale() {
    // IMU under-reports: chassis actually did 10 full turns, IMU only
    // measured 9.8 — scale should be > 1 to correct future readings up.
    expectNear(calibrateHeadingScale(/*actualTurns=*/10.0, /*measuredTurns=*/9.8), 10.0 / 9.8,
               "under-reporting IMU");
    // Exact measurement needs no correction.
    expectNear(calibrateHeadingScale(10.0, 10.0), 1.0, "exact IMU");
}

} // namespace

int main() {
    testRawHeadingDeltaWithinRange();
    testRawHeadingDeltaAcrossSeam();
    testCumulativeTrackingAccumulatesPastOneRevolution();
    testWrapDegrees360();
    testFieldHeadingWithoutOffsetIsWrappedRotation();
    testOffsetRoundTrip();
    testRotationAfterOffsetKeepsTracking();
    testCalibrateHeadingScale();
    std::puts("imu_scale_math_test: all assertions passed");
    return 0;
}
