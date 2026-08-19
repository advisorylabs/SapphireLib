// Host-side unit test for sapphirelib::sensors::rawHeadingDeltaDeg,
// wrapDegrees360, and calibrateHeadingScale — no PROS/embedded
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
    testCalibrateHeadingScale();
    std::puts("imu_scale_math_test: all assertions passed");
    return 0;
}
