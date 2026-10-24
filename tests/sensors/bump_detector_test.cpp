/*
 * Host-side unit test for sapphirelib::sensors::BumpDetector, no
 * PROS/embedded dependencies, so it builds and runs with a normal desktop
 * compiler.
 *
 * (A block comment, not //, so the backslash-continued build line doesn't
 * trip -Wcomment.)
 *
 * Build & run:
 *   g++ -std=c++20 -Iinclude tests/sensors/bump_detector_test.cpp \
 *       src/sapphirelib/sensors/bump_detector.cpp \
 *       -o bump_detector_test && ./bump_detector_test
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "sapphirelib/sensors/bump_detector.hpp"

using sapphirelib::sensors::BumpConfig;
using sapphirelib::sensors::BumpDetector;

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                       \
            std::exit(1);                                                                          \
        }                                                                                          \
    } while (0)

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::uint32_t kTickMs = 10;

/// Feeds `ticks` readings of the same acceleration, one every kTickMs from
/// `nowMs`, and returns how many counted as bumps. `nowMs` ends after the last.
int feed(BumpDetector& bumps, int ticks, double x, double y, double z, std::uint32_t& nowMs) {
    int count = 0;
    for (int i = 0; i < ticks; ++i) {
        if (bumps.update(x, y, z, nowMs)) ++count;
        nowMs += kTickMs;
    }
    return count;
}

void testStillIsNoBump() {
    BumpDetector bumps;
    std::uint32_t now = 1000;
    // flat and at rest: gravity on z, a little noise
    for (int i = 0; i < 200; ++i) {
        const double noise = (i % 2 == 0 ? 0.01 : -0.01);
        CHECK(!bumps.update(noise, -noise, 1.0 + noise, now));
        now += kTickMs;
    }
    CHECK(bumps.lastJoltG() < 0.05);
}

void testGravityOnAnyAxisIsNoBump() {
    // an IMU mounted on its side has gravity on x: still nothing
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 100, 1.0, 0.0, 0.0, now) == 0);
}

void testHitIsOneBump() {
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    // a hit from the side: one reading 1.5g off
    CHECK(bumps.update(0.0, 1.5, 1.0, now));
    CHECK(std::fabs(bumps.lastJoltG() - 1.5) < 1e-9);
    now += kTickMs;
    // the hit ringing for a few readings is still the same bump
    CHECK(feed(bumps, 3, 0.0, -1.2, 1.0, now) == 0);
    CHECK(feed(bumps, 20, 0.0, 0.0, 1.0, now) == 0);
}

void testHoldoff() {
    BumpDetector bumps(BumpConfig{.holdoffMs = 500});
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    CHECK(bumps.update(2.0, 0.0, 1.0, now));
    now += kTickMs;
    CHECK(feed(bumps, 20, 0.0, 0.0, 1.0, now) == 0); // 210ms after
    CHECK(!bumps.update(2.0, 0.0, 1.0, now));
    now += kTickMs;
    CHECK(feed(bumps, 30, 0.0, 0.0, 1.0, now) == 0); // 520ms after
    CHECK(bumps.update(2.0, 0.0, 1.0, now));
}

void testDrivingIsNoBump() {
    // a drivetrain's own start: 0 to 0.6g forward over 200ms, held, then a
    // stop just as hard the other way. Steady changes like these sit in the
    // average
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    for (int i = 1; i <= 20; ++i) {
        CHECK(!bumps.update(0.03 * i, 0.0, 1.0, now));
        now += kTickMs;
    }
    CHECK(feed(bumps, 50, 0.6, 0.0, 1.0, now) == 0);
    for (int i = 20; i >= -20; --i) {
        CHECK(!bumps.update(0.03 * i, 0.0, 1.0, now));
        now += kTickMs;
    }
    CHECK(feed(bumps, 50, -0.6, 0.0, 1.0, now) == 0);
}

void testSmallJoltIsNoBump() {
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    CHECK(!bumps.update(0.5, 0.0, 1.0, now));
    CHECK(std::fabs(bumps.lastJoltG() - 0.5) < 1e-9);
}

void testThresholdIsConfigurable() {
    BumpDetector bumps(BumpConfig{.thresholdG = 0.4});
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    CHECK(bumps.update(0.5, 0.0, 1.0, now));
}

void testEveryAxisAddsUp() {
    // 0.5g on each axis at once is sqrt(0.75) = 0.87g off: a bump, though no
    // one axis is
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    CHECK(bumps.update(0.5, 0.5, 1.5, now));
    CHECK(std::fabs(bumps.lastJoltG() - std::sqrt(0.75)) < 1e-9);
}

void testDropoutStartsOver() {
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    // unplugged (PROS_ERR_F), then back reading something else entirely
    CHECK(!bumps.update(kInf, kInf, kInf, now));
    CHECK(bumps.lastJoltG() == 0.0);
    now += kTickMs;
    CHECK(feed(bumps, 1, 0.0, 0.0, -1.0, now) == 0);
    CHECK(feed(bumps, 20, 0.0, 0.0, -1.0, now) == 0);
    // and it still sees a real one after
    CHECK(bumps.update(1.5, 0.0, -1.0, now));
}

void testGapStartsOver() {
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    // nothing for half a second, then a reading far off: the average was
    // stale, so it starts over from here instead
    now += 500;
    CHECK(!bumps.update(1.5, 0.0, 1.0, now));
    now += kTickMs;
    CHECK(feed(bumps, 20, 1.5, 0.0, 1.0, now) == 0);
}

void testResetForgetsHoldoff() {
    BumpDetector bumps;
    std::uint32_t now = 0;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0);
    CHECK(bumps.update(2.0, 0.0, 1.0, now));
    bumps.reset();
    now += kTickMs;
    CHECK(feed(bumps, 5, 0.0, 0.0, 1.0, now) == 0);
    CHECK(bumps.update(2.0, 0.0, 1.0, now));
}

void testClockWrap() {
    BumpDetector bumps;
    std::uint32_t now = 0xFFFFFF00u;
    CHECK(feed(bumps, 50, 0.0, 0.0, 1.0, now) == 0); // across the wrap
    CHECK(bumps.update(2.0, 0.0, 1.0, now));
}

} // namespace

int main() {
    testStillIsNoBump();
    testGravityOnAnyAxisIsNoBump();
    testHitIsOneBump();
    testHoldoff();
    testDrivingIsNoBump();
    testSmallJoltIsNoBump();
    testThresholdIsConfigurable();
    testEveryAxisAddsUp();
    testDropoutStartsOver();
    testGapStartsOver();
    testResetForgetsHoldoff();
    testClockWrap();
    std::printf("bump_detector_test: all passed\n");
    return 0;
}
