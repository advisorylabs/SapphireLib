// Host-side unit test for sapphirelib::localization's distance sensor model,
// no PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/localization/sensor_model_test.cpp src/sapphirelib/localization/sensor_model.cpp -o sensor_model_test && ./sensor_model_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "sapphirelib/localization/sensor_model.hpp"

using namespace sapphirelib::localization;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectNear(double actual, double expected, const char* label, double tolerance = 1e-9) {
    if (!(std::fabs(actual - expected) < tolerance)) {
        std::printf("FAIL %s: got %.12f, expected %.12f\n", label, actual, expected);
        assert(false);
    }
}

void testSensorRayAtHeadingZero() {
    // facing +y, forward is +y and right is +x
    const SensorRay ray = sensorRay(10, 20, 0, {.forwardIn = 7, .rightIn = 2, .facingDeg = 90});
    expectNear(ray.xIn, 12, "x");
    expectNear(ray.yIn, 27, "y");
    expectNear(ray.dirX, 1, "facing right is +x");
    expectNear(ray.dirY, 0, "facing right has no y");
}

void testSensorRayRotatesWithTheRobot() {
    // facing +x (heading 90), forward is +x and right is -y
    const SensorRay ray = sensorRay(0, 0, 90, {.forwardIn = 7, .rightIn = 2, .facingDeg = 0});
    expectNear(ray.xIn, 7, "x");
    expectNear(ray.yIn, -2, "y");
    expectNear(ray.dirX, 1, "dir x");
    expectNear(ray.dirY, 0, "dir y", 1e-12);

    // a back sensor on a robot facing -x points +x
    const SensorRay back = sensorRay(0, 0, 270, {.forwardIn = -6, .facingDeg = 180});
    expectNear(back.xIn, 6, "back x");
    expectNear(back.dirX, 1, "back dir x");
}

void testSigmaFollowsTheSpec() {
    const BeamModel beam;
    // 15mm floor below ~12in, 5% above
    expectNear(readingSigmaIn(beam, {.distanceIn = 5, .valid = true}), 0.6, "near floor");
    expectNear(readingSigmaIn(beam, {.distanceIn = 40, .valid = true}), 2.0, "5% of 40in");
    expectNear(readingSigmaIn(beam, {.distanceIn = 40, .valid = true, .extraSigmaIn = 1.5}), 2.5,
               "plus extra, in quadrature");
}

void testLikelihoodPeaksAtTheExpectedDistance() {
    const BeamModel beam;
    const double sigma = 2.0;
    const double atExpected = readingLogLikelihood(40, 40, sigma, beam);
    const double oneSigmaOff = readingLogLikelihood(42, 40, sigma, beam);
    const double farOff = readingLogLikelihood(10, 40, sigma, beam);
    assert(atExpected > oneSigmaOff && oneSigmaOff > farOff);

    // exact value at the peak: the Gaussian part plus the outlier floor
    const double peak = 0.9 / (std::sqrt(2 * kPi) * sigma) + 0.1 / 78.0;
    expectNear(atExpected, std::log(peak), "peak value");

    // far off, it bottoms out at the outlier floor instead of going to -inf
    expectNear(farOff, std::log(0.1 / 78.0), "outlier floor", 1e-6);
    const double noHit =
        readingLogLikelihood(40, std::numeric_limits<double>::infinity(), sigma, beam);
    expectNear(noHit, std::log(0.1 / 78.0), "hypothesis that would see nothing");
}

void testReadingConversion() {
    const BeamModel beam;
    const DistanceReading good = distanceReadingFromMm(1016, 63, 20, beam);
    assert(good.valid);
    expectNear(good.distanceIn, 40, "1016mm is 40in");

    assert(!distanceReadingFromMm(9999, 63, 20, beam).valid);                  // no object
    assert(!distanceReadingFromMm(INT32_MAX, INT32_MAX, 20, beam).valid);      // PROS_ERR
    assert(!distanceReadingFromMm(0, 63, 20, beam).valid);                     // nonsense
    assert(!distanceReadingFromMm(1500, 10, 20, beam).valid);                  // low confidence
    assert(distanceReadingFromMm(150, 0, 20, beam).valid);                     // no confidence up close
    assert(!distanceReadingFromMm(2100, 63, 20, beam).valid);                  // past maxRangeIn
}

void testLatencyCompensation() {
    const DistanceReading reading{.distanceIn = 30, .valid = true, .extraSigmaIn = 0};
    // closing at 40in/s with 30ms of latency: 1.2in closer by now
    const DistanceReading now = compensateLatency(reading, 40, 0.03);
    expectNear(now.distanceIn, 28.8, "closing");
    expectNear(now.extraSigmaIn, 0.6, "half the shift as uncertainty");
    const DistanceReading away = compensateLatency(reading, -40, 0.03);
    expectNear(away.distanceIn, 31.2, "moving away");

    // invalid readings pass through
    assert(!compensateLatency(DistanceReading{}, 40, 0.03).valid);
    // closing fast on something close can't produce a negative distance
    assert(!compensateLatency({.distanceIn = 1, .valid = true}, 60, 0.03).valid);
}

} // namespace

void testScorerMatchesTheExactLikelihood() {
    // ReadingScorer replaces readingLogLikelihood()'s exp() and log() with a lookup table: within
    // 1e-5 everywhere, including seeing nothing, no outliers, and nothing but outliers
    double worst = 0.0;
    for (const double outliers : {0.0, 0.001, 0.1, 0.5, 1.0}) {
        for (const double sigma : {0.05, 0.6, 2.0, 5.0}) {
            const BeamModel beam{.outlierProbability = outliers};
            for (double measured = 1.0; measured < 80.0; measured += 7.3) {
                const ReadingScorer scorer(measured, sigma, beam);
                for (double expected = 0.0; expected < 90.0; expected += 0.37) {
                    const double exact = readingLogLikelihood(measured, expected, sigma, beam);
                    const double fast = scorer.logLikelihood(expected);
                    if (exact < -700.0) {
                        // with no outliers, exp() goes subnormal and then 0 far from the reading,
                        // and readingLogLikelihood() loses its precision; the scorer works in
                        // logs there, and is just as hopeless
                        assert(fast < -690.0);
                        continue;
                    }
                    worst = std::max(worst, std::fabs(fast - exact));
                }
                const double inf = std::numeric_limits<double>::infinity();
                const double exactNothing = readingLogLikelihood(measured, inf, sigma, beam);
                if (std::isinf(exactNothing)) {
                    assert(scorer.logLikelihood(inf) < -690.0);
                } else {
                    expectNear(scorer.logLikelihood(inf), exactNothing, "seeing nothing", 1e-12);
                }
            }
        }
    }
    std::printf("  scorer within %.1e of readingLogLikelihood()\n", worst);
    assert(worst < 1e-5);
    // a default scorer gives everything 0
    expectNear(ReadingScorer().logLikelihood(12.0), 0.0, "default scorer");
}

int main() {
    testSensorRayAtHeadingZero();
    testSensorRayRotatesWithTheRobot();
    testSigmaFollowsTheSpec();
    testLikelihoodPeaksAtTheExpectedDistance();
    testReadingConversion();
    testLatencyCompensation();
    testScorerMatchesTheExactLikelihood();
    std::printf("sensor_model_test: all tests passed\n");
    return 0;
}
