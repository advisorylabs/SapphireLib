// Host-side unit test for sapphirelib::localization::ParticleFilter, no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Besides the unit checks, it drives a simulated robot around a field on
// odometry with a known bias and checks that the filter holds the true
// position while odometry alone drifts off, shrugs off a blocked sensor, and
// recovers from a bump odometry never saw. testGoldenRun pins one exact
// seeded run: the simulator's JavaScript port (tools/sim/js/mcl.js) is
// checked against the same numbers in tools/sim/test/mcl.test.js. Change one
// side, change the other.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/localization/particle_filter_test.cpp src/sapphirelib/localization/particle_filter.cpp src/sapphirelib/localization/field_map.cpp src/sapphirelib/localization/sensor_model.cpp -o particle_filter_test && ./particle_filter_test

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sapphirelib/localization/particle_filter.hpp"
#include "sapphirelib/util/random.hpp"

using namespace sapphirelib::localization;
using sapphirelib::Rng;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.12f, expected %.12f (+-%g)\n", label, actual, expected,
                    tolerance);
        assert(false);
    }
}

void expectBelow(double actual, double limit, const char* label) {
    if (!(actual < limit)) {
        std::printf("FAIL %s: got %.4f, expected below %.4f\n", label, actual, limit);
        assert(false);
    }
}

// four sensors, one per side, 7in out from the tracking center
std::vector<DistanceSensorMount> fourSensors() {
    return {
        {.forwardIn = 7, .rightIn = 0, .facingDeg = 0},
        {.forwardIn = 0, .rightIn = 7, .facingDeg = 90},
        {.forwardIn = -7, .rightIn = 0, .facingDeg = 180},
        {.forwardIn = 0, .rightIn = -7, .facingDeg = 270},
    };
}

// what each sensor reads from a true pose: the map's distance plus Gaussian noise at half the
// sensor's spec, with nothing past maxRange. noise == nullptr for exact readings
std::array<DistanceReading, 4> readingsFrom(const FieldMap& world,
                                            const std::vector<DistanceSensorMount>& mounts,
                                            double xIn, double yIn, double headingDeg,
                                            Rng* noise) {
    std::array<DistanceReading, 4> readings{};
    const BeamModel beam;
    for (std::size_t s = 0; s < mounts.size(); ++s) {
        const SensorRay ray = sensorRay(xIn, yIn, headingDeg, mounts[s]);
        double distance = world.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
        if (noise != nullptr) {
            distance += 0.5 * std::max(0.6, 0.05 * distance) * noise->gaussian();
        }
        readings[s].distanceIn = distance;
        readings[s].valid = distance <= beam.maxRangeIn;
    }
    return readings;
}

double distance(double x1, double y1, double x2, double y2) { return std::hypot(x1 - x2, y1 - y2); }

void testResetScattersAroundThePosition() {
    ParticleFilter filter(FieldMap::centered(), fourSensors(), {.particleCount = 4000});
    filter.reset(10, -5, 2.0);
    const Estimate e = filter.estimate();
    expectNear(e.xIn, 10, 0.1, "reset mean x");
    expectNear(e.yIn, -5, 0.1, "reset mean y");
    expectNear(e.spreadIn, 2.0 * std::sqrt(2.0), 0.1, "reset spread");
    expectNear(e.effectiveParticles, 4000, 1e-6, "equal weights");
}

void testPredictMovesTheCloud() {
    ParticleFilter filter(FieldMap::centered(), fourSensors(), {.particleCount = 4000});
    filter.reset(0, 0, 1.0);
    const Estimate before = filter.estimate();
    filter.predict(5, -3, 0);
    const Estimate after = filter.estimate();
    expectNear(after.xIn - before.xIn, 5, 0.05, "predict x");
    expectNear(after.yIn - before.yIn, -3, 0.05, "predict y");
    // and the noise grows the spread: 0.02 + 3% of 5.83in
    assert(after.spreadIn > before.spreadIn);
}

void testWeighPullsTowardTheTruth() {
    const FieldMap map = FieldMap::centered();
    ParticleFilter filter(map, fourSensors());
    filter.reset(0, 0, 8.0);
    const double trueX = 6;
    const double trueY = -4;
    const double heading = 30;
    const auto readings = readingsFrom(map, fourSensors(), trueX, trueY, heading, nullptr);

    const double startError = distance(filter.estimate().xIn, filter.estimate().yIn, trueX, trueY);
    for (int i = 0; i < 20; ++i) {
        filter.predict(0, 0, 0);
        assert(filter.weigh(heading, readings));
        filter.resampleIfNeeded();
    }
    const Estimate e = filter.estimate();
    const double endError = distance(e.xIn, e.yIn, trueX, trueY);
    assert(endError < startError);
    expectBelow(endError, 0.3, "converged on the truth");
    expectBelow(e.spreadIn, 1.0, "and tightened up");
}

void testNoValidReadingsChangesNothing() {
    ParticleFilter filter(FieldMap::centered(), fourSensors());
    filter.reset(0, 0, 3.0);
    const Estimate before = filter.estimate();
    const std::array<DistanceReading, 4> none{};
    assert(!filter.weigh(0, none));
    const Estimate after = filter.estimate();
    expectNear(after.xIn, before.xIn, 0, "x unchanged");
    expectNear(after.effectiveParticles, before.effectiveParticles, 0, "weights unchanged");
}

void testResamplingFollowsTheWeights() {
    // weigh a wide cloud with one sensor, then resample: the new, equally weighted cloud should
    // describe the same distribution the weights did
    ParticleFilter oneSensor(FieldMap(0, 0, 100, 100), {{.facingDeg = 90}},
                             {.particleCount = 1000,
                              .motionNoise = {.baseIn = 0, .perInch = 0, .perDegreeIn = 0},
                              .beam = {.outlierProbability = 0.0},
                              .resampleThreshold = 1.1,
                              .recovery = {.enabled = false}});
    oneSensor.reset(50, 50, 10.0);
    const DistanceReading reading{.distanceIn = 50, .valid = true};
    oneSensor.weigh(0, std::span(&reading, 1));
    const Estimate before = oneSensor.estimate();
    assert(oneSensor.resampleIfNeeded());
    const Estimate after = oneSensor.estimate();
    // same distribution, drawn again: the mean barely moves and weights are equal again
    expectNear(after.xIn, before.xIn, 0.15, "resampled mean x");
    expectNear(after.effectiveParticles, 1000, 1e-6, "equal weights after resampling");
    expectNear(std::sqrt(after.varianceXIn2), std::sqrt(before.varianceXIn2), 0.1,
               "resampled x spread");
}

void testCheckSensors() {
    const FieldMap map = FieldMap::centered(140);
    ParticleFilter filter(map, fourSensors());
    auto readings = readingsFrom(map, fourSensors(), 0, 0, 0, nullptr);
    readings[0].distanceIn = 20; // something in front of the robot
    readings[3].valid = false;   // and the left sensor saw nothing
    std::array<SensorCheck, 4> checks{};
    const std::size_t agreeing = filter.checkSensors(0, 0, 0, readings, 3.0, checks);
    assert(agreeing == 2);
    assert(checks[0].used && !checks[0].agrees);
    expectNear(checks[0].expectedIn, 63, 1e-9, "front expected");
    assert(checks[1].agrees && checks[2].agrees);
    assert(!checks[3].used);
}

void testClosedLoopBeatsBiasedOdometry() {
    // odometry over-reads distance by 4%, the IMU is 1.5 degrees off, and a slipping wheel creeps
    // the pose 0.3in/s in +x; the sensors are noisy and drop 5% of readings. Drive laps of a 90in
    // square at 30in/s, turning as it goes. Scale and rotation errors mostly cancel over a lap, the
    // creep doesn't
    const FieldMap world = FieldMap::centered();
    const auto mounts = fourSensors();
    ParticleFilter filter(world, mounts, {.seed = 11});
    Rng noise(2024);

    const double corners[4][2] = {{-45, -45}, {45, -45}, {45, 45}, {-45, 45}};
    double x = -45;
    double y = -45;
    double heading = 0;
    double odomX = x;
    double odomY = y;
    filter.reset(x, y, 1.0);

    double worstMclError = 0;
    int corner = 1;
    const double dt = 0.05;
    for (int step = 0; step < 1600; ++step) { // 80s, 5+ laps
        const double tx = corners[corner][0];
        const double ty = corners[corner][1];
        const double toGo = distance(x, y, tx, ty);
        if (toGo < 1.0) {
            corner = (corner + 1) % 4;
            continue;
        }
        const double stepIn = std::min(30 * dt, toGo);
        const double dx = (tx - x) / toGo * stepIn;
        const double dy = (ty - y) / toGo * stepIn;
        const double turned = 45 * dt;
        x += dx;
        y += dy;
        heading += turned;

        // odometry's view of the same step
        const double biasRad = 1.5 * kPi / 180;
        const double odomDx = 1.04 * (dx * std::cos(biasRad) + dy * std::sin(biasRad)) + 0.3 * dt;
        const double odomDy = 1.04 * (-dx * std::sin(biasRad) + dy * std::cos(biasRad));
        odomX += odomDx;
        odomY += odomDy;

        filter.predict(odomDx, odomDy, turned);
        auto readings = readingsFrom(world, mounts, x, y, heading, &noise);
        for (DistanceReading& r : readings) {
            if (noise.uniform() < 0.05) r.valid = false;
        }
        filter.weigh(heading + 1.5, readings);
        const Estimate e = filter.estimate();
        filter.resampleIfNeeded();
        if (step > 20) worstMclError = std::max(worstMclError, distance(e.xIn, e.yIn, x, y));
    }

    const Estimate e = filter.estimate();
    const double odomError = distance(odomX, odomY, x, y);
    const double mclError = distance(e.xIn, e.yIn, x, y);
    std::printf("  closed loop: odometry off by %.1fin, MCL by %.2fin (worst %.2fin)\n", odomError,
                mclError, worstMclError);
    assert(odomError > 20);
    expectBelow(mclError, 1.5, "MCL final error");
    expectBelow(worstMclError, 3.5, "MCL worst error");
}

void testBlockedSensorDoesNotDragTheEstimate() {
    // a robot parks 15in in front of us for 5 seconds: the front sensor reads 15in the whole time
    const FieldMap world = FieldMap::centered();
    const auto mounts = fourSensors();
    ParticleFilter filter(world, mounts, {.seed = 3});
    Rng noise(8);
    filter.reset(10, 20, 1.0);
    for (int step = 0; step < 100; ++step) {
        filter.predict(0, 0, 0);
        auto readings = readingsFrom(world, mounts, 10, 20, 0, &noise);
        readings[0].distanceIn = 15 + 0.3 * noise.gaussian();
        filter.weigh(0, readings);
        filter.resampleIfNeeded();
    }
    const Estimate e = filter.estimate();
    expectBelow(distance(e.xIn, e.yIn, 10, 20), 1.0, "estimate with a blocked sensor");
}

void testRecoveryAfterABump() {
    // a hard hit shoves the robot 8in sideways and the tracking wheels miss it
    const FieldMap world = FieldMap::centered();
    const auto mounts = fourSensors();
    for (const bool recovery : {true, false}) {
        ParticleFilter filter(world, mounts, {.recovery = {.enabled = recovery}, .seed = 5});
        Rng noise(9);
        filter.reset(0, 0, 1.0);
        for (int step = 0; step < 20; ++step) {
            filter.predict(0, 0, 0);
            filter.weigh(0, readingsFrom(world, mounts, 0, 0, 0, &noise));
            filter.resampleIfNeeded();
        }
        int stepsToRecover = -1;
        for (int step = 0; step < 200; ++step) {
            filter.predict(0, 0, 0);
            filter.weigh(0, readingsFrom(world, mounts, 8, 0, 0, &noise));
            const Estimate e = filter.estimate();
            filter.resampleIfNeeded();
            if (distance(e.xIn, e.yIn, 8, 0) < 1.0) {
                stepsToRecover = step;
                break;
            }
        }
        std::printf("  bump: recovered in %d updates with recovery %s\n", stepsToRecover,
                    recovery ? "on" : "off");
        if (recovery) {
            assert(stepsToRecover >= 0);
            expectBelow(stepsToRecover, 20, "updates to recover with recovery on");
        }
    }
}

void testGoldenRun() {
    // a fixed, seeded sequence. The JavaScript port must land on the same numbers
    const FieldMap map = FieldMap::centered();
    ParticleFilter filter(map, fourSensors(), {.particleCount = 64, .seed = 1234});
    filter.reset(-20, 15, 3.0);
    const double trueX = -18;
    const double trueY = 16;
    for (int step = 0; step < 6; ++step) {
        filter.predict(0.4, -0.2, 3.0);
        auto readings = readingsFrom(map, fourSensors(), trueX + 0.4 * step, trueY - 0.2 * step,
                                     25 + 3 * step, nullptr);
        readings[2].valid = step % 3 != 1;
        filter.weigh(25 + 3 * step, readings);
        filter.resampleIfNeeded();
    }
    const Estimate e = filter.estimate();
    std::printf("  golden: x=%.12f y=%.12f spread=%.12f neff=%.12f p0=(%.12f, %.12f)\n", e.xIn,
                e.yIn, e.spreadIn, e.effectiveParticles, filter.particles()[0].xIn,
                filter.particles()[0].yIn);
    expectNear(e.xIn, -16.074287648507, 1e-9, "golden x");
    expectNear(e.yIn, 14.950802327907, 1e-9, "golden y");
    expectNear(e.spreadIn, 1.064527667393, 1e-9, "golden spread");
    expectNear(e.effectiveParticles, 62.107761325610, 1e-9, "golden effective particles");
    expectNear(filter.particles()[0].xIn, -16.682482235779, 1e-9, "golden particle 0 x");
    expectNear(filter.particles()[0].yIn, 17.062945229742, 1e-9, "golden particle 0 y");
}

} // namespace

int main() {
    testResetScattersAroundThePosition();
    testPredictMovesTheCloud();
    testWeighPullsTowardTheTruth();
    testNoValidReadingsChangesNothing();
    testResamplingFollowsTheWeights();
    testCheckSensors();
    testClosedLoopBeatsBiasedOdometry();
    testBlockedSensorDoesNotDragTheEstimate();
    testRecoveryAfterABump();
    testGoldenRun();
    std::printf("particle_filter_test: all tests passed\n");
    return 0;
}
