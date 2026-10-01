// Host-side unit test for sapphirelib::localization::fitSensorMounts(), no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Each test spins a simulated robot in place, one turn each way, at the speed
// MonteCarloLocalizer::calibrateSensorMounts() spins it, on a field whose
// walls (and anything else in the way) are a FieldMap the fit is never shown.
// The sensors really sit somewhere other than where the configured mounts say,
// odometry's frame is turned and shifted from the field's, and the tracking
// center drifts as the robot spins. The readings come from the true mounts, so
// the fit has to find them.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/localization/mount_calibration_test.cpp
//   src/sapphirelib/localization/mount_calibration.cpp src/sapphirelib/localization/field_map.cpp
//   src/sapphirelib/localization/sensor_model.cpp -o mount_calibration_test &&
//   ./mount_calibration_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "sapphirelib/localization/mount_calibration.hpp"
#include "sapphirelib/util/random.hpp"

using namespace sapphirelib::localization;
using sapphirelib::Rng;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.6f, expected %.6f (+-%g)\n", label, actual, expected,
                    tolerance);
        assert(false);
    }
}

void expectStatus(MountFitStatus actual, MountFitStatus expected, const char* label) {
    if (actual != expected) {
        std::printf("FAIL %s: got \"%s\", expected \"%s\"\n", label, mountFitStatusText(actual),
                    mountFitStatusText(expected));
        assert(false);
    }
}

// where the config says the sensors are: one per side, 7in out
std::vector<DistanceSensorMount> configuredMounts() {
    return {
        {.forwardIn = 7.0, .rightIn = 0.0, .facingDeg = 0},
        {.forwardIn = 0.0, .rightIn = 7.0, .facingDeg = 90},
        {.forwardIn = -7.0, .rightIn = 0.0, .facingDeg = 180},
        {.forwardIn = 0.0, .rightIn = -7.0, .facingDeg = 270},
    };
}

// where they really are: up to 1.4in off along the beam and 0.8in sideways, both ways
std::vector<DistanceSensorMount> trueMounts() {
    return {
        {.forwardIn = 8.2, .rightIn = 0.6, .facingDeg = 0},
        {.forwardIn = -0.8, .rightIn = 5.9, .facingDeg = 90},
        {.forwardIn = -6.3, .rightIn = -0.5, .facingDeg = 180},
        {.forwardIn = 0.7, .rightIn = -8.4, .facingDeg = 270},
    };
}

struct Spin {
    // what the sensors see: the walls, plus anything in the way
    FieldMap world = FieldMap::centered();
    std::vector<DistanceSensorMount> mounts = trueMounts();
    // where the tracking center starts on the field, and which way the robot faces
    double startXIn = 0.0;
    double startYIn = 0.0;
    double startHeadingDeg = 0.0;
    // odometry's own frame: where it thinks the robot starts
    double rawXIn = 12.0;
    double rawYIn = -30.0;
    double rawHeadingDeg = 200.0;
    // the tracking center wanders this far as the robot spins
    double wobbleIn = 0.8;
    // each reading's noise as a fraction of the distance, never under noiseFloorIn (1 sigma)
    double noiseFraction = 0.0;
    double noiseFloorIn = 0.0;
    // the chance a reading hits something closer that isn't there (another robot walking by)
    double outlierChance = 0.0;
    // round readings to whole millimeters, as the sensor reports them
    bool millimeters = false;
    // the heading the readings were really taken at, behind odometry's by this much in the spin's
    // direction: sensor delay the latency compensation didn't catch
    double headingLagDeg = 0.0;
    // which sensors are plugged in
    std::vector<bool> plugged = {true, true, true, true};
};

// one turn each way at 45 deg/s, a reading from every sensor every 30 ms, as the routine does
std::vector<MountSample> spin(const Spin& setup, std::uint32_t seed) {
    Rng rng(seed);
    const BeamModel beam;
    std::vector<MountSample> samples;
    const double stepDeg = 45.0 * 0.030;
    const int steps = static_cast<int>(std::ceil(360.0 / stepDeg));
    // odometry's frame is the field's turned by this, about the start
    const double frameTurn = (setup.rawHeadingDeg - setup.startHeadingDeg) * kDegToRad;
    double turnedDeg = 0.0;
    for (const double direction : {1.0, -1.0}) {
        for (int step = 0; step <= steps; ++step) {
            if (step > 0) turnedDeg += direction * stepDeg;
            const double headingDeg = setup.startHeadingDeg + turnedDeg;
            // the center drifts around a small circle as the wheels scrub
            const double phase = turnedDeg * kDegToRad;
            const double x = setup.startXIn + setup.wobbleIn * std::sin(phase);
            const double y = setup.startYIn + setup.wobbleIn * (1.0 - std::cos(phase));
            // odometry's record of it, in its own frame
            const double dx = x - setup.startXIn;
            const double dy = y - setup.startYIn;
            const double rawX = setup.rawXIn + dx * std::cos(frameTurn) + dy * std::sin(frameTurn);
            const double rawY = setup.rawYIn - dx * std::sin(frameTurn) + dy * std::cos(frameTurn);
            const double rawHeading = setup.rawHeadingDeg + turnedDeg;

            for (std::size_t s = 0; s < setup.mounts.size(); ++s) {
                if (!setup.plugged[s]) continue;
                const SensorRay ray =
                    sensorRay(x, y, headingDeg - direction * setup.headingLagDeg, setup.mounts[s]);
                double distance = setup.world.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
                const double sigma = std::max(setup.noiseFloorIn, setup.noiseFraction * distance);
                distance += sigma * rng.gaussian();
                if (rng.uniform() < setup.outlierChance) distance *= rng.uniform();
                if (setup.millimeters) distance = std::round(distance * 25.4) / 25.4;
                if (!(distance <= beam.maxRangeIn) || distance < 0.8) continue;
                samples.push_back(MountSample{.sensor = s,
                                              .xIn = rawX,
                                              .yIn = rawY,
                                              .headingDeg = std::fmod(rawHeading + 720.0, 360.0),
                                              .distanceIn = distance});
            }
        }
    }
    return samples;
}

MountCalibrationResult fit(const std::vector<MountSample>& samples,
                           const MountCalibrationConfig& config = {}) {
    const std::vector<DistanceSensorMount> mounts = configuredMounts();
    return fitSensorMounts(samples, mounts, FieldMap::centered(), BeamModel{}, config);
}

// every sensor calibrated, within tolerance of where it really is
void expectFound(const MountCalibrationResult& result, double alongTolerance,
                 double lateralTolerance, const char* label) {
    char text[128];
    std::snprintf(text, sizeof(text), "%s: ok", label);
    if (!result.ok) {
        std::printf("FAIL %s (%s)\n", text, result.error);
        assert(false);
    }
    const std::vector<DistanceSensorMount> truth = trueMounts();
    for (std::size_t s = 0; s < truth.size(); ++s) {
        const SensorMountFit& sensor = result.sensors[s];
        std::snprintf(text, sizeof(text), "%s: sensor %zu status", label, s);
        expectStatus(sensor.status, MountFitStatus::calibrated, text);
        // along and sideways, by the sensor's facing
        const double facing = truth[s].facingDeg * kDegToRad;
        const double trueAlong =
            truth[s].forwardIn * std::cos(facing) + truth[s].rightIn * std::sin(facing);
        const double trueLateral =
            -truth[s].forwardIn * std::sin(facing) + truth[s].rightIn * std::cos(facing);
        const double fitAlong =
            sensor.mount.forwardIn * std::cos(facing) + sensor.mount.rightIn * std::sin(facing);
        const double fitLateral =
            -sensor.mount.forwardIn * std::sin(facing) + sensor.mount.rightIn * std::cos(facing);
        std::snprintf(text, sizeof(text), "%s: sensor %zu along the beam", label, s);
        expectNear(fitAlong, trueAlong, alongTolerance, text);
        // sideways only when it was pinned down, the configured one otherwise
        const double configuredFacing = configuredMounts()[s].facingDeg * kDegToRad;
        const double configuredLateral =
            -configuredMounts()[s].forwardIn * std::sin(configuredFacing) +
            configuredMounts()[s].rightIn * std::cos(configuredFacing);
        std::snprintf(text, sizeof(text), "%s: sensor %zu sideways", label, s);
        expectNear(fitLateral, sensor.lateralFitted ? trueLateral : configuredLateral,
                   sensor.lateralFitted ? lateralTolerance : 1e-12, text);
        std::snprintf(text, sizeof(text), "%s: sensor %zu alongIn", label, s);
        expectNear(sensor.alongIn, fitAlong, 1e-9, text);
        std::snprintf(text, sizeof(text), "%s: sensor %zu facing kept", label, s);
        expectNear(sensor.mount.facingDeg, truth[s].facingDeg, 0.0, text);
    }
}

// The fit's model is exact for these readings, so it should land on the true mounts and the true
// start, up to the quarter turns a square field can't tell apart. With the sideways prior off: it
// pulls toward the configured mount, a few ten-thousandths of an inch even here
void testExactReadingsGiveTheExactMounts() {
    Spin setup;
    setup.startXIn = 5.0;
    setup.startYIn = -8.0;
    setup.startHeadingDeg = 17.0;
    const MountCalibrationResult result = fit(spin(setup, 1), {.lateralPriorIn = 1e4});
    expectFound(result, 1e-4, 1e-4, "exact");
    // the rest hit a wall too slanted, or were out of range
    assert(result.readingsUsed > result.readings / 2);

    bool startFound = false;
    double x = setup.startXIn;
    double y = setup.startYIn;
    double heading = setup.startHeadingDeg;
    for (int quarter = 0; quarter < 4; ++quarter) {
        double gap = std::fmod(std::fabs(result.start.headingDeg - heading), 360.0);
        gap = std::min(gap, 360.0 - gap);
        if (std::fabs(result.start.xIn - x) < 1e-3 && std::fabs(result.start.yIn - y) < 1e-3 &&
            gap < 1e-3) {
            startFound = true;
        }
        // a quarter turn clockwise about the field's middle
        const double turnedX = y;
        y = -x;
        x = turnedX;
        heading += 90.0;
    }
    if (!startFound) {
        std::printf(
            "FAIL exact: start (%.4f, %.4f, %.4f) isn't (5, -8, 17) or a quarter turn of it\n",
            result.start.xIn, result.start.yIn, result.start.headingDeg);
        assert(false);
    }
}

// The sensor's real noise (half its spec, as the simulator models it), millimeter rounding, a few
// readings off things that aren't walls, and 1.5 deg of sensor delay the compensation missed
void testNoisyReadings() {
    Spin setup;
    setup.startXIn = -6.0;
    setup.startYIn = 4.0;
    setup.startHeadingDeg = -32.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.outlierChance = 0.03;
    setup.millimeters = true;
    setup.headingLagDeg = 1.5;
    double squaredError = 0.0;
    double stderrSum = 0.0;
    int fits = 0;
    for (std::uint32_t seed = 1; seed <= 8; ++seed) {
        const MountCalibrationResult result = fit(spin(setup, seed));
        // about 0.12in standard error at this noise: within 3.5 of them
        expectFound(result, 0.42, 0.6, "noisy");
        const std::vector<DistanceSensorMount> truth = trueMounts();
        for (std::size_t s = 0; s < truth.size(); ++s) {
            const double facing = truth[s].facingDeg * kDegToRad;
            const double trueAlong =
                truth[s].forwardIn * std::cos(facing) + truth[s].rightIn * std::sin(facing);
            const double error = result.sensors[s].alongIn - trueAlong;
            squaredError += error * error;
            stderrSum += result.sensors[s].alongStderrIn;
            ++fits;
        }
    }
    // the standard error it reports is about the error it makes
    const double rmsError = std::sqrt(squaredError / fits);
    const double meanStderr = stderrSum / fits;
    std::printf("noisy: along-beam error %.3f in RMS over %d fits, standard error %.3f in\n",
                rmsError, fits, meanStderr);
    expectNear(rmsError / meanStderr, 1.0, 0.5, "noisy: standard error matches the error");
}

// Another robot parked 20in to the right blocks a wall for part of every turn. Its readings match
// no wall and drop out of the fit
void testSomethingInTheWay() {
    Spin setup;
    setup.startXIn = 3.0;
    setup.startYIn = -2.0;
    setup.startHeadingDeg = 5.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    setup.world.addBox(22.0, -12.0, 40.0, 6.0);
    const MountCalibrationResult result = fit(spin(setup, 3));
    expectFound(result, 0.3, 0.6, "blocked");
}

// On the line halfway between the side walls but near the back one: every sensor sees both side
// walls as it turns, which is all the fit needs
void testHalfwayLineNearAWall() {
    Spin setup;
    setup.startXIn = 2.0;
    setup.startYIn = -48.0;
    setup.startHeadingDeg = 90.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    const MountCalibrationResult result = fit(spin(setup, 4));
    expectFound(result, 0.3, 0.6, "halfway line");
    // the far wall is out of range the whole time, so each sees three walls. Which bits they are
    // depends on which quarter turn of the start the fit landed on
    for (const SensorMountFit& sensor : result.sensors) {
        int walls = 0;
        for (int bit = 0; bit < 4; ++bit) walls += (sensor.walls >> bit) & 1;
        assert(walls == 3);
    }
}

// Sideways offsets show only in slanted readings, and every sensor shifted sideways the same way
// looks like the robot facing a little differently unless the walls are at quite different
// distances. Near a wall, with sensors much quieter than their spec, they're pinned down too
void testQuietSensorsPinTheSidewaysOffsets() {
    Spin setup;
    setup.startXIn = 2.0;
    setup.startYIn = -48.0;
    setup.startHeadingDeg = 90.0;
    setup.noiseFraction = 0.005;
    setup.noiseFloorIn = 0.1;
    setup.millimeters = true;
    const MountCalibrationResult result = fit(spin(setup, 7));
    expectFound(result, 0.08, 0.25, "quiet");
    for (const SensorMountFit& sensor : result.sensors) assert(sensor.lateralFitted);
}

// Near a corner nothing reaches the far walls, so a robot an inch nearer the corner with sensors an
// inch further back reads the same. Every sensor is left as it was
void testCornerCannotPinTheOffsets() {
    Spin setup;
    setup.startXIn = -45.0;
    setup.startYIn = -45.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    const MountCalibrationResult result = fit(spin(setup, 5));
    assert(!result.ok);
    assert(std::strcmp(result.error, "no sensor saw two opposite walls") == 0);
    const std::vector<DistanceSensorMount> configured = configuredMounts();
    for (std::size_t s = 0; s < configured.size(); ++s) {
        expectStatus(result.sensors[s].status, MountFitStatus::noOppositeWalls, "corner");
        expectNear(result.sensors[s].mount.forwardIn, configured[s].forwardIn, 0.0,
                   "corner: mount left as it was");
        expectNear(result.sensors[s].mount.rightIn, configured[s].rightIn, 0.0,
                   "corner: mount left as it was");
    }
}

// An unplugged sensor is reported, and the rest are still calibrated
void testUnpluggedSensor() {
    Spin setup;
    setup.startXIn = 1.0;
    setup.startYIn = 1.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.plugged = {true, true, false, true};
    const MountCalibrationResult result = fit(spin(setup, 6));
    assert(result.ok);
    expectStatus(result.sensors[2].status, MountFitStatus::noReadings, "unplugged");
    expectNear(result.sensors[2].mount.forwardIn, -7.0, 0.0, "unplugged: left as it was");
    for (const std::size_t s : {0u, 1u, 3u}) {
        expectStatus(result.sensors[s].status, MountFitStatus::calibrated, "unplugged: the rest");
    }
}

void testNoReadings() {
    const MountCalibrationResult result = fit({});
    assert(!result.ok);
    assert(std::strcmp(result.error, "no readings") == 0);
    assert(result.sensors.size() == 4);
    expectStatus(result.sensors[0].status, MountFitStatus::noReadings, "empty");
    expectNear(result.sensors[1].mount.rightIn, 7.0, 0.0, "empty: left as it was");
}

void testStatusText() {
    for (const MountFitStatus status :
         {MountFitStatus::calibrated, MountFitStatus::noReadings, MountFitStatus::tooFewReadings,
          MountFitStatus::noOppositeWalls, MountFitStatus::uncertain,
          MountFitStatus::implausible}) {
        assert(std::strlen(mountFitStatusText(status)) > 0);
    }
}

} // namespace

int main() {
    testExactReadingsGiveTheExactMounts();
    testNoisyReadings();
    testSomethingInTheWay();
    testHalfwayLineNearAWall();
    testQuietSensorsPinTheSidewaysOffsets();
    testCornerCannotPinTheOffsets();
    testUnpluggedSensor();
    testNoReadings();
    testStatusText();
    std::printf("mount_calibration_test: all tests passed\n");
    return 0;
}
