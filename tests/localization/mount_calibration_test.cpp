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
#include <limits>
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

// where they really are when the sensors were measured right but from the wrong spot: the
// tracking center is really 2.4in behind and 1.8in right of where the configured mounts put it
std::vector<DistanceSensorMount> trueMountsFromAnotherCenter() {
    std::vector<DistanceSensorMount> mounts = configuredMounts();
    for (DistanceSensorMount& mount : mounts) {
        mount.forwardIn += 2.4;
        mount.rightIn -= 1.8;
    }
    return mounts;
}

double alongOf(const DistanceSensorMount& mount) {
    const double facing = mount.facingDeg * kDegToRad;
    return mount.forwardIn * std::cos(facing) + mount.rightIn * std::sin(facing);
}

int wallsSeen(const SensorMountFit& sensor) {
    int walls = 0;
    for (int bit = 0; bit < 4; ++bit) walls += (sensor.walls >> bit) & 1;
    return walls;
}

// how much further out than the truth the configured mounts' spacing says the sensors are, on
// average, by the weights the fit holds the spacing with: with a sensor on each side, the mean of
// their along-beam errors. From one wall or a corner, every sensor lands that much off
double spacingError(const std::vector<DistanceSensorMount>& truth,
                    const std::vector<double>& weights) {
    const std::vector<DistanceSensorMount> configured = configuredMounts();
    double error = 0.0;
    for (std::size_t s = 0; s < weights.size(); ++s) {
        error += weights[s] * (alongOf(configured[s]) - alongOf(truth[s]));
    }
    return error;
}

// a room far bigger than the sensors reach, with nothing in it but what the test adds
FieldMap openRoom() { return FieldMap::centered(10000.0); }

// one 4ft wall, 22in right of the tracking center at (0, 0)
FieldMap oneWall() {
    FieldMap world = openRoom();
    world.addSegment({.x1In = 22.0, .y1In = -24.0, .x2In = 22.0, .y2In = 24.0});
    return world;
}

// two 5ft walls meeting at a right angle: 20in right of the tracking center at (0, 0), and 30in
// ahead of it
FieldMap corner() {
    FieldMap world = openRoom();
    world.addSegment({.x1In = 20.0, .y1In = -30.0, .x2In = 20.0, .y2In = 30.0});
    world.addSegment({.x1In = -40.0, .y1In = 30.0, .x2In = 20.0, .y2In = 30.0});
    return world;
}

// the rest of a classroom around the corner, all in the sensors' reach: a table off to the left, a
// cabinet behind, a cart off the corner, and a chair
void furnish(FieldMap& world) {
    world.addBox(-60.0, -20.0, -36.0, 18.0);
    world.addBox(-25.0, -55.0, 30.0, -40.0);
    world.addBox(30.0, 40.0, 48.0, 52.0);
    world.addBox(-32.0, 34.0, -26.0, 40.0);
}

// a widthIn by depthIn box centered at (xIn, yIn), turned turnDeg clockwise: furniture that isn't
// square to the walls
void addTurnedBox(FieldMap& world, double xIn, double yIn, double widthIn, double depthIn,
                  double turnDeg) {
    const double c = std::cos(turnDeg * kDegToRad);
    const double s = std::sin(turnDeg * kDegToRad);
    const double halfX[4] = {-widthIn / 2, widthIn / 2, widthIn / 2, -widthIn / 2};
    const double halfY[4] = {-depthIn / 2, -depthIn / 2, depthIn / 2, depthIn / 2};
    double cornerX[4];
    double cornerY[4];
    for (int k = 0; k < 4; ++k) {
        cornerX[k] = xIn + halfX[k] * c + halfY[k] * s;
        cornerY[k] = yIn - halfX[k] * s + halfY[k] * c;
    }
    for (int k = 0; k < 4; ++k) {
        world.addSegment({.x1In = cornerX[k],
                          .y1In = cornerY[k],
                          .x2In = cornerX[(k + 1) % 4],
                          .y2In = cornerY[(k + 1) % 4]});
    }
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
    // the beam's cone, its half angle in degrees: 0 reads along the center ray alone. Otherwise a
    // reading is the cone's (coneReading())
    double coneHalfDeg = 0.0;
    bool coneNearest = false;
};

// what a beam with a cone reads: with nearest, the nearest thing anywhere in it, as a sensor that
// only reports its first return would. Otherwise the cone's returns averaged, brightest in the
// middle (a Gaussian, the cone's edge 2 sigma out) and fading with distance squared, scaled so a
// wall square on reads true, as the sensor is calibrated at the factory
double coneReading(const FieldMap& world, double x, double y, double headingDeg,
                   const DistanceSensorMount& mount, double halfDeg, bool nearest) {
    double nearestIn = std::numeric_limits<double>::infinity();
    double weightSum = 0.0;
    double distanceSum = 0.0;
    double squareWeightSum = 0.0;
    double squareDistanceSum = 0.0;
    for (double offset = -halfDeg; offset <= halfDeg + 1e-9; offset += 0.5) {
        DistanceSensorMount turned = mount;
        turned.facingDeg += offset;
        const SensorRay ray = sensorRay(x, y, headingDeg, turned);
        const double distance = world.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
        nearestIn = std::min(nearestIn, distance);
        const double z = offset / (halfDeg / 2.0);
        const double brightness = std::exp(-0.5 * z * z);
        const double c = std::cos(offset * kDegToRad);
        // off a square wall 1 in away, this ray would come back from 1 / c, as bright as c^2
        squareWeightSum += brightness * c * c;
        squareDistanceSum += brightness * c;
        if (!std::isfinite(distance)) continue;
        weightSum += brightness / (distance * distance);
        distanceSum += brightness / distance;
    }
    if (nearest) return nearestIn;
    if (!(weightSum > 0.0)) return std::numeric_limits<double>::infinity();
    return distanceSum / weightSum / (squareDistanceSum / squareWeightSum);
}

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
                const double measuredDeg = headingDeg - direction * setup.headingLagDeg;
                const SensorRay ray = sensorRay(x, y, measuredDeg, setup.mounts[s]);
                double distance =
                    setup.coneHalfDeg > 0.0
                        ? coneReading(setup.world, x, y, measuredDeg, setup.mounts[s],
                                      setup.coneHalfDeg, setup.coneNearest)
                        : setup.world.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
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
                           const MountCalibrationConfig& config = {},
                           const std::vector<DistanceSensorMount>& mounts = configuredMounts()) {
    // the field, or the box the config describes, as calibrateSensorMounts() does
    return fitSensorMounts(samples, mounts, calibrationWalls(config, FieldMap::centered()),
                           BeamModel{}, config);
}

// a box's walls, inside measured widthIn by depthIn, centered on the origin, with nothing else
// around: what the robot spins in
FieldMap box(double widthIn, double depthIn) {
    return FieldMap(-widthIn / 2, -depthIn / 2, widthIn / 2, depthIn / 2);
}

// two 6 ft walls widthIn apart, either side of the origin, in a room with a cabinet down the
// corridor and a cart off one end
FieldMap corridor(double widthIn) {
    FieldMap world = openRoom();
    world.addSegment({.x1In = -widthIn / 2, .y1In = -36.0, .x2In = -widthIn / 2, .y2In = 36.0});
    world.addSegment({.x1In = widthIn / 2, .y1In = -36.0, .x2In = widthIn / 2, .y2In = 36.0});
    world.addBox(-30.0, 50.0, 10.0, 66.0);
    world.addBox(-8.0, -70.0, 14.0, -52.0);
    return world;
}

// every sensor calibrated, within tolerance of where it really is, or of that plus alongOffIn
// along the beam: how far a wrong spacing between the configured mounts moves every sensor from
// one wall or a corner (spacingError())
void expectFound(const MountCalibrationResult& result, double alongTolerance,
                 double lateralTolerance, const char* label,
                 const std::vector<DistanceSensorMount>& truth = trueMounts(),
                 double alongOffIn = 0.0) {
    char text[128];
    std::snprintf(text, sizeof(text), "%s: ok", label);
    if (!result.ok) {
        std::printf("FAIL %s (%s)\n", text, result.error);
        assert(false);
    }
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
        expectNear(fitAlong, trueAlong + alongOffIn, alongTolerance, text);
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
    // the field's width, not the configured spacing
    assert(!result.spacingFromMounts);
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
        assert(!result.spacingFromMounts);
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
    assert(!result.spacingFromMounts);
    // the far wall is out of range the whole time, so each sees three walls. Which bits they are
    // depends on which quarter turn of the start the fit landed on
    for (const SensorMountFit& sensor : result.sensors) assert(wallsSeen(sensor) == 3);
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

// From one wall a robot an inch nearer it with every sensor an inch further out reads the same, so
// the fit holds the configured mounts' spacing. They're 0.2in short of the truth on average, so
// every sensor lands 0.2in in from where it really is: everything but that is found. The standard
// error it reports is about the error it makes on top of that
void testOneWall() {
    Spin setup;
    setup.world = oneWall();
    setup.startHeadingDeg = 40.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    setup.headingLagDeg = 1.5;
    const double off = spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25});
    expectNear(off, -0.2, 1e-12, "one wall: the configured spacing's error");
    double squaredError = 0.0;
    double stderrSum = 0.0;
    int fits = 0;
    for (std::uint32_t seed = 1; seed <= 8; ++seed) {
        const MountCalibrationResult result = fit(spin(setup, seed));
        assert(result.spacingFromMounts);
        expectFound(result, 0.3, 0.6, "one wall", trueMounts(), off);
        const std::vector<DistanceSensorMount> truth = trueMounts();
        for (std::size_t s = 0; s < truth.size(); ++s) {
            const double error = result.sensors[s].alongIn - (alongOf(truth[s]) + off);
            squaredError += error * error;
            stderrSum += result.sensors[s].alongStderrIn;
            ++fits;
            assert(wallsSeen(result.sensors[s]) == 1);
            // every sensor shifted sideways alike looks like the robot facing a little differently
            assert(!result.sensors[s].lateralFitted);
        }
    }
    const double rmsError = std::sqrt(squaredError / fits);
    const double meanStderr = stderrSum / fits;
    std::printf("one wall: along-beam error %.3f in RMS over %d fits, standard error %.3f in\n",
                rmsError, fits, meanStderr);
    expectNear(rmsError / meanStderr, 1.0, 0.5, "one wall: standard error matches the error");
}

// Mounts measured right, from a tracking center that's really a couple of inches from there: the
// readings see that, since it moves opposite sensors' offsets opposite ways, so one wall finds them
// with nothing left over
void testOneWallFindsTheTrackingCenter() {
    Spin setup;
    setup.world = oneWall();
    setup.mounts = trueMountsFromAnotherCenter();
    setup.startHeadingDeg = -70.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    const std::vector<double> evenly = {0.25, 0.25, 0.25, 0.25};
    expectNear(spacingError(setup.mounts, evenly), 0.0, 1e-12, "tracking center: spacing right");
    const MountCalibrationResult result = fit(spin(setup, 9));
    assert(result.spacingFromMounts);
    expectFound(result, 0.3, 0.6, "tracking center", setup.mounts);
}

// Exact readings from one wall: every sensor off by exactly the spacing's error, give or take the
// little a slanted reading says about the robot's distance to the wall
void testOneWallExactReadings() {
    Spin setup;
    setup.world = oneWall();
    setup.startHeadingDeg = 12.0;
    const double off = spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25});
    const MountCalibrationResult result = fit(spin(setup, 1));
    assert(result.spacingFromMounts);
    expectFound(result, 0.02, 0.6, "one wall exact", trueMounts(), off);
}

// A corner: two walls at a right angle, at different distances, so the sideways offsets can show
// too. Same spacing as one wall
void testCorner() {
    Spin setup;
    setup.world = corner();
    setup.startHeadingDeg = 200.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    setup.headingLagDeg = 1.5;
    const double off = spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25});
    const MountCalibrationResult result = fit(spin(setup, 10));
    assert(result.spacingFromMounts);
    expectFound(result, 0.25, 0.6, "corner", trueMounts(), off);
    for (const SensorMountFit& sensor : result.sensors) assert(wallsSeen(sensor) == 2);
}

// A corner of a real field, the far walls out of reach, is the same thing
void testFieldCorner() {
    Spin setup;
    setup.startXIn = -45.0;
    setup.startYIn = -45.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    const MountCalibrationResult result = fit(spin(setup, 5));
    assert(result.spacingFromMounts);
    expectFound(result, 0.25, 0.6, "field corner", trueMounts(),
                spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25}));
}

// The corner in a classroom: a table, a cabinet, a cart and a chair in reach of the sensors when
// they face away from the walls, and someone walking by. Their readings outnumber the walls', and
// match nothing
void testCornerInAClutteredRoom() {
    Spin setup;
    setup.world = corner();
    furnish(setup.world);
    setup.startHeadingDeg = 75.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.outlierChance = 0.05;
    setup.millimeters = true;
    setup.headingLagDeg = 1.5;
    const double off = spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25});
    for (std::uint32_t seed = 11; seed <= 14; ++seed) {
        const MountCalibrationResult result = fit(spin(setup, seed));
        assert(result.spacingFromMounts);
        expectFound(result, 0.3, 0.6, "cluttered corner", trueMounts(), off);
    }
}

// One wall in the same room, the furniture all around it
void testOneWallInAClutteredRoom() {
    Spin setup;
    setup.world = oneWall();
    furnish(setup.world);
    setup.startHeadingDeg = -15.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.outlierChance = 0.05;
    setup.millimeters = true;
    const double off = spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25});
    for (std::uint32_t seed = 15; seed <= 18; ++seed) {
        const MountCalibrationResult result = fit(spin(setup, seed));
        assert(result.spacingFromMounts);
        expectFound(result, 0.3, 0.6, "cluttered wall", trueMounts(), off);
    }
}

// One wall, and a cabinet across the room turned 30 deg from square to it. The walls square to the
// one seen are free to sit anywhere, and one comes to rest on the cabinet's face; the two can't
// agree on the robot's heading, so the cabinet is left out and the wall alone calibrates
void testOneWallWithACabinetAskew() {
    const double off = spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25});
    for (std::uint32_t seed = 1; seed <= 6; ++seed) {
        Spin setup;
        setup.world = oneWall();
        addTurnedBox(setup.world, -4.0, 36.0, 40.0, 12.0, 30.0);
        setup.startHeadingDeg = 25.0 * seed;
        setup.noiseFraction = 0.025;
        setup.noiseFloorIn = 0.3;
        setup.millimeters = true;
        setup.headingLagDeg = 1.5;
        const MountCalibrationResult result = fit(spin(setup, seed));
        assert(result.spacingFromMounts);
        // within 0.25in of the truth with the cabinet in the fit too, but never this close
        expectFound(result, 0.15, 0.6, "cabinet askew", trueMounts(), off);
        for (const SensorMountFit& sensor : result.sensors) assert(wallsSeen(sensor) == 1);
    }
}

// The robot 5ft from one wall, and a long flat face 73.5in the other way: a pair of walls to the
// fit, but 132.5in apart, not the field's 140.5. Taking them for the field's width would move every
// sensor out 4in, further than maxSpacingChangeIn, so the spacing stays held, and the readings,
// which can't agree with both, leave every sensor as it was rather than 4in out
void testSomethingLinedUpAcrossTheRoom() {
    const double off = spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25});
    for (std::uint32_t seed = 1; seed <= 4; ++seed) {
        Spin setup;
        setup.world = openRoom();
        setup.world.addSegment({.x1In = 59.0, .y1In = -40.0, .x2In = 59.0, .y2In = 40.0});
        setup.world.addSegment({.x1In = -73.5, .y1In = -40.0, .x2In = -73.5, .y2In = 40.0});
        setup.startHeadingDeg = 30.0 * seed;
        setup.noiseFraction = 0.025;
        setup.noiseFloorIn = 0.3;
        setup.millimeters = true;
        const MountCalibrationResult result = fit(spin(setup, seed));
        assert(result.spacingFromMounts);
        const std::vector<DistanceSensorMount> truth = trueMounts();
        for (std::size_t s = 0; s < truth.size(); ++s) {
            if (result.sensors[s].status != MountFitStatus::calibrated) continue;
            expectNear(result.sensors[s].alongIn, alongOf(truth[s]) + off, 0.6, "lined up");
        }
    }
}

// Forty rooms: one wall or a corner, 14 to 36in away, and eight pieces of furniture turned every
// which way, 20 to 75in out and in reach of the sensors, anywhere but the walls' view (nothing
// within 40 deg of square on to a wall from the robot, nor within 4in of one). Nothing comes out
// wrong, and the furniture's readings, though they outnumber the walls', don't pass for the
// sensors' noise and leave them too uncertain: no more than one room in forty is left uncalibrated
void testRandomClassrooms() {
    Rng rng(2026);
    int roomsLeft = 0;
    for (std::uint32_t room = 0; room < 40; ++room) {
        Spin setup;
        setup.world = openRoom();
        const bool isCorner = room % 2 == 0;
        const double rightIn = 14.0 + 22.0 * rng.uniform();
        const double aheadIn = 14.0 + 22.0 * rng.uniform();
        setup.world.addSegment(
            {.x1In = rightIn, .y1In = -36.0, .x2In = rightIn, .y2In = isCorner ? aheadIn : 36.0});
        if (isCorner) {
            setup.world.addSegment(
                {.x1In = -48.0, .y1In = aheadIn, .x2In = rightIn, .y2In = aheadIn});
        }
        for (int piece = 0; piece < 8; ++piece) {
            for (int tries = 0; tries < 50; ++tries) {
                const double bearing = 2.0 * kPi * rng.uniform();
                const double reach = 20.0 + 55.0 * rng.uniform();
                const double x = reach * std::sin(bearing);
                const double y = reach * std::cos(bearing);
                const double width = 2.0 + 30.0 * rng.uniform();
                const double depth = 2.0 + 30.0 * rng.uniform();
                const double turn = 90.0 * rng.uniform();
                // out of the walls' view, checked at points along its edges
                const double c = std::cos(turn * kDegToRad);
                const double s = std::sin(turn * kDegToRad);
                const double halfX[4] = {-width / 2, width / 2, width / 2, -width / 2};
                const double halfY[4] = {-depth / 2, -depth / 2, depth / 2, depth / 2};
                bool clear = true;
                for (int k = 0; k < 4 && clear; ++k) {
                    const double x1 = x + halfX[k] * c + halfY[k] * s;
                    const double y1 = y - halfX[k] * s + halfY[k] * c;
                    const double x2 = x + halfX[(k + 1) % 4] * c + halfY[(k + 1) % 4] * s;
                    const double y2 = y - halfX[(k + 1) % 4] * s + halfY[(k + 1) % 4] * c;
                    for (int step = 0; step <= 8 && clear; ++step) {
                        const double px = x1 + (x2 - x1) * step / 8.0;
                        const double py = y1 + (y2 - y1) * step / 8.0;
                        const bool inWallsView =
                            (px > 0.0 && std::fabs(std::atan2(py, px)) < 40.0 * kDegToRad) ||
                            (isCorner && py > 0.0 &&
                             std::fabs(std::atan2(px, py)) < 40.0 * kDegToRad);
                        clear = px < rightIn - 4.0 && (!isCorner || py < aheadIn - 4.0) &&
                                std::hypot(px, py) >= 14.0 && !inWallsView;
                    }
                }
                if (!clear) continue;
                addTurnedBox(setup.world, x, y, width, depth, turn);
                break;
            }
        }
        // measured from a tracking center up to 3in off each way, each face to within half an inch
        std::vector<DistanceSensorMount> truth = configuredMounts();
        const double centerForwardIn = 3.0 * (2.0 * rng.uniform() - 1.0);
        const double centerRightIn = 3.0 * (2.0 * rng.uniform() - 1.0);
        for (DistanceSensorMount& mount : truth) {
            mount.forwardIn += centerForwardIn + 0.5 * (2.0 * rng.uniform() - 1.0);
            mount.rightIn += centerRightIn + 0.5 * (2.0 * rng.uniform() - 1.0);
        }
        setup.mounts = truth;
        const double off = spacingError(truth, {0.25, 0.25, 0.25, 0.25});
        setup.startHeadingDeg = 360.0 * rng.uniform();
        setup.noiseFraction = 0.025;
        setup.noiseFloorIn = 0.3;
        setup.outlierChance = 0.05;
        setup.millimeters = true;
        setup.headingLagDeg = 1.5;
        const MountCalibrationResult result = fit(spin(setup, 100 + room));
        bool left = false;
        for (std::size_t s = 0; s < truth.size(); ++s) {
            if (result.sensors[s].status != MountFitStatus::calibrated) {
                left = true;
                continue;
            }
            expectNear(result.sensors[s].alongIn, alongOf(truth[s]) + off, 0.4, "classroom");
        }
        roomsLeft += left ? 1 : 0;
    }
    std::printf("classrooms: %d of 40 left a sensor uncalibrated\n", roomsLeft);
    assert(roomsLeft <= 1);
}

// Front, right and back sensors: what's left to hold is the front and back sensors' span, and the
// right one follows from the readings
void testThreeSensorsKeepTheirSpan() {
    Spin setup;
    setup.world = oneWall();
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    setup.plugged = {true, true, true, false};
    const double off = spacingError(trueMounts(), {0.5, 0.0, 0.5, 0.0});
    const MountCalibrationResult result = fit(spin(setup, 19));
    assert(result.ok && result.spacingFromMounts);
    expectStatus(result.sensors[3].status, MountFitStatus::noReadings, "three sensors: left");
    const std::vector<DistanceSensorMount> truth = trueMounts();
    for (const std::size_t s : {0u, 1u, 2u}) {
        expectStatus(result.sensors[s].status, MountFitStatus::calibrated, "three sensors");
        expectNear(result.sensors[s].alongIn, alongOf(truth[s]) + off, 0.3, "three sensors: along");
    }
}

// Only the front and right sensors: every offset growing alike is then just the tracking center
// sitting further back and further left, so there's no spacing to hold, and nothing to calibrate
void testAdjacentSensorsCannotKeepASpacing() {
    Spin setup;
    setup.world = oneWall();
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.plugged = {true, true, false, false};
    const MountCalibrationResult result = fit(spin(setup, 20));
    assert(!result.ok && !result.spacingFromMounts);
    assert(std::strcmp(result.error, "no two opposite sensors saw a wall") == 0);
    const std::vector<DistanceSensorMount> configured = configuredMounts();
    for (const std::size_t s : {0u, 1u}) {
        expectStatus(result.sensors[s].status, MountFitStatus::noOppositeSensor, "adjacent");
        expectNear(result.sensors[s].mount.forwardIn, configured[s].forwardIn, 0.0,
                   "adjacent: mount left as it was");
        expectNear(result.sensors[s].mount.rightIn, configured[s].rightIn, 0.0,
                   "adjacent: mount left as it was");
    }
}

// In a box whose inside is measured, a robot spinning a few inches off center sees both walls of
// each pair, so the box's width pins the spacing the configured mounts get 0.2in wrong: every
// sensor lands where it really is, with nothing on the robot measured. With the sensor's real
// noise, a cone (calibrated square on, wider at a slant), readings off nothing, and delay. The
// standard error it reports is about the error it makes
void testBoxFindsTheSpacing() {
    const MountCalibrationConfig inBox{.boxWidthIn = 40.0, .boxDepthIn = 36.0};
    double squaredError = 0.0;
    double stderrSum = 0.0;
    int fits = 0;
    for (std::uint32_t seed = 1; seed <= 8; ++seed) {
        Spin setup;
        setup.world = box(40.0, 36.0);
        setup.startXIn = seed % 2 == 0 ? 4.0 : -3.0;
        setup.startYIn = seed % 3 == 0 ? -4.0 : 3.0;
        setup.startHeadingDeg = 55.0 * seed;
        setup.noiseFraction = 0.025;
        setup.noiseFloorIn = 0.3;
        setup.outlierChance = 0.03;
        setup.millimeters = true;
        setup.headingLagDeg = 1.5;
        setup.coneHalfDeg = 12.0;
        const MountCalibrationResult result = fit(spin(setup, seed), inBox);
        assert(!result.spacingFromMounts);
        expectFound(result, 0.15, 0.6, "box");
        const std::vector<DistanceSensorMount> truth = trueMounts();
        for (std::size_t s = 0; s < truth.size(); ++s) {
            const double error = result.sensors[s].alongIn - alongOf(truth[s]);
            squaredError += error * error;
            stderrSum += result.sensors[s].alongStderrIn;
            ++fits;
            // all four walls, from every sensor
            assert(wallsSeen(result.sensors[s]) == 4);
        }
    }
    const double rmsError = std::sqrt(squaredError / fits);
    const double meanStderr = stderrSum / fits;
    std::printf("box: along-beam error %.3f in RMS over %d fits, standard error %.3f in\n",
                rmsError, fits, meanStderr);
    expectNear(rmsError / meanStderr, 1.0, 0.5, "box: standard error matches the error");
}

// Two walls a measured 48in apart, the corridor's ends open onto the room: the same, from
// anywhere between them the robot can spin. The cabinet and the cart down the corridor match no
// wall and drop out
void testCorridorFindsTheSpacing() {
    const MountCalibrationConfig between{.boxWidthIn = 48.0};
    for (std::uint32_t seed = 1; seed <= 4; ++seed) {
        Spin setup;
        setup.world = corridor(48.0);
        setup.startXIn = seed % 2 == 0 ? 6.0 : -5.0;
        setup.startYIn = 5.0 * seed - 12.0;
        setup.startHeadingDeg = 80.0 * seed;
        setup.noiseFraction = 0.025;
        setup.noiseFloorIn = 0.3;
        setup.outlierChance = 0.03;
        setup.millimeters = true;
        setup.headingLagDeg = 1.5;
        setup.coneHalfDeg = 12.0;
        const MountCalibrationResult result = fit(spin(setup, 30 + seed), between);
        assert(!result.spacingFromMounts);
        expectFound(result, 0.2, 0.6, "corridor");
        // the two walls, never the cabinet or the cart down the corridor taken for an end wall
        for (const SensorMountFit& sensor : result.sensors) {
            assert(sensor.walls == ((1u << 0) | (1u << 1)));
        }
    }
}

// Twenty-four boxes and corridors, 34 to 56in across, the robot up to 6in off center facing any
// way, its mounts measured from a tracking center up to 3in off and each face to within half an
// inch, the corridors' ends open onto furniture turned every which way. Every sensor calibrated,
// within 0.2in of where it really is
void testRandomBoxes() {
    Rng rng(4242);
    double worst = 0.0;
    for (std::uint32_t room = 0; room < 24; ++room) {
        const bool isCorridor = room % 2 == 1;
        const double widthIn = 34.0 + 22.0 * rng.uniform();
        const double depthIn = isCorridor ? 0.0 : 34.0 + 22.0 * rng.uniform();
        Spin setup;
        if (isCorridor) {
            setup.world = openRoom();
            setup.world.addSegment(
                {.x1In = -widthIn / 2, .y1In = -40.0, .x2In = -widthIn / 2, .y2In = 40.0});
            setup.world.addSegment(
                {.x1In = widthIn / 2, .y1In = -40.0, .x2In = widthIn / 2, .y2In = 40.0});
            for (const double end : {-1.0, 1.0}) {
                addTurnedBox(setup.world, 30.0 * (2.0 * rng.uniform() - 1.0),
                             end * (50.0 + 20.0 * rng.uniform()), 10.0 + 30.0 * rng.uniform(),
                             6.0 + 20.0 * rng.uniform(), 90.0 * rng.uniform());
            }
        } else {
            setup.world = box(widthIn, depthIn);
        }
        std::vector<DistanceSensorMount> truth = configuredMounts();
        const double centerForwardIn = 3.0 * (2.0 * rng.uniform() - 1.0);
        const double centerRightIn = 3.0 * (2.0 * rng.uniform() - 1.0);
        for (DistanceSensorMount& mount : truth) {
            mount.forwardIn += centerForwardIn + 0.5 * (2.0 * rng.uniform() - 1.0);
            mount.rightIn += centerRightIn + 0.5 * (2.0 * rng.uniform() - 1.0);
        }
        setup.mounts = truth;
        setup.startXIn = 6.0 * (2.0 * rng.uniform() - 1.0);
        setup.startYIn = (isCorridor ? 12.0 : 6.0) * (2.0 * rng.uniform() - 1.0);
        setup.startHeadingDeg = 360.0 * rng.uniform();
        setup.noiseFraction = 0.025;
        setup.noiseFloorIn = 0.3;
        setup.outlierChance = 0.05;
        setup.millimeters = true;
        setup.headingLagDeg = 1.5;
        setup.coneHalfDeg = 12.0;
        const MountCalibrationResult result =
            fit(spin(setup, 200 + room), {.boxWidthIn = widthIn, .boxDepthIn = depthIn});
        if (result.spacingFromMounts) {
            std::printf("FAIL random boxes: room %u kept the spacing\n", room);
            assert(false);
        }
        for (std::size_t s = 0; s < truth.size(); ++s) {
            expectStatus(result.sensors[s].status, MountFitStatus::calibrated, "random boxes");
            const double error = result.sensors[s].alongIn - alongOf(truth[s]);
            worst = std::max(worst, std::fabs(error));
            expectNear(error, 0.0, 0.2, "random boxes");
        }
    }
    std::printf("random boxes: worst along-beam error %.3f in over 24\n", worst);
}

// Mounts measured 3.5in short of the truth on every side, 7in short across the robot: further
// than a field's width may move them (maxSpacingChangeIn), but a box's walls are there by design,
// so the box sets them
void testBoxCorrectsMountsFarOff() {
    std::vector<DistanceSensorMount> truth = configuredMounts();
    for (DistanceSensorMount& mount : truth) {
        const double facing = mount.facingDeg * kDegToRad;
        mount.forwardIn += 3.5 * std::cos(facing);
        mount.rightIn += 3.5 * std::sin(facing);
    }
    Spin setup;
    setup.world = box(46.0, 46.0);
    setup.mounts = truth;
    setup.startXIn = 3.0;
    setup.startYIn = -4.0;
    setup.startHeadingDeg = 130.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    setup.coneHalfDeg = 12.0;
    const MountCalibrationResult result =
        fit(spin(setup, 40), {.boxWidthIn = 46.0, .boxDepthIn = 46.0});
    assert(!result.spacingFromMounts);
    expectFound(result, 0.15, 0.6, "far off", truth);
}

// How a sensor reads a wall at a slant decides where the fit puts its offsets once a width pins
// them: readings from a sensor that reports the nearest thing in its cone, taken for its center
// ray, put every sensor about 0.2in too far out. The fit finds the shape, so each comes out right:
// near 1 for a beam that reads its center ray, and for the factory calibrated cone; near 0 for
// the nearest thing in it
void testBeamShape() {
    struct Beam {
        const char* label;
        double coneHalfDeg;
        bool nearest;
        double minShape;
        double maxShape;
    };
    const Beam beams[] = {
        {"center ray", 0.0, false, 0.8, 1.0},
        {"calibrated cone", 12.0, false, 0.8, 1.0},
        {"nearest in cone", 12.0, true, 0.0, 0.3},
    };
    for (const Beam& beam : beams) {
        Spin setup;
        setup.world = box(40.0, 40.0);
        setup.startXIn = -4.0;
        setup.startYIn = 3.0;
        setup.startHeadingDeg = 20.0;
        setup.noiseFraction = 0.025;
        setup.noiseFloorIn = 0.3;
        setup.millimeters = true;
        setup.coneHalfDeg = beam.coneHalfDeg;
        setup.coneNearest = beam.nearest;
        const MountCalibrationResult result =
            fit(spin(setup, 50), {.boxWidthIn = 40.0, .boxDepthIn = 40.0});
        std::printf("beam shape: %s fits %.2f\n", beam.label, result.beamShape);
        if (!(result.beamShape >= beam.minShape && result.beamShape <= beam.maxShape)) {
            std::printf("FAIL beam shape: %s is %.2f, not %.1f to %.1f\n", beam.label,
                        result.beamShape, beam.minShape, beam.maxShape);
            assert(false);
        }
        expectFound(result, 0.15, 0.6, beam.label);
    }
}

// From a corner the spacing comes from the mounts whatever the beam's shape, so a cone that reads
// short at a slant changes nothing there
void testCornerWithACone() {
    Spin setup;
    setup.world = corner();
    setup.startHeadingDeg = 300.0;
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    setup.millimeters = true;
    setup.headingLagDeg = 1.5;
    setup.coneHalfDeg = 12.0;
    setup.coneNearest = true;
    const MountCalibrationResult result = fit(spin(setup, 60));
    assert(result.spacingFromMounts);
    expectFound(result, 0.25, 0.6, "corner, cone", trueMounts(),
                spacingError(trueMounts(), {0.25, 0.25, 0.25, 0.25}));
}

void testCalibrationWalls() {
    const FieldMap field = FieldMap::centered();
    const FieldMap none = calibrationWalls({}, field);
    expectNear(none.minXIn(), field.minXIn(), 0.0, "walls: no box is the field");
    expectNear(none.maxYIn(), field.maxYIn(), 0.0, "walls: no box is the field");
    const FieldMap walls = calibrationWalls({.boxWidthIn = 40.0, .boxDepthIn = 30.0}, field);
    expectNear(walls.minXIn(), -20.0, 0.0, "walls: box left");
    expectNear(walls.maxXIn(), 20.0, 0.0, "walls: box right");
    expectNear(walls.minYIn(), -15.0, 0.0, "walls: box near");
    expectNear(walls.maxYIn(), 15.0, 0.0, "walls: box far");
    // one side 0: two walls, the other two far out of reach
    const FieldMap pair = calibrationWalls({.boxWidthIn = 48.0}, field);
    expectNear(pair.maxXIn(), 24.0, 0.0, "walls: corridor");
    assert(pair.maxYIn() > 1000.0 && pair.minYIn() < -1000.0);
    const FieldMap across = calibrationWalls({.boxDepthIn = 48.0}, field);
    expectNear(across.minYIn(), -24.0, 0.0, "walls: corridor the other way");
    assert(across.maxXIn() > 1000.0);
}

// Nothing in reach: every reading is the room's clutter, which matches no wall
void testNoWallInReach() {
    Spin setup;
    setup.world = openRoom();
    setup.world.addBox(-32.0, 34.0, -26.0, 40.0);
    setup.noiseFraction = 0.025;
    setup.noiseFloorIn = 0.3;
    const MountCalibrationResult result = fit(spin(setup, 21));
    assert(!result.ok);
    std::printf("no wall: %s\n", result.error);
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
          MountFitStatus::noOppositeWalls, MountFitStatus::noOppositeSensor,
          MountFitStatus::uncertain, MountFitStatus::implausible}) {
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
    testOneWall();
    testOneWallFindsTheTrackingCenter();
    testOneWallExactReadings();
    testCorner();
    testFieldCorner();
    testCornerInAClutteredRoom();
    testOneWallInAClutteredRoom();
    testOneWallWithACabinetAskew();
    testSomethingLinedUpAcrossTheRoom();
    testRandomClassrooms();
    testThreeSensorsKeepTheirSpan();
    testAdjacentSensorsCannotKeepASpacing();
    testBoxFindsTheSpacing();
    testCorridorFindsTheSpacing();
    testRandomBoxes();
    testBoxCorrectsMountsFarOff();
    testBeamShape();
    testCornerWithACone();
    testCalibrationWalls();
    testNoWallInReach();
    testUnpluggedSensor();
    testNoReadings();
    testStatusText();
    std::printf("mount_calibration_test: all tests passed\n");
    return 0;
}
