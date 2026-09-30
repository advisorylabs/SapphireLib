#include "sapphirelib/localization/sensor_model.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib::localization {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kInvSqrtTwoPi = 0.39894228040143267794;
constexpr double kMmPerIn = 25.4;

// what the V5 Distance Sensor reports when it sees nothing, and its shortest reading
constexpr std::int32_t kNoObjectMm = 9999;
constexpr std::int32_t kMinRangeMm = 20;

// confidence only means something past this distance
constexpr std::int32_t kConfidenceMinMm = 200;

} // namespace

SensorRay sensorRay(double robotXIn, double robotYIn, double headingDeg,
                    const DistanceSensorMount& mount) {
    const double headingRad = headingDeg * kDegToRad;
    const double sinHeading = std::sin(headingRad);
    const double cosHeading = std::cos(headingRad);
    const double facingRad = (headingDeg + mount.facingDeg) * kDegToRad;

    // the robot's forward is (sin, cos) and its right is (cos, -sin), matching odom::Pose
    return SensorRay{
        .xIn = robotXIn + mount.forwardIn * sinHeading + mount.rightIn * cosHeading,
        .yIn = robotYIn + mount.forwardIn * cosHeading - mount.rightIn * sinHeading,
        .dirX = std::sin(facingRad),
        .dirY = std::cos(facingRad),
    };
}

double readingSigmaIn(const BeamModel& beam, const DistanceReading& reading) {
    const double sensorSigma = std::max(beam.minSigmaIn, beam.sigmaFraction * reading.distanceIn);
    return std::hypot(sensorSigma, reading.extraSigmaIn);
}

double readingLogLikelihood(double measuredIn, double expectedIn, double sigmaIn,
                            const BeamModel& beam) {
    // a hypothesis that would see nothing gets exp(-inf) = 0 here, leaving just the outlier part
    const double z = (measuredIn - expectedIn) / sigmaIn;
    const double hit = std::exp(-0.5 * z * z) * kInvSqrtTwoPi / sigmaIn;
    const double density =
        (1.0 - beam.outlierProbability) * hit + beam.outlierProbability / beam.maxRangeIn;
    return std::log(density);
}

DistanceReading distanceReadingFromMm(std::int32_t millimeters, std::int32_t confidence,
                                      std::int32_t minConfidence, const BeamModel& beam) {
    DistanceReading reading;
    // 9999 is no object; PROS_ERR is far above it
    if (millimeters < kMinRangeMm || millimeters >= kNoObjectMm) return reading;
    if (millimeters > kConfidenceMinMm && confidence < minConfidence) return reading;

    reading.distanceIn = millimeters / kMmPerIn;
    reading.valid = reading.distanceIn <= beam.maxRangeIn;
    return reading;
}

DistanceReading compensateLatency(DistanceReading reading, double closingSpeedInPerS,
                                  double latencyS) {
    if (!reading.valid) return reading;
    const double shiftIn = closingSpeedInPerS * latencyS;
    reading.distanceIn -= shiftIn;
    reading.extraSigmaIn = std::hypot(reading.extraSigmaIn, 0.5 * shiftIn);
    // moving fast at something close can shift the reading past the sensor itself
    if (reading.distanceIn <= 0.0) reading.valid = false;
    return reading;
}

} // namespace sapphirelib::localization
