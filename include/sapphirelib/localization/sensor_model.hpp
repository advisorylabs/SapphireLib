#pragma once

#include <cstdint>

namespace sapphirelib::localization {

/**
 * @brief Where a distance sensor sits on the robot, and which way it faces
 *
 * Measured from the tracking center (the point odometry tracks) to the sensor's face, the spot its
 * reading is measured from
 *
 * @b Example
 * @code {.cpp}
 * // a sensor on the back of the robot, 6.5in behind center and 1in left of it, facing backward
 * sapphirelib::localization::DistanceSensorMount back{
 *     .forwardIn = -6.5, .rightIn = -1.0, .facingDeg = 180};
 * @endcode
 */
struct DistanceSensorMount {
    /** how far ahead of the tracking center the sensor's face is, in inches. Negative is behind */
    double forwardIn = 0.0;

    /** how far right of the tracking center the sensor's face is, in inches. Negative is left */
    double rightIn = 0.0;

    /**
     * which way the sensor faces relative to the robot's front, in degrees clockwise: 0 forward,
     * 90 right, 180 backward, 270 left
     */
    double facingDeg = 0.0;
};

/**
 * @brief How much a distance reading can be trusted, as a probability model
 *
 * A reading is modeled as either a real measurement of what the map says is there (Gaussian noise
 * around the expected distance) or an outlier (another robot, a game element, a bad bounce)
 * equally likely anywhere in range. The outlier part is what keeps one blocked sensor from
 * dragging the whole estimate off: every hypothesis explains an outlier equally badly, so it
 * stops telling them apart instead of pulling them toward the wrong answer
 *
 * The defaults are the V5 Distance Sensor's spec: +-15mm below 200mm, +-5% above, 2000mm range
 */
struct BeamModel {
    /** longest distance a reading can be, in inches. Longer readings are ignored */
    double maxRangeIn = 78.0;

    /** smallest noise, in inches (1 standard deviation). 0.6in is 15mm */
    double minSigmaIn = 0.6;

    /** noise as a fraction of the distance (1 standard deviation). 0.05 is 5% */
    double sigmaFraction = 0.05;

    /** the chance any one reading is an outlier, 0 to 1 */
    double outlierProbability = 0.1;
};

/**
 * @brief One distance sensor's reading, ready for the localizer
 */
struct DistanceReading {
    /** distance to what the sensor sees, in inches */
    double distanceIn = 0.0;

    /** whether to use this reading. False for no object, a failed read, or low confidence */
    bool valid = false;

    /**
     * extra uncertainty on top of BeamModel's, in inches (1 standard deviation). For example, how
     * far the robot may have moved between the sensor measuring and the reading arriving
     */
    double extraSigmaIn = 0.0;
};

/**
 * @brief Where a sensor is on the field, and the direction it points
 */
struct SensorRay {
    /** the sensor face's x, in inches */
    double xIn = 0.0;
    /** the sensor face's y, in inches */
    double yIn = 0.0;
    /** unit direction x */
    double dirX = 0.0;
    /** unit direction y */
    double dirY = 0.0;
};

/**
 * @brief Work out where a sensor is on the field for a robot pose
 *
 * @param robotXIn tracking center x, in inches
 * @param robotYIn tracking center y, in inches
 * @param headingDeg robot heading, in degrees clockwise from +y
 * @param mount where the sensor is on the robot
 * @return SensorRay the sensor's position and direction on the field
 */
SensorRay sensorRay(double robotXIn, double robotYIn, double headingDeg,
                    const DistanceSensorMount& mount);

/**
 * @brief Get a reading's noise, in inches (1 standard deviation)
 *
 * From the measured distance rather than each hypothesis's expected one, so every hypothesis is
 * judged against the same yardstick
 *
 * @param beam the sensor model
 * @param reading the reading
 * @return double BeamModel's noise at that distance, combined with the reading's extraSigmaIn
 */
double readingSigmaIn(const BeamModel& beam, const DistanceReading& reading);

/**
 * @brief Get how likely a reading is if the robot were where a hypothesis says, as a log
 *
 * @param measuredIn the reading, in inches
 * @param expectedIn what the map says the sensor should read from the hypothesis, in inches.
 * Infinity if it would see nothing
 * @param sigmaIn the reading's noise, from readingSigmaIn()
 * @param beam the sensor model
 * @return double natural log of the reading's probability density, per inch. Higher is likelier.
 * Never below log(outlierProbability / maxRangeIn), however far off the reading is
 */
double readingLogLikelihood(double measuredIn, double expectedIn, double sigmaIn,
                            const BeamModel& beam);

/**
 * @brief Turn a V5 Distance Sensor's raw values into a reading
 *
 * The sensor reports 9999 when it sees nothing, and PROS reports PROS_ERR (a huge value) when the
 * read fails; both come out invalid, as does anything past maxRangeIn. Confidence is only
 * meaningful past 200mm, so it's only checked there
 *
 * @param millimeters pros::Distance::get_distance()
 * @param confidence pros::Distance::get_confidence(), 0 to 63
 * @param minConfidence the lowest confidence to accept past 200mm, 0 to 63
 * @param beam the sensor model, for its range
 * @return DistanceReading the reading in inches, valid or not
 */
DistanceReading distanceReadingFromMm(std::int32_t millimeters, std::int32_t confidence,
                                      std::int32_t minConfidence, const BeamModel& beam);

/**
 * @brief Correct a reading for the time it took to arrive
 *
 * A reading describes where the robot was when the sensor measured, a few tens of milliseconds
 * ago. Moving toward what the sensor sees, the distance has shrunk since, by the speed along the
 * beam times the latency. Half of that shift is added as extra uncertainty, since the speed and
 * the latency are both estimates
 *
 * @param reading the reading
 * @param closingSpeedInPerS how fast the sensor moves along its beam, in in/s. Positive is toward
 * what it sees
 * @param latencyS how old the reading is, in seconds
 * @return DistanceReading the reading as of now. Invalid readings come back unchanged
 */
DistanceReading compensateLatency(DistanceReading reading, double closingSpeedInPerS,
                                  double latencyS);

} // namespace sapphirelib::localization
