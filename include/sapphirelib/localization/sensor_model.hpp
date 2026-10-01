#pragma once

#include <cstdint>

#include "sapphirelib/util/lookup_table.hpp"

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
 * @brief readingLogLikelihood() for one reading, scored against many expected distances
 *
 * The particle filter scores every particle against every reading. This works out what depends
 * only on the reading once, and looks the rest up in a table instead of calling exp() and log():
 * the log of the beam model's mixture is log(floor) + softplus(log(peak / floor) - z^2 / 2), and
 * softplus(x) = log(1 + e^x) is the same curve for every reading. Within 1e-5 of
 * readingLogLikelihood() (the table's interpolation), which stays the reference
 *
 * @b Example
 * @code {.cpp}
 * using namespace sapphirelib::localization;
 * const BeamModel beam;
 * const ReadingScorer scorer(40.0, readingSigmaIn(beam, {.distanceIn = 40.0, .valid = true}),
 * beam); double score = scorer.logLikelihood(41.0); // as readingLogLikelihood(40, 41, sigma, beam)
 * @endcode
 */
class ReadingScorer {
public:
    /**
     * @brief Construct a scorer that gives every expected distance 0
     */
    ReadingScorer() = default;

    /**
     * @brief Construct a scorer for one reading
     *
     * @param measuredIn the reading, in inches
     * @param sigmaIn the reading's noise, from readingSigmaIn()
     * @param beam the sensor model
     */
    ReadingScorer(double measuredIn, double sigmaIn, const BeamModel& beam);

    /**
     * @brief Get how likely the reading is if the sensor should have read expectedIn, as a log
     *
     * @note always inlined: the particle filter calls it per particle per sensor, and -Os (the
     * PROS build) would otherwise make each a call
     *
     * @param expectedIn what the map says the sensor should read, in inches. Infinity if it would
     * see nothing
     * @return double as readingLogLikelihood(), within 1e-5
     */
    [[gnu::always_inline]] double logLikelihood(double expectedIn) const {
        // seeing nothing (infinity) leaves just the floor, as in readingLogLikelihood()
        const double z = (measuredIn_ - expectedIn) * invSigma_;
        const double x = logPeak_ - 0.5 * z * z;
        if (noOutliers_) return x;
        // softplus(x) = log(1 + e^x) is within 1.2e-7 of 0 below the table, and of x above it
        if (!(x > softplus_->minX())) return logFloor_;
        if (x >= softplus_->maxX()) return logFloor_ + x;
        return logFloor_ + softplus_->at(x);
    }

private:
    double measuredIn_ = 0.0;
    double invSigma_ = 0.0;
    // log(outlierProbability / maxRangeIn), the outlier floor. 0 with no outliers
    double logFloor_ = 0.0;
    // the hit part's peak over the floor, as a log. The peak itself with no outliers
    double logPeak_ = 0.0;
    bool noOutliers_ = true;
    // softplus(x) from -16 to 16, 64 samples to the unit: within 7.6e-6. Shared by every scorer;
    // unused with no outliers
    const LookupTable* softplus_ = nullptr;
};

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
