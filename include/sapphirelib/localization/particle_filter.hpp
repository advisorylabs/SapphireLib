#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "sapphirelib/localization/field_map.hpp"
#include "sapphirelib/localization/sensor_model.hpp"
#include "sapphirelib/util/random.hpp"

namespace sapphirelib::localization {

/**
 * @brief One guess at where the robot is
 */
struct Particle {
    /** x, in inches */
    double xIn = 0.0;
    /** y, in inches */
    double yIn = 0.0;
    /** how much this guess counts, 0 to 1. All the weights add up to 1 */
    double weight = 0.0;
};

/**
 * @brief How much uncertainty each predict() adds, as the standard deviation of a random nudge in
 * x and in y
 *
 * The three parts add up. Too little and the particles can't keep up with odometry's real error;
 * too much and the estimate gets noisy between sensor readings
 */
struct MotionNoise {
    /** added every predict(), even standing still, in inches. Keeps copied particles apart */
    double baseIn = 0.02;

    /**
     * added per inch traveled, as a fraction. 0.05 is 5%: a little above what good tracking wheels
     * get wrong, so the cloud stays wide enough to cover wear and slip too
     */
    double perInch = 0.05;

    /** added per degree turned, in inches. Turning scrubs tracking wheels */
    double perDegreeIn = 0.005;
};

/**
 * @brief When and where to scatter fresh particles, to recover from a bad estimate
 *
 * Tracks how well the particles explain the readings (as a fraction of a perfect fit), as a slow
 * and a fast running average. When the fast one falls well below the slow one, the readings have
 * suddenly stopped matching (a hard bump, wheel slip) and a share of the particles is replaced by
 * fresh ones near the estimate, which the next readings sort out. This is augmented MCL (Thrun,
 * Probabilistic Robotics 8.3.5), with a trigger threshold added: without it, the ordinary
 * ups and downs of the fit keep scattering particles while nothing is wrong
 */
struct RecoveryConfig {
    /** whether to scatter fresh particles at all. True by default */
    bool enabled = true;

    /** how quickly the slow average follows, 0 to 1 per update. Must be well below fastRate */
    double slowRate = 0.01;

    /** how quickly the fast average follows, 0 to 1 per update */
    double fastRate = 0.2;

    /**
     * recover once the fast average falls below this fraction of the slow one, 0 to 1. The share
     * replaced grows from 0 there, up to maxFraction. In the simulator, ordinary driving stays
     * above about 0.9 and a 7in bump drops it under 0.6
     */
    double triggerRatio = 0.8;

    /** how far from the estimate fresh particles go, in inches. 0 spreads them over the field */
    double radiusIn = 18.0;

    /** the most particles replaced in one resample, as a fraction, 0 to 1 */
    double maxFraction = 0.1;
};

/**
 * @brief Settings for a ParticleFilter
 */
struct ParticleFilterConfig {
    /** number of particles. More is steadier and slower; every one costs a raycast per sensor */
    std::size_t particleCount = 300;

    /** uncertainty added as the robot moves */
    MotionNoise motionNoise;

    /** how much to trust a distance reading */
    BeamModel beam;

    /**
     * resample when the effective particle count falls below this fraction of particleCount, 0 to
     * 1. Resampling less often keeps more variety; more often drops bad guesses sooner
     */
    double resampleThreshold = 0.5;

    /** recovering from a bad estimate */
    RecoveryConfig recovery;

    /** random seed. The same seed and inputs give the same particles every time */
    std::uint32_t seed = 1;
};

/**
 * @brief What the particles say, summed up
 */
struct Estimate {
    /** weighted mean x, in inches */
    double xIn = 0.0;
    /** weighted mean y, in inches */
    double yIn = 0.0;
    /** weighted variance of x, in square inches */
    double varianceXIn2 = 0.0;
    /** weighted variance of y, in square inches */
    double varianceYIn2 = 0.0;
    /** weighted covariance of x and y, in square inches */
    double covarianceXYIn2 = 0.0;
    /** how far particles are from the mean on average (root mean square), in inches */
    double spreadIn = 0.0;
    /**
     * how many particles are really carrying the estimate: particleCount when all weigh the same,
     * 1 when a single one has all the weight
     */
    double effectiveParticles = 0.0;
};

/**
 * @brief How one sensor's reading compares with what the map says it should read from a pose
 */
struct SensorCheck {
    /** whether the reading was valid, so the rest means anything */
    bool used = false;
    /** the reading, in inches */
    double measuredIn = 0.0;
    /** what the map says it should read, in inches. Infinity if it would see nothing */
    double expectedIn = 0.0;
    /** the reading's noise, in inches */
    double sigmaIn = 0.0;
    /** (measured - expected) / sigma */
    double residualSigmas = 0.0;
    /** whether the residual is within the agreement threshold */
    bool agrees = false;
};

/**
 * @brief Monte Carlo localization: a cloud of guesses at the robot's position, kept honest by
 * distance sensors
 *
 * Each cycle has three steps:
 *   1. predict(): move every particle by what odometry measured, plus a little random noise,
 *      since odometry isn't perfect
 *   2. weigh(): for every particle, work out what each distance sensor would read from there and
 *      compare it with what it did read. Particles that explain the readings gain weight
 *   3. resampleIfNeeded(): copy heavy particles and drop light ones, so the cloud gathers where
 *      the robot really is
 *
 * Particles only guess x and y. Heading comes from the IMU, which on a V5 robot is far better than
 * anything distance sensors could work out
 *
 * Pure (no PROS), and not thread-safe: one task owns it. localization::MonteCarloLocalizer runs
 * one on the robot, feeding it odometry and distance sensors
 *
 * @b Example
 * @code {.cpp}
 * using namespace sapphirelib::localization;
 * ParticleFilter filter(FieldMap::centered(), {{.forwardIn = 7, .facingDeg = 0}});
 * filter.reset(0, 0, 2.0); // start around (0, 0), give or take 2in
 * filter.predict(0.5, 0, 0); // odometry says we moved 0.5in in +x
 * DistanceReading front{.distanceIn = 60.1, .valid = true};
 * filter.weigh(0.0, std::span(&front, 1)); // facing +y, the front sensor reads 60.1in
 * Estimate where = filter.estimate();
 * filter.resampleIfNeeded();
 * @endcode
 */
class ParticleFilter {
public:
    /**
     * @brief Construct a new ParticleFilter
     *
     * Particles start at (0, 0); call reset() to place them
     *
     * @param map what the sensors can see
     * @param sensors where each sensor is on the robot. weigh() takes readings in this order
     * @param config particle count, noise, and sensor model
     */
    ParticleFilter(FieldMap map, std::vector<DistanceSensorMount> sensors,
                   ParticleFilterConfig config = {});

    /**
     * @brief Scatter the particles around a known position
     *
     * @param xIn x, in inches
     * @param yIn y, in inches
     * @param spreadIn how far off the position may be, in inches (1 standard deviation)
     */
    void reset(double xIn, double yIn, double spreadIn);

    /**
     * @brief Scatter the particles evenly over the whole field, to find the robot from scratch
     *
     * Needs the robot to move around and plenty of particles before it converges, and a symmetric
     * field can fool it
     */
    void resetUniform();

    /**
     * @brief Move every particle by one odometry step, plus noise
     *
     * @param dxIn how far odometry moved in x since the last predict(), in inches
     * @param dyIn how far odometry moved in y, in inches
     * @param turnedDeg how far the robot turned, in degrees. Only its size matters
     */
    void predict(double dxIn, double dyIn, double turnedDeg);

    /**
     * @brief Weigh every particle by how well it explains the sensor readings
     *
     * @param headingDeg the robot's heading, in degrees clockwise from +y
     * @param readings one per sensor, in the order they were given to the constructor. Invalid
     * readings are skipped
     * @return true if any reading was valid. With none, nothing changes
     */
    bool weigh(double headingDeg, std::span<const DistanceReading> readings);

    /**
     * @brief Resample, if the weights have become lopsided or recovery wants fresh particles
     *
     * Draws a new cloud with systematic (low variance) resampling, so a particle with twice the
     * weight gets about twice the copies, then swaps in fresh particles for recovery. Every
     * weight ends up equal
     *
     * @return true if it resampled
     */
    bool resampleIfNeeded();

    /**
     * @brief Get the weighted mean and spread of the particles
     */
    Estimate estimate() const;

    /**
     * @brief Compare each reading with what the map says it should read from one pose
     *
     * Used to decide whether an estimate is trustworthy: a sensor blocked by another robot
     * disagrees with the map, the rest still agree
     *
     * @param xIn the pose's x, in inches
     * @param yIn the pose's y, in inches
     * @param headingDeg the pose's heading, in degrees
     * @param readings one per sensor
     * @param agreementSigmas a reading agrees when it's within this many sigmas of the map
     * @param out one SensorCheck per sensor, filled in. May be empty
     * @return std::size_t how many valid readings agree
     */
    std::size_t checkSensors(double xIn, double yIn, double headingDeg,
                             std::span<const DistanceReading> readings, double agreementSigmas,
                             std::span<SensorCheck> out) const;

    /**
     * @brief Get the fraction of particles the next resample would replace for recovery, 0 to 1
     */
    double recoveryFraction() const;

    /**
     * @brief Get how many particles the last resample replaced for recovery
     */
    std::size_t lastRecovered() const;

    /**
     * @brief Get the particles
     */
    const std::vector<Particle>& particles() const;

    /**
     * @brief Get the map
     */
    const FieldMap& map() const;

    /**
     * @brief Get the sensor mounts
     */
    const std::vector<DistanceSensorMount>& sensors() const;

    /**
     * @brief Get the settings
     */
    const ParticleFilterConfig& config() const;

private:
    FieldMap map_;
    std::vector<DistanceSensorMount> sensors_;
    ParticleFilterConfig config_;
    Rng rng_;

    std::vector<Particle> particles_;

    // scratch space, sized once so no update allocates
    std::vector<Particle> resampled_;
    std::vector<double> logWeights_;
    std::vector<double> logLikelihoods_;
    std::vector<SensorRay> rays_;
    std::vector<double> sigmas_;

    // recovery's running averages of how well particles explain one reading
    double slowAverage_ = 0.0;
    double fastAverage_ = 0.0;
    bool averagesPrimed_ = false;
    std::size_t lastRecovered_ = 0;
};

} // namespace sapphirelib::localization
