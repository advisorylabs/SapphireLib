#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/localization/field_map.hpp"
#include "sapphirelib/localization/particle_filter.hpp"
#include "sapphirelib/localization/sensor_model.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/pose.hpp"

namespace sapphirelib::localization {

/**
 * @brief One distance sensor the localizer reads
 */
struct DistanceSensorConfig {
    /** smart port, 1 to 21 */
    std::uint8_t port = 0;

    /** where it's mounted and which way it faces */
    DistanceSensorMount mount;
};

/**
 * @brief Settings for a MonteCarloLocalizer
 *
 * The defaults suit a V5 robot with good tracking wheels and four distance sensors. The simulator
 * (tools/sim) runs the same settings, so try changes there first
 */
struct LocalizerConfig {
    /** the particle filter: particle count, noise, the sensor model, recovery */
    ParticleFilterConfig filter;

    /**
     * how far odometry's pose may be off, in inches (1 standard deviation), when particles are
     * placed around it: at the first update and after every Odometry::setPose()
     */
    double startSpreadIn = 2.0;

    /**
     * how old a distance reading is by the time it's read, in milliseconds. The sensor measures
     * at about 30Hz, so a reading is on average a few tens of milliseconds old; at 60in/s that's
     * nearly 2in. Compensated for with odometry's velocity. 0 turns compensation off
     */
    double sensorLatencyMs = 30.0;

    /** readings past 200mm with a lower confidence (0 to 63) are skipped */
    std::int32_t minConfidence = 20;

    /**
     * skip the sensors while turning faster than this, in degrees per second. Mid-spin, a beam
     * sweeps across whatever it's pointed at while the sensor measures. Prediction keeps running
     */
    double maxTurnRateDegPerS = 200.0;

    /** whether to correct odometry's pose. See MonteCarloLocalizer::setCorrectionEnabled() */
    bool correctOdometry = true;

    /**
     * don't correct odometry until Odometry::setPose() has run once. Until then the pose is
     * relative to wherever the robot sat at startup, not the field the map describes, and the
     * particles can only hunt for the robot. True by default; set it false if the Odometry is
     * constructed with the robot's real field pose
     */
    bool waitForSetPose = true;

    /** only correct while the particles' spread is below this, in inches */
    double maxCorrectionSpreadIn = 3.0;

    /**
     * only correct when at least this many sensors agree with the estimate: a sensor blocked by
     * another robot disagrees, the walls the others see still agree
     */
    std::size_t minAgreeingSensors = 2;

    /** a sensor agrees when it's within this many sigmas of what the map says from the estimate */
    double agreementSigmas = 3.0;

    /**
     * how fast a correction may move odometry's pose, in inches per second. A motion's derivative
     * term sees this as extra speed, so keep it well under the drive's top speed. 0 or less jumps
     */
    double maxCorrectionRateInPerS = 4.0;
};

/**
 * @brief What the localizer thinks, as of its last update
 */
struct LocalizationStatus {
    /** the particles' weighted mean, with odometry's heading */
    odom::Pose estimate;

    /** how far particles are from the estimate on average, in inches. Smaller is surer */
    double spreadIn = 0.0;

    /** how many particles carry the estimate. See Estimate::effectiveParticles */
    double effectiveParticles = 0.0;

    /** how many sensors had a usable reading */
    std::uint32_t sensorsUsed = 0;

    /** how many of those agree with the estimate */
    std::uint32_t sensorsAgreeing = 0;

    /** whether the last update passed every check and set odometry's correction */
    bool correcting = false;

    /** the correction odometry is easing toward, in inches: how far off raw odometry is */
    double correctionXIn = 0.0;

    /** the correction odometry is easing toward in y, in inches */
    double correctionYIn = 0.0;

    /** how many updates have run */
    std::uint32_t updates = 0;
};

/**
 * @brief Monte Carlo localization on top of odometry: distance sensors keep the pose honest
 *
 * Odometry drifts: tracking wheels slip, get bumped, and read a percent or two long or short, and
 * every error stays in the pose for the rest of the match. This runs a ParticleFilter beside it,
 * moving particles by odometry's measured travel and weighing them against what the distance
 * sensors see of the field walls, then feeds the difference back as
 * Odometry::setPositionCorrection(). Every motion reads getPose(), so moveToPoint(), moveToPose()
 * and followPath(), with whatever gains Auto-Tune gave them, drive by the corrected pose with no
 * other changes. Heading is left to the IMU
 *
 * A correction is only set while the estimate is trustworthy: the particles agree
 * (maxCorrectionSpreadIn) and enough sensors match the map from it (minAgreeingSensors).
 * Otherwise odometry keeps the last correction and carries on alone
 *
 * The map is in field coordinates, so odometry has to be too: start autonomous with
 * Odometry::setPose() at the robot's real place on the field. The localizer notices every
 * setPose() and starts its particles over around the new pose, and by default doesn't correct
 * anything before the first one (LocalizerConfig::waitForSetPose)
 *
 * @b Example
 * @code {.cpp}
 * using namespace sapphirelib::localization;
 *
 * MonteCarloLocalizer& localizer() {
 *     static MonteCarloLocalizer instance(
 *         odometry(),
 *         {
 *             {.port = 1, .mount = {.forwardIn = 6.5, .rightIn = 0, .facingDeg = 0}},
 *             {.port = 4, .mount = {.forwardIn = 0, .rightIn = 7, .facingDeg = 90}},
 *             {.port = 6, .mount = {.forwardIn = -6.5, .rightIn = 0, .facingDeg = 180}},
 *             {.port = 11, .mount = {.forwardIn = 0, .rightIn = -7, .facingDeg = 270}},
 *         },
 *         FieldMap::centered());
 *     return instance;
 * }
 *
 * void initialize() {
 *     odometry().startTask();
 *     localizer().startTask();
 * }
 *
 * void autonomous() {
 *     odometry().setPose({-48, -60, 90}); // where the robot really starts, in field coordinates
 *     drivetrain().moveToPoint(-24, -24);  // drives by the corrected pose
 * }
 * @endcode
 */
class MonteCarloLocalizer {
public:
    /**
     * @brief Construct a new MonteCarloLocalizer
     *
     * @param odometry the odometry to follow and correct. Must outlive the localizer
     * @param sensors the distance sensors, in any order
     * @param map what the sensors can see, in field coordinates
     * @param config filter, gating, and correction settings
     */
    MonteCarloLocalizer(odom::Odometry& odometry, std::vector<DistanceSensorConfig> sensors,
                        FieldMap map, LocalizerConfig config = {});

    /**
     * @brief Run one update: predict from odometry, weigh against the sensors, resample, correct
     *
     * Call this on your own loop, or use startTask()
     *
     * @note only one task may call it
     */
    void update();

    /**
     * @brief Start a task that calls update() every periodMs
     *
     * Only the first call starts a task. The task runs for the rest of the program, so the
     * localizer must be static
     *
     * @param periodMs update period, in milliseconds. 50 by default: the sensors measure at about
     * 30Hz, so updating much faster weighs the same reading twice
     */
    void startTask(std::uint32_t periodMs = 50);

    /**
     * @brief Get what the localizer thinks, as of its last update
     *
     * @note safe to call from any task. Fields are separate atomics, so a read can mix two updates
     */
    LocalizationStatus status() const;

    /**
     * @brief Turn correcting odometry on or off. Safe to call from any task
     *
     * Off, the filter keeps running (status() stays live) but leaves odometry alone. The
     * correction already applied stays until the next Odometry::setPose()
     *
     * @param enabled whether to correct odometry
     *
     * @b Example
     * @code {.cpp}
     * // odometry alone for a test run, to compare against
     * localizer().setCorrectionEnabled(false);
     * @endcode
     */
    void setCorrectionEnabled(bool enabled);

    /**
     * @brief Whether the localizer is correcting odometry
     */
    bool correctionEnabled() const;

    /**
     * @brief Scatter the particles over the whole field at the next update, to find the robot
     * from scratch. Safe to call from any task
     *
     * For when odometry's pose is no use at all. Needs the robot to move around with clear views
     * of the walls before it converges
     */
    void relocalizeGlobally();

private:
    odom::Odometry& odometry_;
    std::vector<pros::Distance> sensors_;
    LocalizerConfig config_;
    ParticleFilter filter_;

    // update() task only
    std::vector<DistanceReading> readings_;
    std::vector<SensorCheck> checks_;
    bool started_ = false;
    std::uint32_t lastResetCount_ = 0;
    odom::Pose lastRawPose_;
    std::uint32_t lastUpdateMs_ = 0;

    std::atomic<bool> correctionEnabled_;
    std::atomic<bool> relocalizeRequested_{false};

    // see status()
    std::atomic<double> estimateXIn_{0.0};
    std::atomic<double> estimateYIn_{0.0};
    std::atomic<double> estimateHeadingDeg_{0.0};
    std::atomic<double> spreadIn_{0.0};
    std::atomic<double> effectiveParticles_{0.0};
    std::atomic<std::uint32_t> sensorsUsed_{0};
    std::atomic<std::uint32_t> sensorsAgreeing_{0};
    std::atomic<bool> correcting_{false};
    std::atomic<double> correctionXIn_{0.0};
    std::atomic<double> correctionYIn_{0.0};
    std::atomic<std::uint32_t> updates_{0};

    std::unique_ptr<pros::Task> task_;
};

} // namespace sapphirelib::localization
