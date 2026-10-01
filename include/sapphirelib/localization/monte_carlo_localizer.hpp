#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/localization/field_map.hpp"
#include "sapphirelib/localization/localizer_config.hpp"
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
 * @brief What the localizer thinks, as of its last update
 */
struct LocalizationStatus {
    /** blockedBy flag: setCorrectionEnabled(false) */
    static constexpr std::uint32_t kCorrectionOff = 1u << 0;
    /** blockedBy flag: no Odometry::setPose() yet (LocalizerConfig::waitForSetPose) */
    static constexpr std::uint32_t kNoSetPose = 1u << 1;
    /** blockedBy flag: no sensor had a usable reading */
    static constexpr std::uint32_t kNoReadings = 1u << 2;
    /** blockedBy flag: the particles were too spread out (LocalizerConfig::maxCorrectionSpreadIn)
     */
    static constexpr std::uint32_t kTooSpread = 1u << 3;
    /** blockedBy flag: too few sensors agreed with the map (LocalizerConfig::minAgreeingSensors) */
    static constexpr std::uint32_t kTooFewAgree = 1u << 4;
    /** blockedBy flag: odometry turned it down, since a setPose() landed during the update */
    static constexpr std::uint32_t kRefused = 1u << 5;
    /**
     * blockedBy flag: turning faster than LocalizerConfig::maxTurnRateDegPerS, so every reading was
     * skipped. Comes with kNoReadings
     */
    static constexpr std::uint32_t kSpinning = 1u << 6;

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

    /** why the last update didn't correct odometry: the k* flags above. 0 when it did */
    std::uint32_t blockedBy = 0;

    /** the correction odometry is easing toward, in inches: how far off raw odometry is */
    double correctionXIn = 0.0;

    /** the correction odometry is easing toward in y, in inches */
    double correctionYIn = 0.0;

    /** how many updates have run */
    std::uint32_t updates = 0;

    /**
     * how long the last update took, in microseconds: wall time, so time other tasks spent
     * preempting it counts too, making it an upper bound on what it cost
     */
    std::uint32_t updateUs = 0;
};

/**
 * @brief One distance sensor in one update, for telemetry: the reading against the map
 */
struct BeamSample {
    /** whether the reading was usable. measuredIn is NaN when it wasn't */
    bool used = false;

    /** the reading, latency compensated, in inches */
    double measuredIn = 0.0;

    /**
     * what the map says the sensor should read from the estimate, in inches. Worked out for every
     * sensor, used or not, so a reading that went missing while a wall was in range shows up.
     * Infinity when the beam would see nothing
     */
    double expectedIn = 0.0;

    /**
     * how fast the sensor was moving toward what it points at, in in/s, by odometry. What latency
     * compensation scaled sensorLatencyMs by, so a reading that's still off in proportion to it
     * says the latency is set wrong
     */
    double closingSpeedInPerS = 0.0;
};

/**
 * @brief Everything one update worked out, handed to the update callback
 */
struct LocalizerUpdate {
    /** when the update ran, in milliseconds since the program started */
    std::uint32_t timeMs = 0;

    /** odometry's pose before any correction, and the correction applied to it so far */
    odom::Pose rawPose;

    /** the estimate the particles settled on this update */
    Estimate estimate;

    /** the same fields status() returns, as of this update */
    LocalizationStatus status;

    /**
     * the heading the beams were cast at, in degrees: odometry's, wound back by
     * LocalizerConfig::sensorLatencyMs of turning
     */
    double beamHeadingDeg = 0.0;

    /** whether this update resampled the particles */
    bool resampled = false;

    /** how many particles recovery replaced with fresh ones this update */
    std::uint32_t recovered = 0;

    /** how well the particles explained this update's readings, and recovery's averages of it */
    RecoveryFit fit;

    /** one per sensor, in the order the localizer was given them. Valid during the call only */
    std::span<const BeamSample> beams;

    /**
     * the particles, as this update left them (after resampling). Valid during the call only.
     * sampleParticles() picks a few that stand for them all
     */
    std::span<const Particle> particles;
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

    /**
     * @brief Get the settings the localizer was built with
     */
    const LocalizerConfig& config() const;

    /**
     * @brief Get the sensors' mounts, in the order given to the constructor
     */
    const std::vector<DistanceSensorMount>& sensorMounts() const;

    /**
     * @brief Call a function at the end of every update, on the localizer's task
     *
     * For telemetry: it gets each sensor's reading next to what the map says it should read,
     * which is what tuning the sensor model from a log needs. Safe to set from any task, before
     * or after startTask(). Keep it quick: it runs on the localizer's task, after the update
     *
     * @param callback called with what the update worked out. nullptr removes it
     *
     * @b Example
     * @code {.cpp}
     * localizer().setUpdateCallback([](const LocalizerUpdate& update) {
     *     beamLog.record({update.beams[0].measuredIn, update.beams[0].expectedIn});
     * });
     * @endcode
     */
    void setUpdateCallback(std::function<void(const LocalizerUpdate&)> callback);

private:
    odom::Odometry& odometry_;
    std::vector<pros::Distance> sensors_;
    LocalizerConfig config_;
    ParticleFilter filter_;

    // update() task only
    std::vector<DistanceReading> readings_;
    std::vector<SensorCheck> checks_;
    std::vector<BeamSample> beams_;
    pros::Mutex callbackMutex_;
    std::function<void(const LocalizerUpdate&)> callback_; // under callbackMutex_
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
    std::atomic<std::uint32_t> blockedBy_{0};
    std::atomic<double> correctionXIn_{0.0};
    std::atomic<double> correctionYIn_{0.0};
    std::atomic<std::uint32_t> updates_{0};
    std::atomic<std::uint32_t> updateUs_{0};

    std::unique_ptr<pros::Task> task_;
};

} // namespace sapphirelib::localization
