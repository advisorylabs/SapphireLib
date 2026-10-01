#include "sapphirelib/localization/monte_carlo_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <utility>

#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/clock.hpp"

namespace sapphirelib::localization {

namespace {

std::vector<DistanceSensorMount> mountsOf(const std::vector<DistanceSensorConfig>& sensors) {
    std::vector<DistanceSensorMount> mounts;
    mounts.reserve(sensors.size());
    for (const DistanceSensorConfig& sensor : sensors) mounts.push_back(sensor.mount);
    return mounts;
}

} // namespace

MonteCarloLocalizer::MonteCarloLocalizer(odom::Odometry& odometry,
                                         std::vector<DistanceSensorConfig> sensors, FieldMap map,
                                         LocalizerConfig config)
    : odometry_(odometry), config_(config),
      filter_(std::move(map), mountsOf(sensors), config.filter),
      correctionEnabled_(config.correctOdometry) {
    sensors_.reserve(sensors.size());
    for (const DistanceSensorConfig& sensor : sensors) sensors_.emplace_back(sensor.port);
    readings_.resize(sensors.size());
    checks_.resize(sensors.size());
    beams_.resize(sensors.size());
}

void MonteCarloLocalizer::update() {
    const std::uint64_t startUs = sapphirelib::micros();
    const std::uint32_t nowMs = pros::millis();
    const odom::Odometry::Snapshot snapshot = odometry_.snapshot();
    const odom::Pose& raw = snapshot.rawPose;

    // start over around odometry's pose at the first update and after every setPose(), since the
    // particles belong to the old frame
    const bool relocalize = relocalizeRequested_.exchange(false);
    if (!started_ || snapshot.resetCount != lastResetCount_ || relocalize) {
        if (relocalize) {
            filter_.resetUniform();
        } else {
            filter_.reset(snapshot.pose.xIn, snapshot.pose.yIn, config_.startSpreadIn);
        }
        started_ = true;
        lastResetCount_ = snapshot.resetCount;
        lastRawPose_ = raw;
        lastUpdateMs_ = nowMs;
        // setPose() cleared odometry's correction
        correctionXIn_.store(0.0);
        correctionYIn_.store(0.0);
    }

    // predict from raw odometry's travel, which a correction easing in never moves
    const double dxIn = raw.xIn - lastRawPose_.xIn;
    const double dyIn = raw.yIn - lastRawPose_.yIn;
    const double turnedDeg = wrapDegrees180(raw.headingDeg - lastRawPose_.headingDeg);
    const double dtS = (nowMs - lastUpdateMs_) / 1000.0;
    lastRawPose_ = raw;
    lastUpdateMs_ = nowMs;
    filter_.predict(dxIn, dyIn, turnedDeg);

    // readings describe a moment about sensorLatencyMs ago: point the beams the way the robot
    // faced then, and move each reading along its beam to where the robot is now
    const double latencyS = config_.sensorLatencyMs / 1000.0;
    const double turnRateDegPerS = dtS > 0.0 ? turnedDeg / dtS : 0.0;
    const double velocityX = dtS > 0.0 ? dxIn / dtS : 0.0;
    const double velocityY = dtS > 0.0 ? dyIn / dtS : 0.0;
    const double beamHeadingDeg = raw.headingDeg - turnRateDegPerS * latencyS;
    const bool spinning = std::fabs(turnRateDegPerS) > config_.maxTurnRateDegPerS;

    std::uint32_t used = 0;
    for (std::size_t s = 0; s < sensors_.size(); ++s) {
        DistanceReading reading =
            distanceReadingFromMm(sensors_[s].get_distance(), sensors_[s].get_confidence(),
                                  config_.minConfidence, config_.filter.beam);
        if (spinning) reading.valid = false;
        const SensorRay ray = sensorRay(0.0, 0.0, beamHeadingDeg, filter_.sensors()[s]);
        const double closingSpeed = velocityX * ray.dirX + velocityY * ray.dirY;
        reading = compensateLatency(reading, closingSpeed, latencyS);
        readings_[s] = reading;
        beams_[s].closingSpeedInPerS = closingSpeed;
        if (reading.valid) ++used;
    }

    const bool weighed = filter_.weigh(beamHeadingDeg, readings_);
    const Estimate estimate = filter_.estimate();
    const std::size_t agreeing = filter_.checkSensors(estimate.xIn, estimate.yIn, beamHeadingDeg,
                                                      readings_, config_.agreementSigmas, checks_);
    const bool resampled = filter_.resampleIfNeeded();

    // only correct odometry in the field frame, from an estimate that's sure of itself and
    // matches the walls
    const bool inFieldFrame = !config_.waitForSetPose || snapshot.resetCount > 0;
    std::uint32_t blockedBy = 0;
    if (!correctionEnabled_.load()) blockedBy |= LocalizationStatus::kCorrectionOff;
    if (!inFieldFrame) blockedBy |= LocalizationStatus::kNoSetPose;
    if (!weighed) blockedBy |= LocalizationStatus::kNoReadings;
    if (spinning) blockedBy |= LocalizationStatus::kSpinning;
    if (!(estimate.spreadIn <= config_.maxCorrectionSpreadIn)) {
        blockedBy |= LocalizationStatus::kTooSpread;
    }
    if (agreeing < config_.minAgreeingSensors) blockedBy |= LocalizationStatus::kTooFewAgree;
    const double correctionXIn = estimate.xIn - raw.xIn;
    const double correctionYIn = estimate.yIn - raw.yIn;
    // refused if a setPose() landed since the snapshot; the next update starts over
    if (blockedBy == 0 &&
        !odometry_.setPositionCorrection(correctionXIn, correctionYIn,
                                         config_.maxCorrectionRateInPerS, snapshot.resetCount)) {
        blockedBy |= LocalizationStatus::kRefused;
    }
    const bool correcting = blockedBy == 0;

    estimateXIn_.store(estimate.xIn);
    estimateYIn_.store(estimate.yIn);
    estimateHeadingDeg_.store(raw.headingDeg);
    spreadIn_.store(estimate.spreadIn);
    effectiveParticles_.store(estimate.effectiveParticles);
    sensorsUsed_.store(used);
    sensorsAgreeing_.store(static_cast<std::uint32_t>(agreeing));
    correcting_.store(correcting);
    blockedBy_.store(blockedBy);
    if (correcting) {
        correctionXIn_.store(correctionXIn);
        correctionYIn_.store(correctionYIn);
    }
    updates_.fetch_add(1);
    // the callback's own cost isn't the localizer's, so the clock stops here
    const auto updateUs = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(sapphirelib::micros() - startUs, UINT32_MAX));
    updateUs_.store(updateUs);

    // uncontended except the moment it's set, and this task is never deleted holding it
    std::lock_guard<pros::Mutex> lock(callbackMutex_);
    if (!callback_) return;
    constexpr double kNoReading = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t s = 0; s < beams_.size(); ++s) {
        BeamSample& beam = beams_[s];
        beam.used = readings_[s].valid;
        beam.measuredIn = beam.used ? readings_[s].distanceIn : kNoReading;
        if (checks_[s].used) {
            beam.expectedIn = checks_[s].expectedIn;
        } else {
            // checkSensors() skips unused readings, but a wall in range with no reading is worth
            // knowing about too
            const SensorRay ray =
                sensorRay(estimate.xIn, estimate.yIn, beamHeadingDeg, filter_.sensors()[s]);
            beam.expectedIn = filter_.map().castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
        }
    }
    callback_(LocalizerUpdate{.timeMs = nowMs,
                              .rawPose = raw,
                              .estimate = estimate,
                              .status = status(),
                              .beamHeadingDeg = beamHeadingDeg,
                              .resampled = resampled,
                              .recovered = static_cast<std::uint32_t>(filter_.lastRecovered()),
                              .fit = filter_.recoveryFit(),
                              .beams = std::span<const BeamSample>(beams_),
                              .particles = std::span<const Particle>(filter_.particles())});
}

void MonteCarloLocalizer::startTask(std::uint32_t periodMs) {
    if (task_) return;
    task_ = std::make_unique<pros::Task>(
        [this, periodMs] {
            while (true) {
                update();
                pros::delay(periodMs);
            }
        },
        "Localizer");
}

LocalizationStatus MonteCarloLocalizer::status() const {
    return LocalizationStatus{
        .estimate = odom::Pose{.xIn = estimateXIn_.load(),
                               .yIn = estimateYIn_.load(),
                               .headingDeg = estimateHeadingDeg_.load()},
        .spreadIn = spreadIn_.load(),
        .effectiveParticles = effectiveParticles_.load(),
        .sensorsUsed = sensorsUsed_.load(),
        .sensorsAgreeing = sensorsAgreeing_.load(),
        .correcting = correcting_.load(),
        .blockedBy = blockedBy_.load(),
        .correctionXIn = correctionXIn_.load(),
        .correctionYIn = correctionYIn_.load(),
        .updates = updates_.load(),
        .updateUs = updateUs_.load(),
    };
}

void MonteCarloLocalizer::setCorrectionEnabled(bool enabled) { correctionEnabled_.store(enabled); }

bool MonteCarloLocalizer::correctionEnabled() const { return correctionEnabled_.load(); }

void MonteCarloLocalizer::relocalizeGlobally() { relocalizeRequested_.store(true); }

const LocalizerConfig& MonteCarloLocalizer::config() const { return config_; }

const std::vector<DistanceSensorMount>& MonteCarloLocalizer::sensorMounts() const {
    return filter_.sensors();
}

void MonteCarloLocalizer::setUpdateCallback(std::function<void(const LocalizerUpdate&)> callback) {
    std::lock_guard<pros::Mutex> lock(callbackMutex_);
    callback_ = std::move(callback);
}

} // namespace sapphirelib::localization
