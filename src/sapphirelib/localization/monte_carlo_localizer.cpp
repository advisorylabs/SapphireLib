#include "sapphirelib/localization/monte_carlo_localizer.hpp"

#include <cmath>
#include <utility>

#include "sapphirelib/util/angle.hpp"

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
}

void MonteCarloLocalizer::update() {
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
        reading = compensateLatency(reading, velocityX * ray.dirX + velocityY * ray.dirY, latencyS);
        readings_[s] = reading;
        if (reading.valid) ++used;
    }

    const bool weighed = filter_.weigh(beamHeadingDeg, readings_);
    const Estimate estimate = filter_.estimate();
    const std::size_t agreeing = filter_.checkSensors(estimate.xIn, estimate.yIn, beamHeadingDeg,
                                                      readings_, config_.agreementSigmas, checks_);
    filter_.resampleIfNeeded();

    // only correct odometry in the field frame, from an estimate that's sure of itself and
    // matches the walls
    const bool inFieldFrame = !config_.waitForSetPose || snapshot.resetCount > 0;
    bool correcting = correctionEnabled_.load() && inFieldFrame && weighed &&
                      estimate.spreadIn <= config_.maxCorrectionSpreadIn &&
                      agreeing >= config_.minAgreeingSensors;
    const double correctionXIn = estimate.xIn - raw.xIn;
    const double correctionYIn = estimate.yIn - raw.yIn;
    if (correcting) {
        // refused if a setPose() landed since the snapshot; the next update starts over
        correcting = odometry_.setPositionCorrection(
            correctionXIn, correctionYIn, config_.maxCorrectionRateInPerS, snapshot.resetCount);
    }

    estimateXIn_.store(estimate.xIn);
    estimateYIn_.store(estimate.yIn);
    estimateHeadingDeg_.store(raw.headingDeg);
    spreadIn_.store(estimate.spreadIn);
    effectiveParticles_.store(estimate.effectiveParticles);
    sensorsUsed_.store(used);
    sensorsAgreeing_.store(static_cast<std::uint32_t>(agreeing));
    correcting_.store(correcting);
    if (correcting) {
        correctionXIn_.store(correctionXIn);
        correctionYIn_.store(correctionYIn);
    }
    updates_.fetch_add(1);
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
        .correctionXIn = correctionXIn_.load(),
        .correctionYIn = correctionYIn_.load(),
        .updates = updates_.load(),
    };
}

void MonteCarloLocalizer::setCorrectionEnabled(bool enabled) { correctionEnabled_.store(enabled); }

bool MonteCarloLocalizer::correctionEnabled() const { return correctionEnabled_.load(); }

void MonteCarloLocalizer::relocalizeGlobally() { relocalizeRequested_.store(true); }

} // namespace sapphirelib::localization
