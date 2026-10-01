#include "sapphirelib/localization/monte_carlo_localizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <utility>

#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"

namespace sapphirelib::localization {

namespace {

std::vector<DistanceSensorMount> mountsOf(const std::vector<DistanceSensorConfig>& sensors) {
    std::vector<DistanceSensorMount> mounts;
    mounts.reserve(sensors.size());
    for (const DistanceSensorConfig& sensor : sensors) mounts.push_back(sensor.mount);
    return mounts;
}

// calibrateSensorMounts()'s spin: its loop, and how often it reads the sensors (about their rate)
constexpr std::uint32_t kSpinLoopMs = 10;
constexpr std::uint32_t kSpinReadMs = 30;
// the turn rate is measured over this long
constexpr std::uint32_t kSpinRateWindowMs = 50;
// the turn command starts here, and grows or shrinks by this much a second per deg/s off the rate:
// a robot that turns 450 deg/s flat out settles on its rate in about half a second
constexpr double kSpinStartPower = 0.1;
constexpr double kSpinPowerGain = 0.004;
// how long to let the robot stop before turning back
constexpr std::uint32_t kSpinStopMs = 500;
// a turn that takes this many times as long as it should, plus the margin, is stuck
constexpr double kSpinTimeoutFactor = 3.0;
constexpr std::uint32_t kSpinTimeoutMarginMs = 3000;

// odometry's raw pose over the last 640 ms, with the heading unwrapped, so each reading can be
// paired with where the robot was when the sensor measured it, sensorLatencyMs before it's read
class PoseHistory {
public:
    struct Entry {
        std::uint32_t timeMs = 0;
        double xIn = 0.0;
        double yIn = 0.0;
        double headingDeg = 0.0;
    };

    void add(std::uint32_t timeMs, const odom::Pose& pose) {
        heading_ = count_ == 0 ? pose.headingDeg
                               : heading_ + wrapDegrees180(pose.headingDeg - lastWrappedDeg_);
        lastWrappedDeg_ = pose.headingDeg;
        entries_[next_] = Entry{timeMs, pose.xIn, pose.yIn, heading_};
        next_ = (next_ + 1) % kSize;
        count_ = std::min(count_ + 1, kSize);
    }

    const Entry& latest() const { return entries_[(next_ + kSize - 1) % kSize]; }

    // the pose at a moment, interpolated between the two it falls between. The oldest one kept
    // for a moment before it
    Entry at(std::uint32_t timeMs) const {
        Entry newer = latest();
        if (timeMs >= newer.timeMs) return newer;
        for (std::size_t back = 1; back < count_; ++back) {
            const Entry& older = entries_[(next_ + kSize - 1 - back) % kSize];
            if (older.timeMs <= timeMs) {
                const double span = static_cast<double>(newer.timeMs - older.timeMs);
                const double t = span > 0.0 ? (timeMs - older.timeMs) / span : 0.0;
                return Entry{timeMs, older.xIn + t * (newer.xIn - older.xIn),
                             older.yIn + t * (newer.yIn - older.yIn),
                             older.headingDeg + t * (newer.headingDeg - older.headingDeg)};
            }
            newer = older;
        }
        return newer;
    }

    // degrees per second, clockwise positive, over the last windowMs
    double turnRateDegPerS(std::uint32_t windowMs) const {
        const Entry& now = latest();
        const Entry then = at(now.timeMs > windowMs ? now.timeMs - windowMs : 0);
        const std::uint32_t elapsedMs = now.timeMs - then.timeMs;
        return elapsedMs > 0 ? 1000.0 * (now.headingDeg - then.headingDeg) / elapsedMs : 0.0;
    }

private:
    static constexpr std::size_t kSize = 64;
    std::array<Entry, kSize> entries_{};
    std::size_t next_ = 0;
    std::size_t count_ = 0;
    double heading_ = 0.0;
    double lastWrappedDeg_ = 0.0;
};

// a word for a sensor by which way it faces, for the log
const char* sideOf(const DistanceSensorMount& mount) {
    const double facing = std::fmod(std::fmod(mount.facingDeg, 360.0) + 360.0, 360.0);
    const char* sides[] = {"front", "right", "back", "left"};
    for (int side = 0; side < 4; ++side) {
        if (std::fabs(wrapDegrees180(facing - 90.0 * side)) < 1.0) return sides[side];
    }
    return "angled";
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
    mounts_ = filter_.sensors();
}

void MonteCarloLocalizer::update() {
    const std::uint64_t startUs = sapphirelib::micros();
    const std::uint32_t nowMs = pros::millis();

    // mounts from setSensorMounts(), handed over here since only this task touches the filter.
    // Locked only when there's something new, so a task deleted holding the lock can't stall it
    if (mountsChanged_.exchange(false)) {
        std::lock_guard<pros::Mutex> lock(mountsMutex_);
        filter_.setSensors(mounts_);
    }
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

std::vector<DistanceSensorMount> MonteCarloLocalizer::sensorMounts() const {
    std::lock_guard<pros::Mutex> lock(mountsMutex_);
    return mounts_;
}

bool MonteCarloLocalizer::setSensorMounts(std::vector<DistanceSensorMount> mounts) {
    if (mounts.size() != sensors_.size()) return false;
    {
        std::lock_guard<pros::Mutex> lock(mountsMutex_);
        mounts_ = std::move(mounts);
    }
    mountsChanged_.store(true);
    return true;
}

MountCalibrationResult
MonteCarloLocalizer::calibrateSensorMounts(std::function<void(double)> setSpin,
                                           const MountCalibrationConfig& config) {
    const std::vector<DistanceSensorMount> mounts = sensorMounts();
    MountCalibrationResult result;
    result.sensors.resize(mounts.size());
    for (std::size_t s = 0; s < mounts.size(); ++s) result.sensors[s].mount = mounts[s];
    if (!setSpin) {
        result.error = "nothing to spin the robot with";
        return result;
    }
    if (calibratingMounts_.exchange(true)) {
        result.error = "already calibrating";
        return result;
    }
    SAPPHIRELIB_LOG_INFO("mcl", "calibrating sensor mounts: spinning %.1f turns each way",
                         std::fabs(config.turns));
    telemetry::event("mcl", "mount_calibration,start");

    // one turn (or config.turns) each way: the sensors' leftover delay reads one way turning
    // clockwise and the other way back, and cancels
    const double turnDeg = std::fabs(config.turns) * 360.0;
    const double rateDegPerS = std::max(config.turnRateDegPerS, 1.0);
    const double maxPower = std::clamp(config.maxSpinPower, 0.0, 1.0);
    const auto latencyMs =
        static_cast<std::uint32_t>(std::lround(std::max(config_.sensorLatencyMs, 0.0)));
    const auto timeoutMs =
        static_cast<std::uint32_t>(kSpinTimeoutFactor * 1000.0 * turnDeg / rateDegPerS) +
        kSpinTimeoutMarginMs;
    std::vector<MountSample> samples;
    samples.reserve(static_cast<std::size_t>(2.0 * 1000.0 * turnDeg / rateDegPerS / kSpinReadMs) *
                    sensors_.size());

    PoseHistory history;
    // the raw pose: a correction easing in mid-spin isn't the robot moving
    const auto record = [&] { history.add(pros::millis(), odometry_.snapshot().rawPose); };
    record();
    bool stuck = false;
    for (const double direction : {1.0, -1.0}) {
        const double startDeg = history.latest().headingDeg;
        const std::uint32_t startMs = pros::millis();
        std::uint32_t lastMs = startMs;
        std::uint32_t lastReadMs = 0;
        double power = std::min(kSpinStartPower, maxPower);
        while (std::fabs(history.latest().headingDeg - startDeg) < turnDeg) {
            if (pros::millis() - startMs > timeoutMs) {
                stuck = true;
                break;
            }
            setSpin(direction * power);
            pros::delay(kSpinLoopMs);
            record();
            const PoseHistory::Entry& now = history.latest();

            // hold the turn rate: more power while it's slow, less while it's fast
            const double dtS = (now.timeMs - lastMs) / 1000.0;
            lastMs = now.timeMs;
            const double rate = std::fabs(history.turnRateDegPerS(kSpinRateWindowMs));
            power = std::clamp(power + kSpinPowerGain * (rateDegPerS - rate) * dtS, 0.0, maxPower);

            if (now.timeMs - lastReadMs < kSpinReadMs) continue;
            lastReadMs = now.timeMs;
            // a reading describes the robot as it was about sensorLatencyMs ago
            const PoseHistory::Entry then =
                history.at(now.timeMs > latencyMs ? now.timeMs - latencyMs : 0);
            for (std::size_t s = 0; s < sensors_.size(); ++s) {
                const DistanceReading reading =
                    distanceReadingFromMm(sensors_[s].get_distance(), sensors_[s].get_confidence(),
                                          config_.minConfidence, config_.filter.beam);
                if (!reading.valid) continue;
                samples.push_back(MountSample{.sensor = s,
                                              .xIn = then.xIn,
                                              .yIn = then.yIn,
                                              .headingDeg = then.headingDeg,
                                              .distanceIn = reading.distanceIn});
            }
        }
        setSpin(0.0);
        // let it stop before turning back, still watching it
        const std::uint32_t stopMs = pros::millis();
        while (pros::millis() - stopMs < kSpinStopMs) {
            pros::delay(kSpinLoopMs);
            record();
        }
        if (stuck) break;
    }
    setSpin(0.0);

    if (stuck) {
        result.error = "the spin didn't finish: blocked?";
        SAPPHIRELIB_LOG_ERROR("mcl", "sensor mount calibration: %s", result.error);
        telemetry::event("mcl", "mount_calibration,failed,%s", result.error);
        calibratingMounts_.store(false);
        return result;
    }

    // the map never changes after construction, so reading it from this task is safe
    result = fitSensorMounts(samples, mounts, filter_.map(), config_.filter.beam, config);
    if (result.ok) {
        std::vector<DistanceSensorMount> fitted;
        fitted.reserve(result.sensors.size());
        for (const SensorMountFit& sensor : result.sensors) fitted.push_back(sensor.mount);
        setSensorMounts(std::move(fitted));
    }

    // what it found, ready to copy into the code that builds the localizer
    SAPPHIRELIB_LOG_INFO(
        "mcl", "sensor mounts: %u readings, %u used%s%s", static_cast<unsigned>(result.readings),
        static_cast<unsigned>(result.readingsUsed), result.ok ? "" : ": ", result.error);
    for (std::size_t s = 0; s < result.sensors.size(); ++s) {
        const SensorMountFit& sensor = result.sensors[s];
        const bool calibrated = sensor.status == MountFitStatus::calibrated;
        SAPPHIRELIB_LOG_INFO("mcl",
                             "  sensor %u (%s, port %d): %s, {.forwardIn = %.2f, .rightIn = %.2f, "
                             ".facingDeg = %.0f}, %+.2f in along the beam (+-%.2f), sideways %s",
                             static_cast<unsigned>(s), sideOf(sensor.mount),
                             static_cast<int>(sensors_[s].get_port()),
                             mountFitStatusText(sensor.status), sensor.mount.forwardIn,
                             sensor.mount.rightIn, sensor.mount.facingDeg,
                             calibrated ? sensor.alongChangeIn : 0.0, sensor.alongStderrIn,
                             sensor.lateralFitted ? "fitted" : "kept");
        telemetry::event("mcl", "mount,%u,%s,forward=%.3f,right=%.3f,facing=%.1f,stderr=%.3f",
                         static_cast<unsigned>(s), calibrated ? "calibrated" : "kept",
                         sensor.mount.forwardIn, sensor.mount.rightIn, sensor.mount.facingDeg,
                         sensor.alongStderrIn);
    }
    calibratingMounts_.store(false);
    return result;
}

void MonteCarloLocalizer::setUpdateCallback(std::function<void(const LocalizerUpdate&)> callback) {
    std::lock_guard<pros::Mutex> lock(callbackMutex_);
    callback_ = std::move(callback);
}

} // namespace sapphirelib::localization
