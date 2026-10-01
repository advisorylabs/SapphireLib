#include "sapphirelib/localization/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "sapphirelib/util/lookup_table.hpp"

namespace sapphirelib::localization {

namespace {

// added to the log weight of a particle outside the walls, where the robot can't be. Finite, so a
// cloud that's all outside (odometry not in the field frame yet) is left alone instead of breaking
constexpr double kOutsideLogPenalty = -30.0;

// e^x for recovery's fit, which only ever takes x <= 0: 64 samples to the unit, so within
// (1/64)^2 / 8 = 3.1e-5 of exp(x), relatively. Below -32 a particle explains the readings
// e^-32 = 1.3e-14 as well as a perfect one, and counts as 0
const LookupTable& fitExpTable() {
    static const LookupTable table(-32.0, 0.0, 64, [](double x) { return std::exp(x); });
    return table;
}

} // namespace

ParticleFilter::ParticleFilter(FieldMap map, std::vector<DistanceSensorMount> sensors,
                               ParticleFilterConfig config)
    : map_(std::move(map)), sensors_(std::move(sensors)), config_(config), rng_(config.seed) {
    const std::size_t count = std::max<std::size_t>(config_.particleCount, 1);
    config_.particleCount = count;
    particles_.assign(count, Particle{.weight = 1.0 / static_cast<double>(count)});
    resampled_.resize(count);
    logWeights_.resize(count);
    logLikelihoods_.resize(count);
    rays_.resize(sensors_.size());
    casters_.resize(sensors_.size());
    scorers_.resize(sensors_.size());
    usedSensors_.reserve(sensors_.size());
    // aim each caster once, so weigh() never allocates
    for (ParallelRayCaster& caster : casters_) caster.aim(map_, 0.0, 1.0);
}

void ParticleFilter::reset(double xIn, double yIn, double spreadIn) {
    const double weight = 1.0 / static_cast<double>(particles_.size());
    for (Particle& p : particles_) {
        p.xIn = xIn + spreadIn * rng_.gaussian();
        p.yIn = yIn + spreadIn * rng_.gaussian();
        p.weight = weight;
    }
    uniformWeights_ = true;
    averagesPrimed_ = false;
    lastRecovered_ = 0;
}

void ParticleFilter::resetUniform() {
    const double weight = 1.0 / static_cast<double>(particles_.size());
    const double width = map_.maxXIn() - map_.minXIn();
    const double height = map_.maxYIn() - map_.minYIn();
    for (Particle& p : particles_) {
        p.xIn = map_.minXIn() + rng_.uniform() * width;
        p.yIn = map_.minYIn() + rng_.uniform() * height;
        p.weight = weight;
    }
    uniformWeights_ = true;
    averagesPrimed_ = false;
    lastRecovered_ = 0;
}

void ParticleFilter::predict(double dxIn, double dyIn, double turnedDeg) {
    const MotionNoise& noise = config_.motionNoise;
    const double sigmaIn = noise.baseIn + noise.perInch * std::hypot(dxIn, dyIn) +
                           noise.perDegreeIn * std::fabs(turnedDeg);
    for (Particle& p : particles_) {
        p.xIn += dxIn + sigmaIn * rng_.fastGaussian();
        p.yIn += dyIn + sigmaIn * rng_.fastGaussian();
    }
}

bool ParticleFilter::weigh(double headingDeg, std::span<const DistanceReading> readings) {
    const std::size_t sensorCount = std::min(readings.size(), sensors_.size());

    // every particle shares the heading, so each sensor's offset, direction and scoring are worked
    // out once
    usedSensors_.clear();
    double logPerfectFit = 0.0;
    for (std::size_t s = 0; s < sensorCount; ++s) {
        if (!readings[s].valid) continue;
        rays_[s] = sensorRay(0.0, 0.0, headingDeg, sensors_[s]);
        casters_[s].aim(map_, rays_[s].dirX, rays_[s].dirY);
        scorers_[s] = ReadingScorer(readings[s].distanceIn,
                                    readingSigmaIn(config_.beam, readings[s]), config_.beam);
        // the most this reading could score, from a particle exactly where it says
        logPerfectFit += scorers_[s].logLikelihood(readings[s].distanceIn);
        usedSensors_.push_back(s);
    }
    const std::size_t validCount = usedSensors_.size();
    if (validCount == 0) return false;

    // in logs, so four sensors' small densities multiplied together can't underflow
    const double uniformLogWeight = -std::log(static_cast<double>(particles_.size()));
    double maxLogWeight = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        const Particle& p = particles_[i];
        double logLikelihood = map_.contains(p.xIn, p.yIn) ? 0.0 : kOutsideLogPenalty;
        for (const std::size_t s : usedSensors_) {
            const SensorRay& ray = rays_[s];
            logLikelihood +=
                scorers_[s].logLikelihood(casters_[s].castIn(p.xIn + ray.xIn, p.yIn + ray.yIn));
        }
        logLikelihoods_[i] = logLikelihood;
        logWeights_[i] = (uniformWeights_ ? uniformLogWeight : logWeights_[i]) + logLikelihood;
        maxLogWeight = std::max(maxLogWeight, logWeights_[i]);
    }

    // the weights, and alongside them recovery's average: how well the particles explain the
    // readings, as a fraction of a perfect fit, per reading. Against a perfect fit because a far
    // reading's likelihood is lower however right it is (its noise is wider), and per reading so a
    // sensor dropping in or out doesn't look like the match suddenly getting worse or better
    const LookupTable& fitExp = fitExpTable();
    const double inverseValid = 1.0 / static_cast<double>(validCount);
    double sum = 0.0;
    double average = 0.0;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        particles_[i].weight = std::exp(logWeights_[i] - maxLogWeight);
        sum += particles_[i].weight;
        const double fit = (logLikelihoods_[i] - logPerfectFit) * inverseValid;
        if (fit > fitExp.minX()) average += fitExp.at(fit);
    }
    average /= static_cast<double>(particles_.size());
    latestFit_ = average;

    // normalized, and the logs kept to match for the next weigh()
    const double inverseSum = 1.0 / sum;
    const double logSum = std::log(sum);
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        particles_[i].weight *= inverseSum;
        logWeights_[i] = logWeights_[i] - maxLogWeight - logSum;
    }
    uniformWeights_ = false;

    if (!averagesPrimed_) {
        slowAverage_ = average;
        fastAverage_ = average;
        averagesPrimed_ = true;
    } else {
        slowAverage_ += config_.recovery.slowRate * (average - slowAverage_);
        fastAverage_ += config_.recovery.fastRate * (average - fastAverage_);
    }
    return true;
}

double ParticleFilter::recoveryFraction() const {
    const RecoveryConfig& recovery = config_.recovery;
    if (!recovery.enabled || !averagesPrimed_ || !(slowAverage_ > 0.0) ||
        !(recovery.triggerRatio > 0.0)) {
        return 0.0;
    }
    const double ratio = fastAverage_ / slowAverage_;
    return std::clamp(1.0 - ratio / recovery.triggerRatio, 0.0, recovery.maxFraction);
}

std::size_t ParticleFilter::lastRecovered() const { return lastRecovered_; }

RecoveryFit ParticleFilter::recoveryFit() const {
    return RecoveryFit{.latest = latestFit_, .fast = fastAverage_, .slow = slowAverage_};
}

bool ParticleFilter::resampleIfNeeded() {
    const std::size_t count = particles_.size();
    double sumSquares = 0.0;
    for (const Particle& p : particles_) sumSquares += p.weight * p.weight;
    const double effective = 1.0 / sumSquares;

    const double fraction = recoveryFraction();
    const auto recovered = static_cast<std::size_t>(fraction * static_cast<double>(count));
    if (effective >= config_.resampleThreshold * static_cast<double>(count) && recovered == 0) {
        lastRecovered_ = 0;
        return false;
    }

    // systematic resampling: one random offset, then evenly spaced picks along the cumulative
    // weights, so a particle's copies track its weight closely
    const std::size_t kept = count - recovered;
    if (kept > 0) {
        const double step = 1.0 / static_cast<double>(kept);
        const double start = rng_.uniform() * step;
        double cumulative = particles_[0].weight;
        std::size_t source = 0;
        for (std::size_t j = 0; j < kept; ++j) {
            const double target = start + static_cast<double>(j) * step;
            while (target > cumulative && source + 1 < count) {
                ++source;
                cumulative += particles_[source].weight;
            }
            resampled_[j] = particles_[source];
        }
    }

    // fresh particles near the estimate, from before resampling
    if (recovered > 0) {
        const Estimate center = estimate();
        const double radius = config_.recovery.radiusIn;
        const bool wholeField = !(radius > 0.0);
        const double minX =
            wholeField ? map_.minXIn() : std::max(map_.minXIn(), center.xIn - radius);
        const double maxX =
            wholeField ? map_.maxXIn() : std::min(map_.maxXIn(), center.xIn + radius);
        const double minY =
            wholeField ? map_.minYIn() : std::max(map_.minYIn(), center.yIn - radius);
        const double maxY =
            wholeField ? map_.maxYIn() : std::min(map_.maxYIn(), center.yIn + radius);
        for (std::size_t j = kept; j < count; ++j) {
            resampled_[j].xIn = minX + rng_.uniform() * (maxX - minX);
            resampled_[j].yIn = minY + rng_.uniform() * (maxY - minY);
        }
    }

    const double weight = 1.0 / static_cast<double>(count);
    for (Particle& p : resampled_) p.weight = weight;
    std::swap(particles_, resampled_);
    uniformWeights_ = true;
    lastRecovered_ = recovered;
    return true;
}

Estimate ParticleFilter::estimate() const {
    Estimate out;
    double sumSquares = 0.0;
    for (const Particle& p : particles_) {
        out.xIn += p.weight * p.xIn;
        out.yIn += p.weight * p.yIn;
        sumSquares += p.weight * p.weight;
    }
    for (const Particle& p : particles_) {
        const double dx = p.xIn - out.xIn;
        const double dy = p.yIn - out.yIn;
        out.varianceXIn2 += p.weight * dx * dx;
        out.varianceYIn2 += p.weight * dy * dy;
        out.covarianceXYIn2 += p.weight * dx * dy;
    }
    out.spreadIn = std::sqrt(out.varianceXIn2 + out.varianceYIn2);
    out.effectiveParticles = sumSquares > 0.0 ? 1.0 / sumSquares : 0.0;
    return out;
}

std::size_t ParticleFilter::checkSensors(double xIn, double yIn, double headingDeg,
                                         std::span<const DistanceReading> readings,
                                         double agreementSigmas, std::span<SensorCheck> out) const {
    std::size_t agreeing = 0;
    const std::size_t sensorCount = std::min(readings.size(), sensors_.size());
    for (std::size_t s = 0; s < sensorCount; ++s) {
        SensorCheck check;
        check.used = readings[s].valid;
        if (check.used) {
            const SensorRay ray = sensorRay(xIn, yIn, headingDeg, sensors_[s]);
            check.measuredIn = readings[s].distanceIn;
            check.expectedIn = map_.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
            check.sigmaIn = readingSigmaIn(config_.beam, readings[s]);
            check.residualSigmas = (check.measuredIn - check.expectedIn) / check.sigmaIn;
            check.agrees = std::fabs(check.residualSigmas) <= agreementSigmas;
            if (check.agrees) ++agreeing;
        }
        if (s < out.size()) out[s] = check;
    }
    return agreeing;
}

const std::vector<Particle>& ParticleFilter::particles() const { return particles_; }

const FieldMap& ParticleFilter::map() const { return map_; }

const std::vector<DistanceSensorMount>& ParticleFilter::sensors() const { return sensors_; }

const ParticleFilterConfig& ParticleFilter::config() const { return config_; }

void sampleParticles(std::span<const Particle> particles, std::span<Particle> out) {
    if (particles.empty() || out.empty()) return;
    double total = 0.0;
    for (const Particle& p : particles) total += p.weight;
    // the middle of each of out.size() equal shares of the weight
    const double step = total / static_cast<double>(out.size());
    double cumulative = particles[0].weight;
    std::size_t source = 0;
    for (std::size_t j = 0; j < out.size(); ++j) {
        const double target = (static_cast<double>(j) + 0.5) * step;
        while (target > cumulative && source + 1 < particles.size()) {
            ++source;
            cumulative += particles[source].weight;
        }
        out[j] = particles[source];
    }
}

} // namespace sapphirelib::localization
