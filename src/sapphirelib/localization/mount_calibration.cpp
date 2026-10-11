#include "sapphirelib/localization/mount_calibration.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace sapphirelib::localization {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Tukey's biweight, which gives a reading that matches no wall (something in the way) no weight at
// all. A weight that never reaches 0 (Huber's) lets a wall's worth of blocked readings drag the fit
// to a compromise that fits nothing
constexpr double kTukeyC = 4.685;
constexpr int kMaxIterations = 60;
constexpr int kMaxDampingTries = 12;
// a reading that was in the fit but misses the walls entirely from a trial step costs this much
constexpr double kMissedU = 10.0;

// the errors' spread, in units of each reading's beam-model noise: the median absolute error
// scaled to a standard deviation, and never below this, so near-perfect readings don't make every
// reading an outlier
constexpr double kMadToSigma = 1.4826;
constexpr double kMinScale = 0.02;

// weak priors, so a parameter no reading pins down stays put instead of making the fit singular
constexpr double kCenterPriorIn = 200.0;
constexpr double kHeadingPriorRad = 2.0;
constexpr double kAlongPriorIn = 12.0;

// the start's heading is searched on a grid before the fit refines it. Half a turn covers every
// start, since the walls look the same from half a turn around
constexpr double kHeadingGridStepDeg = 2.0;
constexpr std::size_t kStartsToRefine = 2;
constexpr double kDistinctStartsDeg = 10.0;
// how close square-on readings' guesses at the center must be to count as agreeing, in inches:
// wide enough for a few inches of mount error between opposite sensors
constexpr double kStartClusterIn = 6.0;

// a step this small (in inches, the heading's as its arc 70 in out) ends the fit
constexpr double kConvergedStepIn = 1e-5;
constexpr double kHeadingArmIn = 70.0;

enum Wall : int { kLeft = 0, kRight = 1, kNear = 2, kFar = 3 };

struct Box {
    double minX;
    double maxX;
    double minY;
    double maxY;
};

// one reading, with everything that doesn't depend on the fit's parameters worked out once
struct Reading {
    std::size_t sensor;
    // how far the tracking center had moved since the first reading, in a frame turned with the
    // robot's heading at the first reading, so the start's field heading turns it onto the field
    double vx;
    double vy;
    // the beam's heading relative to the robot's at the first reading: turned since, plus facing
    double sinBeam;
    double cosBeam;
    double distanceIn;
    double sigmaIn;
};

// the parameters: the tracking center's field position and heading at the first reading, then each
// sensor's offset along its beam and sideways from it (right of the beam positive)
constexpr std::size_t kX = 0;
constexpr std::size_t kY = 1;
constexpr std::size_t kHeading = 2;
constexpr std::size_t kShared = 3;
constexpr std::size_t alongIndex(std::size_t sensor) { return kShared + 2 * sensor; }
constexpr std::size_t lateralIndex(std::size_t sensor) { return kShared + 2 * sensor + 1; }

struct Problem {
    std::vector<Reading> readings;
    std::size_t sensorCount = 0;
    Box box{};
    BeamModel beam;
    double cosMaxIncidence = 1.0;
    // each parameter's prior: mean and standard deviation
    std::vector<double> priorMean;
    std::vector<double> priorSigma;
};

// a reading as the parameters predict it, and how the prediction changes with each of them
struct Prediction {
    bool usable = false;
    double expectedIn = 0.0;
    int wall = kLeft;
    // |cos| of the angle between the beam and the wall's normal: 1 square on
    double cosIncidence = 0.0;
    double dX = 0.0;
    double dY = 0.0;
    double dHeading = 0.0;
    double dAlong = 0.0;
    double dLateral = 0.0;
};

Prediction predict(const Reading& reading, const std::vector<double>& p, double sinHeading,
                   double cosHeading, const Box& box) {
    Prediction out;
    const double along = p[alongIndex(reading.sensor)];
    const double lateral = p[lateralIndex(reading.sensor)];
    // the center's travel turned onto the field. Turning the heading turns it too: d(dx)/dH = dy,
    // d(dy)/dH = -dx
    const double dx = reading.vx * cosHeading + reading.vy * sinHeading;
    const double dy = -reading.vx * sinHeading + reading.vy * cosHeading;
    // the beam on the field, (sin, cos) of its heading like odom::Pose, and n, its right
    const double ux = reading.sinBeam * cosHeading + reading.cosBeam * sinHeading;
    const double uy = reading.cosBeam * cosHeading - reading.sinBeam * sinHeading;
    const double nx = uy;
    const double ny = -ux;
    const double faceX = p[kX] + dx + along * ux + lateral * nx;
    const double faceY = p[kY] + dy + along * uy + lateral * ny;
    if (!(faceX > box.minX && faceX < box.maxX && faceY > box.minY && faceY < box.maxY)) {
        return out;
    }
    const double tx = ux > 0.0   ? (box.maxX - faceX) / ux
                      : ux < 0.0 ? (box.minX - faceX) / ux
                                 : kInfinity;
    const double ty = uy > 0.0   ? (box.maxY - faceY) / uy
                      : uy < 0.0 ? (box.minY - faceY) / uy
                                 : kInfinity;
    // u and n along the wall's normal, and how the center's travel along it turns with the heading
    double u = 0.0;
    double n = 0.0;
    double travelTurn = 0.0;
    if (tx <= ty) {
        out.expectedIn = tx;
        out.wall = ux > 0.0 ? kRight : kLeft;
        out.dX = -1.0 / ux;
        u = ux;
        n = nx;
        travelTurn = dy;
    } else {
        out.expectedIn = ty;
        out.wall = uy > 0.0 ? kFar : kNear;
        out.dY = -1.0 / uy;
        u = uy;
        n = ny;
        travelTurn = -dx;
    }
    out.usable = std::isfinite(out.expectedIn);
    out.cosIncidence = std::fabs(u);
    // expected = (wall - face) / u, with the face moving along u by `along` and along n by
    // `lateral`; turning moves u by n and n by -u
    out.dAlong = -1.0;
    out.dLateral = -n / u;
    out.dHeading = -travelTurn / u + lateral - (along + out.expectedIn) * n / u;
    return out;
}

// Tukey's biweight with cutoff c: the cost of an error u (in units of the noise), and its weight
double rho(double u, double c) {
    if (std::fabs(u) >= c) return c * c / 6.0;
    const double q = 1.0 - (u / c) * (u / c);
    return c * c / 6.0 * (1.0 - q * q * q);
}

double weight(double u, double c) {
    if (std::fabs(u) >= c) return 0.0;
    const double q = 1.0 - (u / c) * (u / c);
    return q * q;
}

double median(std::vector<double> values) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    const std::size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid),
                     values.end());
    const double upper = values[mid];
    if (values.size() % 2 == 1) return upper;
    const double lower =
        *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
    return 0.5 * (lower + upper);
}

double priorCost(const Problem& problem, const std::vector<double>& p) {
    double total = 0.0;
    for (std::size_t k = 0; k < p.size(); ++k) {
        const double z = (p[k] - problem.priorMean[k]) / problem.priorSigma[k];
        total += 0.5 * z * z;
    }
    return total;
}

// which readings this iteration fits: a wall hit nearly square on
void selectReadings(const Problem& problem, const std::vector<double>& p,
                    std::vector<char>& selected) {
    const double sinHeading = std::sin(p[kHeading]);
    const double cosHeading = std::cos(p[kHeading]);
    selected.assign(problem.readings.size(), 0);
    for (std::size_t j = 0; j < problem.readings.size(); ++j) {
        const Prediction prediction =
            predict(problem.readings[j], p, sinHeading, cosHeading, problem.box);
        selected[j] = prediction.usable && prediction.cosIncidence >= problem.cosMaxIncidence;
    }
}

// each selected reading's error in units of its noise, NaN where it misses the walls
std::vector<double> normalizedErrors(const Problem& problem, const std::vector<double>& p,
                                     const std::vector<char>& selected) {
    const double sinHeading = std::sin(p[kHeading]);
    const double cosHeading = std::cos(p[kHeading]);
    std::vector<double> errors(problem.readings.size(), std::numeric_limits<double>::quiet_NaN());
    for (std::size_t j = 0; j < problem.readings.size(); ++j) {
        if (!selected[j]) continue;
        const Reading& reading = problem.readings[j];
        const Prediction prediction = predict(reading, p, sinHeading, cosHeading, problem.box);
        if (prediction.usable) {
            errors[j] = (reading.distanceIn - prediction.expectedIn) / reading.sigmaIn;
        }
    }
    return errors;
}

double robustScale(const std::vector<double>& errors) {
    std::vector<double> sizes;
    sizes.reserve(errors.size());
    for (const double z : errors) {
        if (std::isfinite(z)) sizes.push_back(std::fabs(z));
    }
    if (sizes.empty()) return 1.0;
    return std::max(kMinScale, kMadToSigma * median(std::move(sizes)));
}

double cost(const Problem& problem, const std::vector<double>& p, const std::vector<char>& selected,
            double scale, double cutoff) {
    const std::vector<double> errors = normalizedErrors(problem, p, selected);
    double total = priorCost(problem, p);
    for (std::size_t j = 0; j < errors.size(); ++j) {
        if (!selected[j]) continue;
        total += std::isfinite(errors[j]) ? rho(errors[j] / scale, cutoff) : rho(kMissedU, cutoff);
    }
    return total;
}

// the Gauss-Newton system for one iteration: hessian (row-major, n by n) and gradient
void normalEquations(const Problem& problem, const std::vector<double>& p,
                     const std::vector<char>& selected, double scale, double cutoff,
                     std::vector<double>& hessian, std::vector<double>& gradient) {
    const std::size_t n = p.size();
    hessian.assign(n * n, 0.0);
    gradient.assign(n, 0.0);
    const double sinHeading = std::sin(p[kHeading]);
    const double cosHeading = std::cos(p[kHeading]);
    for (std::size_t j = 0; j < problem.readings.size(); ++j) {
        if (!selected[j]) continue;
        const Reading& reading = problem.readings[j];
        const Prediction prediction = predict(reading, p, sinHeading, cosHeading, problem.box);
        if (!prediction.usable) continue;
        const double z = (reading.distanceIn - prediction.expectedIn) / reading.sigmaIn;
        const double w = weight(z / scale, cutoff) / (scale * scale);
        if (w == 0.0) continue;
        // d(error)/d(parameter): the prediction's slopes, negated, over the reading's noise
        const std::size_t index[5] = {kX, kY, kHeading, alongIndex(reading.sensor),
                                      lateralIndex(reading.sensor)};
        const double slope[5] = {-prediction.dX / reading.sigmaIn, -prediction.dY / reading.sigmaIn,
                                 -prediction.dHeading / reading.sigmaIn,
                                 -prediction.dAlong / reading.sigmaIn,
                                 -prediction.dLateral / reading.sigmaIn};
        for (int a = 0; a < 5; ++a) {
            gradient[index[a]] += w * slope[a] * z;
            for (int b = 0; b < 5; ++b) {
                hessian[index[a] * n + index[b]] += w * slope[a] * slope[b];
            }
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        const double inverseVariance = 1.0 / (problem.priorSigma[k] * problem.priorSigma[k]);
        hessian[k * n + k] += inverseVariance;
        gradient[k] += (p[k] - problem.priorMean[k]) * inverseVariance;
    }
}

// solves a x = b for a symmetric positive definite a (row-major, n by n), leaving x in b. False if
// a isn't positive definite
bool choleskySolve(std::vector<double> a, std::size_t n, std::vector<double>& b) {
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j <= i; ++j) {
            double sum = a[i * n + j];
            for (std::size_t k = 0; k < j; ++k) sum -= a[i * n + k] * a[j * n + k];
            if (i == j) {
                if (!(sum > 0.0)) return false;
                a[i * n + i] = std::sqrt(sum);
            } else {
                a[i * n + j] = sum / a[j * n + j];
            }
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        double sum = b[i];
        for (std::size_t k = 0; k < i; ++k) sum -= a[i * n + k] * b[k];
        b[i] = sum / a[i * n + i];
    }
    for (std::size_t i = n; i-- > 0;) {
        double sum = b[i];
        for (std::size_t k = i + 1; k < n; ++k) sum -= a[k * n + i] * b[k];
        b[i] = sum / a[i * n + i];
    }
    return true;
}

// one coordinate of the center, from where each square-on reading of the two walls across that axis
// puts it. Readings off something in the way can be most of one wall's, so only the densest cluster
// counts; within it, each wall's median, averaged, so the two sides' mount errors cancel
double startCoordinate(const std::vector<double>& fromOneWall,
                       const std::vector<double>& fromOtherWall, double fallback) {
    std::vector<double> all = fromOneWall;
    all.insert(all.end(), fromOtherWall.begin(), fromOtherWall.end());
    if (all.empty()) return fallback;
    std::sort(all.begin(), all.end());
    std::size_t bestStart = 0;
    std::size_t bestCount = 0;
    std::size_t end = 0;
    for (std::size_t start = 0; start < all.size(); ++start) {
        while (end < all.size() && all[end] - all[start] <= kStartClusterIn) ++end;
        if (end - start > bestCount) {
            bestCount = end - start;
            bestStart = start;
        }
    }
    const double low = all[bestStart];
    const double high = all[bestStart + bestCount - 1];
    const auto inCluster = [low, high](const std::vector<double>& values) {
        std::vector<double> kept;
        for (const double v : values) {
            if (v >= low && v <= high) kept.push_back(v);
        }
        return kept;
    };
    std::vector<double> one = inCluster(fromOneWall);
    std::vector<double> other = inCluster(fromOtherWall);
    if (!one.empty() && !other.empty()) {
        return 0.5 * (median(std::move(one)) + median(std::move(other)));
    }
    return median(one.empty() ? std::move(other) : std::move(one));
}

// a start for the fit at one heading: the configured mounts, and the center where the square-on
// readings put it
std::vector<double> startAt(const Problem& problem, double headingRad) {
    std::vector<double> p = problem.priorMean;
    p[kHeading] = headingRad;
    const double sinHeading = std::sin(headingRad);
    const double cosHeading = std::cos(headingRad);
    std::vector<double> fromRight;
    std::vector<double> fromLeft;
    std::vector<double> fromFar;
    std::vector<double> fromNear;
    for (const Reading& reading : problem.readings) {
        const double along = p[alongIndex(reading.sensor)];
        const double lateral = p[lateralIndex(reading.sensor)];
        const double dx = reading.vx * cosHeading + reading.vy * sinHeading;
        const double dy = -reading.vx * sinHeading + reading.vy * cosHeading;
        const double ux = reading.sinBeam * cosHeading + reading.cosBeam * sinHeading;
        const double uy = reading.cosBeam * cosHeading - reading.sinBeam * sinHeading;
        // where the center must be for this reading to reach the wall it points at
        if (std::fabs(ux) >= problem.cosMaxIncidence) {
            const double wall = ux > 0.0 ? problem.box.maxX : problem.box.minX;
            const double x = wall - dx - along * ux - lateral * uy - reading.distanceIn * ux;
            (ux > 0.0 ? fromRight : fromLeft).push_back(x);
        } else if (std::fabs(uy) >= problem.cosMaxIncidence) {
            const double wall = uy > 0.0 ? problem.box.maxY : problem.box.minY;
            const double y = wall - dy - along * uy + lateral * ux - reading.distanceIn * uy;
            (uy > 0.0 ? fromFar : fromNear).push_back(y);
        }
    }
    p[kX] = startCoordinate(fromRight, fromLeft, 0.5 * (problem.box.minX + problem.box.maxX));
    p[kY] = startCoordinate(fromFar, fromNear, 0.5 * (problem.box.minY + problem.box.maxY));
    return p;
}

// how well parameters explain every reading, slanted ones too, comparable between starts: Tukey's
// cost at the beam model's own noise, a reading that misses the walls costing the most
double score(const Problem& problem, const std::vector<double>& p) {
    const std::vector<char> all(problem.readings.size(), 1);
    const std::vector<double> errors = normalizedErrors(problem, p, all);
    double total = 0.0;
    for (const double z : errors)
        total += std::isfinite(z) ? rho(z, kTukeyC) : rho(kMissedU, kTukeyC);
    return total;
}

// each reading's noise, from the distance the parameters predict for it. From the reading itself,
// a reading that happens to be short would count as less noisy and weigh more, which biases every
// offset outward: about 2 sigma^2 / distance, 0.08 in at 66 in with 2.5% noise
void noiseFromPredictions(Problem& problem, const std::vector<double>& p) {
    const double sinHeading = std::sin(p[kHeading]);
    const double cosHeading = std::cos(p[kHeading]);
    for (Reading& reading : problem.readings) {
        const Prediction prediction = predict(reading, p, sinHeading, cosHeading, problem.box);
        const double distance = prediction.usable ? prediction.expectedIn : reading.distanceIn;
        reading.sigmaIn = readingSigmaIn(
            problem.beam, DistanceReading{.distanceIn = std::max(distance, 0.0), .valid = true});
    }
}

// Levenberg-Marquardt on the robust cost, reweighting each iteration. The noise's scale is measured
// from the start's errors and may only shrink: allowed to grow, a fit drifting toward a compromise
// between two explanations (the wall, and something flat in front of it) widens the scale, which
// lets more of the readings it doesn't explain back in, which drags it further
std::vector<double> refine(Problem problem, std::vector<double> p) {
    const std::size_t n = p.size();
    std::vector<char> selected;
    std::vector<double> hessian;
    std::vector<double> gradient;
    double damping = 1e-3;
    double scale = kInfinity;
    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        noiseFromPredictions(problem, p);
        selectReadings(problem, p, selected);
        scale = std::min(scale, robustScale(normalizedErrors(problem, p, selected)));
        normalEquations(problem, p, selected, scale, kTukeyC, hessian, gradient);
        const double current = cost(problem, p, selected, scale, kTukeyC);

        bool accepted = false;
        double stepIn = 0.0;
        for (int attempt = 0; attempt < kMaxDampingTries; ++attempt) {
            std::vector<double> damped = hessian;
            for (std::size_t k = 0; k < n; ++k) damped[k * n + k] *= 1.0 + damping;
            std::vector<double> step(n);
            for (std::size_t k = 0; k < n; ++k) step[k] = -gradient[k];
            if (!choleskySolve(std::move(damped), n, step)) {
                damping *= 10.0;
                continue;
            }
            std::vector<double> trial = p;
            for (std::size_t k = 0; k < n; ++k) trial[k] += step[k];
            if (cost(problem, trial, selected, scale, kTukeyC) < current) {
                stepIn = 0.0;
                for (std::size_t k = 0; k < n; ++k) {
                    const double size = std::fabs(step[k]) * (k == kHeading ? kHeadingArmIn : 1.0);
                    stepIn = std::max(stepIn, size);
                }
                p = std::move(trial);
                damping = std::max(damping / 10.0, 1e-9);
                accepted = true;
                break;
            }
            damping *= 10.0;
        }
        if (!accepted || stepIn < kConvergedStepIn) break;
    }
    return p;
}

double wrapDegrees360(double degrees) {
    const double wrapped = std::fmod(degrees, 360.0);
    return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

// the circular distance between two headings that look the same half a turn apart, in degrees
double headingGap(double a, double b) {
    const double gap = std::fmod(std::fabs(a - b), 180.0);
    return std::min(gap, 180.0 - gap);
}

} // namespace

const char* mountFitStatusText(MountFitStatus status) {
    switch (status) {
        case MountFitStatus::calibrated: return "calibrated";
        case MountFitStatus::noReadings: return "no readings";
        case MountFitStatus::tooFewReadings: return "too few readings of a wall";
        case MountFitStatus::noOppositeWalls: return "no two opposite walls seen";
        case MountFitStatus::uncertain: return "too uncertain";
        case MountFitStatus::implausible: return "moved too far, view blocked?";
    }
    return "?";
}

MountCalibrationResult fitSensorMounts(std::span<const MountSample> samples,
                                       std::span<const DistanceSensorMount> mounts,
                                       const FieldMap& map, const BeamModel& beam,
                                       const MountCalibrationConfig& config) {
    MountCalibrationResult result;
    const std::size_t sensorCount = mounts.size();
    result.sensors.resize(sensorCount);
    for (std::size_t s = 0; s < sensorCount; ++s) result.sensors[s].mount = mounts[s];

    Problem problem;
    problem.sensorCount = sensorCount;
    problem.box = Box{map.minXIn(), map.maxXIn(), map.minYIn(), map.maxYIn()};
    problem.beam = beam;
    problem.cosMaxIncidence = std::cos(config.maxIncidenceDeg * kDegToRad);

    // the configured mounts, as offsets along and sideways from each beam
    const std::size_t parameterCount = kShared + 2 * sensorCount;
    problem.priorMean.assign(parameterCount, 0.0);
    problem.priorSigma.assign(parameterCount, 1.0);
    problem.priorSigma[kX] = kCenterPriorIn;
    problem.priorSigma[kY] = kCenterPriorIn;
    problem.priorSigma[kHeading] = kHeadingPriorRad;
    std::vector<double> sinFacing(sensorCount);
    std::vector<double> cosFacing(sensorCount);
    for (std::size_t s = 0; s < sensorCount; ++s) {
        const double facing = mounts[s].facingDeg * kDegToRad;
        sinFacing[s] = std::sin(facing);
        cosFacing[s] = std::cos(facing);
        problem.priorMean[alongIndex(s)] =
            mounts[s].forwardIn * cosFacing[s] + mounts[s].rightIn * sinFacing[s];
        problem.priorMean[lateralIndex(s)] =
            -mounts[s].forwardIn * sinFacing[s] + mounts[s].rightIn * cosFacing[s];
        problem.priorSigma[alongIndex(s)] = kAlongPriorIn;
        problem.priorSigma[lateralIndex(s)] = std::max(config.lateralPriorIn, 1e-3);
    }

    // every reading relative to the first: how far the center moved, turned into the robot's
    // frame at that moment, and how far the robot has turned
    const auto usable = [sensorCount](const MountSample& sample) {
        return sample.sensor < sensorCount && std::isfinite(sample.distanceIn) &&
               sample.distanceIn > 0.0 && std::isfinite(sample.xIn) && std::isfinite(sample.yIn) &&
               std::isfinite(sample.headingDeg);
    };
    const auto first = std::find_if(samples.begin(), samples.end(), usable);
    std::vector<std::size_t> perSensor(sensorCount, 0);
    if (first != samples.end()) {
        // turning the raw frame back by the first heading makes the robot face +y at the start
        const double sinBack = std::sin(-first->headingDeg * kDegToRad);
        const double cosBack = std::cos(-first->headingDeg * kDegToRad);
        for (const MountSample& sample : samples) {
            if (!usable(sample)) continue;
            const double rawX = sample.xIn - first->xIn;
            const double rawY = sample.yIn - first->yIn;
            const double turned = (sample.headingDeg - first->headingDeg) * kDegToRad;
            problem.readings.push_back(Reading{
                .sensor = sample.sensor,
                .vx = rawX * cosBack + rawY * sinBack,
                .vy = -rawX * sinBack + rawY * cosBack,
                .sinBeam = std::sin(turned) * cosFacing[sample.sensor] +
                           std::cos(turned) * sinFacing[sample.sensor],
                .cosBeam = std::cos(turned) * cosFacing[sample.sensor] -
                           std::sin(turned) * sinFacing[sample.sensor],
                .distanceIn = sample.distanceIn,
                .sigmaIn = readingSigmaIn(
                    beam, DistanceReading{.distanceIn = sample.distanceIn, .valid = true}),
            });
            ++perSensor[sample.sensor];
        }
    }
    result.readings = problem.readings.size();
    if (sensorCount == 0) {
        result.error = "no sensors";
        return result;
    }
    if (problem.readings.empty()) {
        result.error = "no readings";
        return result;
    }

    // search the start's heading on a grid, then refine the best few starts and keep the best
    std::vector<std::pair<double, double>> starts; // (score, heading in degrees)
    for (double heading = -90.0; heading < 90.0; heading += kHeadingGridStepDeg) {
        starts.emplace_back(score(problem, startAt(problem, heading * kDegToRad)), heading);
    }
    std::sort(starts.begin(), starts.end());
    std::vector<double> best;
    double bestScore = kInfinity;
    std::vector<double> refined;
    for (const auto& [startScore, heading] : starts) {
        if (refined.size() >= kStartsToRefine) break;
        bool distinct = true;
        for (const double other : refined) {
            if (headingGap(heading, other) < kDistinctStartsDeg) distinct = false;
        }
        if (!distinct) continue;
        refined.push_back(heading);
        std::vector<double> p = refine(problem, startAt(problem, heading * kDegToRad));
        const double finalScore = score(problem, p);
        if (finalScore < bestScore) {
            bestScore = finalScore;
            best = std::move(p);
        }
    }

    // the readings the fit kept, sensor by sensor and wall by wall
    noiseFromPredictions(problem, best);
    std::vector<char> selected;
    selectReadings(problem, best, selected);
    const std::vector<double> errors = normalizedErrors(problem, best, selected);
    const double scale = robustScale(errors);
    const double sinHeading = std::sin(best[kHeading]);
    const double cosHeading = std::cos(best[kHeading]);
    std::vector<std::array<std::size_t, 4>> wallReadings(sensorCount, {0, 0, 0, 0});
    std::vector<double> squaredError(sensorCount, 0.0);
    for (std::size_t j = 0; j < problem.readings.size(); ++j) {
        if (!selected[j] || !std::isfinite(errors[j]) ||
            weight(errors[j] / scale, kTukeyC) == 0.0) {
            continue;
        }
        const Reading& reading = problem.readings[j];
        const Prediction prediction = predict(reading, best, sinHeading, cosHeading, problem.box);
        ++wallReadings[reading.sensor][static_cast<std::size_t>(prediction.wall)];
        ++result.sensors[reading.sensor].readings;
        const double errorIn = reading.distanceIn - prediction.expectedIn;
        squaredError[reading.sensor] += errorIn * errorIn;
        ++result.readingsUsed;
    }

    // a pair of opposite walls pins the robot's distance to both, since the field's width is known
    bool pinnedX = false;
    bool pinnedY = false;
    for (std::size_t s = 0; s < sensorCount; ++s) {
        const auto seen = [&](int wall) {
            return wallReadings[s][static_cast<std::size_t>(wall)] >= config.minWallReadings;
        };
        pinnedX = pinnedX || (seen(kLeft) && seen(kRight));
        pinnedY = pinnedY || (seen(kNear) && seen(kFar));
    }

    std::vector<double> hessian;
    std::vector<double> gradient;
    normalEquations(problem, best, selected, scale, kTukeyC, hessian, gradient);

    // the offsets' standard errors: their diagonal of the inverse hessian
    const auto stderrOf = [&](std::size_t index) {
        std::vector<double> unit(parameterCount, 0.0);
        unit[index] = 1.0;
        return choleskySolve(hessian, parameterCount, unit) ? std::sqrt(std::max(0.0, unit[index]))
                                                            : kInfinity;
    };

    for (std::size_t s = 0; s < sensorCount; ++s) {
        SensorMountFit& fit = result.sensors[s];
        const double configuredLateral = problem.priorMean[lateralIndex(s)];
        const double along = best[alongIndex(s)];
        fit.alongIn = along;
        fit.alongChangeIn = along - problem.priorMean[alongIndex(s)];
        fit.alongStderrIn = stderrOf(alongIndex(s));
        fit.lateralStderrIn = stderrOf(lateralIndex(s));
        // sideways only when the readings pinned it down. Shifting every sensor sideways the same
        // way looks much like the robot facing a little differently, so from the middle of the
        // field it mostly isn't, and the configured one is the better guess
        fit.lateralFitted = fit.lateralStderrIn <= config.maxStderrIn;
        fit.lateralIn = fit.lateralFitted ? best[lateralIndex(s)] : configuredLateral;
        fit.lateralChangeIn = fit.lateralIn - configuredLateral;
        fit.rmsErrorIn =
            fit.readings > 0 ? std::sqrt(squaredError[s] / static_cast<double>(fit.readings)) : 0.0;
        bool onPinnedWall = false;
        for (int wall = kLeft; wall <= kFar; ++wall) {
            if (wallReadings[s][static_cast<std::size_t>(wall)] < config.minWallReadings) continue;
            fit.walls |= static_cast<std::uint8_t>(1u << wall);
            const bool xWall = wall == kLeft || wall == kRight;
            if ((xWall && pinnedX) || (!xWall && pinnedY)) onPinnedWall = true;
        }

        if (perSensor[s] == 0) {
            fit.status = MountFitStatus::noReadings;
        } else if (fit.readings < config.minReadings) {
            fit.status = MountFitStatus::tooFewReadings;
        } else if (!onPinnedWall) {
            fit.status = MountFitStatus::noOppositeWalls;
        } else if (!(fit.alongStderrIn <= config.maxStderrIn)) {
            fit.status = MountFitStatus::uncertain;
        } else if (std::fabs(fit.alongChangeIn) > config.maxChangeIn ||
                   std::fabs(fit.lateralChangeIn) > config.maxChangeIn) {
            fit.status = MountFitStatus::implausible;
        } else {
            fit.status = MountFitStatus::calibrated;
            fit.mount.forwardIn = along * cosFacing[s] - fit.lateralIn * sinFacing[s];
            fit.mount.rightIn = along * sinFacing[s] + fit.lateralIn * cosFacing[s];
            result.ok = true;
        }
        if (fit.status != MountFitStatus::calibrated) {
            // nothing changes for a sensor left as it was
            fit.lateralFitted = false;
            fit.lateralIn = configuredLateral;
            fit.lateralChangeIn = 0.0;
        }
    }

    result.start = odom::Pose{
        .xIn = best[kX], .yIn = best[kY], .headingDeg = wrapDegrees360(best[kHeading] / kDegToRad)};
    if (!result.ok) {
        result.error = (pinnedX || pinnedY) ? "no sensor could be calibrated"
                                            : "no sensor saw two opposite walls";
    }
    return result;
}

} // namespace sapphirelib::localization
