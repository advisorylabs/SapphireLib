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
// and never above this once the fit is near. The beam model's noise is the sensor's spec, so a
// wider spread comes from readings that aren't walls at all, as around one wall in a room full of
// things, where they can be most of the readings, and a scale that wide would let them all in
constexpr double kMaxScale = 1.0;
// nor above this before then, while the configured mounts can put a wall's readings a few inches
// off. Near a wall that's several times their noise, and the tighter limit would throw them out
// before the fit could move the mount to them
constexpr double kMaxStartScale = 3.0;

// weak priors, so a parameter no reading pins down stays put instead of making the fit singular
constexpr double kCenterPriorIn = 200.0;
constexpr double kHeadingPriorRad = 2.0;
constexpr double kAlongPriorIn = 12.0;

// from walls on one or two sides, how tightly the fit holds the configured mounts' spacing, in
// inches: close enough to exact, loose enough to keep the system well conditioned
constexpr double kSpacingSigmaIn = 1e-3;
// the sensors' facings leave no spacing to hold when less than this share of "every offset grows
// alike" is left once a shift of the tracking center explains what it can
constexpr double kMinSpacingShare = 1e-3;
// from one wall or a corner, a wall this far out of square with the robot's heading, in degrees, is
// something else across the room the fit took for one (skewedWall()). A corner of the field is
// square to well within it, and the fit tells a wall's skew to about a degree
constexpr double kMaxWallSkewDeg = 3.0;

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
    // from walls on one or two sides only: a weight per sensor (spacingWeights()), and the
    // weighted sum of the along-beam offsets the fit holds them to. Empty when the walls pin them
    std::vector<double> spacing;
    double spacingTarget = 0.0;
    // walls the fit leaves out, a bit each as in SensorMountFit::walls: from one wall, ones that
    // came to rest on something else across the room (skewedWall())
    std::uint8_t ignoredWalls = 0;
};

// a reading as the parameters predict it, and how the prediction changes with each of them
struct Prediction {
    bool usable = false;
    double expectedIn = 0.0;
    int wall = kLeft;
    // |cos| of the angle between the beam and the wall's normal: 1 square on
    double cosIncidence = 0.0;
    // sin of that angle, signed: which way along the wall the beam leans
    double slant = 0.0;
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
        out.slant = uy;
        out.dX = -1.0 / ux;
        u = ux;
        n = nx;
        travelTurn = dy;
    } else {
        out.expectedIn = ty;
        out.wall = uy > 0.0 ? kFar : kNear;
        out.slant = ux;
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

// the along-beam offsets' weighted sum the spacing holds, 0 with none
double spacingSum(const Problem& problem, const std::vector<double>& p) {
    double sum = 0.0;
    for (std::size_t s = 0; s < problem.spacing.size(); ++s) {
        sum += problem.spacing[s] * p[alongIndex(s)];
    }
    return sum;
}

double priorCost(const Problem& problem, const std::vector<double>& p) {
    double total = 0.0;
    for (std::size_t k = 0; k < p.size(); ++k) {
        const double z = (p[k] - problem.priorMean[k]) / problem.priorSigma[k];
        total += 0.5 * z * z;
    }
    if (!problem.spacing.empty()) {
        const double z = (spacingSum(problem, p) - problem.spacingTarget) / kSpacingSigmaIn;
        total += 0.5 * z * z;
    }
    return total;
}

// which readings this iteration fits: a wall hit nearly square on, and not one it leaves out
void selectReadings(const Problem& problem, const std::vector<double>& p,
                    std::vector<char>& selected) {
    const double sinHeading = std::sin(p[kHeading]);
    const double cosHeading = std::cos(p[kHeading]);
    selected.assign(problem.readings.size(), 0);
    for (std::size_t j = 0; j < problem.readings.size(); ++j) {
        const Prediction prediction =
            predict(problem.readings[j], p, sinHeading, cosHeading, problem.box);
        selected[j] = prediction.usable && prediction.cosIncidence >= problem.cosMaxIncidence &&
                      (problem.ignoredWalls & (1u << prediction.wall)) == 0;
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

double robustScale(const std::vector<double>& errors, double maxScale) {
    std::vector<double> sizes;
    sizes.reserve(errors.size());
    for (const double z : errors) {
        if (std::isfinite(z)) sizes.push_back(std::fabs(z));
    }
    if (sizes.empty()) return 1.0;
    return std::clamp(kMadToSigma * median(std::move(sizes)), kMinScale, maxScale);
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
    if (!problem.spacing.empty()) {
        const double inverseVariance = 1.0 / (kSpacingSigmaIn * kSpacingSigmaIn);
        const double offBy = spacingSum(problem, p) - problem.spacingTarget;
        for (std::size_t a = 0; a < problem.spacing.size(); ++a) {
            gradient[alongIndex(a)] += problem.spacing[a] * offBy * inverseVariance;
            for (std::size_t b = 0; b < problem.spacing.size(); ++b) {
                hessian[alongIndex(a) * n + alongIndex(b)] +=
                    problem.spacing[a] * problem.spacing[b] * inverseVariance;
            }
        }
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
std::vector<double> refine(Problem problem, std::vector<double> p, double maxScale) {
    const std::size_t n = p.size();
    std::vector<char> selected;
    std::vector<double> hessian;
    std::vector<double> gradient;
    double damping = 1e-3;
    double scale = kInfinity;
    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        noiseFromPredictions(problem, p);
        selectReadings(problem, p, selected);
        scale = std::min(scale, robustScale(normalizedErrors(problem, p, selected), maxScale));
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

// refine() with room for readings the configured mounts put inches off, then again with the noise's
// scale held near the beam model's, once the mounts have moved to their walls
std::vector<double> fitFrom(const Problem& problem, std::vector<double> p) {
    return refine(problem, refine(problem, std::move(p), kMaxStartScale), kMaxScale);
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

// the readings a fit kept, sensor by sensor and wall by wall
struct Tally {
    std::vector<char> selected;
    double scale = 1.0;
    std::vector<std::array<std::size_t, 4>> wallReadings;
    std::vector<std::size_t> readings;
    std::vector<double> squaredError;
    std::size_t used = 0;
    // whether some sensor saw both walls of a pair, which the field's known width pins the robot
    // between
    bool pinnedX = false;
    bool pinnedY = false;
    // each wall's readings, every sensor's together: how many, and sums for a line through their
    // errors (in inches) by how they slanted (Prediction::slant), and for the tracking center's
    // mean distance from the wall
    struct WallSums {
        std::size_t count = 0;
        double slant = 0.0;
        double slantSquared = 0.0;
        double error = 0.0;
        double slantError = 0.0;
        double distance = 0.0;
    };
    std::array<WallSums, 4> walls{};
};

Tally tally(Problem& problem, const std::vector<double>& p, std::size_t minWallReadings) {
    Tally out;
    noiseFromPredictions(problem, p);
    selectReadings(problem, p, out.selected);
    const std::vector<double> errors = normalizedErrors(problem, p, out.selected);
    out.scale = robustScale(errors, kMaxScale);
    out.wallReadings.assign(problem.sensorCount, {0, 0, 0, 0});
    out.readings.assign(problem.sensorCount, 0);
    out.squaredError.assign(problem.sensorCount, 0.0);
    const double sinHeading = std::sin(p[kHeading]);
    const double cosHeading = std::cos(p[kHeading]);
    for (std::size_t j = 0; j < problem.readings.size(); ++j) {
        if (!out.selected[j] || !std::isfinite(errors[j]) ||
            weight(errors[j] / out.scale, kTukeyC) == 0.0) {
            continue;
        }
        const Reading& reading = problem.readings[j];
        const Prediction prediction = predict(reading, p, sinHeading, cosHeading, problem.box);
        const auto wall = static_cast<std::size_t>(prediction.wall);
        ++out.wallReadings[reading.sensor][wall];
        ++out.readings[reading.sensor];
        const double errorIn = reading.distanceIn - prediction.expectedIn;
        out.squaredError[reading.sensor] += errorIn * errorIn;
        ++out.used;
        Tally::WallSums& sums = out.walls[wall];
        ++sums.count;
        sums.slant += prediction.slant;
        sums.slantSquared += prediction.slant * prediction.slant;
        sums.error += errorIn;
        sums.slantError += prediction.slant * errorIn;
        sums.distance +=
            (prediction.expectedIn + p[alongIndex(reading.sensor)]) * prediction.cosIncidence;
    }
    for (const std::array<std::size_t, 4>& walls : out.wallReadings) {
        const auto seen = [&](int wall) {
            return walls[static_cast<std::size_t>(wall)] >= minWallReadings;
        };
        out.pinnedX = out.pinnedX || (seen(kLeft) && seen(kRight));
        out.pinnedY = out.pinnedY || (seen(kNear) && seen(kFar));
    }
    return out;
}

// From one wall, the walls square to it are free to sit anywhere, and one can come to rest on
// something flat across the room: a cabinet, a box. Few of those are square to the wall, and one
// that isn't can't agree with it on the robot's heading. Whatever heading the fit settles on, the
// readings off one of them then run long on one side of square on and short on the other: their
// error, by how far the beam leans along it, slopes by the robot's distance from it times the
// angle it's out of square. Real walls, square to each other, show no such slope. With two or more
// walls seen, the one most out of square, as a bit like SensorMountFit::walls, when that's over
// kMaxWallSkewDeg; 0 when none is, or only one wall was seen
std::uint8_t skewedWall(const Tally& kept, std::size_t minWallReadings) {
    int seen = 0;
    int worst = -1;
    double worstSkew = 0.0;
    for (int wall = kLeft; wall <= kFar; ++wall) {
        const Tally::WallSums& sums = kept.walls[static_cast<std::size_t>(wall)];
        if (sums.count < minWallReadings) continue;
        ++seen;
        const double n = static_cast<double>(sums.count);
        const double spread = n * sums.slantSquared - sums.slant * sums.slant;
        if (!(spread > 0.0) || !(sums.distance > 0.0)) continue;
        const double slope = (n * sums.slantError - sums.slant * sums.error) / spread;
        const double skewDeg = std::fabs(std::atan(slope / (sums.distance / n))) / kDegToRad;
        if (skewDeg > worstSkew) {
            worstSkew = skewDeg;
            worst = wall;
        }
    }
    if (seen < 2 || worst < 0 || !(worstSkew > kMaxWallSkewDeg)) return 0;
    return static_cast<std::uint8_t>(1u << worst);
}

// From walls on one or two sides, a robot nearer them with every sensor that much further out reads
// the same, so one length has to come from the configured mounts. Not all of "every offset grows
// alike", though: the tracking center sitting somewhere else changes opposite sensors' offsets
// opposite ways, which the readings do see, so it's what's left once that's taken out. With a
// sensor on each side it's simply their mean; with front, right and back it's the front and back
// sensors' span, the right one's following from the readings.
//
// Weights for the along-beam offsets of the sensors in the fit, summing to 1: the fit holds their
// weighted sum at the configured mounts'. Empty when nothing's left to hold, which is when those
// sensors face no more than two ways and not opposite ones: any growth is then the tracking center
// moving
std::vector<double> spacingWeights(const std::vector<char>& inFit,
                                   const std::vector<double>& sinFacing,
                                   const std::vector<double>& cosFacing) {
    const std::size_t count = inFit.size();
    std::size_t sensorsInFit = 0;
    // moving the tracking center right or forward shortens each offset by sin or cos of its facing.
    // Made orthonormal, so the weights are the ones less their projections onto them
    std::vector<std::vector<double>> shifts;
    for (const std::vector<double>* along : {&sinFacing, &cosFacing}) {
        std::vector<double> shift(count, 0.0);
        for (std::size_t s = 0; s < count; ++s) {
            if (inFit[s]) shift[s] = (*along)[s];
        }
        for (const std::vector<double>& other : shifts) {
            double dot = 0.0;
            for (std::size_t s = 0; s < count; ++s) dot += shift[s] * other[s];
            for (std::size_t s = 0; s < count; ++s) shift[s] -= dot * other[s];
        }
        double norm = 0.0;
        for (const double v : shift) norm += v * v;
        norm = std::sqrt(norm);
        if (norm < 1e-6) continue;
        for (double& v : shift) v /= norm;
        shifts.push_back(std::move(shift));
    }
    std::vector<double> weights(count, 0.0);
    for (std::size_t s = 0; s < count; ++s) {
        if (!inFit[s]) continue;
        weights[s] = 1.0;
        ++sensorsInFit;
    }
    for (const std::vector<double>& shift : shifts) {
        double dot = 0.0;
        for (std::size_t s = 0; s < count; ++s) dot += shift[s];
        for (std::size_t s = 0; s < count; ++s) weights[s] -= dot * shift[s];
    }
    // the weights are a projection of the ones, so their sum is also their squared length
    double sum = 0.0;
    for (const double w : weights) sum += w;
    if (sensorsInFit == 0 || !(sum >= kMinSpacingShare * static_cast<double>(sensorsInFit))) {
        return {};
    }
    for (double& w : weights) w /= sum;
    return weights;
}

} // namespace

const char* mountFitStatusText(MountFitStatus status) {
    switch (status) {
        case MountFitStatus::calibrated: return "calibrated";
        case MountFitStatus::noReadings: return "no readings";
        case MountFitStatus::tooFewReadings: return "too few readings of a wall";
        case MountFitStatus::noOppositeWalls: return "no two opposite walls seen";
        case MountFitStatus::noOppositeSensor: return "no opposite sensor saw a wall";
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

    // The fit holds the configured mounts' spacing from the start, until a sensor turns out to see
    // both walls of a pair. From walls on one or two sides, nothing in the readings stops every
    // sensor moving out and the robot toward the walls together, and far enough along that, the
    // walls it can't see come into reach of whatever else is around the room. Held over every
    // sensor with readings for now
    std::vector<char> withReadings(sensorCount, 0);
    for (std::size_t s = 0; s < sensorCount; ++s) withReadings[s] = perSensor[s] > 0;
    const auto holdSpacing = [&](const std::vector<char>& sensors) {
        problem.spacing = spacingWeights(sensors, sinFacing, cosFacing);
        problem.spacingTarget = spacingSum(problem, problem.priorMean);
    };
    holdSpacing(withReadings);

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
        std::vector<double> p = fitFrom(problem, startAt(problem, heading * kDegToRad));
        const double finalScore = score(problem, p);
        if (finalScore < bestScore) {
            bestScore = finalScore;
            best = std::move(p);
        }
    }

    // the readings the fit kept, sensor by sensor and wall by wall
    Tally kept = tally(problem, best, config.minWallReadings);
    bool pinned = kept.pinnedX || kept.pinnedY;
    if (pinned && !problem.spacing.empty()) {
        // A sensor saw both walls of a pair, so the field's width pins the spacing: let it go.
        // Unless that moves it further than a tape measure across the robot is ever off, when the
        // pair was really one wall and something across the room that happened to line up 140 in
        // from it
        Problem released = problem;
        released.spacing.clear();
        std::vector<double> p = refine(released, best, kMaxScale);
        if (std::fabs(spacingSum(problem, p) - problem.spacingTarget) <=
            config.maxSpacingChangeIn) {
            problem = std::move(released);
            best = std::move(p);
            kept = tally(problem, best, config.minWallReadings);
        } else {
            pinned = false;
        }
    }
    bool sawWall = false;
    if (!pinned) {
        // Walls on one or two sides only (one wall, or a corner): the spacing stays. Only real
        // walls, though: one that came to rest on something out of square across the room is
        // left out, and the fit goes again without it, until none is
        for (int pass = 0; pass < 3 && !problem.spacing.empty(); ++pass) {
            const std::uint8_t skewed = skewedWall(kept, config.minWallReadings);
            if (skewed == 0) break;
            problem.ignoredWalls |= skewed;
            best = refine(problem, std::move(best), kMaxScale);
            kept = tally(problem, best, config.minWallReadings);
        }
        // and held over just the sensors that saw a wall
        std::vector<char> inFit(sensorCount, 0);
        for (std::size_t s = 0; s < sensorCount; ++s) {
            inFit[s] = kept.readings[s] >= config.minReadings;
            sawWall = sawWall || inFit[s];
        }
        if (inFit != withReadings) {
            holdSpacing(inFit);
            if (!problem.spacing.empty()) best = refine(problem, std::move(best), kMaxScale);
            kept = tally(problem, best, config.minWallReadings);
        }
    }
    const bool spacingKept = !problem.spacing.empty();
    result.spacingFromMounts = spacingKept;
    result.readingsUsed = kept.used;

    std::vector<double> hessian;
    std::vector<double> gradient;
    normalEquations(problem, best, kept.selected, kept.scale, kTukeyC, hessian, gradient);

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
        fit.readings = kept.readings[s];
        fit.rmsErrorIn = fit.readings > 0
                             ? std::sqrt(kept.squaredError[s] / static_cast<double>(fit.readings))
                             : 0.0;
        bool onPinnedWall = false;
        for (int wall = kLeft; wall <= kFar; ++wall) {
            if (kept.wallReadings[s][static_cast<std::size_t>(wall)] < config.minWallReadings) {
                continue;
            }
            fit.walls |= static_cast<std::uint8_t>(1u << wall);
            const bool xWall = wall == kLeft || wall == kRight;
            if ((xWall && kept.pinnedX) || (!xWall && kept.pinnedY)) onPinnedWall = true;
        }

        if (perSensor[s] == 0) {
            fit.status = MountFitStatus::noReadings;
        } else if (fit.readings < config.minReadings) {
            fit.status = MountFitStatus::tooFewReadings;
        } else if (!spacingKept && !onPinnedWall) {
            // with the spacing held, every wall is pinned
            fit.status =
                pinned ? MountFitStatus::noOppositeWalls : MountFitStatus::noOppositeSensor;
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
        result.error = pinned || spacingKept ? "no sensor could be calibrated"
                       : sawWall             ? "no two opposite sensors saw a wall"
                                             : "no sensor saw a wall";
    }
    return result;
}

} // namespace sapphirelib::localization
