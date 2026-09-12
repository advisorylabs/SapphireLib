#include "sapphirelib/tuning/characterization_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace sapphirelib::tuning {

namespace {

constexpr double kLn2 = 0.69314718055994530942;

/// Fewer rows than this can't meaningfully constrain three parameters
/// against real sensor noise.
constexpr int kMinFitRows = 20;

/// Delays longer than this aren't latency, they're a run that didn't
/// measure what it meant to (a stalled chassis that eventually broke free).
constexpr double kMaxPlausibleDelayS = 0.5;

double signOf(double value) { return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0); }

/// Central-difference velocity at every index with a full window either
/// side; std::nullopt elsewhere. Central differences add no lag — a lagging
/// estimate would read as extra delay and bias the fit.
std::vector<std::optional<double>> velocities(const CharacterizationRun& run, int halfWindow) {
    const int count = static_cast<int>(run.size());
    std::vector<std::optional<double>> result(run.size());
    for (int i = halfWindow; i < count - halfWindow; ++i) {
        const double dtS = (run[i + halfWindow].timeMs - run[i - halfWindow].timeMs) / 1000.0;
        if (dtS > 0.0) {
            result[i] = (run[i + halfWindow].position - run[i - halfWindow].position) / dtS;
        }
    }
    return result;
}

/// Solves the 3×3 system `a`·x = `b` by Gaussian elimination with partial
/// pivoting. False if a pivot is too small relative to the matrix's scale to
/// trust — i.e. the data didn't constrain every parameter.
bool solve3(std::array<std::array<double, 3>, 3> a, std::array<double, 3> b,
            std::array<double, 3>& x) {
    double scale = 0.0;
    for (const auto& row : a) {
        for (double value : row) scale = std::fmax(scale, std::fabs(value));
    }
    if (!(scale > 0.0)) return false;

    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int r = col + 1; r < 3; ++r) {
            if (std::fabs(a[r][col]) > std::fabs(a[pivot][col])) pivot = r;
        }
        if (std::fabs(a[pivot][col]) < scale * 1e-12) return false;
        std::swap(a[pivot], a[col]);
        std::swap(b[pivot], b[col]);

        for (int r = col + 1; r < 3; ++r) {
            const double factor = a[r][col] / a[col][col];
            for (int c = col; c < 3; ++c) a[r][c] -= factor * a[col][c];
            b[r] -= factor * b[col];
        }
    }

    for (int r = 2; r >= 0; --r) {
        double sum = b[r];
        for (int c = r + 1; c < 3; ++c) sum -= a[r][c] * x[c];
        x[r] = sum / a[r][r];
    }
    return true;
}

struct FitRow {
    double velocity;     // v[k]
    double nextVelocity; // v[k+m]
    double volts;        // mean command over the interval, delay-shifted
    double intervalS;    // T
};

} // namespace

FeedforwardFit fitFeedforward(const std::vector<CharacterizationRun>& runs, double minSpeed,
                              int halfWindow, int delayTicks, double minRSquared) {
    FeedforwardFit fit;
    const int w = std::max(1, halfWindow);
    const int shift = std::max(0, delayTicks);
    // One past the combined width of two windows, so v[k] and v[k+m] are
    // built from disjoint samples.
    const int m = 2 * w + 1;

    std::vector<FitRow> rows;
    for (const CharacterizationRun& run : runs) {
        const std::vector<std::optional<double>> v = velocities(run, w);
        const int count = static_cast<int>(run.size());
        for (int k = std::max(w, shift); k + m < count; ++k) {
            if (!v[k] || !v[k + m]) continue;
            if (std::fabs(*v[k]) < minSpeed) continue;
            const double intervalS = (run[k + m].timeMs - run[k].timeMs) / 1000.0;
            if (!(intervalS > 0.0)) continue;

            double voltsSum = 0.0;
            for (int j = k; j < k + m; ++j) voltsSum += run[j - shift].volts;
            rows.push_back(FitRow{*v[k], *v[k + m], voltsSum / m, intervalS});
        }
    }
    fit.samplesUsed = static_cast<int>(rows.size());
    if (fit.samplesUsed < kMinFitRows) return fit;

    // Normal equations for v[k+m] = α·v[k] + β·u + γ·sign(v[k]).
    std::array<std::array<double, 3>, 3> normal{};
    std::array<double, 3> rhs{};
    double intervalSum = 0.0;
    for (const FitRow& row : rows) {
        const std::array<double, 3> f{row.velocity, row.volts, signOf(row.velocity)};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) normal[r][c] += f[r] * f[c];
            rhs[r] += f[r] * row.nextVelocity;
        }
        intervalSum += row.intervalS;
    }

    std::array<double, 3> x{};
    if (!solve3(normal, rhs, x)) return fit;
    const double alpha = x[0];
    const double beta = x[1];
    const double gamma = x[2];

    // α = e^(−kV·T/kA) must be a decay, and β must push the right way, for
    // the model to describe a physical axis at all.
    if (!(alpha > 0.0 && alpha < 1.0 && beta > 0.0)) return fit;

    const double intervalS = intervalSum / fit.samplesUsed;
    const double kV = (1.0 - alpha) / beta;
    const MotorFeedforward model{
        // Negative friction is only ever noise around a near-zero kS.
        .kS = std::fmax(0.0, -gamma / beta),
        .kV = kV,
        .kA = -kV * intervalS / std::log(alpha),
    };
    fit.model = model;

    // R² in voltage terms — how well the fitted model explains what was
    // commanded — which is the question that matters for feedforward, and
    // far more discriminating than R² on next-velocity, where "same as last
    // time" already scores near 1.
    double meanVolts = 0.0;
    for (const FitRow& row : rows) meanVolts += row.volts;
    meanVolts /= fit.samplesUsed;
    double residual = 0.0;
    double total = 0.0;
    for (const FitRow& row : rows) {
        const double midVelocity = 0.5 * (row.velocity + row.nextVelocity);
        const double acceleration = (row.nextVelocity - row.velocity) / row.intervalS;
        const double predicted = model.volts(midVelocity, acceleration);
        residual += (row.volts - predicted) * (row.volts - predicted);
        total += (row.volts - meanVolts) * (row.volts - meanVolts);
    }
    fit.rSquared = total > 0.0 ? 1.0 - residual / total : 0.0;

    fit.ok = model.valid() && fit.rSquared >= minRSquared;
    return fit;
}

double estimateResponseDelayS(const CharacterizationRun& stepRun, const MotorFeedforward& model,
                              int halfWindow) {
    if (!model.valid()) return -1.0;

    // The step is the first tick that commands anything.
    const auto stepIt = std::find_if(stepRun.begin(), stepRun.end(),
                                     [](const CharacterizationSample& s) { return s.volts != 0.0; });
    if (stepIt == stepRun.end()) return -1.0;
    const double stepTimeS = stepIt->timeMs / 1000.0;
    const double stepVolts = stepIt->volts;

    const double halfSpeed = 0.5 * model.maxVelocity(stepVolts);
    if (!(halfSpeed > 0.0)) return -1.0;
    const double direction = signOf(stepVolts);

    const int w = std::max(1, halfWindow);
    const std::vector<std::optional<double>> v = velocities(stepRun, w);
    std::optional<std::size_t> prev;
    for (std::size_t i = 0; i < stepRun.size(); ++i) {
        if (!v[i]) continue;
        const double timeS = stepRun[i].timeMs / 1000.0;
        const double speed = *v[i] * direction;

        if (timeS >= stepTimeS && speed >= halfSpeed) {
            // Interpolate the crossing between ticks — at a 10ms period, a
            // whole tick is a sizeable fraction of the delay being measured.
            double crossingS = timeS;
            if (prev) {
                const double prevTimeS = stepRun[*prev].timeMs / 1000.0;
                const double prevSpeed = *v[*prev] * direction;
                if (prevSpeed < halfSpeed && speed > prevSpeed) {
                    crossingS = prevTimeS + (timeS - prevTimeS) * (halfSpeed - prevSpeed) /
                                                (speed - prevSpeed);
                }
            }
            const double idealS = model.kA / model.kV * kLn2;
            return std::clamp(crossingS - stepTimeS - idealS, 0.0, kMaxPlausibleDelayS);
        }
        prev = i;
    }
    return -1.0;
}

AxisCharacterization characterizeAxis(const CharacterizationData& data, double minSpeed,
                                      int halfWindow, double minRSquared) {
    AxisCharacterization result;

    std::vector<CharacterizationRun> all = data.ramps;
    all.insert(all.end(), data.steps.begin(), data.steps.end());

    // Mean tick period across everything, to turn a delay into a shift.
    double periodSumS = 0.0;
    int periodCount = 0;
    for (const CharacterizationRun& run : all) {
        if (run.size() < 2) continue;
        periodSumS += (run.back().timeMs - run.front().timeMs) / 1000.0;
        periodCount += static_cast<int>(run.size()) - 1;
    }
    const double periodS = periodCount > 0 ? periodSumS / periodCount : 0.0;

    const auto meanDelay = [&](const MotorFeedforward& model) {
        double sum = 0.0;
        int valid = 0;
        for (const CharacterizationRun& step : data.steps) {
            const double delayS = estimateResponseDelayS(step, model, halfWindow);
            if (delayS >= 0.0) {
                sum += delayS;
                ++valid;
            }
        }
        return valid > 0 ? sum / valid : 0.0;
    };

    FeedforwardFit fit = fitFeedforward(all, minSpeed, halfWindow, 0, minRSquared);
    if (!fit.model.valid()) {
        result.fit = fit;
        return result;
    }

    double delayS = meanDelay(fit.model);
    if (periodS > 0.0) {
        const int shift = static_cast<int>(std::lround(delayS / periodS));
        if (shift > 0) {
            const FeedforwardFit shifted =
                fitFeedforward(all, minSpeed, halfWindow, shift, minRSquared);
            if (shifted.model.valid()) {
                fit = shifted;
                delayS = meanDelay(fit.model);
            }
        }
    }

    result.fit = fit;
    result.delayS = delayS;
    result.ok = fit.ok;
    return result;
}

} // namespace sapphirelib::tuning
