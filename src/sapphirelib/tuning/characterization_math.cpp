#include "sapphirelib/tuning/characterization_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace sapphirelib::tuning {

namespace {

constexpr double kLn2 = 0.69314718055994530942;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

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

/// A velocity that exists and is a number — a NaN position (a sensor that
/// didn't answer) makes one that exists but isn't.
bool usable(const std::optional<double>& velocity) {
    return velocity.has_value() && std::isfinite(*velocity);
}

/// Solves the N×N system `a`·x = `b` by Gaussian elimination with partial
/// pivoting. False if a pivot is too small relative to the matrix's scale to
/// trust — i.e. the data didn't constrain every parameter.
template <std::size_t N>
bool solve(std::array<std::array<double, N>, N> a, std::array<double, N> b,
           std::array<double, N>& x) {
    double scale = 0.0;
    for (const auto& row : a) {
        for (double value : row) scale = std::fmax(scale, std::fabs(value));
    }
    if (!(scale > 0.0)) return false;

    for (std::size_t col = 0; col < N; ++col) {
        std::size_t pivot = col;
        for (std::size_t r = col + 1; r < N; ++r) {
            if (std::fabs(a[r][col]) > std::fabs(a[pivot][col])) pivot = r;
        }
        if (std::fabs(a[pivot][col]) < scale * 1e-12) return false;
        std::swap(a[pivot], a[col]);
        std::swap(b[pivot], b[col]);

        for (std::size_t r = col + 1; r < N; ++r) {
            const double factor = a[r][col] / a[col][col];
            for (std::size_t c = col; c < N; ++c) a[r][c] -= factor * a[col][c];
            b[r] -= factor * b[col];
        }
    }

    for (std::size_t i = N; i-- > 0;) {
        double sum = b[i];
        for (std::size_t c = i + 1; c < N; ++c) sum -= a[i][c] * x[c];
        x[i] = sum / a[i][i];
    }
    return true;
}

struct FitRow {
    double velocity;     // v[k]
    double nextVelocity; // v[k+m]
    double volts;        // mean command over the interval, delay-shifted
    double gravity;      // mean gravity factor over the interval (0 without gravity)
    double intervalS;    // T
};

/// The regression behind fitFeedforward() and fitMechanism(): N = 3
/// parameters (α, β, γ) without gravity, 4 (plus δ) with it.
template <std::size_t N>
MechanismFit fitDiscrete(const std::vector<CharacterizationRun>& runs, const GravityShape& gravity,
                         double minSpeed, int halfWindow, int delayTicks, double minRSquared) {
    static_assert(N == 3 || N == 4);
    constexpr bool kWithGravity = N == 4;

    MechanismFit fit;
    fit.model.gravity = gravity;
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
            if (!usable(v[k]) || !usable(v[k + m])) continue;
            if (std::fabs(*v[k]) < minSpeed) continue;
            const double intervalS = (run[k + m].timeMs - run[k].timeMs) / 1000.0;
            if (!(intervalS > 0.0)) continue;

            double voltsSum = 0.0;
            for (int j = k; j < k + m; ++j) voltsSum += run[j - shift].volts;
            // A held (NaN-volts) sample anywhere in the interval: nobody knows
            // what the brake applied, so the row can't say anything.
            if (!std::isfinite(voltsSum)) continue;

            double gravitySum = 0.0;
            if constexpr (kWithGravity) {
                // Gravity acts on where the axis is, so it isn't delay-shifted
                // the way the command is.
                for (int j = k; j < k + m; ++j) gravitySum += gravity.factor(run[j].position);
                if (!std::isfinite(gravitySum)) continue;
            }
            rows.push_back(FitRow{*v[k], *v[k + m], voltsSum / m, gravitySum / m, intervalS});
        }
    }
    fit.samplesUsed = static_cast<int>(rows.size());
    if (fit.samplesUsed < kMinFitRows) return fit;

    // Normal equations for v[k+m] = α·v[k] + β·u + γ·sign(v[k]) [+ δ·g].
    std::array<std::array<double, N>, N> normal{};
    std::array<double, N> rhs{};
    double intervalSum = 0.0;
    for (const FitRow& row : rows) {
        std::array<double, N> f;
        f[0] = row.velocity;
        f[1] = row.volts;
        f[2] = signOf(row.velocity);
        if constexpr (kWithGravity) f[3] = row.gravity;
        for (std::size_t r = 0; r < N; ++r) {
            for (std::size_t c = 0; c < N; ++c) normal[r][c] += f[r] * f[c];
            rhs[r] += f[r] * row.nextVelocity;
        }
        intervalSum += row.intervalS;
    }

    std::array<double, N> x{};
    if (!solve<N>(normal, rhs, x)) return fit;
    const double alpha = x[0];
    const double beta = x[1];
    const double gamma = x[2];

    // α = e^(−kV·T/kA) must be a decay, and β must push the right way, for
    // the model to describe a physical axis at all.
    if (!(alpha > 0.0 && alpha < 1.0 && beta > 0.0)) return fit;

    const double intervalS = intervalSum / fit.samplesUsed;
    const double kV = (1.0 - alpha) / beta;
    fit.model.motion = MotorFeedforward{
        // Negative friction is only ever noise around a near-zero kS.
        .kS = std::fmax(0.0, -gamma / beta),
        .kV = kV,
        .kA = -kV * intervalS / std::log(alpha),
    };
    // Not clamped: rubber bands or a counterweight that more than carry the
    // mechanism make a genuinely negative kG.
    if constexpr (kWithGravity) fit.model.kG = -x[3] / beta;

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
        double predicted = fit.model.motion.volts(midVelocity, acceleration);
        if constexpr (kWithGravity) predicted += fit.model.kG * row.gravity;
        residual += (row.volts - predicted) * (row.volts - predicted);
        total += (row.volts - meanVolts) * (row.volts - meanVolts);
    }
    fit.rSquared = total > 0.0 ? 1.0 - residual / total : 0.0;

    fit.ok = fit.model.valid() && fit.rSquared >= minRSquared;
    return fit;
}

} // namespace

double GravityShape::factor(double position) const {
    switch (kind) {
        case GravityKind::none: return 0.0;
        case GravityKind::constant: return 1.0;
        case GravityKind::cosine:
            // The same expression as mechanism::GravityFeedforward::volts(), so
            // a fitted kG drops into its cosineVolts unchanged.
            return std::cos((position - horizontalPosition) * armDegreesPerUnit * kDegToRad);
    }
    return 0.0;
}

double MechanismModel::gravityVolts(double position) const { return kG * gravity.factor(position); }

mechanism::GravityFeedforward MechanismModel::gravityFeedforward() const {
    switch (gravity.kind) {
        case GravityKind::none: return {};
        case GravityKind::constant: return {.constantVolts = kG};
        case GravityKind::cosine:
            return {.cosineVolts = kG,
                    .horizontalPosition = gravity.horizontalPosition,
                    .armDegreesPerUnit = gravity.armDegreesPerUnit};
    }
    return {};
}

FeedforwardFit fitFeedforward(const std::vector<CharacterizationRun>& runs, double minSpeed,
                              int halfWindow, int delayTicks, double minRSquared) {
    const MechanismFit fit =
        fitDiscrete<3>(runs, GravityShape{}, minSpeed, halfWindow, delayTicks, minRSquared);
    return FeedforwardFit{.ok = fit.ok,
                          .model = fit.model.motion,
                          .rSquared = fit.rSquared,
                          .samplesUsed = fit.samplesUsed};
}

MechanismFit fitMechanism(const std::vector<CharacterizationRun>& runs, GravityShape gravity,
                          double minSpeed, int halfWindow, int delayTicks, double minRSquared) {
    if (gravity.kind == GravityKind::none) {
        return fitDiscrete<3>(runs, gravity, minSpeed, halfWindow, delayTicks, minRSquared);
    }
    return fitDiscrete<4>(runs, gravity, minSpeed, halfWindow, delayTicks, minRSquared);
}

double estimateResponseDelayS(const CharacterizationRun& stepRun, const MotorFeedforward& model,
                              int halfWindow) {
    return estimateMechanismDelayS(stepRun, MechanismModel{.motion = model}, halfWindow);
}

double estimateMechanismDelayS(const CharacterizationRun& stepRun, const MechanismModel& model,
                               int halfWindow) {
    if (!model.valid()) return -1.0;
    const MotorFeedforward& motion = model.motion;

    // The step is the first tick that commands anything. A held (NaN) tick
    // commands nothing the model knows about; it's the rest before the step.
    const auto stepIt =
        std::find_if(stepRun.begin(), stepRun.end(), [](const CharacterizationSample& s) {
            return std::isfinite(s.volts) && s.volts != 0.0;
        });
    if (stepIt == stepRun.end()) return -1.0;
    const double stepTimeS = stepIt->timeMs / 1000.0;

    // What the step has to work with once gravity has taken its share —
    // for a drive axis (no gravity) exactly the step's volts.
    const double netVolts = stepIt->volts - model.gravityVolts(stepIt->position);
    const double halfSpeed = 0.5 * std::fmax(0.0, (std::fabs(netVolts) - motion.kS) / motion.kV);
    if (!(halfSpeed > 0.0)) return -1.0;
    const double direction = signOf(netVolts);

    const int w = std::max(1, halfWindow);
    const std::vector<std::optional<double>> v = velocities(stepRun, w);
    std::optional<std::size_t> prev;
    for (std::size_t i = 0; i < stepRun.size(); ++i) {
        if (!usable(v[i])) continue;
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
                    crossingS = prevTimeS +
                                (timeS - prevTimeS) * (halfSpeed - prevSpeed) / (speed - prevSpeed);
                }
            }
            const double idealS = motion.kA / motion.kV * kLn2;
            return std::clamp(crossingS - stepTimeS - idealS, 0.0, kMaxPlausibleDelayS);
        }
        prev = i;
    }
    return -1.0;
}

MechanismCharacterization characterizeMechanism(const CharacterizationData& data,
                                                GravityShape gravity, double minSpeed,
                                                int halfWindow, double minRSquared) {
    MechanismCharacterization result;

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

    const auto meanDelay = [&](const MechanismModel& model) {
        double sum = 0.0;
        int valid = 0;
        for (const CharacterizationRun& step : data.steps) {
            const double delayS = estimateMechanismDelayS(step, model, halfWindow);
            if (delayS >= 0.0) {
                sum += delayS;
                ++valid;
            }
        }
        return valid > 0 ? sum / valid : 0.0;
    };

    MechanismFit fit = fitMechanism(all, gravity, minSpeed, halfWindow, 0, minRSquared);
    if (!fit.model.valid()) {
        result.fit = fit;
        return result;
    }

    double delayS = meanDelay(fit.model);
    if (periodS > 0.0) {
        const int shift = static_cast<int>(std::lround(delayS / periodS));
        if (shift > 0) {
            const MechanismFit shifted =
                fitMechanism(all, gravity, minSpeed, halfWindow, shift, minRSquared);
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

AxisCharacterization characterizeAxis(const CharacterizationData& data, double minSpeed,
                                      int halfWindow, double minRSquared) {
    const MechanismCharacterization result =
        characterizeMechanism(data, GravityShape{}, minSpeed, halfWindow, minRSquared);
    return AxisCharacterization{.ok = result.ok,
                                .fit = FeedforwardFit{.ok = result.fit.ok,
                                                      .model = result.fit.model.motion,
                                                      .rSquared = result.fit.rSquared,
                                                      .samplesUsed = result.fit.samplesUsed},
                                .delayS = result.delayS};
}

} // namespace sapphirelib::tuning
