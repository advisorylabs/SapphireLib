#include "sapphirelib/tuning/auto_tune_math.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib::tuning {

namespace {
constexpr double kGrowFactor = 1.1;
constexpr double kShrinkFactor = 0.9;

void clampNonNegative(double (&params)[3]) {
    for (double& p : params) p = std::max(0.0, p);
}
} // namespace

OvershootResult measureOvershoot(const std::vector<double>& samples, double target) {
    const double start = samples.front();
    const double direction = (target >= start) ? 1.0 : -1.0;

    double peak = start;
    for (double sample : samples) {
        if (direction > 0.0) {
            peak = std::max(peak, sample);
        } else {
            peak = std::min(peak, sample);
        }
    }

    return OvershootResult{
        .overshoot = std::max(0.0, direction * (peak - target)),
        .finalError = std::fabs(target - samples.back()),
    };
}

double autoTuneCost(const OvershootResult& result, double finalErrorWeight) {
    return result.overshoot + finalErrorWeight * result.finalError;
}

TwiddleState makeTwiddleState(PIDGains initial, PIDGains stepSizes) {
    TwiddleState state;
    state.params[0] = initial.kP;
    state.params[1] = initial.kI;
    state.params[2] = initial.kD;
    state.stepSizes[0] = stepSizes.kP;
    state.stepSizes[1] = stepSizes.kI;
    state.stepSizes[2] = stepSizes.kD;
    return state;
}

PIDGains currentGains(const TwiddleState& state) {
    return PIDGains{.kP = state.params[0], .kI = state.params[1], .kD = state.params[2]};
}

void reportCost(TwiddleState& state, double cost) {
    if (state.bestCost < 0.0) {
        // First-ever measurement: this is the baseline before any
        // perturbation. Record it, then perturb params[0] upward to set up
        // the first trial.
        state.bestCost = cost;
        state.params[state.index] += state.stepSizes[state.index];
        clampNonNegative(state.params);
        return;
    }

    bool advance = false;

    if (!state.triedDecrease) {
        // We just tested params[index] with +stepSizes[index] applied.
        if (cost < state.bestCost) {
            state.bestCost = cost;
            state.stepSizes[state.index] *= kGrowFactor;
            advance = true;
        } else {
            state.params[state.index] -= 2.0 * state.stepSizes[state.index];
            clampNonNegative(state.params);
            state.triedDecrease = true;
        }
    } else {
        // We just tested params[index] with -stepSizes[index] applied
        // (net, relative to before we tried +).
        if (cost < state.bestCost) {
            state.bestCost = cost;
            state.stepSizes[state.index] *= kGrowFactor;
        } else {
            // Neither direction helped — revert to the original value and
            // search more finely next time.
            state.params[state.index] += state.stepSizes[state.index];
            clampNonNegative(state.params);
            state.stepSizes[state.index] *= kShrinkFactor;
        }
        state.triedDecrease = false;
        advance = true;
    }

    if (advance) {
        state.index = (state.index + 1) % 3;
        state.params[state.index] += state.stepSizes[state.index];
        clampNonNegative(state.params);
    }
}

double totalStepSize(const TwiddleState& state) {
    return state.stepSizes[0] + state.stepSizes[1] + state.stepSizes[2];
}

} // namespace sapphirelib::tuning
