#include "sapphirelib/tuning/gain_design.hpp"

#include <algorithm>
#include <cmath>

namespace sapphirelib::tuning {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSettleBand = 0.02;
constexpr double kMinDamping = 0.1;
constexpr double kMaxDamping = 5.0;

/// Unit step response of ω = 1 spring-damper with damping ratio `zeta`.
double stepResponse(double zeta, double t) {
    if (std::fabs(zeta - 1.0) < 1e-6) return 1.0 - (1.0 + t) * std::exp(-t);
    if (zeta < 1.0) {
        const double wd = std::sqrt(1.0 - zeta * zeta);
        return 1.0 - std::exp(-zeta * t) / wd * std::sin(wd * t + std::acos(zeta));
    }
    const double root = std::sqrt(zeta * zeta - 1.0);
    const double fast = zeta + root;
    const double slow = zeta - root;
    return 1.0 - (fast * std::exp(-slow * t) - slow * std::exp(-fast * t)) / (fast - slow);
}

PIDGains placePoles(const MotorFeedforward& model, double omega, double zeta) {
    return PIDGains{
        .kP = model.kA * omega * omega,
        .kI = 0.0,
        .kD = std::fmax(0.0, 2.0 * zeta * omega * model.kA - model.kV),
    };
}

} // namespace

double normalizedSettleTime(double dampingRatio) {
    const double zeta = std::clamp(dampingRatio, kMinDamping, kMaxDamping);
    constexpr double kStep = 0.001;

    if (zeta >= 1.0 - 1e-6) {
        // Monotonic rise: settled the first time it's inside the band.
        for (double t = 0.0; t < 1000.0; t += kStep) {
            if (1.0 - stepResponse(zeta, t) <= kSettleBand) return t;
        }
        return 1000.0;
    }

    // Oscillatory: it may leave the band again after entering, so scan until
    // the decay envelope guarantees it can't, keeping the last exit.
    const double envelopeEndT =
        -std::log(kSettleBand * std::sqrt(1.0 - zeta * zeta)) / zeta + kStep;
    double lastOutside = 0.0;
    for (double t = 0.0; t < envelopeEndT; t += kStep) {
        if (std::fabs(1.0 - stepResponse(zeta, t)) > kSettleBand) lastOutside = t;
    }
    return lastOutside + kStep;
}

double phaseMarginDeg(const MotorFeedforward& model, PIDGains gains, double delayS) {
    if (!(gains.kP > 0.0) || !model.valid()) return 180.0;

    // Open loop L(jω) = (kP + jω·kD) / (jω·(jω·kA + kV)) · e^(−jωθ).
    const auto magnitude = [&](double w) {
        const double num = std::hypot(gains.kP, w * gains.kD);
        const double den = w * std::hypot(w * model.kA, model.kV);
        return num / den;
    };

    // |L| falls monotonically (numerator grows like ω, denominator like ω²),
    // so the crossover can be bisected — in log space, since plausible
    // crossovers span several decades.
    double lo = std::log(1e-4);
    double hi = std::log(1e5);
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (magnitude(std::exp(mid)) > 1.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const double crossover = std::exp(0.5 * (lo + hi));

    const double phaseRad = std::atan2(crossover * gains.kD, gains.kP) - kPi / 2.0 -
                            std::atan2(crossover * model.kA, model.kV) -
                            crossover * std::fmax(0.0, delayS);
    return 180.0 + phaseRad * 180.0 / kPi;
}

GainDesign designPositionGains(const MotorFeedforward& model, ResponseSpec spec, double delayS) {
    GainDesign design;
    if (!model.valid() || !(spec.settleTimeS > 0.0)) return design;

    const double zeta = std::clamp(spec.dampingRatio, kMinDamping, kMaxDamping);
    const double unitSettle = normalizedSettleTime(zeta);
    double omega = unitSettle / spec.settleTimeS;

    if (phaseMarginDeg(model, placePoles(model, omega, zeta), delayS) < spec.minPhaseMarginDeg) {
        // Phase margin shrinks as ω rises, so the fastest acceptable design
        // is a bisection away. The lower bound is slow enough that delay
        // can't matter.
        double slow = 0.0;
        double fast = omega;
        for (int i = 0; i < 60; ++i) {
            const double mid = 0.5 * (slow + fast);
            if (phaseMarginDeg(model, placePoles(model, mid, zeta), delayS) >=
                spec.minPhaseMarginDeg) {
                slow = mid;
            } else {
                fast = mid;
            }
        }
        omega = slow;
        design.limitedByDelay = true;
    }

    if (!(omega > 0.0)) return design;

    design.ok = true;
    design.gains = placePoles(model, omega, zeta);
    design.naturalFrequency = omega;
    design.settleTimeS = unitSettle / omega;
    design.phaseMarginDeg = phaseMarginDeg(model, design.gains, delayS);
    return design;
}

} // namespace sapphirelib::tuning
