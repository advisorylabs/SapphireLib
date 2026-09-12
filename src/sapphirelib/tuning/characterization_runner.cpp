#include "sapphirelib/tuning/characterization_runner.hpp"

#include <algorithm>
#include <cmath>

#include "pros/rtos.hpp"

namespace sapphirelib::tuning {

namespace {

/// The axis counts as stopped once it moves less than this fraction of the
/// segment's travel limit (or of one unit, for an unlimited axis) across
/// one check interval.
constexpr double kStoppedFraction = 0.002;
constexpr std::uint32_t kStoppedCheckMs = 100;

void waitUntilStopped(const CharacterizationConfig& config) {
    config.actuate(0.0);
    const double threshold =
        std::fmax(config.maxTravel > 0.0 ? config.maxTravel * kStoppedFraction : 0.0, 0.01);
    const std::uint32_t start = pros::millis();
    double last = config.measure();
    while (pros::millis() - start < config.settleTimeoutMs) {
        pros::delay(kStoppedCheckMs);
        const double now = config.measure();
        if (std::fabs(now - last) < threshold) return;
        last = now;
    }
}

/// Runs one segment: `preRollMs` at 0V, then `voltsAt(seconds since the
/// voltage started)` until the travel limit or duration cap.
template <typename VoltsAt>
CharacterizationRun runSegment(const CharacterizationConfig& config, VoltsAt voltsAt) {
    CharacterizationRun run;
    const std::uint32_t startMs = pros::millis();
    const double origin = config.measure();

    while (true) {
        const std::uint32_t elapsedMs = pros::millis() - startMs;
        if (elapsedMs >= config.preRollMs + config.maxSegmentMs) break;

        const double position = config.measure();
        if (config.maxTravel > 0.0 && std::fabs(position - origin) >= config.maxTravel) break;

        const double volts =
            elapsedMs < config.preRollMs ? 0.0 : voltsAt((elapsedMs - config.preRollMs) / 1000.0);
        config.actuate(volts);
        run.push_back(CharacterizationSample{.timeMs = elapsedMs, .volts = volts, .position = position});
        pros::delay(config.samplePeriodMs);
    }

    config.actuate(0.0);
    return run;
}

} // namespace

CharacterizationData runCharacterization(const CharacterizationConfig& config) {
    CharacterizationData data;
    const double rampRate = std::fabs(config.rampVoltsPerS);
    const double rampMax = std::fabs(config.rampMaxVolts);
    const double step = std::fabs(config.stepVolts);

    for (const double direction : {1.0, -1.0}) {
        waitUntilStopped(config);
        data.ramps.push_back(runSegment(config, [&](double t) {
            return direction * std::min(rampRate * t, rampMax);
        }));
    }
    for (const double direction : {1.0, -1.0}) {
        waitUntilStopped(config);
        data.steps.push_back(runSegment(config, [&](double) { return direction * step; }));
    }

    waitUntilStopped(config);
    return data;
}

} // namespace sapphirelib::tuning
