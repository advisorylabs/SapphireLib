#include "sapphirelib/tuning/characterization_runner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sapphirelib/util/clock.hpp"

namespace sapphirelib::tuning {

namespace {

/// The axis counts as stopped once it moves less than this fraction of the
/// segment's travel limit (or of one unit, for an unlimited axis) across
/// one check interval.
constexpr double kStoppedFraction = 0.002;
constexpr std::uint32_t kStoppedCheckMs = 100;

/// One run's shared state: the config, and whether it has been told to stop.
struct Run {
    const CharacterizationConfig& config;
    bool aborted = false;

    /// Checks (and latches) the config's abort condition.
    bool abortRequested() {
        if (!aborted && config.shouldAbort && config.shouldAbort()) aborted = true;
        return aborted;
    }
};

/// Applies `rest` (0V, or a mechanism's hold) and waits for the axis to
/// stop moving, for at most settleTimeoutMs. Returns early on abort.
template <typename Rest> void waitUntilStopped(Run& run, double threshold, Rest rest) {
    const CharacterizationConfig& config = run.config;
    rest();
    const std::uint32_t start = sapphirelib::millis();
    double last = config.measure();
    while (sapphirelib::millis() - start < config.settleTimeoutMs) {
        if (run.abortRequested()) return;
        sapphirelib::delayMs(kStoppedCheckMs);
        const double now = config.measure();
        if (std::fabs(now - last) < threshold) return;
        last = now;
    }
}

/// Runs one segment: `preRollMs` of `preRoll()` (which applies whatever
/// holds the axis and returns the volts to record for it), then
/// `voltsAt(seconds since the voltage started)` until `outOfRange(position)`,
/// a non-finite reading, the duration cap, or an abort. Ends with `rest()`.
template <typename OutOfRange, typename PreRoll, typename VoltsAt, typename Rest>
CharacterizationRun runSegment(Run& runState, OutOfRange outOfRange, PreRoll preRoll,
                               VoltsAt voltsAt, Rest rest) {
    const CharacterizationConfig& config = runState.config;
    CharacterizationRun run;
    const std::uint32_t startMs = sapphirelib::millis();

    while (!runState.abortRequested()) {
        const std::uint32_t elapsedMs = sapphirelib::millis() - startMs;
        if (elapsedMs >= config.preRollMs + config.maxSegmentMs) break;

        const double position = config.measure();
        if (!std::isfinite(position) || outOfRange(position)) break;

        double volts;
        if (elapsedMs < config.preRollMs) {
            volts = preRoll();
        } else {
            volts = voltsAt((elapsedMs - config.preRollMs) / 1000.0);
            config.actuate(volts);
        }
        run.push_back(
            CharacterizationSample{.timeMs = elapsedMs, .volts = volts, .position = position});
        sapphirelib::delayMs(config.samplePeriodMs);
    }

    rest();
    return run;
}

} // namespace

CharacterizationData runCharacterization(const CharacterizationConfig& config) {
    CharacterizationData data;
    Run run{config};
    const double rampRate = std::fabs(config.rampVoltsPerS);
    const double rampMax = std::fabs(config.rampMaxVolts);
    const double step = std::fabs(config.stepVolts);
    const double threshold =
        std::fmax(config.maxTravel > 0.0 ? config.maxTravel * kStoppedFraction : 0.0, 0.01);
    const auto zero = [&] { config.actuate(0.0); };
    const auto preRoll = [&] {
        config.actuate(0.0);
        return 0.0;
    };

    if (config.start) config.start();

    // Each segment measures its travel from where it starts.
    const auto segment = [&](auto voltsAt) {
        const double origin = config.measure();
        return runSegment(
            run,
            [&](double position) {
                return config.maxTravel > 0.0 && std::fabs(position - origin) >= config.maxTravel;
            },
            preRoll, voltsAt, zero);
    };

    for (const double direction : {1.0, -1.0}) {
        if (run.abortRequested()) break;
        waitUntilStopped(run, threshold, zero);
        if (run.abortRequested()) break;
        data.ramps.push_back(
            segment([&](double t) { return direction * std::min(rampRate * t, rampMax); }));
    }
    for (const double direction : {1.0, -1.0}) {
        if (run.abortRequested()) break;
        waitUntilStopped(run, threshold, zero);
        if (run.abortRequested()) break;
        data.steps.push_back(segment([&](double) { return direction * step; }));
    }

    if (run.abortRequested()) {
        zero();
    } else {
        waitUntilStopped(run, threshold, zero);
    }
    data.aborted = run.aborted;
    if (config.finish) config.finish();
    return data;
}

CharacterizationData
runMechanismCharacterization(const MechanismCharacterizationConfig& mechanism) {
    const CharacterizationConfig& config = mechanism.axis;
    CharacterizationData data;
    Run run{config};
    const double rampRate = std::fabs(config.rampVoltsPerS);
    const double rampMax = std::fabs(config.rampMaxVolts);
    const double stepUp = std::fabs(config.stepVolts);
    const double stepDown =
        mechanism.downStepVolts != 0.0 ? std::fabs(mechanism.downStepVolts) : stepUp;
    const double lower = mechanism.lowerLimit;
    const double upper = mechanism.upperLimit;
    const double threshold = std::fmax(std::fabs(upper - lower) * kStoppedFraction, 0.01);

    const auto hold = [&] {
        if (mechanism.hold) {
            mechanism.hold();
        } else {
            config.actuate(0.0);
        }
    };
    // A held sample's volts are unknown — whatever the brake applied — so
    // NaN, which the fit skips. Only a mechanism with no hold really sits at
    // 0V.
    const auto preRoll = [&] {
        hold();
        return mechanism.hold ? std::numeric_limits<double>::quiet_NaN() : 0.0;
    };
    const auto up = [&](double position) { return position >= upper; };
    const auto down = [&](double position) { return position <= lower; };

    if (config.start) config.start();

    struct Segment {
        bool upward;
        bool ramp;
    };
    for (const Segment s :
         {Segment{true, true}, Segment{false, true}, Segment{true, false}, Segment{false, false}}) {
        if (run.abortRequested()) break;
        waitUntilStopped(run, threshold, hold);
        if (run.abortRequested()) break;

        const double direction = s.upward ? 1.0 : -1.0;
        const double step = s.upward ? stepUp : stepDown;
        const auto voltsAt = [&](double t) {
            return direction * (s.ramp ? std::min(rampRate * t, rampMax) : step);
        };
        CharacterizationRun segment = s.upward ? runSegment(run, up, preRoll, voltsAt, hold)
                                               : runSegment(run, down, preRoll, voltsAt, hold);
        (s.ramp ? data.ramps : data.steps).push_back(std::move(segment));
    }

    if (run.abortRequested()) {
        hold();
    } else {
        waitUntilStopped(run, threshold, hold);
    }
    data.aborted = run.aborted;
    if (config.finish) config.finish();
    return data;
}

} // namespace sapphirelib::tuning
