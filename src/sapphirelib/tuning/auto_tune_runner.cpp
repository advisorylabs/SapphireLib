#include "sapphirelib/tuning/auto_tune_runner.hpp"

#include "pros/rtos.hpp"

namespace sapphirelib::tuning {

std::vector<RelaySample> runRelayExperiment(const RelayTuneConfig& config) {
    std::vector<RelaySample> samples;

    const std::uint32_t startMs = pros::millis();
    const double target = config.measure() + config.setpointDelta;
    double relayOutput = 0.0;

    while (pros::millis() - startMs < config.timeoutMs) {
        const double measurement = config.measure();
        const double error = target - measurement;

        // Only flip the relay once error crosses to the opposite side of
        // the hysteresis band — inside the band, keep whatever sign it
        // last had (matters most right at startup, before the relay has
        // picked a side at all, and at each crossing, where sensor noise
        // would otherwise cause rapid re-switching).
        if (error > config.hysteresis) {
            relayOutput = config.relayAmplitude;
        } else if (error < -config.hysteresis) {
            relayOutput = -config.relayAmplitude;
        }

        config.actuate(relayOutput);
        samples.push_back(RelaySample{.timeMs = pros::millis() - startMs, .value = measurement});
        pros::delay(config.samplePeriodMs);
    }

    config.actuate(0.0);
    return samples;
}

} // namespace sapphirelib::tuning
