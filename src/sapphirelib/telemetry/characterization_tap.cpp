#include "sapphirelib/telemetry/characterization_tap.hpp"

#include <limits>
#include <memory>
#include <utility>

namespace sapphirelib::telemetry {

namespace {

// what the wrapped measure() last read, for the next actuate() or hold(). Shared since the
// wrappers are separate functions; they all run on one task, so no synchronization
struct Tap {
    double position = std::numeric_limits<double>::quiet_NaN();
    // a measure() happened since the last row, which pairs rows one to one with samples
    bool fresh = false;

    void log(Channel& channel, double volts) {
        if (!fresh) return;
        fresh = false;
        channel.record({volts, position});
    }
};

// wrap measure() and actuate() in place, and return the shared state
std::shared_ptr<Tap> wrap(tuning::CharacterizationConfig& config, Channel& channel) {
    auto tap = std::make_shared<Tap>();
    config.measure = [tap, measure = std::move(config.measure)] {
        const double position = measure();
        tap->position = position;
        tap->fresh = true;
        return position;
    };
    config.actuate = [tap, actuate = std::move(config.actuate), &channel](double volts) {
        actuate(volts);
        tap->log(channel, volts);
    };
    return tap;
}

} // namespace

tuning::CharacterizationConfig tapCharacterization(tuning::CharacterizationConfig config,
                                                   Channel& channel) {
    if (!config.measure || !config.actuate) return config;
    wrap(config, channel);
    return config;
}

tuning::MechanismCharacterizationConfig
tapMechanismCharacterization(tuning::MechanismCharacterizationConfig config, Channel& channel) {
    if (!config.axis.measure || !config.axis.actuate) return config;
    std::shared_ptr<Tap> tap = wrap(config.axis, channel);
    if (config.hold) {
        config.hold = [tap, hold = std::move(config.hold), &channel] {
            hold();
            tap->log(channel, std::numeric_limits<double>::quiet_NaN());
        };
    }
    return config;
}

} // namespace sapphirelib::telemetry
