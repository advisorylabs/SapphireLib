#include "sapphirelib/telemetry/characterization_tap.hpp"

#include <limits>
#include <memory>
#include <utility>

namespace sapphirelib::telemetry {

namespace {

// What the wrapped measure() last read, handed to the next actuate() (or
// hold()). Shared (rather than captured by value) because the wrappers are
// separate std::functions; all of them run on the characterization task, so
// it needs no synchronization.
struct Tap {
    double position = std::numeric_limits<double>::quiet_NaN();
    /// A measure() has happened since the last logged row. This is what
    /// pairs rows one-to-one with the runner's samples — see the header.
    bool fresh = false;

    void log(Channel& channel, double volts) {
        if (!fresh) return;
        fresh = false;
        channel.record({volts, position});
    }
};

/// Wraps measure() and actuate() in place; returns the shared state for any
/// further wrappers.
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
