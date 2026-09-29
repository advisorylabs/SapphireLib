#include "sapphirelib/telemetry/characterization_tap.hpp"

#include <limits>
#include <memory>
#include <utility>

namespace sapphirelib::telemetry {

tuning::CharacterizationConfig tapCharacterization(tuning::CharacterizationConfig config,
                                                   Channel& channel) {
    if (!config.measure || !config.actuate) return config;

    // What the wrapped measure() last read, handed to the next actuate().
    // Shared (rather than captured by value) because the two wrappers are
    // separate std::functions; both run on the characterization task, so it
    // needs no synchronization.
    struct Tap {
        double position = std::numeric_limits<double>::quiet_NaN();
        /// A measure() has happened since the last logged row. This is what
        /// pairs rows one-to-one with the runner's samples — see the header.
        bool fresh = false;
    };
    auto tap = std::make_shared<Tap>();

    config.measure = [tap, measure = std::move(config.measure)] {
        const double position = measure();
        tap->position = position;
        tap->fresh = true;
        return position;
    };
    config.actuate = [tap, actuate = std::move(config.actuate), &channel](double volts) {
        actuate(volts);
        if (tap->fresh) {
            tap->fresh = false;
            channel.record({volts, tap->position});
        }
    };
    return config;
}

} // namespace sapphirelib::telemetry
