#include "sapphirelib/mechanism/roller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sapphirelib::mechanism {

Roller::Roller(std::initializer_list<std::int8_t> ports, JamConfig jam)
    : motors_(ports), jam_(jam) {}

void Roller::spin(double volts, std::uint32_t nowMs) {
    // Speed only matters to the jam detector, so skip the device reads when
    // it's off.
    const double rpm = jam_.config().enabled ? speedRpm() : 0.0;
    double applied = jam_.update(volts, rpm, nowMs);
    // A NaN would survive the clamp, and converting it to an integer is
    // undefined behavior.
    if (std::isnan(applied)) applied = 0.0;
    applied = std::clamp(applied, -12.0, 12.0);
    motors_.move_voltage(static_cast<std::int32_t>(std::lround(applied * 1000.0)));
}

void Roller::stop() {
    jam_.reset();
    motors_.move_voltage(0);
}

bool Roller::jammed() const { return jam_.reversing(); }

pros::MotorGroup& Roller::motors() { return motors_; }

double Roller::speedRpm() const {
    double total = 0.0;
    int answering = 0;
    const int count = motors_.size();
    for (int i = 0; i < count; ++i) {
        // An unplugged motor reads PROS_ERR_F (infinity); leave it out
        // rather than letting it hide a jam on the others.
        const double rpm = motors_.get_actual_velocity(static_cast<std::uint8_t>(i));
        if (!std::isfinite(rpm)) continue;
        total += std::fabs(rpm);
        ++answering;
    }
    if (answering == 0) return std::numeric_limits<double>::quiet_NaN();
    return total / answering;
}

} // namespace sapphirelib::mechanism
