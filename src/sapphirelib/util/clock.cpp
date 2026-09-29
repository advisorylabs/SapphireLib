#include "sapphirelib/util/clock.hpp"

#include "pros/rtos.hpp"

namespace sapphirelib {

std::uint32_t millis() { return pros::millis(); }

std::uint64_t micros() { return pros::micros(); }

void delayMs(std::uint32_t ms) { pros::delay(ms); }

} // namespace sapphirelib
