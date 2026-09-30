#pragma once

#include <cstdint>

// the library's only access to the system clock. There's no PROS include here, so pure modules
// can still be built and tested on a computer: the robot defines these in util/clock.cpp, and
// host tests define them as a fake clock

namespace sapphirelib {

/**
 * @brief Get the time since the program started
 *
 * @note wraps after ~49.7 days, so compare two readings with elapsedMs(), never with <
 *
 * @return std::uint32_t time, in milliseconds
 */
std::uint32_t millis();

/**
 * @brief Get the time since the program started, in microseconds
 *
 * 64 bits, so it never wraps in practice. Telemetry timestamps use this
 *
 * @return std::uint64_t time, in microseconds
 */
std::uint64_t micros();

/**
 * @brief Block the calling task
 *
 * @note for autonomous and your own tasks. Never call it from a per-tick driver control
 * function, which it would stall
 *
 * @param ms how long to wait, in milliseconds
 */
void delayMs(std::uint32_t ms);

} // namespace sapphirelib
