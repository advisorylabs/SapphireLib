#pragma once

#include <cstdint>
#include <functional>

namespace sapphirelib {

/**
 * @brief Wait until a condition is true, or until a timeout
 *
 * The condition is checked before the timeout on every pass, so a condition that comes true right
 * at the deadline still counts. Always give a timeout in a match: a wait with no way out leaves
 * the robot frozen if a sensor comes unplugged
 *
 * @note for autonomous and your own tasks only. Calling it from opcontrol's loop stalls every
 * per-tick function
 *
 * @param done the condition to wait for
 * @param timeoutMs longest time to wait, in milliseconds. 0 waits forever
 * @param pollMs how often to check the condition, in milliseconds. 10 by default
 * @return true the condition came true
 * @return false the wait timed out
 *
 * @b Example
 * @code {.cpp}
 * // wait up to 1.5 seconds for the lift to arrive
 * if (!sapphirelib::waitUntil([] { return lift.settled(); }, 1500)) {
 *     // the lift didn't make it, skip the next step
 * }
 * @endcode
 */
bool waitUntil(const std::function<bool()>& done, std::uint32_t timeoutMs,
               std::uint32_t pollMs = 10);

} // namespace sapphirelib
