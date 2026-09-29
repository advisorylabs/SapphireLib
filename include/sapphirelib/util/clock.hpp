/**
 * \file sapphirelib/util/clock.hpp
 *
 * The library's one seam onto the system clock. Declared with no PROS
 * include, so pure modules that need to wait or timestamp (util/wait.hpp,
 * Sequence::runBlocking(), telemetry channels) still build on a desktop
 * compiler. On the robot these are defined in src/sapphirelib/util/clock.cpp
 * on top of pros::millis(), pros::micros(), and pros::delay(); a host test
 * defines them itself, as a fake clock it can step deterministically (its
 * delayMs() just advances the fake time).
 *
 * Per-tick code should not call millis() over and over: take the tick's one
 * "now" from input::Controller::now() (or read it once at the top of your own
 * loop) and hand that same value to everything that tick — see
 * util/timing.hpp for why.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

namespace sapphirelib {

/// Milliseconds since the program started. Wraps after ~49.7 days, so compare
/// two readings with elapsedMs() (util/timing.hpp), never with `<`.
std::uint32_t millis();

/// Microseconds since the program started. 64-bit, so it never wraps in
/// practice — which is why telemetry timestamps use it.
std::uint64_t micros();

/// Blocks the calling task for `ms` milliseconds. For autonomous and tasks of
/// your own — never from a per-tick driver-control function, which it would
/// stall.
void delayMs(std::uint32_t ms);

} // namespace sapphirelib
