/**
 * \file sapphirelib/util/wait.hpp
 *
 * A blocking wait for autonomous code that always has a timeout. A wait with
 * no way out is how a robot sits frozen for the rest of a match when a
 * sensor comes unplugged. Pure through util/clock.hpp; see
 * tests/util/wait_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <functional>

namespace sapphirelib {

/// Blocks until `done()` returns true or `timeoutMs` passes, checking every
/// `pollMs`. `done` is checked before the timeout on every pass, so a
/// condition that comes true right at the deadline still counts. Returns
/// true if `done()` came true, false on timeout.
///
/// `timeoutMs` = 0 waits with no limit (the chassis::ExitConditions
/// convention, and just as not-recommended in a match).
///
/// For autonomous or a task of your own only. Called from opcontrol()'s loop
/// it stalls every per-tick function, and input::Controller reports
/// resumed() once it returns.
bool waitUntil(const std::function<bool()>& done, std::uint32_t timeoutMs,
               std::uint32_t pollMs = 10);

} // namespace sapphirelib
