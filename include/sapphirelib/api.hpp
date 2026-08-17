/**
 * \file sapphirelib/api.hpp
 *
 * Umbrella header for SapphireLib. Include this single header to pull in the
 * full public API of the library.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/log.hpp"
#include "sapphirelib/version.hpp"

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/holonomic_drivetrain.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/chassis/tank_drivetrain.hpp"
#include "sapphirelib/control/joystick_curve.hpp"
#include "sapphirelib/control/pid.hpp"

// Phase 2+: odometry headers
// #include "sapphirelib/odom/odometry.hpp"

namespace sapphirelib {

/**
 * Call once at the start of `initialize()` in your PROS project before using
 * any SapphireLib functionality. Reserved for future setup (logging, task
 * scheduling, etc.) as later phases land.
 */
void initialize();

} // namespace sapphirelib
