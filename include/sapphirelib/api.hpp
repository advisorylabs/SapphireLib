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

#include "sapphirelib/odom/motor_group_tracking_wheel.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/odometry_config.hpp"
#include "sapphirelib/odom/odometry_math.hpp"
#include "sapphirelib/odom/pose.hpp"
#include "sapphirelib/odom/rotation_tracking_wheel.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"

#include "sapphirelib/motion/motion_config.hpp"
#include "sapphirelib/motion/motion_queue.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"

#include "sapphirelib/sensors/imu.hpp"
#include "sapphirelib/sensors/imu_scale_math.hpp"

#include "sapphirelib/diag/sensor_check.hpp"

#include "sapphirelib/tuning/auto_tune_math.hpp"
#include "sapphirelib/tuning/auto_tune_runner.hpp"

#include "sapphirelib/gui/auton_selector_page.hpp"
#include "sapphirelib/gui/diagnostics_page.hpp"
#include "sapphirelib/gui/field_view_math.hpp"
#include "sapphirelib/gui/gui.hpp"
#include "sapphirelib/gui/home_page.hpp"
#include "sapphirelib/gui/odometry_page.hpp"
#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/gui/pid_tuner_page.hpp"

namespace sapphirelib {

/**
 * Call once at the start of `initialize()` in your PROS project before using
 * any SapphireLib functionality. Reserved for future setup (logging, task
 * scheduling, etc.) as later phases land.
 */
void initialize();

} // namespace sapphirelib
