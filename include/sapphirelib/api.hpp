#pragma once

// include this one header to get all of SapphireLib

#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"
#include "sapphirelib/util/random.hpp"
#include "sapphirelib/util/sequence.hpp"
#include "sapphirelib/util/timing.hpp"
#include "sapphirelib/util/wait.hpp"
#include "sapphirelib/version.hpp"

#include "sapphirelib/chassis/drift_math.hpp"
#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/holonomic_drivetrain.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/chassis/tank_drivetrain.hpp"
#include "sapphirelib/chassis/thermal_math.hpp"
#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/control/heading_hold.hpp"
#include "sapphirelib/control/joystick_curve.hpp"
#include "sapphirelib/control/pid.hpp"

#include "sapphirelib/odom/motor_group_tracking_wheel.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/odometry_config.hpp"
#include "sapphirelib/odom/odometry_math.hpp"
#include "sapphirelib/odom/pose.hpp"
#include "sapphirelib/odom/rotation_tracking_wheel.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"

#include "sapphirelib/localization/field_map.hpp"
#include "sapphirelib/localization/monte_carlo_localizer.hpp"
#include "sapphirelib/localization/particle_filter.hpp"
#include "sapphirelib/localization/sensor_model.hpp"

#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/motion/motion_config.hpp"
#include "sapphirelib/motion/motion_queue.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"

#include "sapphirelib/sensors/imu.hpp"
#include "sapphirelib/sensors/imu_scale_math.hpp"

#include "sapphirelib/input/button_tracker.hpp"
#include "sapphirelib/input/controller.hpp"
#include "sapphirelib/input/controller_screen.hpp"

#include "sapphirelib/mechanism/jam_detector.hpp"
#include "sapphirelib/mechanism/piston.hpp"
#include "sapphirelib/mechanism/position_control.hpp"
#include "sapphirelib/mechanism/position_mechanism.hpp"
#include "sapphirelib/mechanism/preset_ladder.hpp"
#include "sapphirelib/mechanism/roller.hpp"

#include "sapphirelib/diag/sensor_check.hpp"

#include "sapphirelib/telemetry/channel.hpp"
#include "sapphirelib/telemetry/characterization_tap.hpp"
#include "sapphirelib/telemetry/csv_format.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/telemetry/file_naming.hpp"
#include "sapphirelib/telemetry/logger.hpp"
#include "sapphirelib/telemetry/motor_row.hpp"
#include "sapphirelib/telemetry/record.hpp"
#include "sapphirelib/telemetry/record_ring.hpp"

#include "sapphirelib/tuning/characterization_math.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"
#include "sapphirelib/tuning/gain_design.hpp"

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
 * @brief Initialize SapphireLib. This should be called at the start of initialize()
 *
 * Logs the library version to the terminal. Nothing else is started here: the GUI, odometry,
 * telemetry and mechanism tasks each start only when you call their own start()/startTask()
 *
 * @b Example
 * @code {.cpp}
 * void initialize() {
 *     // initialize the library before anything else
 *     sapphirelib::initialize();
 * }
 * @endcode
 */
void initialize();

} // namespace sapphirelib
