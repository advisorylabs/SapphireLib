/**
 * \file telemetry.cpp
 *
 * Wires the robot's devices into the SD-card log. The char.* channels Auto-Tune
 * logs into are registered with the tuner, in tuning.cpp.
 *
 * Team 96671H — Hitmen
 */

#include "robot/telemetry.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include "pros/error.h"
#include "pros/misc.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"

namespace robot {

using sapphirelib::chassis::AxisVolts;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::telemetry::Logger;

Logger& logger() {
    // A function-local static, like the devices: constructing a Logger only
    // sets up its tables — no tasks and no card access until start().
    static Logger instance({.directory = "/usd/sl", .robotName = "96671H"});
    return instance;
}

void startTelemetry() {
    Logger& log = logger();
    HolonomicDrivetrain& chassis = drivetrain();

    // Every drivetrain controller, through the same accessors PidTunerPage
    // uses. Gains the tuner or Auto-Tune set later show up as G rows on their
    // own.
    log.pid("drive", chassis.drivePID());
    log.pid("turn", chassis.turnPID());
    log.pid("hold", chassis.headingHoldPID());

    // x, y, heading at 100Hz.
    log.pose(odometry(), "odom", 10);

    // What the drivetrain actually commanded on each axis (the PIDs' "out" is
    // only the loop's share, before mixing and heading correction) — the
    // input side of the chassis's response, for fitting its model offline.
    // Read from the sampler task; appliedAxisVolts() is safe from any task.
    log.poll(
        "chassis", {"fwd_v", "strafe_v", "turn_v"}, 10,
        [drive = &chassis](double* values) {
            const AxisVolts volts = drive->appliedAxisVolts();
            values[0] = volts.forward;
            values[1] = volts.strafe;
            values[2] = volts.turn;
        },
        {.decimals = 3});

    // Battery voltage: sag under load skews any kV fitted from this log, so the
    // app needs it next to the PID rows. Once a second is plenty for something
    // this slow.
    log.poll(
        "batt", {"volts", "pct"}, 1000,
        [](double* values) {
            // The reads fail (PROS_ERR / PROS_ERR_F) while another task holds
            // the battery port: a gap in the column rather than a
            // 2-million-volt spike or an "inf" percent.
            constexpr double kNoReading = std::numeric_limits<double>::quiet_NaN();
            const std::int32_t millivolts = pros::battery::get_voltage();
            values[0] = millivolts == PROS_ERR ? kNoReading : millivolts / 1000.0;
            const double percent = pros::battery::get_capacity();
            values[1] = std::isfinite(percent) ? percent : kNoReading;
        },
        {.capacity = 32, .decimals = 2});

    // "lift" PID + "lift.act".
    macros::attachTelemetry(log);

    log.start();
}

} // namespace robot
