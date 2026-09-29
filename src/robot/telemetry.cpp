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
#include "robot/config.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"

namespace robot {

using sapphirelib::chassis::AxisVolts;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::input::Axis;
using sapphirelib::input::Controller;
using sapphirelib::telemetry::Channel;
using sapphirelib::telemetry::Logger;

namespace {

// The "driver" channel, once startTelemetry() has made it.
Channel* driverLog = nullptr;

} // namespace

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

    // The battery: sag under load skews any kV fitted from this log, and a
    // deep sag under a burst of current is how a brownout starts. Volts,
    // charge, current drawn (A) and the pack's temperature, 5 times a second —
    // fast enough to catch a sag, slow enough to cost nothing.
    log.poll(
        "batt", {"volts", "pct", "amps", "temp"}, 200,
        [](double* values) {
            // The reads fail (PROS_ERR / PROS_ERR_F) while another task holds
            // the battery port: a gap in the column rather than a
            // 2-million-volt spike or an "inf" percent.
            constexpr double kNoReading = std::numeric_limits<double>::quiet_NaN();
            const std::int32_t millivolts = pros::battery::get_voltage();
            values[0] = millivolts == PROS_ERR ? kNoReading : millivolts / 1000.0;
            const double percent = pros::battery::get_capacity();
            values[1] = std::isfinite(percent) ? percent : kNoReading;
            const std::int32_t milliamps = pros::battery::get_current();
            values[2] = milliamps == PROS_ERR ? kNoReading : milliamps / 1000.0;
            const double tempC = pros::battery::get_temperature();
            values[3] = std::isfinite(tempC) ? tempC : kNoReading;
        },
        {.capacity = 32, .decimals = 2});

    // Every motor's volts, current, temperature, speed and fault bits, 10
    // times a second: the evidence for a mechanism that faded mid-match (V5
    // motors cut their own power as they heat, and nothing else says so),
    // one that stalled, and one that came unplugged (rows of NaN).
    log.motor("motor.fl", ports::kFrontLeft);
    log.motor("motor.fr", ports::kFrontRight);
    log.motor("motor.bl", ports::kBackLeft);
    log.motor("motor.br", ports::kBackRight);
    log.motor("motor.ml", ports::kMiddleLeft);
    log.motor("motor.mr", ports::kMiddleRight);
    log.motor("motor.liftA", ports::kLiftA);
    log.motor("motor.liftB", ports::kLiftB);
    // Placeholder ports today (see config.hpp): until they're real these log
    // NaN, which the analyzer reports as "never answered" — a reminder, not a
    // fault.
    log.motor("motor.intake", ports::kIntake);
    log.motor("motor.claw", ports::kClawMotor);

    // The driver's sticks and buttons, recorded by logDriver() each opcontrol
    // tick. Capacity 128 is 2.5s at 20ms.
    driverLog = &log.channel("driver", {"lx", "ly", "rx", "ry", "buttons", "connected"},
                             {.capacity = 128, .decimals = 3});

    // "lift" PID + "lift.act".
    macros::attachTelemetry(log);

    log.start();
}

void logDriver(const Controller& controller) {
    if (driverLog == nullptr) return;
    driverLog->record({controller.axis(Axis::leftX), controller.axis(Axis::leftY),
                       controller.axis(Axis::rightX), controller.axis(Axis::rightY),
                       static_cast<double>(controller.buttons().heldMask()),
                       controller.connected() ? 1.0 : 0.0});
}

} // namespace robot
