/**
 * \file telemetry.cpp
 *
 * Wires the robot's devices into the SD-card log. The char.* channels Auto-Tune
 * logs into are registered with the tuner, in tuning.cpp.
 *
 * Team 96671H: Hitmen
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
#include "robot/tune.hpp"

namespace robot {

using sapphirelib::chassis::AxisVolts;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::input::Axis;
using sapphirelib::input::Controller;
using sapphirelib::localization::BeamSample;
using sapphirelib::localization::LocalizerUpdate;
using sapphirelib::telemetry::Channel;
using sapphirelib::telemetry::Logger;

namespace {

// The "driver" channel, once startTelemetry() has made it.
Channel* driverLog = nullptr;

} // namespace

Logger& logger() {
    // A function-local static, like the devices: constructing a Logger only
    // sets up its tables, no tasks and no card access until start().
    //
    // Recording waits for the Home page's Start log button, so pit testing
    // doesn't fill the card with every power-on, and a match records itself:
    // plugging into a competition switch or the field starts a new file, and
    // it closes 5s after the tether comes out. Logging costs the control
    // loops nothing (rows are copied into rings; formatting and SD writes
    // happen on the lowest-priority task), and every file's H rows say what
    // the logger itself took (samp_us, fmt_us) next to the localizer's own
    // update time (mcl's "us"), to check that on the real brain.
    static Logger instance({.directory = "/usd/sl",
                            .robotName = "96671H",
                            .recordAtStart = false,
                            .recordUnderCompetition = true});
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

    // x, y, heading at 100Hz. The corrected pose, what every motion drives by.
    log.pose(odometry(), "odom", 10);

    // The localizer, one row per update (20Hz), recorded by the localizer's
    // own task as each update finishes:
    //  - "mcl": its estimate, how sure it is (spread in inches, effective
    //    particle count), how many of the four sensors gave a reading and how
    //    many of those agree with the walls, whether that update corrected
    //    odometry, the correction odometry is easing toward (how far raw
    //    odometry had drifted), how long the update took (us), and raw
    //    odometry's position, so drift can be measured against distance
    //    driven;
    //  - "mcl.beams": each sensor's reading (m, nan if it had none), what the
    //    map says it should read from the estimate (e, inf if nothing in
    //    range), and how fast it was closing on what it points at (v, in/s).
    //    What the simulator's Tune tab fits the sensor model, the latency and
    //    the tracking wheels' scale from (docs/TUNING.md).
    // Like the poll() sources, nothing while disabled under competition
    // control, when the robot sits still in the queue.
    Channel& mclLog = log.channel("mcl",
                                  {"x", "y", "spread", "neff", "used", "agree", "correcting",
                                   "corr_x", "corr_y", "us", "raw_x", "raw_y"},
                                  {.capacity = 64, .decimals = 2});
    Channel& beamLog = log.channel("mcl.beams",
                                   {"m0", "e0", "v0", "m1", "e1", "v1", "m2", "e2", "v2", "m3",
                                    "e3", "v3"},
                                   {.capacity = 64, .decimals = 2});
    localizer().setUpdateCallback([mcl = &mclLog, beams = &beamLog](const LocalizerUpdate& update) {
        const std::uint8_t competition = pros::competition::get_status();
        if ((competition & COMPETITION_DISABLED) != 0 &&
            (competition & COMPETITION_CONNECTED) != 0) {
            return;
        }
        const sapphirelib::localization::LocalizationStatus& status = update.status;
        mcl->record({status.estimate.xIn, status.estimate.yIn, status.spreadIn,
                     status.effectiveParticles, static_cast<double>(status.sensorsUsed),
                     static_cast<double>(status.sensorsAgreeing), status.correcting ? 1.0 : 0.0,
                     status.correctionXIn, status.correctionYIn,
                     static_cast<double>(status.updateUs), update.rawPose.xIn,
                     update.rawPose.yIn});
        // four sensors fill 12 of a row's 13 columns
        double values[12];
        for (std::size_t i = 0; i < 4; ++i) {
            const bool present = i < update.beams.size();
            const BeamSample beam = present ? update.beams[i] : BeamSample{};
            constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
            values[3 * i] = present ? beam.measuredIn : kNone;
            values[3 * i + 1] = present ? beam.expectedIn : kNone;
            values[3 * i + 2] = present ? beam.closingSpeedInPerS : kNone;
        }
        beams->record(values, 12);
    });

    // What the drivetrain actually commanded on each axis (the PIDs' "out" is
    // only the loop's share, before mixing and heading correction): the
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
    // charge, current drawn (A) and the pack's temperature, 5 times a second,
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
    // NaN, which the analyzer reports as "never answered", a reminder, not a
    // fault.
    log.motor("motor.intake", ports::kIntake);
    log.motor("motor.claw", ports::kClawMotor);

    // The driver's sticks and buttons, recorded by logDriver() each opcontrol
    // tick. Capacity 128 is 2.5s at 20ms.
    driverLog = &log.channel("driver", {"lx", "ly", "rx", "ry", "buttons", "connected"},
                             {.capacity = 128, .decimals = 3});

    // "lift" PID + "lift.act".
    macros::attachTelemetry(log);

    // #meta lines at the top of every file: which TUNE.CFG revision this run
    // used, and the localizer's settings and sensor mounts, so each log says
    // what produced it.
    describeTuning(log);

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
