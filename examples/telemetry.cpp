/**
 * \file examples/telemetry.cpp
 *
 * Phase 4 example: SD-card telemetry with telemetry::Logger, on a tank chassis
 * with one vertical tracking wheel. Logs every step of the drivetrain's PIDs,
 * the odometry pose, the volts the drivetrain commanded, every motor's health,
 * the battery, a custom channel recorded from driver control, and event
 * markers, one file per recording, for tuning and troubleshooting off the
 * robot. Recording starts with the Home page's Start log button, and on its
 * own whenever a competition switch or the field is plugged in. docs/TELEMETRY_FORMAT.md is the file format (and what each column
 * means for tuning); open the files in tools/analyzer/index.html to see what
 * went wrong in a match, replay it, and tune from it, or read them with
 * tools/telemetry/slt_read.py.
 *
 * Needs a FAT32 microSD card with an `sl` folder at its root. The V5 can't
 * create folders, so make it on a computer once; without it, files land in
 * the card's root instead.
 *
 * Not built into the program (the Makefile only compiles src/), but
 * `make check-examples` compiles it against the current headers so it can't
 * silently fall out of date. Copy the relevant pieces into your own files.
 *
 * Team 96671H: Hitmen
 */

#include <memory>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::TankDrivetrain;
using sapphirelib::gui::Gui;
using sapphirelib::gui::HomePage;
using sapphirelib::input::Axis;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
using sapphirelib::odom::Odometry;
using sapphirelib::odom::OdometryConfig;
using sapphirelib::odom::RotationTrackingWheel;
using sapphirelib::telemetry::Channel;
using sapphirelib::telemetry::Logger;

namespace {

constexpr std::uint8_t kImuPort = 10;
constexpr std::int8_t kVerticalWheelPort = 11;
constexpr double kTrackingWheelDiameterIn = 2.0;

Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(12);

/// Built on the first call, from initialize(): the constructor blocks while
/// the IMU calibrates, which is no job for static initialization.
TankDrivetrain& drivetrain() {
    static TankDrivetrain instance(
        /*leftPorts=*/{1, -2, 3}, /*rightPorts=*/{-4, 5, -6}, Gearset::green, kImuPort,
        DrivetrainConfig{
            .wheelDiameterIn = 3.25, .externalGearRatio = 1.0, .headingCorrectionKP = 0.4},
        /*drivePIDConfig=*/
        PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
        /*turnPIDConfig=*/
        PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0});
    return instance;
}

RotationTrackingWheel& verticalWheel() {
    static RotationTrackingWheel instance(kVerticalWheelPort, kTrackingWheelDiameterIn);
    return instance;
}

Odometry& odometry() {
    // Shares the drivetrain's IMU, so this builds the drivetrain first if
    // nothing has yet.
    static Odometry instance(
        Odometry::Sensors{.imu = &drivetrain().imu(), .vertical = &verticalWheel()},
        OdometryConfig{.verticalOffsetIn = 0.0});
    return instance;
}

/// Exactly one Logger per program, with static storage: its tasks run for the
/// rest of the program. `directory` and `robotName` are read for as long, so
/// they're string literals.
///
/// recordAtStart = false: nothing is recorded until the Home page's Start log
/// button (or startRecording()), so pit testing doesn't fill the card, and a
/// match records itself (recordUnderCompetition, on by default): a new file
/// when the competition switch or field connects, closed 5s after it's
/// unplugged. Leave recordAtStart at its default (true) to record every
/// program run from power-on instead.
Logger& logger() {
    static Logger instance(
        {.directory = "/usd/sl", .robotName = "1234A", .recordAtStart = false});
    return instance;
}

// Registered in initialize(), recorded into from opcontrol().
Channel* intakeLog = nullptr;

} // namespace

void initialize() {
    sapphirelib::initialize();

    // First, as in examples/gui.cpp, so the header is up while the IMU
    // calibrates.
    static Gui gui("SapphireLib - 1234A");

    drivetrain(); // blocks while the IMU calibrates
    odometry().startTask();
    drivetrain().setOdometry(&odometry());

    // Register every channel here, before start() and before any task runs
    // these PIDs: registration takes a mutex that a competition task deleted
    // mid-call would leave locked, and attaching a PID's observer isn't
    // synchronized with the loop running it.
    Logger& log = logger();

    // Every update() of each PID, plus its config and gains at the top of the
    // file and again whenever the gains change (a tuner page's edits
    // included).
    log.pid("drive", drivetrain().drivePID());
    log.pid("turn", drivetrain().turnPID());

    // x, y, heading every 10ms, as channel "odom".
    log.pose(odometry());

    // Polled on the logger's sampler task: what the drivetrain commanded on
    // each axis (the input side of the chassis's response, for fitting a
    // model offline), and the battery, whose sag skews any fit.
    log.poll("chassis", {"fwd_v", "strafe_v", "turn_v"}, 10, [](double* values) {
        const sapphirelib::chassis::AxisVolts volts = drivetrain().appliedAxisVolts();
        values[0] = volts.forward;
        values[1] = volts.strafe;
        values[2] = volts.turn;
    });
    log.poll("batt", {"volts", "pct"}, 1000, [](double* values) {
        values[0] = pros::battery::get_voltage() / 1000.0;
        values[1] = pros::battery::get_capacity();
    });

    // Each motor's volts, current, temperature, speed, efficiency and fault
    // bits, every 100ms: how a motor that overheated (V5 motors cut their own
    // current as they heat, and say so nowhere else), stalled, or came
    // unplugged mid-match shows up in the log. The analyzer finds motor
    // channels by these columns, so any names work.
    log.motor("motor.l1", 1);
    log.motor("motor.l2", -2);
    log.motor("motor.l3", 3);
    log.motor("motor.r1", -4);
    log.motor("motor.r2", 5);
    log.motor("motor.r3", -6);
    log.motor("motor.intake", 12);

    // A channel of your own, recorded wherever the values are computed.
    intakeLog = &log.channel("intake", {"volts", "rpm"});

    // Starts the sampler and writer tasks and returns at once. The card is
    // checked and the file opened on the writer task, so a missing card never
    // holds up initialize().
    log.start();

    // An "SD: logging SL000042 (button)" line on the home page, with a Start
    // log / Stop log button beside it, so a missing, full, or pulled card is
    // noticed in the pits rather than after the match. "SD: ready, not
    // logging" means the file is closed and the card is safe to pull.
    auto home = std::make_unique<HomePage>(&drivetrain().imu());
    home->setTelemetry(&log);
    gui.addPage(std::move(home));
    gui.start();
}

void autonomous() {
    // Markers the tuning app can cut the log on. They're formatted on the
    // calling task, so they're for moments, not data: never inside a 100Hz
    // loop. (Logger::event() does the same with a Logger in hand.)
    sapphirelib::telemetry::event("auton", "start,%s", "two points");
    // Each motion logs its own start and end events, and its PID steps land in
    // "drive" and "turn".
    drivetrain().moveToPoint(0.0, 24.0);
    drivetrain().moveToPoint(24.0, 24.0);
    sapphirelib::telemetry::event("auton", "end,%s", "two points");
}

void opcontrol() {
    while (true) {
        master.update();

        const double intakeVolts = master.held(Button::r1) ? 12.0 : 0.0;
        intake.move_voltage(static_cast<std::int32_t>(intakeVolts * 1000.0));
        // Never blocks or allocates: a row that can't be taken right now is
        // dropped and counted in the file instead.
        intakeLog->record({intakeVolts, intake.get_actual_velocity()});

        if (master.pressed(Button::x)) {
            // A driver's "that looked wrong" button, to find the moment later.
            sapphirelib::telemetry::event("mark", "driver");
        }

        drivetrain().arcade(master.axis(Axis::leftY), master.axis(Axis::rightX));
        pros::delay(20);
    }
}
