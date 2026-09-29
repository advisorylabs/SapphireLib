/**
 * \file main.cpp
 *
 * PROS's competition callbacks, and nothing else: each one hands off to the
 * robot program in src/robot/ (ports in include/robot/config.hpp, devices,
 * screen, autons, tuning, telemetry, driver control, and the intake/claw/lift
 * macros).
 *
 * Team 96671H — Hitmen
 */

#include "main.h"
#include "robot/devices.hpp"
#include "robot/driver.hpp"
#include "robot/macros.hpp"
#include "robot/screen.hpp"
#include "robot/telemetry.hpp"
#include "sapphirelib/api.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    sapphirelib::initialize();

    // The GUI shell first, ahead of any hardware, so the branded header is
    // on the screen even if a device below blocks or faults — see screen().
    robot::screen();
    SAPPHIRELIB_LOG_INFO("init", "GUI shell created");

    // Blocks ~2-3s while the IMU calibrates. Also zeroes the lift, so start
    // the program with the lift all the way down.
    robot::initDevices();

    // Before anything can run a motion or opcontrol, since it attaches
    // observers to PIDs those would be running. Costs initialize() nothing:
    // the card is checked and the file opened on the logger's own task.
    robot::startTelemetry();
    SAPPHIRELIB_LOG_INFO("init", "telemetry started");

    // Pages need the devices (and the Home page the logger); also starts the
    // screen refreshing.
    robot::buildScreen();
    SAPPHIRELIB_LOG_INFO("init", "initialize() complete");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    // VEXos ignores motor commands while disabled anyway; this makes sure
    // nothing picks up where it left off at the next enable.
    robot::macros::stop();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() { robot::runSelectedAuton(); }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    sapphirelib::input::Controller& controller = robot::controller();

    while (true) {
        // One sample of every button and stick, and this tick's "now".
        controller.update();

        // Intake, claw, and lift buttons (R1/R2/L1/L2/Y/RIGHT/LEFT). Runs
        // ahead of the busy check below, so they keep working during a GUI
        // routine.
        robot::macros::update(controller);

        // Skip driving from the sticks while a GUI routine (the Odom page's
        // offset calibration, or a PID tuner test or Auto-Tune run) is
        // driving the chassis on its own — see robot::screenBusy().
        if (!robot::screenBusy()) robot::drive(controller);

        // A resets field-centric "forward"; B+DOWN runs the selected auton,
        // off the field only. Every tick, busy or not — see shortcuts().
        robot::shortcuts(controller);

        pros::delay(robot::kDriverLoopMs);
    }
}
