#include <cmath>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::ExitConditions;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;

namespace {

constexpr std::uint8_t kImuPort = 10;
constexpr double kJoystickCurve = 0.3;  // 0 = linear, 1 = full cubic
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

HolonomicDrivetrain* drivetrain = nullptr;

// Heading at initialize() time, i.e. "field 0 degrees" for headless control.
double startHeadingDeg = 0.0;

}  // namespace

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello Sapphire User!");

	pros::lcd::register_btn1_cb(on_center_button);

	sapphirelib::initialize();

	static HolonomicDrivetrain chassis(
	    /*frontLeftPort=*/-7, /*frontRightPort=*/4, /*backLeftPort=*/-8, /*backRightPort=*/3,
	    Gearset::green, kImuPort,
	    DrivetrainConfig{.wheelDiameterIn = 4.0, .externalGearRatio = 1.0, .headingCorrectionKP = 0.4},
	    /*drivePIDConfig=*/
	    PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.1}, .outputLimit = 12.0},
	    /*turnPIDConfig=*/
	    PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.02}, .outputLimit = 12.0});
	drivetrain = &chassis;
	startHeadingDeg = drivetrain->headingDeg();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void autonomous() {}

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Field-relative stick input: "forward" always means field 0 degrees
		// (the heading captured in initialize()), not the robot's nose.
		const double fieldThrottle =
		    sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_Y) / 127.0, kJoystickCurve);
		const double fieldStrafe =
		    sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_X) / 127.0, kJoystickCurve);
		const double turn =
		    sapphirelib::curveJoystick(master.get_analog(ANALOG_RIGHT_X) / 127.0, kJoystickCurve);

		// Rotate the field-relative stick vector into the robot's current
		// frame by the heading it has picked up since initialize() — see
		// HolonomicDrivetrain::headingDeg() for the clockwise-positive
		// convention this matches.
		const double headingDeltaRad =
		    sapphirelib::wrapDegrees180(drivetrain->headingDeg() - startHeadingDeg) * kDegToRad;
		const double cosHeading = std::cos(headingDeltaRad);
		const double sinHeading = std::sin(headingDeltaRad);
		const double throttle = fieldThrottle * cosHeading + fieldStrafe * sinHeading;
		const double strafe = -fieldThrottle * sinHeading + fieldStrafe * cosHeading;

		drivetrain->holonomic(throttle, strafe, turn);
		pros::delay(20);
	}
}