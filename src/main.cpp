#include <cmath>
#include <memory>
#include <vector>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::ExitConditions;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::chassis::MotorGroup;
using sapphirelib::diag::DeviceKind;
using sapphirelib::diag::SensorCheck;
using sapphirelib::gui::AutonSelectorPage;
using sapphirelib::gui::DiagnosticsPage;
using sapphirelib::gui::Gui;
using sapphirelib::gui::HomePage;
using sapphirelib::gui::OdometryPage;
using sapphirelib::gui::PidTunerPage;
using sapphirelib::odom::MotorGroupTrackingWheel;
using sapphirelib::odom::Odometry;
using sapphirelib::odom::OdometryConfig;
using sapphirelib::odom::Pose;
using sapphirelib::tuning::autoTuneCost;
using sapphirelib::tuning::measureOvershoot;
using sapphirelib::tuning::runAndSample;

namespace {

constexpr std::uint8_t kImuPort = 10;
constexpr double kJoystickCurve = 0.3;  // 0 = linear, 1 = full cubic
constexpr double kWheelDiameterIn = 4.0;

HolonomicDrivetrain* drivetrain = nullptr;
Odometry* odometry = nullptr;
AutonSelectorPage* autonSelector = nullptr;

void doNothingAuton() {}

void driveForwardAuton() { drivetrain->driveDistance(24.0); }

// Round-trip test motions for PidTunerPage — repeated "Run Test" taps don't
// walk the robot off the field, since each one returns to where it started.
void driveTuningTest() {
	drivetrain->driveDistance(24.0);
	drivetrain->driveDistance(-24.0);
}

void turnTuningTest() {
	drivetrain->turnToHeading(90.0);
	drivetrain->turnToHeading(0.0);
}

// --- Auto-tune cost callbacks for PidTunerPage ---
//
// Each "leg" runs one bounded motion while sampling odometry, measures how
// far it overshot the target (plus how far off it finished, in case it
// undershot instead), and turns that into a single cost the Twiddle search
// tries to minimize. The drive/turn cycles below each alternate between two
// legs that return to roughly where they started, so repeated auto-tune
// trials don't walk the robot off the field or spin it away from its
// starting heading.

constexpr double kDriveTestDistanceIn = 24.0;  // ~2ft, alternating +/-
constexpr double kTurnTestHighHeadingDeg = 90.0;
constexpr double kTurnTestLowHeadingDeg = 30.0;  // stays clear of the 0/360 seam
constexpr double kAutoTuneFinalErrorWeight = 4.0;

double measureDriveLegCost(double distanceIn) {
	const Pose start = odometry->getPose();
	const std::vector<double> samples = runAndSample(
	    [&] { drivetrain->driveDistance(distanceIn); },
	    [&] {
		    const Pose pose = odometry->getPose();
		    return std::hypot(pose.xIn - start.xIn, pose.yIn - start.yIn);
	    });
	return autoTuneCost(measureOvershoot(samples, std::fabs(distanceIn)), kAutoTuneFinalErrorWeight);
}

double driveAutoTuneCost() {
	return measureDriveLegCost(kDriveTestDistanceIn) + measureDriveLegCost(-kDriveTestDistanceIn);
}

double measureTurnLegCost(double headingDeg) {
	const std::vector<double> samples = runAndSample([&] { drivetrain->turnToHeading(headingDeg); },
	                                                  [&] { return odometry->getPose().headingDeg; });
	return autoTuneCost(measureOvershoot(samples, headingDeg), kAutoTuneFinalErrorWeight);
}

double turnAutoTuneCost() {
	return measureTurnLegCost(kTurnTestHighHeadingDeg) + measureTurnLegCost(kTurnTestLowHeadingDeg);
}

}  // namespace

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
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
	// HolonomicDrivetrain zeros its field heading at construction time, so
	// holonomicFieldCentric() below is already field-centric from here on.

	// Odometry needs a forward-distance source of its own — since
	// HolonomicDrivetrain doesn't expose its internal motor groups, this
	// reads the front-left drive motor a second time (safe: reading a
	// motor's position from more than one object is fine, only *commanding*
	// it from more than one would conflict) as the "IMU + drive encoders
	// only" fallback config. Swap in a RotationTrackingWheel instead if you
	// wire up a dedicated tracking wheel — see docs/ROADMAP.md Phase 2.
	static MotorGroup forwardEncoder({-7}, Gearset::green);
	static MotorGroupTrackingWheel forwardWheel(forwardEncoder, kWheelDiameterIn);
	static Odometry odom(
	    Odometry::Sensors{.imu = &chassis.imu(), .vertical = &forwardWheel, .horizontal = nullptr},
	    OdometryConfig{.verticalOffsetIn = 0.0}, Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0});
	odometry = &odom;
	odometry->startTask();

	// SapphireLib's default brain-screen GUI — fully replaces LLEMU (which
	// wasn't loading here anyway). This is entirely opt-in: if you'd rather
	// build your own UI, just don't construct a Gui — nothing else in the
	// library depends on it. See sapphirelib::gui::Gui's class comment.
	static Gui gui("SapphireLib - 96671H");

	gui.addPage(std::make_unique<HomePage>(&chassis.imu()));

	auto autonSelectorPage = std::make_unique<AutonSelectorPage>();
	autonSelector = autonSelectorPage.get();
	autonSelector->addRoutine("Do Nothing", &doNothingAuton);
	autonSelector->addRoutine("Drive Forward", &driveForwardAuton);
	gui.addPage(std::move(autonSelectorPage));

	// Re-checks these every tick (not just at startup) — a sensor that
	// works at power-on but gets knocked loose mid-match should still show
	// up. Ports here must match the ones passed to the chassis above.
	gui.addPage(std::make_unique<DiagnosticsPage>(
	    std::vector<SensorCheck>{
	        SensorCheck{.label = "Front-left drive motor", .port = -7, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Front-right drive motor", .port = 4, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Back-left drive motor", .port = -8, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Back-right drive motor", .port = 3, .expected = DeviceKind::motor},
	        SensorCheck{.label = "IMU", .port = static_cast<std::int8_t>(kImuPort), .expected = DeviceKind::imu},
	    },
	    &gui));

	auto pidTunerPage = std::make_unique<PidTunerPage>();
	pidTunerPage->addController("Drive", chassis.drivePID(), &driveTuningTest, &driveAutoTuneCost);
	pidTunerPage->addController("Turn", chassis.turnPID(), &turnTuningTest, &turnAutoTuneCost);
	gui.addPage(std::move(pidTunerPage));

	// fieldWidthIn/fieldHeightIn default to a 144x144in (12x12ft) VRC field
	// — pass your own for a different game/field size.
	gui.addPage(std::make_unique<OdometryPage>(*odometry));

	// Add your own pages here too — gui.addPage(std::make_unique<MyPage>(...))

	gui.start();
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
void autonomous() {
	if (autonSelector) autonSelector->run();
}

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
		if (master.get_digital_new_press(DIGITAL_A)) {
			// Redefine "forward" as whichever way the chassis is facing now.
			drivetrain->resetFieldHeading();
		}

		// Field-relative stick input: "forward" always means the field
		// heading captured at initialize() (or the last resetFieldHeading()
		// press above), not the robot's nose.
		const double fieldThrottle =
		    sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_Y) / 127.0, kJoystickCurve);
		const double fieldStrafe =
		    sapphirelib::curveJoystick(master.get_analog(ANALOG_LEFT_X) / 127.0, kJoystickCurve);
		const double turn =
		    sapphirelib::curveJoystick(master.get_analog(ANALOG_RIGHT_X) / 127.0, kJoystickCurve);

		drivetrain->holonomicFieldCentric(fieldThrottle, fieldStrafe, turn);
		pros::delay(20);
	}
}