#include <memory>
#include <vector>

#include "main.h"
#include "sapphirelib/api.hpp"

using sapphirelib::PID;
using sapphirelib::chassis::AsteriskConfig;
using sapphirelib::chassis::DrivetrainConfig;
using sapphirelib::chassis::ExitConditions;
using sapphirelib::chassis::Gearset;
using sapphirelib::chassis::HolonomicDrivetrain;
using sapphirelib::diag::DeviceKind;
using sapphirelib::diag::SensorCheck;
using sapphirelib::gui::AutonSelectorPage;
using sapphirelib::gui::DiagnosticsPage;
using sapphirelib::gui::Gui;
using sapphirelib::gui::HomePage;
using sapphirelib::gui::OdometryPage;
using sapphirelib::gui::PidTunerPage;
using sapphirelib::motion::LocalOffset;
using sapphirelib::motion::toLocalFrame;
using sapphirelib::odom::Odometry;
using sapphirelib::odom::OdometryConfig;
using sapphirelib::odom::Pose;
using sapphirelib::odom::RotationTrackingWheel;
using sapphirelib::tuning::RelayTuneConfig;

namespace {

constexpr std::uint8_t kImuPort = 7;
constexpr double kJoystickCurve = 0.3;  // 0 = linear, 1 = full cubic

// TODO: measure your actual tracking wheels and set this to their real
// diameter (2.75in is just a common off-the-shelf omni size) — odometry
// distance is directly proportional to this value.
constexpr double kTrackingWheelDiameterIn = 2.75;
constexpr std::int8_t kVerticalTrackingPort = 13;
constexpr std::int8_t kHorizontalTrackingPort = 14;

HolonomicDrivetrain* drivetrain = nullptr;
Odometry* odometry = nullptr;
AutonSelectorPage* autonSelector = nullptr;
OdometryPage* odometryPage = nullptr;
PidTunerPage* pidTunerPage = nullptr;

void doNothingAuton() {}

void driveForwardAuton() { drivetrain->driveDistance(24); }

void turnTestingAuton() { drivetrain->turnToHeading(90);
						  pros::delay(1000);
						  drivetrain->turnToHeading(0); }

// Round-trip test motions for PidTunerPage — repeated "Run Test" taps don't
// walk the robot off the field, since each one returns to where it started.
void driveTuningTest() {
	drivetrain->driveDistance(24);
	drivetrain->driveDistance(0);
}

void turnTuningTest() {
	drivetrain->turnToHeading(90);
	drivetrain->turnToHeading(0);
}

// --- Ziegler-Nichols relay auto-tune configs for PidTunerPage ---
//
// Each factory below is called fresh every time "Auto-Tune" is tapped (not
// once at startup) — see PidTunerPage::addController()'s
// buildAutoTuneConfig doc comment for why: it lets `driveRelayConfig()`
// capture the chassis's current pose/heading as the relay experiment's
// reference frame each time, rather than a frame frozen at registration.
//
// TODO: relayAmplitude/setpointDelta/hysteresis below are conservative
// starting points, not measured for this robot — a relay amplitude too
// small to overcome friction/static load will make Auto-Tune report
// "no clean oscillation"; too large risks a rougher, more aggressive
// oscillation than necessary. Nudge them on the real robot and re-run.

constexpr double kDriveRelayAmplitudeVolts = 4.0;
constexpr double kDriveRelaySetpointIn = 12.0;
constexpr double kDriveRelayHysteresisIn = 0.5;

constexpr double kTurnRelayAmplitudeVolts = 4.0;
constexpr double kTurnRelaySetpointDeg = 90.0;
constexpr double kTurnRelayHysteresisDeg = 2.0;

constexpr std::uint32_t kAutoTuneTimeoutMs = 8000;

RelayTuneConfig driveRelayConfig() {
	// Captured by value into `measure` below, so every sample during this
	// one experiment reads distance traveled along *this* run's starting
	// heading — not the live heading, which open-loop relay driving (no
	// heading correction while the relay bypasses drivePID_) can let drift
	// slightly over the experiment.
	const Pose reference = odometry->getPose();

	return RelayTuneConfig{
	    .actuate = [](double v) { drivetrain->holonomic(v / 12.0, 0.0, 0.0); },
	    .measure =
	        [reference] {
		        const Pose pose = odometry->getPose();
		        const LocalOffset local = toLocalFrame(pose.xIn - reference.xIn, pose.yIn - reference.yIn,
		                                               reference.headingDeg);
		        return local.forwardIn;
	        },
	    .relayAmplitude = kDriveRelayAmplitudeVolts,
	    .setpointDelta = kDriveRelaySetpointIn,
	    .hysteresis = kDriveRelayHysteresisIn,
	    .timeoutMs = kAutoTuneTimeoutMs,
	};
}

RelayTuneConfig turnRelayConfig() {
	return RelayTuneConfig{
	    .actuate = [](double v) { drivetrain->holonomic(0.0, 0.0, v / 12.0); },
	    // Cumulative (unwrapped) heading, not getHeadingDeg()'s 0-360
	    // reading — so "start + setpointDelta" stays a simple, always-
	    // reachable linear target regardless of where the chassis happens
	    // to be pointed when Auto-Tune is tapped, instead of needing to
	    // dodge the 0/360 seam by hand.
	    .measure = [] { return drivetrain->imu().getCumulativeHeadingDeg(); },
	    .relayAmplitude = kTurnRelayAmplitudeVolts,
	    .setpointDelta = kTurnRelaySetpointDeg,
	    .hysteresis = kTurnRelayHysteresisDeg,
	    .timeoutMs = kAutoTuneTimeoutMs,
	};
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

	// SapphireLib's default brain-screen GUI — fully replaces LLEMU (which
	// wasn't loading here anyway). This is entirely opt-in: if you'd rather
	// build your own UI, just don't construct a Gui — nothing else in the
	// library depends on it. See sapphirelib::gui::Gui's class comment.
	//
	// Constructed first, ahead of any hardware: a Gui's constructor only
	// touches LVGL, so the branded header paints the moment the program
	// starts. If a device constructor below then blocks or faults (an IMU
	// that never finishes calibrating, a sensor in the wrong port), the
	// screen still shows that header instead of staying black — which is
	// the difference between "the GUI itself is broken" and "initialize()
	// never got far enough to build it". The stage logs below narrow it
	// the rest of the way over `pros terminal`.
	static Gui gui("SapphireLib - 96671H");
	SAPPHIRELIB_LOG_INFO("init", "GUI shell created");

	// Asterisk drivetrain: the standard 4 mecanum/X-drive corners plus a
	// straight-facing 5th/6th center wheel pair (middle-left/middle-right)
	// that add power during forward/backward driving and correct forward/
	// back drift while strafing — see AsteriskConfig and setDriftSource()
	// below. All right-side motors are physically mounted reversed.
	static HolonomicDrivetrain chassis(
	    /*frontLeftPort=*/-3, /*frontRightPort=*/8, /*backLeftPort=*/-17, /*backRightPort=*/10,
	    Gearset::green, kImuPort,
	    DrivetrainConfig{.wheelDiameterIn = 4.0, .externalGearRatio = 1.0, .headingCorrectionKP = 0.4},
	    // kI/kD are continuous-time gains (per second), not per control
	    // tick — see PIDGains' comment. The kD values below are the
	    // previous per-tick numbers (0.1 / 0.02) converted at the
	    // drivetrain's 10ms loop period, so they behave identically; run
	    // Auto-Tune to replace them with measured ones.
	    /*drivePIDConfig=*/
	    PID::Config{.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},
	    /*turnPIDConfig=*/
	    PID::Config{.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0},
	    /*imuHeadingScale=*/1.0,
	    /*asterisk=*/
	    // TODO: driftCorrectionKP starts conservative (volts per in/sec of
	    // sensed drift) — tune it up on the real robot until sideways drift
	    // is corrected without the center wheels fighting an intentional
	    // strafe.
	    AsteriskConfig{.middleLeftPort = -2, .middleRightPort = 9, .driftCorrectionKP = 0.5});
	drivetrain = &chassis;
	SAPPHIRELIB_LOG_INFO("init", "chassis ready (IMU calibrated)");
	// HolonomicDrivetrain zeros its field heading at construction time, so
	// holonomicFieldCentric() below is already field-centric from here on.

	// Dedicated tracking wheels on their own rotation sensors — the "IMU +
	// both wheels" odometry config (see Odometry's class comment).
	//
	// Vertical is reversed (negative port): on this chassis, the sensor's
	// raw positive direction is backward — confirmed by driving forward and
	// watching Y decrease instead of increase on the Odom page — so this
	// negation isn't a stylistic choice, it's required for Odometry's
	// forward/backward sign to actually match the chassis's forward
	// direction. If you re-mount or swap this sensor, re-check this.
	static RotationTrackingWheel verticalWheel(-kVerticalTrackingPort, kTrackingWheelDiameterIn);
	static RotationTrackingWheel horizontalWheel(kHorizontalTrackingPort, kTrackingWheelDiameterIn);
	static Odometry odom(
	    Odometry::Sensors{.imu = &chassis.imu(), .vertical = &verticalWheel, .horizontal = &horizontalWheel},
	    // TODO: verticalOffsetIn/horizontalOffsetIn default to 0 — measure
	    // (or run calibrateTrackingWheelOffsetIn()) for the real mounting
	    // offsets from the tracking center.
	    OdometryConfig{.verticalOffsetIn = 3.59, .horizontalOffsetIn = 4.18},
	    Pose{.xIn = 0.0, .yIn = 0.0, .headingDeg = 0.0});
	odometry = &odom;
	odometry->startTask();
	SAPPHIRELIB_LOG_INFO("init", "odometry task started");

	// Center wheels read the same vertical tracking wheel Odometry uses to
	// detect forward/back drift while strafing — reading a sensor from two
	// places is safe, only commanding a motor from two places would
	// conflict.
	chassis.setDriftSource(&verticalWheel);

	gui.addPage(std::make_unique<HomePage>(&chassis.imu()));

	auto autonSelectorPage = std::make_unique<AutonSelectorPage>();
	autonSelector = autonSelectorPage.get();
	autonSelector->addRoutine("Do Nothing", &doNothingAuton);
	autonSelector->addRoutine("Drive Forward", &driveForwardAuton);
	autonSelector->addRoutine("Turn Testing", &turnTestingAuton);
	gui.addPage(std::move(autonSelectorPage));

	// Re-checks these every tick (not just at startup) — a sensor that
	// works at power-on but gets knocked loose mid-match should still show
	// up. Ports here must match the ones passed to the chassis above.
	gui.addPage(std::make_unique<DiagnosticsPage>(
	    std::vector<SensorCheck>{
	        SensorCheck{.label = "Front-left drive motor", .port = -3, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Front-right drive motor", .port = 8, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Back-left drive motor", .port = -17, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Back-right drive motor", .port = 10, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Middle-left drive motor", .port = -2, .expected = DeviceKind::motor},
	        SensorCheck{.label = "Middle-right drive motor", .port = 9, .expected = DeviceKind::motor},
	        SensorCheck{.label = "IMU", .port = static_cast<std::int8_t>(kImuPort), .expected = DeviceKind::imu},
	        SensorCheck{.label = "Vertical tracking wheel", .port = kVerticalTrackingPort, .expected = DeviceKind::rotation},
	        SensorCheck{.label = "Horizontal tracking wheel", .port = kHorizontalTrackingPort, .expected = DeviceKind::rotation},
	    },
	    &gui));

	auto pidTunerPageOwned = std::make_unique<PidTunerPage>();
	pidTunerPage = pidTunerPageOwned.get();
	pidTunerPage->addController("Drive", chassis.drivePID(), &driveTuningTest, &driveRelayConfig);
	pidTunerPage->addController("Turn", chassis.turnPID(), &turnTuningTest, &turnRelayConfig);
	pidTunerPage->setTuningRule(sapphirelib::tuning::TuningRule::pdOnly);
	gui.addPage(std::move(pidTunerPageOwned));

	// fieldWidthIn/fieldHeightIn default to a 144x144in (12x12ft) VRC field
	// — pass your own for a different game/field size.
	auto odometryPageOwned = std::make_unique<OdometryPage>(*odometry);
	odometryPage = odometryPageOwned.get();
	// "Calibrate Offsets" button: spins the chassis in place (via the
	// drivetrain's own holonomic() turn axis, so it stays robot-centric
	// regardless of field heading) and derives verticalOffsetIn/
	// horizontalOffsetIn from how far the tracking wheels move over a known
	// rotation — replaces the hand-measured 3.5/5.5 placeholders above with
	// a calibrated value applied straight to the running Odometry.
	odometryPage->enableOffsetCalibration(
	    chassis.imu(), &verticalWheel, &horizontalWheel,
	    [](double turn) { drivetrain->holonomic(0.0, 0.0, turn); });
	gui.addPage(std::move(odometryPageOwned));
	SAPPHIRELIB_LOG_INFO("init", "all pages built");

	// Add your own pages here too — gui.addPage(std::make_unique<MyPage>(...))

	gui.start();
	SAPPHIRELIB_LOG_INFO("init", "initialize() complete");
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
		if ((master.get_digital_new_press(DIGITAL_B)) && (master.get_digital_new_press(DIGITAL_DOWN))) {
			autonSelector->run();
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

		// Skip driving from the sticks while a background GUI routine
		// (OdometryPage's offset calibration, or a PidTunerPage test/auto-
		// tune run) is driving the chassis on its own — otherwise this
		// loop's every-tick call below would immediately overwrite that
		// routine's commanded voltage with whatever the (likely centered)
		// sticks read, breaking the routine (see
		// OdometryPage::isCalibrating()'s comment for the failure mode in
		// detail).
		const bool backgroundRoutineActive = (odometryPage && odometryPage->isCalibrating()) ||
		                                     (pidTunerPage && pidTunerPage->isRunning());
		if (backgroundRoutineActive) {
			pros::delay(20);
			continue;
		}

		drivetrain->holonomicFieldCentric(fieldThrottle, fieldStrafe, turn);
		pros::delay(20);
	}
}