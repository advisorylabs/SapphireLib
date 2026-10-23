/**
 * \file driver.cpp
 *
 * Driver control: stick shaping, split arcade driving with its invert and
 * orbit modes, and the X+UP testing shortcut. The intake, claw, and lift
 * buttons are in macros.cpp.
 *
 * Team 96671H: Hitmen
 */

#include "robot/driver.hpp"

#include <cmath>

#include "pros/misc.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"
#include "robot/screen.hpp"
#include "robot/teach.hpp"
#include "sapphirelib/chassis/orbit_math.hpp"
#include "sapphirelib/control/joystick_curve.hpp"
#include "sapphirelib/util/timing.hpp"

namespace robot {

using sapphirelib::chassis::OrbitFrame;
using sapphirelib::chassis::OrbitPivot;
using sapphirelib::chassis::OrbitTranslation;
using sapphirelib::input::Axis;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;
using sapphirelib::odom::Pose;

namespace {

constexpr double kJoystickCurve = 0.3; // 0 = linear, 1 = full cubic
constexpr double kFullVolts = 12.0;

// --- Orbit mode (DOWN) ---
// Where the pivot starts when DOWN is pressed: this far straight behind the
// tracking center.
constexpr double kOrbitStartRadiusIn = 12.0;
// The closest the left stick can bring the robot to the pivot; pulling in
// past it pushes the pivot back too. The turn per inch of travel around the
// pivot is 1/radius, so a tiny radius would spin the robot on any touch of
// the right stick.
constexpr double kOrbitMinRadiusIn = 6.0;
// Holding the radius: volts back toward it per inch off. Going around fast,
// the robot's momentum carries it off the circle, and this brings it back;
// in the simulator, 2 kept a full-stick orbit within half an inch. TODO: tune
// on the robot. Too low and fast orbits wander out; too high and it hunts in
// and out at rest.
constexpr double kOrbitRadiusKP = 2.0;
// Left stick Y past this counts as the driver moving the radius, and the hold
// lets go until they stop, then holds wherever they left it.
constexpr double kOrbitRadiusDeadband = 0.05;
// A longer gap than this between orbit ticks means the drive call was skipped
// (a GUI routine had the chassis, maybe moving it), so the pivot is placed
// behind the robot again instead of driving back to wherever it was.
constexpr std::uint32_t kOrbitResumeGapMs = 100;

// Sampled once per opcontrol tick. At namespace scope (not a local in
// opcontrol()) so its button history survives opcontrol() restarting, as
// PROS's own new-press flags do. The controller drops text sent faster than
// about every 50ms across all its lines, so writes are spaced 60ms; and
// opcontrol ticks every 20ms (kDriverLoopMs), so a gap over 100ms means it
// stopped and restarted in between. Constructing it makes no PROS calls, so
// static initialization is safe.
Controller master(pros::E_CONTROLLER_MASTER, {.resumeGapMs = 100, .screenIntervalMs = 60});

// B: front and back swapped. Kept across restarts, like the controller.
bool inverted = false;

// DOWN: orbiting a pivot behind the robot, a fixed point in trackedPose()'s
// frame.
struct Orbit {
    bool active = false;
    OrbitPivot pivot;
    // The radius the driver left it at, which the hold keeps it at.
    double targetRadiusIn = kOrbitStartRadiusIn;
    std::uint32_t lastMs = 0;
    sapphirelib::GapDetector gap{kOrbitResumeGapMs}; // a fresh one reports a gap
};

Orbit orbit;

// Where the robot is by the tracking wheels and IMU alone. Never getPose():
// that adds the localizer's corrections, which are for autonomous motions and
// drift during driver control, and would drag the pivot around with them.
Pose trackedPose() { return odometry().snapshot().rawPose; }

double stick(const Controller& controller, Axis axis) {
    return sapphirelib::curveJoystick(controller.axis(axis), kJoystickCurve);
}

// Holonomic split arcade: the left stick drives and strafes, the right stick
// turns. Robot-centric, so inverting can swap front and back: forward and
// right both flip, while turning doesn't, since clockwise is clockwise
// whichever end leads.
//
// axis() reads 0 while the controller is disconnected, so a dropped
// controller stops the chassis instead of leaving it on its last reading.
void driveArcade(const Controller& controller) {
    const double direction = inverted ? -1.0 : 1.0;
    drivetrain().holonomic(direction * stick(controller, Axis::leftY),
                           direction * stick(controller, Axis::leftX),
                           stick(controller, Axis::rightX));
}

// Orbiting: the right stick carries the robot around the pivot and the left
// stick's Y moves it straight away from or toward it, both measured against
// the real circle (both tracking wheels, through trackedPose()), so the pivot
// stays put. The heading hold PID turns the robot to keep its back to the
// pivot; when the turn lags on a fast orbit, the drive is steered along the
// circle anyway (see orbitTranslation()), so the lag only shows as the robot
// not quite facing away yet. In the driver's frame, so inverting puts the
// pivot ahead of the robot's physical front instead.
void driveOrbit(const Controller& controller) {
    const std::uint32_t now = controller.now();
    const Pose pose = trackedPose();
    const bool fresh = orbit.gap.update(now);
    if (fresh) {
        // The first tick, or the first since a GUI routine had the chassis:
        // the pivot goes straight behind the robot, at the radius it had.
        orbit.pivot = sapphirelib::chassis::pivotBehind(pose, orbit.targetRadiusIn, inverted);
        drivetrain().headingHoldPID().reset();
    }

    const OrbitFrame frame =
        sapphirelib::chassis::measureOrbit(orbit.pivot, pose, inverted, kOrbitMinRadiusIn);
    orbit.pivot = frame.pivot;

    const double radial = stick(controller, Axis::leftY);
    if (std::fabs(radial) > kOrbitRadiusDeadband) orbit.targetRadiusIn = frame.radiusIn;
    const OrbitTranslation move = sapphirelib::chassis::orbitTranslation(
        radial * kFullVolts + kOrbitRadiusKP * (orbit.targetRadiusIn - frame.radiusIn),
        stick(controller, Axis::rightX) * kFullVolts, frame.headingErrorDeg, inverted);

    // The error as the target and 0 as the measurement, like the drivetrain's
    // own heading hold. Skipped on a fresh tick, which has no time step yet
    // (and, with the pivot just placed straight behind, no error either).
    const double turnVolts =
        fresh ? 0.0
              : drivetrain().headingHoldPID().update(
                    frame.headingErrorDeg, 0.0, sapphirelib::elapsedMs(orbit.lastMs, now) / 1000.0);
    orbit.lastMs = now;

    drivetrain().holonomicVolts(move.forward, move.strafe, turnVolts);
}

// Controller line 1 (macros.cpp has 0 and 2): which way is forward, or the
// radius the orbit is holding. Whole inches, since a tenth would change
// nearly every tick while the radius moves and, with one line sent per 60ms
// lowest first, starve the lift readout below it.
void showDriveMode(Controller& controller) {
    // Teaching takes the line while it records, so the driver can see it is;
    // the drive mode comes back when it stops.
    if (teach::recording()) {
        controller.screen().setLine(1, "TEACH REC: X+A"); // 15 columns at most
    } else if (orbit.active) {
        controller.screen().setLine(1, "%s %.0fin", inverted ? "ORBIT INV" : "ORBIT",
                                    orbit.targetRadiusIn);
    } else {
        controller.screen().setLine(1, "DRIVE %s", inverted ? "INVERTED" : "NORMAL");
    }
}

} // namespace

Controller& controller() { return master; }

void drive(Controller& controller) {
    // After autonomous, a disable, or a blocking routine, the robot comes
    // back driving normally, never still orbiting a pivot from before.
    if (controller.resumed()) orbit.active = false;

    if (controller.pressed(Button::b)) {
        inverted = !inverted;
        // Mid-orbit, the pivot moves behind the new back, rather than the
        // robot spinning around to put its new back to the old one.
        if (orbit.active) {
            orbit.pivot =
                sapphirelib::chassis::pivotBehind(trackedPose(), orbit.targetRadiusIn, inverted);
        }
    }
    if (controller.pressed(Button::down)) {
        if (orbit.active) {
            orbit.active = false;
        } else {
            orbit = Orbit{.active = true};
        }
    }

    if (orbit.active) {
        driveOrbit(controller);
    } else {
        driveArcade(controller);
    }
    showDriveMode(controller);
}

void shortcuts(Controller& controller) {
    // Both buttons, pressed in either order: combo() is true on the one tick
    // the second of them goes down while the other is held. Neither does
    // anything on its own, so this can't fire by accident mid-drive.
    if (!controller.combo(Button::x, Button::up)) return;

    // Only on the bench. On a competition switch or field, autonomous()
    // runs the routine for real, and this would block the driver period for
    // as long as the routine takes.
    if (pros::competition::is_connected()) return;

    // Nor on top of a GUI routine that's driving the chassis (a PID tuner
    // test or Auto-Tune run, the Odom page's calibration spin): the two would
    // fight over the drivetrain, spoiling both.
    if (screenBusy()) return;

    // Nothing calls macros::update() until the routine returns, and the
    // mechanisms would otherwise keep their last command the whole time.
    macros::stop();
    runSelectedAuton();
}

} // namespace robot
