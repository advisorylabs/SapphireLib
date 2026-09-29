/**
 * \file driver.cpp
 *
 * Driver control: stick shaping, the drive call, and the B+DOWN testing
 * shortcut. The intake, claw, and lift buttons are in macros.cpp.
 *
 * Team 96671H — Hitmen
 */

#include "robot/driver.hpp"

#include "pros/misc.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"
#include "robot/screen.hpp"
#include "sapphirelib/control/joystick_curve.hpp"

namespace robot {

using sapphirelib::input::Axis;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;

namespace {

constexpr double kJoystickCurve = 0.3; // 0 = linear, 1 = full cubic

// Sampled once per opcontrol tick. At namespace scope (not a local in
// opcontrol()) so its button history survives opcontrol() restarting, as
// PROS's own new-press flags do. The controller drops text sent faster than
// about every 50ms across all its lines, so writes are spaced 60ms; and
// opcontrol ticks every 20ms (kDriverLoopMs), so a gap over 100ms means it
// stopped and restarted in between. Constructing it makes no PROS calls, so
// static initialization is safe.
Controller master(pros::E_CONTROLLER_MASTER, {.resumeGapMs = 100, .screenIntervalMs = 60});

} // namespace

Controller& controller() { return master; }

void drive(Controller& controller) {
    // Field-relative stick input: "forward" always means the field
    // heading captured at initialize() (or the last A press — see
    // shortcuts()), not the robot's nose.
    //
    // The right stick steers a *heading*, not a turn rate — see
    // holonomicFieldCentricHeadingHold() below. Curving it still makes
    // sense: it shapes how fast the stick sweeps that heading, so small
    // deflections give fine aim and large ones swing around quickly.
    //
    // axis() reads 0 while the controller is disconnected, so once the
    // drive call below is back, a dropped controller stops the chassis
    // instead of leaving it on its last stick reading.
    //
    // [[maybe_unused]] only because the drive call is disabled; drop it when
    // re-enabling.
    [[maybe_unused]] const double fieldThrottle =
        sapphirelib::curveJoystick(controller.axis(Axis::leftY), kJoystickCurve);
    [[maybe_unused]] const double fieldStrafe =
        sapphirelib::curveJoystick(controller.axis(Axis::leftX), kJoystickCurve);
    // const double turn =
    //     sapphirelib::curveJoystick(controller.axis(Axis::rightX), kJoystickCurve);

    // Heading-hold turning: the right stick moves a target heading and
    // the turn PID holds the chassis on it continuously, mixed in on top
    // of the translation rather than replacing it. Releasing the stick
    // leaves the bot pointed somewhere definite instead of drifting on
    // through, and a strafe that used to wander off-heading gets
    // straightened as it goes — including the wander an overheating
    // corner motor causes (see AsteriskConfig::thermalCompensation,
    // which attacks the same problem from the feedforward side).
    //
    // Tunable via drivetrain().setHeadingHold(); the defaults on
    // HeadingHoldConfig are the starting point. If turning feels
    // sluggish, raise maxLeadDeg before slewDegPerSec.
    //
    // DISABLED: driving from the sticks is switched off on this robot for
    // now — the robot can't drive in driver control. To drive again,
    // uncomment `turn` above and the call below, and drop the two
    // [[maybe_unused]]s.
    // drivetrain().holonomicFieldCentricHeadingHold(fieldThrottle, fieldStrafe, turn);
}

void shortcuts(Controller& controller) {
    if (controller.pressed(Button::a)) {
        // Redefine "forward" as whichever way the chassis is facing now.
        drivetrain().resetFieldHeading();
    }

    // Both buttons, pressed in either order: combo() is true on the one tick
    // the second of them goes down while the other is held.
    if (!controller.combo(Button::b, Button::down)) return;

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
