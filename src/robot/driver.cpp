/**
 * \file driver.cpp
 *
 * Driver control: stick shaping, split arcade driving with its invert mode,
 * and the X+UP testing shortcut. The claw, lift, and extender buttons are in
 * macros.cpp.
 *
 * Team 96671H: Hitmen
 */

#include "robot/driver.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

#include "pros/misc.hpp"
#include "robot/autocycle.hpp"
#include "robot/config.hpp"
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

// B: front and back swapped. Kept across restarts, like the controller.
bool inverted = false;

double stick(const Controller& controller, Axis axis) {
    return sapphirelib::curveJoystick(controller.axis(axis), kJoystickCurve);
}

// Practice mode, set from the Home page's button on the GUI's task, read here
// on opcontrol's. Off at startup.
std::atomic<bool> practice{false};
// Whether practice mode has the drive's power cut right now.
bool practiceCut = false;

// Whether practice mode cuts the drive this tick: any drive stick past
// kPracticeMaxThrottlePct, after the curve, so it's the power the motors
// would have been asked for. Rumbles once as it cuts.
bool practiceCutsPower(Controller& controller) {
    if (!practice.load()) {
        practiceCut = false;
        return false;
    }
    const double largest =
        std::max({std::fabs(stick(controller, Axis::leftY)),
                  std::fabs(stick(controller, Axis::leftX)),
                  std::fabs(stick(controller, Axis::rightX))});
    const bool over = largest * 100.0 > kPracticeMaxThrottlePct;
    if (over && !practiceCut) controller.rumble(".");
    practiceCut = over;
    return over;
}

// Holonomic split arcade: the left stick drives and strafes, the right stick
// turns. Robot-centric, so inverting can swap front and back: forward and
// right both flip, while turning doesn't, since clockwise is clockwise
// whichever end leads.
//
// axis() reads 0 while the controller is disconnected, so a dropped
// controller stops the chassis instead of leaving it on its last reading.
void driveArcade(const Controller& controller, double maxVolts) {
    const double direction = inverted ? -1.0 : 1.0;
    drivetrain().holonomic(direction * stick(controller, Axis::leftY),
                           direction * stick(controller, Axis::leftX),
                           stick(controller, Axis::rightX), maxVolts);
}

// Controller line 1 (macros.cpp has 0 and 2): which way is forward.
void showDriveMode(Controller& controller) {
    if (practiceCut) {
        controller.screen().setLine(1, "PRACTICE: SLOW");
    } else {
        controller.screen().setLine(1, "DRIVE %s", inverted ? "INVERTED" : "NORMAL");
    }
}

} // namespace

Controller& controller() { return master; }

void setPracticeMode(bool on) { practice.store(on); }

bool practiceMode() { return practice.load(); }

void drive(Controller& controller) {
    if (controller.pressed(Button::b)) inverted = !inverted;

    if (practiceCutsPower(controller)) {
        // Zero volts, so the drive coasts, as with the power off.
        drivetrain().holonomicVolts(0.0, 0.0, 0.0);
    } else {
        driveArcade(controller, macros::driveMaxVolts());
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

    // Nor alongside the auto cycle: the routine's first motion would wait
    // for the cycle's to finish, and the cycle waits on the driver's buttons,
    // which nothing reads until the routine returns.
    autocycle::cancel();

    // Nothing calls macros::update() until the routine returns, and the
    // lift would otherwise keep its last command the whole time.
    macros::stop();
    runSelectedAuton();
}

} // namespace robot
