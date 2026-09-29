/**
 * \file sapphirelib/input/controller.hpp
 *
 * A V5 controller sampled once per tick: button edges and hold times
 * (ButtonTracker), normalized sticks, the tick's timestamp, detection of the
 * loop having stopped and restarted, and a throttled text screen plus rumble
 * (ControllerScreen). The PROS-facing shell around those pure parts.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <array>
#include <cstdint>

#include "pros/misc.hpp"
#include "sapphirelib/input/button_tracker.hpp"
#include "sapphirelib/input/controller_screen.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::input {

enum class Axis : std::uint8_t { leftX, leftY, rightX, rightY };

/// At namespace scope, not nested in Controller, so it can be a `= {}`
/// default argument (GCC rejects that for a nested struct with default
/// member initializers).
struct ControllerConfig {
    /// update() calls further apart than this count as a restart: resumed()
    /// reports it and the screen resends everything. Must be comfortably longer
    /// than your loop period (the default is 5 x a 20ms opcontrol tick).
    std::uint32_t resumeGapMs = 100;

    /// Minimum spacing between controller writes. Text lines and rumble share
    /// it, because the controller drops text sent faster than about every 50ms
    /// across all lines.
    std::uint32_t screenIntervalMs = 60;
};

/// One update() at the top of each tick, one flushScreen() at the end:
///
///   sapphirelib::input::Controller master(pros::E_CONTROLLER_MASTER); // namespace scope
///
///   void opcontrol() {
///       while (true) {
///           master.update();
///           if (master.pressed(Button::a)) ...;
///           master.screen().setLine(0, "LIFT %4.0f", lift.position());
///           master.flushScreen();
///           pros::delay(20);
///       }
///   }
///
/// **Declare it at namespace scope (or `static`), never as a local in
/// opcontrol().** Its button history has to survive opcontrol() restarting,
/// the way PROS's own new-press flags do (they're kernel globals, not part of
/// a pros::Controller object). PROS deletes and recreates the opcontrol task
/// on every disable/enable, so a local Controller starts over with "nothing
/// was held", and a button held through a disable/enable reads as a fresh
/// press on the first tick back — holding L1 through a re-enable would raise
/// the lift. Constructing one is safe during static initialization: it makes
/// no PROS calls until update().
///
/// Buttons are read with get_digital() only, never get_digital_new_press(),
/// so code still calling raw().get_digital_new_press() keeps working
/// unchanged; this class never consumes those flags.
///
/// Not thread-safe: update, query, and flush it from one task.
class Controller {
public:
    explicit Controller(pros::controller_id_e_t id, ControllerConfig config = {});

    /// Samples all twelve buttons, the four sticks and the connection state,
    /// and takes this tick's timestamp. Call once per tick, before any query.
    void update();

    /// sapphirelib::millis() as of the latest update(). Use it as the tick's
    /// one "now" for everything else that takes a time.
    std::uint32_t now() const;

    /// True on the first update() ever, and after any gap longer than
    /// ControllerConfig::resumeGapMs: the loop was stopped (autonomous,
    /// disabled, a blocking routine) and has restarted. The screen has already
    /// been told to resend everything; drop your own stale state (sequences,
    /// PID memory).
    bool resumed() const;

    /// Whether the controller was connected (over VEXnet or tethered) at the
    /// latest update().
    bool connected() const;

    // Buttons (see ButtonTracker for the exact semantics).
    bool held(Button button) const { return buttons_.held(button); }
    bool pressed(Button button) const { return buttons_.pressed(button); }
    bool released(Button button) const { return buttons_.released(button); }
    std::uint32_t heldMs(Button button) const { return buttons_.heldMs(button); }
    bool heldFor(Button button, std::uint32_t ms) const { return buttons_.heldFor(button, ms); }
    bool longPressed(Button button, std::uint32_t ms) const {
        return buttons_.longPressed(button, ms);
    }
    bool repeated(Button button, std::uint32_t delayMs, std::uint32_t periodMs) const {
        return buttons_.repeated(button, delayMs, periodMs);
    }
    bool combo(Button first, Button second) const { return buttons_.combo(first, second); }
    const ButtonTracker& buttons() const { return buttons_; }

    /// A stick, normalized to [-1, 1] (raw / 127). 0 while disconnected. Shape
    /// it with applyDeadband() and curveJoystick() (control/joystick_curve.hpp).
    double axis(Axis axis) const;

    ControllerScreen& screen() { return screen_; }

    /// Queues a rumble pattern; see ControllerScreen::rumble().
    void rumble(const char* pattern) { screen_.rumble(pattern); }

    /// Sends at most one pending screen write (see ControllerScreen::takeWrite()),
    /// timed by now(). Call once at the end of each tick, after setting that
    /// tick's lines.
    void flushScreen();

    /// The underlying pros::Controller, for anything not wrapped here.
    pros::Controller& raw() { return raw_; }

private:
    pros::Controller raw_;
    ButtonTracker buttons_;
    ControllerScreen screen_;
    GapDetector gap_;
    std::array<double, 4> axes_{};
    std::uint32_t now_ = 0;
    bool resumed_ = false;
    bool connected_ = false;
};

} // namespace sapphirelib::input
