#pragma once

#include <array>
#include <cstdint>

#include "pros/misc.hpp"
#include "sapphirelib/input/button_tracker.hpp"
#include "sapphirelib/input/controller_screen.hpp"
#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::input {

/**
 * @brief The controller's sticks
 */
enum class Axis : std::uint8_t { leftX, leftY, rightX, rightY };

/**
 * @brief Settings for a Controller
 */
struct ControllerConfig {
    /**
     * time between update() calls that counts as the loop restarting, in milliseconds. 100 by
     * default. Keep it well above your loop period
     */
    std::uint32_t resumeGapMs = 100;

    /**
     * minimum time between controller screen writes and rumbles, in milliseconds. 60 by default.
     * The controller drops text sent faster than about every 50ms
     */
    std::uint32_t screenIntervalMs = 60;
};

/**
 * @brief A V5 controller, sampled once per tick
 *
 * Tracks button presses and hold times, normalizes the sticks, and sends the screen and rumble
 * without flooding the controller
 *
 * @note declare it at namespace scope, never as a local in opcontrol(). Its button history has to
 * survive opcontrol() restarting, or a button held through a disable reads as a new press
 *
 * @note not thread-safe, so update, query, and flush it from one task
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::input::Controller master(pros::E_CONTROLLER_MASTER);
 *
 * void opcontrol() {
 *     while (true) {
 *         master.update();
 *         if (master.pressed(Button::a)) claw.toggle(master.now());
 *         master.screen().setLine(0, "LIFT %4.0f", lift.position());
 *         master.flushScreen();
 *         pros::delay(20);
 *     }
 * }
 * @endcode
 */
class Controller {
public:
    /**
     * @brief Construct a new Controller. Makes no PROS calls until update()
     *
     * @param id E_CONTROLLER_MASTER or E_CONTROLLER_PARTNER
     * @param config screen and resume settings
     */
    explicit Controller(pros::controller_id_e_t id, ControllerConfig config = {});

    /**
     * @brief Sample the buttons, sticks, and connection, and take this tick's time
     *
     * Call once at the start of each tick, before any query
     */
    void update();

    /**
     * @brief Get the time of the latest update(). Use it as the tick's one "now"
     *
     * @return std::uint32_t time, in milliseconds
     */
    std::uint32_t now() const;

    /**
     * @brief Whether the loop just started or restarted after a gap (autonomous, a disable, a
     * blocking routine)
     *
     * The screen already resends everything; drop your own stale state, like sequences and PID
     * memory
     */
    bool resumed() const;

    /**
     * @brief Whether the controller was connected at the latest update()
     */
    bool connected() const;

    // buttons, see ButtonTracker
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

    /**
     * @brief Get a stick
     *
     * @param axis the stick
     * @return double -1 to 1. 0 while disconnected
     *
     * @b Example
     * @code {.cpp}
     * double throttle = sapphirelib::applyDeadband(master.axis(Axis::leftY), 0.05);
     * @endcode
     */
    double axis(Axis axis) const;

    /**
     * @brief Get the controller screen
     */
    ControllerScreen& screen() { return screen_; }

    /**
     * @brief Queue a rumble pattern. See ControllerScreen::rumble()
     *
     * @param pattern '.' short, '-' long, ' ' pause
     */
    void rumble(const char* pattern) { screen_.rumble(pattern); }

    /**
     * @brief Send at most one pending screen write. Call once at the end of each tick
     */
    void flushScreen();

    /**
     * @brief Get the underlying pros::Controller
     */
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
