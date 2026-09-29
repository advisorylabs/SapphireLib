#include "sapphirelib/input/controller.hpp"

#include <algorithm>
#include <cstddef>

#include "pros/error.h"
#include "sapphirelib/util/clock.hpp"

namespace sapphirelib::input {

// update() turns an index into a PROS button/stick by adding it to the first
// one, which only works while Button and Axis list them in PROS's order.
static_assert(pros::E_CONTROLLER_DIGITAL_L1 + static_cast<int>(Button::a) ==
              pros::E_CONTROLLER_DIGITAL_A);
static_assert(pros::E_CONTROLLER_DIGITAL_L1 + static_cast<int>(Button::down) ==
              pros::E_CONTROLLER_DIGITAL_DOWN);
static_assert(pros::E_CONTROLLER_ANALOG_LEFT_X + static_cast<int>(Axis::rightY) ==
              pros::E_CONTROLLER_ANALOG_RIGHT_Y);

Controller::Controller(pros::controller_id_e_t id, ControllerConfig config)
    : raw_(id), screen_(config.screenIntervalMs), gap_(config.resumeGapMs) {}

void Controller::update() {
    now_ = sapphirelib::millis();
    resumed_ = gap_.update(now_);
    // Whatever was on the screen may have been lost or overwritten while we
    // weren't running, so resend it all.
    if (resumed_) screen_.invalidate();

    // is_connected() hands back VEXos's connection status: 0 offline, 1
    // tethered, 2 VEXnet — so "connected" is any non-zero status, not == 1
    // (which would read a wireless controller as unplugged all match).
    // PROS_ERR (non-zero too) only comes from an invalid controller id.
    const std::int32_t status = raw_.is_connected();
    connected_ = status != 0 && status != PROS_ERR;

    // One sample of every button, every tick, with get_digital() only: edges
    // come from comparing samples in ButtonTracker, so the kernel's
    // new-press flags are left alone for any raw() caller.
    ButtonMask mask = 0;
    for (std::size_t i = 0; i < kButtonCount; ++i) {
        const auto button = static_cast<Button>(i);
        const std::int32_t value = raw_.get_digital(
            static_cast<pros::controller_digital_e_t>(pros::E_CONTROLLER_DIGITAL_L1 + i));
        // PROS_ERR means "no reading"; keep last tick's state instead of
        // treating the error as pressed (PROS_ERR is non-zero). Unreachable in
        // practice — the kernel waits forever for the controller's port
        // mutex, so it only errors on an invalid id or button — but cheap.
        const bool down = value == PROS_ERR ? buttons_.held(button) : value != 0;
        if (down) mask = static_cast<ButtonMask>(mask | maskOf(button));
    }
    buttons_.update(mask, now_);

    for (std::size_t i = 0; i < axes_.size(); ++i) {
        const std::int32_t value = raw_.get_analog(
            static_cast<pros::controller_analog_e_t>(pros::E_CONTROLLER_ANALOG_LEFT_X + i));
        // Sticks read -127..127. Zeroed while disconnected, whatever VEXos
        // reports then, so nothing driven off a stick can keep running on
        // the last reading from a dropped controller; clamped so [-1, 1]
        // holds even for a value outside the documented range.
        axes_[i] = !connected_ || value == PROS_ERR ? 0.0 : std::clamp(value / 127.0, -1.0, 1.0);
    }
}

std::uint32_t Controller::now() const { return now_; }

bool Controller::resumed() const { return resumed_; }

bool Controller::connected() const { return connected_; }

double Controller::axis(Axis axis) const {
    const auto index = static_cast<std::size_t>(axis);
    return index < axes_.size() ? axes_[index] : 0.0;
}

void Controller::flushScreen() {
    const ControllerScreen::Write write = screen_.takeWrite(now_);
    std::int32_t result = PROS_ERR;
    if (write.kind == ControllerScreen::WriteKind::line) {
        // Padded to the full width, so a shorter line fully overwrites a
        // longer one.
        result = raw_.print(write.line, 0, "%-*s", static_cast<int>(ControllerScreen::kColumns),
                            write.text.data());
    } else if (write.kind == ControllerScreen::WriteKind::rumble) {
        result = raw_.rumble(write.text.data());
    } else {
        return; // nothing due this tick
    }
    // VEXos rejects text now and then (PROS_ERR, errno EAGAIN). Left
    // unconfirmed, the write comes up again once the interval has passed.
    if (result != PROS_ERR) screen_.confirm(write);
}

} // namespace sapphirelib::input
