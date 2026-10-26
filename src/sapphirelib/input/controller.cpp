#include "sapphirelib/input/controller.hpp"

#include <algorithm>
#include <cstddef>

#include "pros/error.h"
#include "sapphirelib/util/clock.hpp"

namespace sapphirelib::input {

// update() turns an index into a PROS button or stick by adding it to the first one, which
// needs Button and Axis in PROS's order
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
    // the screen may have been lost or overwritten while we weren't running, so resend it all
    if (resumed_) screen_.invalidate();

    // is_connected() is 0 offline, 1 tethered, 2 VEXnet, so connected is any non-zero status.
    // PROS_ERR only comes from a bad controller id
    const std::int32_t status = raw_.is_connected();
    connected_ = status != 0 && status != PROS_ERR;

    // sample every button with get_digital() only. Edges come from ButtonTracker, so the kernel's
    // new-press flags are left alone for raw() callers
    ButtonMask mask = 0;
    for (std::size_t i = 0; i < kButtonCount; ++i) {
        const auto button = static_cast<Button>(i);
        const std::int32_t value = raw_.get_digital(
            static_cast<pros::controller_digital_e_t>(pros::E_CONTROLLER_DIGITAL_L1 + i));
        // PROS_ERR means no reading, so keep last tick's state instead of reading it as pressed
        const bool down = value == PROS_ERR ? buttons_.held(button) : value != 0;
        if (down) mask = static_cast<ButtonMask>(mask | maskOf(button));
    }
    buttons_.update(mask, now_);

    for (std::size_t i = 0; i < axes_.size(); ++i) {
        const std::int32_t value = raw_.get_analog(
            static_cast<pros::controller_analog_e_t>(pros::E_CONTROLLER_ANALOG_LEFT_X + i));
        // sticks read -127 to 127. Zeroed while disconnected, so nothing keeps running on a dropped
        // controller's last reading
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
        // padded to the full width, so a shorter line fully covers a longer one
        result = raw_.print(write.line, 0, "%-*s", static_cast<int>(ControllerScreen::kColumns),
                            write.text.data());
    } else if (write.kind == ControllerScreen::WriteKind::rumble) {
        result = raw_.rumble(write.text.data());
    } else {
        return; // nothing due this tick
    }
    // VEXos rejects text now and then. Left unconfirmed, the write comes up again
    if (result != PROS_ERR) screen_.confirm(write);
}

} // namespace sapphirelib::input
