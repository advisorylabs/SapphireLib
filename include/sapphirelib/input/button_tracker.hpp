#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sapphirelib::input {

/**
 * @brief The controller's buttons, in PROS's order (E_CONTROLLER_DIGITAL_L1 + index)
 */
enum class Button : std::uint8_t { l1, l2, r1, r2, up, down, left, right, x, b, y, a };

inline constexpr std::size_t kButtonCount = 12;

/**
 * @brief One bit per button (bit i is Button i), so a whole controller sample fits in one value
 */
using ButtonMask = std::uint16_t;

/**
 * @brief Get the bit for a button
 *
 * @param button the button
 * @return ButtonMask its bit. A value outside the enum gets no bit, so it never reads as held
 */
constexpr ButtonMask maskOf(Button button) {
    const auto index = static_cast<unsigned>(button);
    return index < kButtonCount ? static_cast<ButtonMask>(1u << index) : ButtonMask{0};
}

/**
 * @brief Tracks button presses, releases, and hold times across ticks
 *
 * Feed it one sample per tick with update(). Unlike get_digital_new_press(), where the first
 * reader uses up the press, queries never change anything, so it doesn't matter which buttons you
 * ask about, how often, or from where
 *
 * @note not thread-safe, so update and query it from one task
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::input::ButtonTracker buttons;
 * // once per tick
 * buttons.update(heldMask, sapphirelib::millis());
 * if (buttons.pressed(sapphirelib::input::Button::a)) toggleClaw();
 * @endcode
 */
class ButtonTracker {
public:
    /**
     * @brief Record this tick's sample
     *
     * A button already down on the very first update counts as pressed, like
     * get_digital_new_press()
     *
     * @param heldMask a bit for every button down right now. Bits past the twelfth are ignored
     * @param nowMs when the sample was taken, in milliseconds
     */
    void update(ButtonMask heldMask, std::uint32_t nowMs);

    /**
     * @brief Get the time of the latest update(), in milliseconds
     */
    std::uint32_t nowMs() const;

    /**
     * @brief Whether a button is down
     */
    bool held(Button button) const;

    /**
     * @brief Whether a button was just pressed. True for exactly one update per press
     */
    bool pressed(Button button) const;

    /**
     * @brief Whether a button was just released. True for exactly one update per release
     */
    bool released(Button button) const;

    /**
     * @brief Get how long a button has been held
     *
     * @param button the button
     * @return std::uint32_t while held, time since the press. On the update it's released, how
     * long it was held, so a tap and a long hold can be told apart. Otherwise 0
     */
    std::uint32_t heldMs(Button button) const;

    /**
     * @brief Whether a button is held, and has been for at least `ms`
     */
    bool heldFor(Button button, std::uint32_t ms) const;

    /**
     * @brief Whether a button just reached `ms` of holding. True for exactly one update per press
     *
     * For "hold it to do the other thing" buttons
     *
     * @b Example
     * @code {.cpp}
     * // tap B to toggle the claw, hold it for half a second to reset the lift
     * if (master.longPressed(Button::b, 500)) resetLift();
     * @endcode
     */
    bool longPressed(Button button, std::uint32_t ms) const;

    /**
     * @brief Auto-repeat, like holding a key on a keyboard
     *
     * True on the press, then while held: once delayMs after the press and every periodMs after
     * that, at most once per update
     *
     * @param button the button
     * @param delayMs time before the first repeat, in milliseconds
     * @param periodMs time between repeats, in milliseconds. 0 disables repeats
     *
     * @b Example
     * @code {.cpp}
     * // step the lift up while Up is held: now, after 400ms, then every 150ms
     * if (master.repeated(Button::up, 400, 150)) ladder.up();
     * @endcode
     */
    bool repeated(Button button, std::uint32_t delayMs, std::uint32_t periodMs) const;

    /**
     * @brief Whether two buttons just became held together, in either order or at once
     */
    bool combo(Button first, Button second) const;

    /**
     * @brief Get every held button as a mask
     */
    ButtonMask heldMask() const;

    /**
     * @brief Get every just-pressed button as a mask
     */
    ButtonMask pressedMask() const;

    /**
     * @brief Get every just-released button as a mask
     */
    ButtonMask releasedMask() const;

private:
    ButtonMask held_ = 0;
    ButtonMask previous_ = 0;
    std::uint32_t nowMs_ = 0;
    std::uint32_t previousNowMs_ = 0;
    std::array<std::uint32_t, kButtonCount> pressedAtMs_{};
    std::array<std::uint32_t, kButtonCount> releasedHeldMs_{};
};

} // namespace sapphirelib::input
