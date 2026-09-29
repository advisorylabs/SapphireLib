/**
 * \file sapphirelib/input/button_tracker.hpp
 *
 * Edge and hold-time tracking for the V5 controller's twelve buttons, from
 * one sample of all of them per tick. Pure: no PROS, no clock. The sample
 * and its timestamp are handed in (input::Controller does that on the
 * robot). See tests/input/button_tracker_test.cpp.
 *
 * Why not get_digital_new_press()? PROS keeps one "already seen" flag per
 * button and updates it only when that button is *read*. Code that skips
 * reading a button for a while gets a press delivered late, and two places
 * reading the same button steal it from each other. Comparing this tick's
 * sample with last tick's has neither problem: every query is a pure
 * function of the two samples, so asking or not asking changes nothing.
 * Sampled every tick, it reports exactly what get_digital_new_press() would
 * have if every button were read every tick (tested against a copy of the
 * kernel's logic).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sapphirelib::input {

/// The controller's buttons, in PROS's order (E_CONTROLLER_DIGITAL_L1 + index).
enum class Button : std::uint8_t { l1, l2, r1, r2, up, down, left, right, x, b, y, a };

inline constexpr std::size_t kButtonCount = 12;

/// One bit per Button (bit i = Button i): a whole controller sample in one value.
using ButtonMask = std::uint16_t;

/// The bit for `button`. A value cast in from outside the enum gets no bit at
/// all (rather than an oversized shift), so it simply never reads as held.
constexpr ButtonMask maskOf(Button button) {
    const auto index = static_cast<unsigned>(button);
    return index < kButtonCount ? static_cast<ButtonMask>(1u << index) : ButtonMask{0};
}

/// Tracks presses, releases and hold times across ticks. Feed it one sample
/// per tick with update(); every query answers for the latest sample.
/// Timing queries resolve to your tick: heldFor(b, 500) turns true on the
/// first update at least 500ms after the press was sampled.
///
/// Queries never change anything, so it doesn't matter which buttons you
/// ask about, how often, or from how many places in the tick — unlike
/// get_digital_new_press(), where the first reader consumes the press.
///
/// Not thread-safe: update and query it from one task.
class ButtonTracker {
public:
    /// Records this tick's sample. `heldMask` has a bit for every button down
    /// right now (bits past the twelfth are ignored); `nowMs` is when it was
    /// taken. A button already down on the very first update counts as
    /// pressed on it, the same as get_digital_new_press(), whose flags also
    /// start "not seen".
    void update(ButtonMask heldMask, std::uint32_t nowMs);

    /// The `nowMs` of the latest update().
    std::uint32_t nowMs() const;

    /// Down in the latest sample.
    bool held(Button button) const;

    /// Down now and up in the previous sample: true for exactly one update per press.
    bool pressed(Button button) const;

    /// Up now and down in the previous sample: true for exactly one update per release.
    bool released(Button button) const;

    /// While held: ms since the press was sampled. On the update where it's
    /// released: how long it had been held, so a tap and a long hold can be
    /// told apart on release. Otherwise 0.
    std::uint32_t heldMs(Button button) const;

    /// Held, and for at least `ms`.
    bool heldFor(Button button, std::uint32_t ms) const;

    /// True on exactly one update per press: the first where heldFor(button, ms).
    /// For "hold it to do the other thing" buttons.
    bool longPressed(Button button, std::uint32_t ms) const;

    /// Auto-repeat. True on the press, then while still held: once `delayMs`
    /// after the press and every `periodMs` after that, at most once per
    /// update. `periodMs` = 0 disables repeats (the press only).
    bool repeated(Button button, std::uint32_t delayMs, std::uint32_t periodMs) const;

    /// Both held, and this is the first update where they both are: one was
    /// just pressed while the other was already down, in either order, or both
    /// at once.
    bool combo(Button first, Button second) const;

    /// held()/pressed()/released() for the whole controller, e.g. to log every
    /// edge of a tick as one telemetry event.
    ButtonMask heldMask() const;
    ButtonMask pressedMask() const;
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
