#include "sapphirelib/input/button_tracker.hpp"

#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::input {

namespace {

// every real button's bit. A Button value outside the enum gets no bit, so it never reads as
// held, and the arrays below are only indexed behind a held()/released() check
constexpr ButtonMask kAllButtons = static_cast<ButtonMask>((1u << kButtonCount) - 1u);

constexpr std::size_t indexOf(Button button) { return static_cast<std::size_t>(button); }

// how many auto-repeats are due heldMs into a hold: none before delayMs, then one at delayMs
// and one more every periodMs. periodMs must be non-zero
constexpr std::uint32_t repeatCount(std::uint32_t heldMs, std::uint32_t delayMs,
                                    std::uint32_t periodMs) {
    return heldMs < delayMs ? 0u : 1u + (heldMs - delayMs) / periodMs;
}

} // namespace

void ButtonTracker::update(ButtonMask heldMask, std::uint32_t nowMs) {
    previous_ = held_;
    previousNowMs_ = nowMs_;
    held_ = static_cast<ButtonMask>(heldMask & kAllButtons);
    nowMs_ = nowMs;
    for (std::size_t i = 0; i < kButtonCount; ++i) {
        const ButtonMask bit = static_cast<ButtonMask>(1u << i);
        const bool isDown = (held_ & bit) != 0;
        const bool wasDown = (previous_ & bit) != 0;
        if (isDown && !wasDown) pressedAtMs_[i] = nowMs;
        // kept for heldMs() on the release update only
        if (!isDown && wasDown) releasedHeldMs_[i] = elapsedMs(pressedAtMs_[i], nowMs);
    }
}

std::uint32_t ButtonTracker::nowMs() const { return nowMs_; }

bool ButtonTracker::held(Button button) const { return (held_ & maskOf(button)) != 0; }

bool ButtonTracker::pressed(Button button) const { return (pressedMask() & maskOf(button)) != 0; }

bool ButtonTracker::released(Button button) const { return (releasedMask() & maskOf(button)) != 0; }

std::uint32_t ButtonTracker::heldMs(Button button) const {
    if (held(button)) return elapsedMs(pressedAtMs_[indexOf(button)], nowMs_);
    if (released(button)) return releasedHeldMs_[indexOf(button)];
    return 0;
}

bool ButtonTracker::heldFor(Button button, std::uint32_t ms) const {
    return held(button) && heldMs(button) >= ms;
}

bool ButtonTracker::longPressed(Button button, std::uint32_t ms) const {
    if (!heldFor(button, ms)) return false;
    // the first update past the threshold: just pressed (only when ms is 0), or held last sample
    // but not yet for ms
    return pressed(button) || elapsedMs(pressedAtMs_[indexOf(button)], previousNowMs_) < ms;
}

bool ButtonTracker::repeated(Button button, std::uint32_t delayMs, std::uint32_t periodMs) const {
    if (pressed(button)) return true;
    if (!held(button) || periodMs == 0) return false;
    // held at both samples: fire if a repeat came due between them. Comparing counts caps it at
    // one per update and keeps a long tick from skipping one
    const std::uint32_t pressedAtMs = pressedAtMs_[indexOf(button)];
    return repeatCount(elapsedMs(pressedAtMs, nowMs_), delayMs, periodMs) >
           repeatCount(elapsedMs(pressedAtMs, previousNowMs_), delayMs, periodMs);
}

bool ButtonTracker::combo(Button first, Button second) const {
    // both down now, and at least one wasn't a sample ago, whatever order they went down in
    return held(first) && held(second) && (pressed(first) || pressed(second));
}

ButtonMask ButtonTracker::heldMask() const { return held_; }

ButtonMask ButtonTracker::pressedMask() const {
    return static_cast<ButtonMask>(held_ & ~previous_);
}

ButtonMask ButtonTracker::releasedMask() const {
    return static_cast<ButtonMask>(previous_ & ~held_);
}

} // namespace sapphirelib::input
