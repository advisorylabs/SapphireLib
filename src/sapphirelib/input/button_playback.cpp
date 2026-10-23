#include "sapphirelib/input/button_playback.hpp"

#include <utility>

namespace sapphirelib::input {

ButtonPlayback::ButtonPlayback(std::vector<TimedButtons> changes) : changes_(std::move(changes)) {}

ButtonMask ButtonPlayback::advance(double clockS) {
    // changes that don't change anything (the same buttons again) are crossed at once; the first
    // that does is played this tick, and anything after it waits for the next
    while (next_ < changes_.size() && changes_[next_].tS <= clockS) {
        const ButtonMask buttons = changes_[next_++].buttons;
        if (buttons != current_) {
            current_ = buttons;
            break;
        }
    }
    return current_;
}

ButtonMask ButtonPlayback::current() const { return current_; }

bool ButtonPlayback::finished() const { return next_ >= changes_.size(); }

} // namespace sapphirelib::input
