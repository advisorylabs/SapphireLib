#include "sapphirelib/input/controller_screen.hpp"

#include <cstdio>
#include <cstring>

#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::input {

// a rumble pattern travels in Write::text, so it has to fit
static_assert(ControllerScreen::kMaxRumbleChars <= ControllerScreen::kColumns);

ControllerScreen::ControllerScreen(std::uint32_t minWriteIntervalMs)
    : minWriteIntervalMs_(minWriteIntervalMs) {}

void ControllerScreen::setLine(std::size_t line, const char* format, ...) {
    std::va_list args;
    va_start(args, format);
    setLineV(line, format, args);
    va_end(args);
}

void ControllerScreen::setLineV(std::size_t line, const char* format, std::va_list args) {
    if (line >= kLines || format == nullptr) return;
    // format into a scratch buffer first, so an argument that is this very line isn't
    // overwritten while it's read. An encoding error leaves the line empty
    std::array<char, kColumns + 1> text{};
    if (std::vsnprintf(text.data(), text.size(), format, args) < 0) text[0] = '\0';
    wanted_[line] = text;
}

const char* ControllerScreen::line(std::size_t line) const {
    return line < kLines ? wanted_[line].data() : "";
}

void ControllerScreen::rumble(const char* pattern) {
    if (pattern == nullptr || pattern[0] == '\0') return;
    // cleared first so the copy is always terminated, and confirm() can compare whole patterns
    rumble_.fill('\0');
    for (std::size_t i = 0; i < kMaxRumbleChars && pattern[i] != '\0'; ++i) {
        rumble_[i] = pattern[i];
    }
    rumblePending_ = true;
}

void ControllerScreen::invalidate() { shownKnown_.fill(false); }

ControllerScreen::Write ControllerScreen::takeWrite(std::uint32_t nowMs) {
    Write write;
    // lastWriteMs_ starts at 0, so the first send waits one interval
    if (elapsedMs(lastWriteMs_, nowMs) < minWriteIntervalMs_) return write;
    if (rumblePending_) {
        lastWriteMs_ = nowMs;
        write.kind = WriteKind::rumble;
        std::memcpy(write.text.data(), rumble_.data(), rumble_.size());
        return write;
    }
    for (std::size_t i = 0; i < kLines; ++i) {
        if (shownKnown_[i] && std::strcmp(shown_[i].data(), wanted_[i].data()) == 0) continue;
        lastWriteMs_ = nowMs; // stamped whether or not the send succeeds
        write.kind = WriteKind::line;
        write.line = static_cast<std::uint8_t>(i);
        write.text = wanted_[i];
        return write;
    }
    return write;
}

void ControllerScreen::confirm(const Write& write) {
    if (write.kind == WriteKind::rumble) {
        // unless a different pattern was queued since, which is still pending
        if (std::strncmp(rumble_.data(), write.text.data(), rumble_.size()) == 0) {
            rumblePending_ = false;
        }
    } else if (write.kind == WriteKind::line && write.line < kLines) {
        shown_[write.line] = write.text;
        // a hand-built Write might not be terminated, and takeWrite()'s strcmp relies on it
        shown_[write.line][kColumns] = '\0';
        shownKnown_[write.line] = true;
    }
}

} // namespace sapphirelib::input
