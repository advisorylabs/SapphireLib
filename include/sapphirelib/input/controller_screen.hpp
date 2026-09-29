/**
 * \file sapphirelib/input/controller_screen.hpp
 *
 * The controller's three-line text screen (and rumble) as retained state
 * plus a write scheduler. Set what each line should say whenever you like;
 * takeWrite() hands back at most one thing to send per call, only after the
 * minimum interval since the last send, and only for a line whose text
 * differs from what the controller last accepted. That is what stops a
 * controller screen from dropping text: V5 controllers silently discard
 * writes that arrive faster than about every 50ms, counted across all lines.
 * Pure; input::Controller does the actual sending. See
 * tests/input/controller_screen_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>

namespace sapphirelib::input {

/// Not thread-safe: set lines and take writes from one task (the one that
/// runs your per-tick function).
class ControllerScreen {
public:
    static constexpr std::size_t kLines = 3;

    /// Visible columns per line. Lines are cut to this and padded to it when
    /// sent, so a shorter line fully overwrites a longer one.
    static constexpr std::size_t kColumns = 15;

    /// Longest rumble pattern the controller takes ('.' short, '-' long, ' ' pause).
    static constexpr std::size_t kMaxRumbleChars = 8;

    enum class WriteKind : std::uint8_t { none, line, rumble };

    /// One thing to send: a line's text, or a rumble pattern. Self-contained
    /// (the text is copied in), so it stays valid whatever changes next.
    struct Write {
        WriteKind kind = WriteKind::none;
        std::uint8_t line = 0;
        std::array<char, kColumns + 1> text{};
    };

    /// `minWriteIntervalMs` is the spacing between sends. Keep it above the
    /// controller's ~50ms limit.
    explicit ControllerScreen(std::uint32_t minWriteIntervalMs = 60);

    /// printf-style. The result is cut to kColumns characters, as snprintf
    /// into a kColumns + 1 buffer would cut it. Out-of-range lines are
    /// ignored. Safe to call every tick: nothing is sent unless the text
    /// differs from what's shown. The arguments may include line() itself
    /// (e.g. to append to what a line already says).
    void setLine(std::size_t line, const char* format, ...) __attribute__((format(printf, 3, 4)));
    void setLineV(std::size_t line, const char* format, std::va_list args);

    /// The text line `line` is meant to show (not necessarily sent yet); ""
    /// for an out-of-range line.
    const char* line(std::size_t line) const;

    /// Queues a rumble pattern (cut to kMaxRumbleChars), replacing any not yet
    /// sent. It goes ahead of pending line changes, through the same interval.
    /// A null or empty pattern is ignored.
    void rumble(const char* pattern);

    /// Forgets what the controller is showing, so every line is sent again,
    /// empty ones included. Use when the screen may have been changed or lost
    /// behind this object's back (input::Controller does this on resumed()).
    void invalidate();

    /// The next thing to send, if the interval since the last send has passed:
    /// a pending rumble first, then the lowest-numbered line that differs from
    /// what the controller last accepted; `kind == none` if nothing is due.
    /// Returning a write counts as sending for the interval, whether or not
    /// the send succeeds, so a rejected write waits a full interval before
    /// retrying instead of hammering a controller that's already refusing.
    ///
    /// Every line starts out "unknown", so the first writes after
    /// construction repaint all three lines — clearing whatever an earlier
    /// program left on the screen.
    Write takeWrite(std::uint32_t nowMs);

    /// Call after `write` was accepted: its text is now what's shown (or the
    /// rumble is done). A failed write is simply not confirmed and comes up
    /// again.
    void confirm(const Write& write);

private:
    std::uint32_t minWriteIntervalMs_;
    std::uint32_t lastWriteMs_ = 0;
    std::array<std::array<char, kColumns + 1>, kLines> wanted_{};
    std::array<std::array<char, kColumns + 1>, kLines> shown_{};
    std::array<bool, kLines> shownKnown_{}; // false = unknown, so always differs
    std::array<char, kMaxRumbleChars + 1> rumble_{};
    bool rumblePending_ = false;
};

} // namespace sapphirelib::input
