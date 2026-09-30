#pragma once

#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>

namespace sapphirelib::input {

/**
 * @brief The controller's three line screen and rumble, with writes spaced out
 *
 * Set what each line should say whenever you like. takeWrite() hands back at most one thing to
 * send, only after the minimum interval, and only for a line that changed. V5 controllers drop
 * writes sent faster than about every 50ms. input::Controller does the sending
 *
 * @note not thread-safe, so use it from one task
 */
class ControllerScreen {
public:
    static constexpr std::size_t kLines = 3;

    // visible columns per line. Lines are cut and padded to this, so a shorter line fully covers a
    // longer one
    static constexpr std::size_t kColumns = 15;

    // longest rumble pattern ('.' short, '-' long, ' ' pause)
    static constexpr std::size_t kMaxRumbleChars = 8;

    enum class WriteKind : std::uint8_t { none, line, rumble };

    /**
     * @brief One thing to send: a line's text or a rumble pattern. The text is copied in
     */
    struct Write {
        WriteKind kind = WriteKind::none;
        std::uint8_t line = 0;
        std::array<char, kColumns + 1> text{};
    };

    /**
     * @brief Construct a new ControllerScreen
     *
     * @param minWriteIntervalMs time between sends, in milliseconds. Keep it above 50. 60 by
     * default
     */
    explicit ControllerScreen(std::uint32_t minWriteIntervalMs = 60);

    /**
     * @brief Set a line, printf style
     *
     * Cut to 15 characters. Safe to call every tick, since nothing is sent unless the text changed
     *
     * @param line line number, 0-2. Others are ignored
     * @param format printf format string
     *
     * @b Example
     * @code {.cpp}
     * master.screen().setLine(1, "BATT %3.0f%%", pros::battery::get_capacity());
     * @endcode
     */
    void setLine(std::size_t line, const char* format, ...) __attribute__((format(printf, 3, 4)));
    void setLineV(std::size_t line, const char* format, std::va_list args);

    /**
     * @brief Get the text a line should show (not necessarily sent yet). "" for a bad line number
     */
    const char* line(std::size_t line) const;

    /**
     * @brief Queue a rumble pattern, replacing one not yet sent. Goes before pending line changes
     *
     * @param pattern '.' short, '-' long, ' ' pause. Cut to 8 characters. Empty is ignored
     */
    void rumble(const char* pattern);

    /**
     * @brief Forget what the controller shows, so every line is sent again
     *
     * For when the screen may have changed behind this object's back
     */
    void invalidate();

    /**
     * @brief Get the next thing to send, if the interval has passed
     *
     * A pending rumble first, then the lowest line that changed. Returning a write counts as
     * sending, so a rejected write waits a full interval before retrying. Every line starts
     * unknown, so the first writes clear whatever an earlier program left
     *
     * @param nowMs the current time, in milliseconds
     * @return Write what to send. kind is none if nothing is due
     */
    Write takeWrite(std::uint32_t nowMs);

    /**
     * @brief Mark a write as accepted. A write that isn't confirmed comes up again
     *
     * @param write the write that was sent
     */
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
