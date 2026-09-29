// Host-side unit test for sapphirelib::input::ControllerScreen — no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/input/controller_screen_test.cpp src/sapphirelib/input/controller_screen.cpp -o controller_screen_test && ./controller_screen_test

#include <array>
#include <cassert>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "sapphirelib/input/controller_screen.hpp"
#include "sapphirelib/util/timing.hpp"

using sapphirelib::input::ControllerScreen;
using Write = ControllerScreen::Write;
using WriteKind = ControllerScreen::WriteKind;

// As in pros/error.h, so the reference below reads exactly as the original.
#define PROS_ERR (INT32_MAX)

namespace {

void expect(bool ok, const char* label) {
    if (!ok) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

void expectLine(const Write& write, std::uint8_t line, const char* text, const char* label) {
    if (write.kind != WriteKind::line || write.line != line ||
        std::strcmp(write.text.data(), text) != 0) {
        std::printf("FAIL %s: got kind %d line %d \"%s\", expected line %d \"%s\"\n", label,
                    static_cast<int>(write.kind), write.line, write.text.data(), line, text);
        assert(false);
    }
}

void expectNone(const Write& write, const char* label) {
    expect(write.kind == WriteKind::none, label);
}

// --- A fake pros::Controller that records what reaches the screen ---

struct PrintRecord {
    int tick;
    int line;
    std::string text; // after the print's own formatting (the padding)
    bool operator==(const PrintRecord&) const = default;
};

struct FakeController {
    std::vector<PrintRecord> prints;
    int tick = 0;
    bool failThisTick = false;

    // The kernel formats into CONTROLLER_MAX_CHARS + 1 = 32 bytes.
    template <typename... Args>
    std::int32_t print(std::uint8_t line, std::uint8_t col, const char* fmt, Args... args) {
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), fmt, args...);
        assert(col == 0);
        prints.push_back({tick, line, buffer});
        return failThisTick ? PROS_ERR : 1;
    }

    std::int32_t rumble(const char* pattern) {
        prints.push_back({tick, 3, pattern});
        return failThisTick ? PROS_ERR : 1;
    }
};

// --- Reference: robot_macros.cpp's hand-written screen throttle ---
//
// Verbatim from src/robot_macros.cpp (before the port): the restart check at
// the top of update(), and the throttle/diff loop at the end of
// showStatus(). Only the state struct is trimmed to the fields they use.

constexpr std::uint32_t kControllerPrintMs = 60;
constexpr std::uint32_t kRestartGapMs = 100;

struct State {
    std::array<std::string, 3> shownLines; // what each controller line last showed
    std::uint32_t lastPrintMs = 0;
    std::uint32_t lastUpdateMs = 0;
};

void referenceUpdateStart(State& state, std::uint32_t now) {
    if (now - state.lastUpdateMs > kRestartGapMs) {
        for (std::string& line : state.shownLines) line.clear();
    }
    state.lastUpdateMs = now;
}

void referenceShowStatusEnd(State& state, FakeController& controller, char (&lines)[3][16],
                            std::uint32_t now) {
    // At most one line per print interval, so a change that arrives too soon
    // after the last print (or behind another line's change) waits for a
    // later tick.
    if (now - state.lastPrintMs < kControllerPrintMs) return;
    for (std::uint8_t i = 0; i < state.shownLines.size(); ++i) {
        if (state.shownLines[i] == lines[i]) continue;
        state.lastPrintMs = now;
        // Padded to the screen's full 15 columns, so a shorter line fully
        // overwrites a longer one.
        if (controller.print(i, 0, "%-15s", lines[i]) != PROS_ERR) state.shownLines[i] = lines[i];
        return;
    }
}

// --- The port: input::Controller's update()/flushScreen() on the pure parts ---
//
// Mirrors src/sapphirelib/input/controller.cpp line for line, minus the
// button and stick reads (which don't touch the screen).
struct PortedController {
    ControllerScreen screen{60};
    sapphirelib::GapDetector gap{100};
    FakeController& raw;
    std::uint32_t now = 0;

    void update(std::uint32_t nowMs) {
        now = nowMs;
        if (gap.update(now)) screen.invalidate();
    }

    void flushScreen() {
        const Write write = screen.takeWrite(now);
        std::int32_t result = PROS_ERR;
        if (write.kind == WriteKind::line) {
            result = raw.print(write.line, 0, "%-*s", static_cast<int>(ControllerScreen::kColumns),
                               write.text.data());
        } else if (write.kind == WriteKind::rumble) {
            result = raw.rumble(write.text.data());
        } else {
            return;
        }
        if (result != PROS_ERR) screen.confirm(write);
    }
};

// --- Random screen content, rendered identically on both sides ---

/// One line's content: which of the robot's line formats, and its values.
/// Every format here yields a non-empty line, as every robot_macros line
/// does — the one place the port differs is an *empty* line after a restart
/// (see testInvalidateResendsEmptyLines).
struct LineSpec {
    int kind = 0;
    int a = 0;
    int b = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    std::string s;
};

LineSpec randomSpec(std::mt19937& rng) {
    static const char* const kNames[] = {"ALLIANCE", "MEDIUM", "CENTER", "SKILLS99"};
    LineSpec spec;
    spec.kind = std::uniform_int_distribution<int>(0, 6)(rng);
    spec.a = std::uniform_int_distribution<int>(-2, 12)(rng);
    spec.b = std::uniform_int_distribution<int>(0, 9)(rng);
    // Wide ranges, so some values overflow their field and get cut off.
    spec.x = std::uniform_real_distribution<double>(-2000.0, 200000.0)(rng);
    spec.y = std::uniform_real_distribution<double>(-15.0, 15.0)(rng);
    spec.z = std::uniform_real_distribution<double>(0.0, 120.0)(rng);
    spec.s = kNames[std::uniform_int_distribution<int>(0, 3)(rng)];
    if (spec.kind == 6) {
        // Free text, 1 to 24 printable characters (including '%', which must
        // survive as data).
        const int length = std::uniform_int_distribution<int>(1, 24)(rng);
        spec.s.clear();
        for (int i = 0; i < length; ++i) {
            spec.s += static_cast<char>(std::uniform_int_distribution<int>(33, 126)(rng));
        }
    }
    return spec;
}

/// snprintf into the reference's char[16] and setLine() on the port, with
/// the same format and arguments.
template <typename... Args>
void setBoth(char (&referenceLine)[16], ControllerScreen& screen, std::size_t line,
             const char* format, Args... args) {
    std::snprintf(referenceLine, sizeof(referenceLine), format, args...);
    screen.setLine(line, format, args...);
}

void render(const LineSpec& spec, char (&referenceLine)[16], ControllerScreen& screen,
            std::size_t line) {
    switch (spec.kind) {
        case 0:
            setBoth(referenceLine, screen, line, "%-8s L%d/%d", spec.s.c_str(), spec.a, spec.b);
            break;
        case 1: setBoth(referenceLine, screen, line, "%-8s STOW", spec.s.c_str()); break;
        case 2:
            setBoth(referenceLine, screen, line, "%3.0fV %3.1fA %2.0fC", spec.y, spec.z / 40.0,
                    spec.z);
            break;
        case 3:
            setBoth(referenceLine, screen, line, "CLAW %s", spec.a % 2 ? "DEPLOYED" : "RETRACTED");
            break;
        case 4: setBoth(referenceLine, screen, line, "%s", "LIFT NO SENSOR"); break;
        case 5:
            setBoth(referenceLine, screen, line, "LIFT %5.0f/%4.0f", spec.x, spec.z * 8.0);
            break;
        default: setBoth(referenceLine, screen, line, "%s", spec.s.c_str()); break;
    }
}

// The port must send exactly what the hand-written throttle sent: same
// ticks, same lines, same padded text, same retries after a rejected print,
// same resend after a restart gap.
void testMatchesHandWrittenThrottle() {
    long totalPrints = 0;
    long totalFailures = 0;
    for (std::uint32_t seed = 1; seed <= 300; ++seed) {
        std::mt19937 rng(seed);
        const double changeChance = std::uniform_real_distribution<double>(0.01, 0.6)(rng);
        const double failChance = std::uniform_real_distribution<double>(0.0, 0.5)(rng);

        FakeController referenceRaw;
        FakeController portedRaw;
        State state;
        PortedController ported{.raw = portedRaw};

        std::array<LineSpec, 3> specs{randomSpec(rng), randomSpec(rng), randomSpec(rng)};
        // Starts both before and after the first interval has passed.
        std::uint32_t now = std::uniform_int_distribution<std::uint32_t>(0, 3000)(rng);

        for (int tick = 0; tick < 3000; ++tick) {
            for (LineSpec& spec : specs) {
                if (std::bernoulli_distribution(changeChance)(rng)) spec = randomSpec(rng);
            }
            const bool fail = std::bernoulli_distribution(failChance)(rng);
            referenceRaw.tick = portedRaw.tick = tick;
            referenceRaw.failThisTick = portedRaw.failThisTick = fail;

            // One opcontrol tick of each: the port's master.update() sits
            // where the original read the clock, then the lines are set, then
            // the one write.
            referenceUpdateStart(state, now);
            ported.update(now);
            char lines[3][16];
            for (std::size_t i = 0; i < 3; ++i) render(specs[i], lines[i], ported.screen, i);
            for (std::size_t i = 0; i < 3; ++i) {
                expect(std::strcmp(lines[i], ported.screen.line(i)) == 0,
                       "setLine text equals snprintf into char[16]");
            }
            referenceShowStatusEnd(state, referenceRaw, lines, now);
            ported.flushScreen();

            if (referenceRaw.prints != portedRaw.prints) {
                std::printf("FAIL throttle equivalence: seed %lu tick %d (%zu vs %zu prints)\n",
                            static_cast<unsigned long>(seed), tick, referenceRaw.prints.size(),
                            portedRaw.prints.size());
                assert(false);
            }
            if (!referenceRaw.prints.empty() && referenceRaw.prints.back().tick == tick && fail) {
                ++totalFailures;
            }

            // Mostly 20 +- 5 ms ticks, sometimes a pause: right at the
            // restart threshold (100 = not a restart, 101 = a restart), or a
            // short stall, or autonomous/disabled-length gaps.
            const int roll = std::uniform_int_distribution<int>(0, 99)(rng);
            if (roll < 90) {
                now += std::uniform_int_distribution<std::uint32_t>(15, 25)(rng);
            } else if (roll < 93) {
                now += 100;
            } else if (roll < 96) {
                now += 101;
            } else if (roll < 98) {
                now += std::uniform_int_distribution<std::uint32_t>(26, 99)(rng);
            } else {
                now += std::uniform_int_distribution<std::uint32_t>(102, 15000)(rng);
            }
        }
        totalPrints += static_cast<long>(referenceRaw.prints.size());
    }
    std::printf("  throttle equivalence: %ld identical prints (%ld rejected and retried)\n",
                totalPrints, totalFailures);
}

void testTruncationMatchesSnprintfIntoSixteenChars() {
    ControllerScreen screen;
    const char* const texts[] = {"", "a", "exactly15chars!", "sixteen chars!!!",
                                 "a much longer line than the screen can show"};
    for (const char* text : texts) {
        char expected[16];
        std::snprintf(expected, sizeof(expected), "%s", text);
        screen.setLine(1, "%s", text);
        expect(std::strcmp(screen.line(1), expected) == 0, "cut like snprintf into char[16]");
        expect(std::strlen(screen.line(1)) <= ControllerScreen::kColumns, "at most 15 columns");
    }
    char expected[16];
    std::snprintf(expected, sizeof(expected), "LIFT %5.0f/%4.0f", 123456.0, 900.0);
    screen.setLine(2, "LIFT %5.0f/%4.0f", 123456.0, 900.0);
    expect(std::strcmp(screen.line(2), expected) == 0, "numeric overflow cut the same way");
}

void testFirstWritesRepaintEveryLineEvenEmpty() {
    // Nothing is known about the screen at startup (an earlier program may
    // have left text), so every line goes out once, empty ones included.
    ControllerScreen screen(60);
    screen.setLine(1, "HELLO");
    expectNone(screen.takeWrite(59), "the first send waits one interval from 0");
    Write write = screen.takeWrite(60);
    expectLine(write, 0, "", "empty line 0 still sent first");
    screen.confirm(write);
    write = screen.takeWrite(120);
    expectLine(write, 1, "HELLO", "then line 1");
    screen.confirm(write);
    write = screen.takeWrite(180);
    expectLine(write, 2, "", "then empty line 2");
    screen.confirm(write);
    expectNone(screen.takeWrite(240), "then nothing, since nothing changed");
    expectNone(screen.takeWrite(100000), "still nothing much later");
}

/// A screen whose three lines are all confirmed as shown, last send at `atMs`.
ControllerScreen settledScreen(std::uint32_t atMs, const char* l0, const char* l1, const char* l2) {
    ControllerScreen screen(60);
    screen.setLine(0, "%s", l0);
    screen.setLine(1, "%s", l1);
    screen.setLine(2, "%s", l2);
    for (std::uint32_t t = atMs - 120; t <= atMs; t += 60) screen.confirm(screen.takeWrite(t));
    return screen;
}

void testOneWritePerIntervalLowestLineFirst() {
    ControllerScreen screen = settledScreen(1000, "A", "B", "C");
    screen.setLine(2, "C2");
    screen.setLine(1, "B2");
    screen.setLine(0, "A2");
    expectNone(screen.takeWrite(1059), "59ms after the last send is too soon");
    Write write = screen.takeWrite(1060);
    expectLine(write, 0, "A2", "line 0 first");
    screen.confirm(write);
    expectNone(screen.takeWrite(1070), "one write per interval");
    write = screen.takeWrite(1120);
    expectLine(write, 1, "B2", "then line 1");
    screen.confirm(write);
    write = screen.takeWrite(1200);
    expectLine(write, 2, "C2", "then line 2");
    screen.confirm(write);
    expectNone(screen.takeWrite(1300), "all shown");
}

void testUnchangedTextIsNeverResent() {
    ControllerScreen screen = settledScreen(1000, "SAME", "SAME", "SAME");
    for (std::uint32_t t = 1020; t < 5000; t += 20) {
        screen.setLine(0, "SAME"); // set every tick, as per-tick code does
        expectNone(screen.takeWrite(t), "unchanged text is not resent");
    }
    // Changed and changed back before it could be sent: nothing to send.
    screen.setLine(1, "OTHER");
    screen.setLine(1, "SAME");
    expectNone(screen.takeWrite(6000), "changed back before sending");
}

void testRejectedWriteWaitsAFullIntervalThenRetries() {
    ControllerScreen screen = settledScreen(1000, "A", "B", "C");
    screen.setLine(1, "NEW");
    const Write rejected = screen.takeWrite(1100);
    expectLine(rejected, 1, "NEW", "first attempt");
    // Not confirmed: the controller refused it. The interval still counts
    // from that attempt.
    expectNone(screen.takeWrite(1159), "no retry inside the interval");
    const Write retry = screen.takeWrite(1160);
    expectLine(retry, 1, "NEW", "retried after a full interval");
    screen.confirm(retry);
    expectNone(screen.takeWrite(1300), "shown once confirmed");
}

void testWriteIsACopy() {
    ControllerScreen screen = settledScreen(1000, "A", "B", "C");
    screen.setLine(0, "FIRST");
    const Write write = screen.takeWrite(1060);
    screen.setLine(0, "SECOND"); // changes after the write was taken...
    expectLine(write, 0, "FIRST", "...don't reach a taken write");
    screen.confirm(write);
    // What's shown is what was sent, so the newer text is still pending.
    expectLine(screen.takeWrite(1120), 0, "SECOND", "newer text sent next");
}

void testInvalidateResendsEmptyLines() {
    // The one deliberate difference from the hand-written throttle, which
    // reset its record to "" and so never resent an empty line. Unknown
    // means unknown: an empty line may be hiding stale text.
    ControllerScreen screen = settledScreen(1000, "A", "", "C");
    screen.invalidate();
    Write write = screen.takeWrite(1060);
    expectLine(write, 0, "A", "resend line 0");
    screen.confirm(write);
    write = screen.takeWrite(1120);
    expectLine(write, 1, "", "resend empty line 1");
    screen.confirm(write);
    write = screen.takeWrite(1180);
    expectLine(write, 2, "C", "resend line 2");
    screen.confirm(write);
    expectNone(screen.takeWrite(1240), "then quiet again");
}

void testRumble() {
    ControllerScreen screen = settledScreen(1000, "A", "B", "C");
    screen.setLine(0, "CHANGED");
    screen.rumble(".-.-.-.-.-.-"); // longer than the controller takes
    Write write = screen.takeWrite(1060);
    expect(write.kind == WriteKind::rumble, "rumble goes ahead of line changes");
    expect(std::strcmp(write.text.data(), ".-.-.-.-") == 0, "cut to 8 characters");
    // Rejected: comes up again after the interval, still ahead of the line.
    expectNone(screen.takeWrite(1100), "rumble retry waits the interval");
    write = screen.takeWrite(1120);
    expect(write.kind == WriteKind::rumble, "rejected rumble retried");
    screen.confirm(write);
    expectLine(screen.takeWrite(1180), 0, "CHANGED", "line change after the rumble");

    // A newer pattern replaces one not yet sent.
    ControllerScreen replaced = settledScreen(1000, "A", "B", "C");
    replaced.rumble("...");
    replaced.rumble("--");
    write = replaced.takeWrite(1060);
    expect(write.kind == WriteKind::rumble && std::strcmp(write.text.data(), "--") == 0,
           "newest pattern wins");
    replaced.confirm(write);
    expectNone(replaced.takeWrite(1120), "rumble done once confirmed");

    // Queued between takeWrite() and confirm(): the confirm is for the old
    // pattern, so the new one still goes out.
    ControllerScreen overlapping = settledScreen(1000, "A", "B", "C");
    overlapping.rumble(".");
    const Write first = overlapping.takeWrite(1060);
    overlapping.rumble("-");
    overlapping.confirm(first);
    write = overlapping.takeWrite(1120);
    expect(write.kind == WriteKind::rumble && std::strcmp(write.text.data(), "-") == 0,
           "a pattern queued mid-send still goes out");

    // Null or empty patterns queue nothing.
    ControllerScreen ignored = settledScreen(1000, "A", "B", "C");
    ignored.rumble(nullptr);
    ignored.rumble("");
    expectNone(ignored.takeWrite(1060), "null/empty rumble ignored");
}

void testOutOfRangeLinesAreIgnored() {
    ControllerScreen screen = settledScreen(1000, "A", "B", "C");
    screen.setLine(3, "NOPE %d", 1);
    screen.setLine(99, "NOPE");
    expect(std::strcmp(screen.line(3), "") == 0, "line(3) reads empty");
    expectNone(screen.takeWrite(2000), "nothing queued for a nonexistent line");
    // confirm() of a nonsense write changes nothing.
    Write bogus;
    bogus.kind = WriteKind::line;
    bogus.line = 7;
    screen.confirm(bogus);
    screen.confirm(Write{});
    expectNone(screen.takeWrite(3000), "bogus confirms ignored");
}

void testSetLineMayReadItsOwnLine() {
    ControllerScreen screen;
    screen.setLine(0, "ABC");
    screen.setLine(0, "%s%s", screen.line(0), screen.line(0));
    expect(std::strcmp(screen.line(0), "ABCABC") == 0, "a line can append to itself");
}

} // namespace

int main() {
    testMatchesHandWrittenThrottle();
    testTruncationMatchesSnprintfIntoSixteenChars();
    testFirstWritesRepaintEveryLineEvenEmpty();
    testOneWritePerIntervalLowestLineFirst();
    testUnchangedTextIsNeverResent();
    testRejectedWriteWaitsAFullIntervalThenRetries();
    testWriteIsACopy();
    testInvalidateResendsEmptyLines();
    testRumble();
    testOutOfRangeLinesAreIgnored();
    testSetLineMayReadItsOwnLine();
    std::puts("controller_screen_test: all assertions passed");
    return 0;
}
