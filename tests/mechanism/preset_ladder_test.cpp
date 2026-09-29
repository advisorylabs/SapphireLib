/*
 * Host-side unit test for sapphirelib::mechanism::PresetLadder — no
 * PROS/embedded dependencies, so it builds and runs with a normal desktop
 * compiler.
 *
 * Besides the unit cases, it replays random button sequences through a
 * verbatim copy of the scoring-mode/level logic from the robot's original
 * driver macros (src/robot_macros.cpp) and checks the ladder agrees with it
 * on every step.
 *
 * (A block comment, not //, so the backslash-continued build line doesn't
 * trip -Wcomment.)
 *
 * Build & run:
 *   g++ -std=c++20 -Iinclude tests/mechanism/preset_ladder_test.cpp \
 *       src/sapphirelib/mechanism/preset_ladder.cpp \
 *       -o preset_ladder_test && ./preset_ladder_test
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "sapphirelib/mechanism/preset_ladder.hpp"

using sapphirelib::mechanism::PresetLadder;
using sapphirelib::mechanism::PresetTable;

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                       \
            std::exit(1);                                                                          \
        }                                                                                          \
    } while (0)

namespace {

bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

PresetLadder robotLadder() {
    // The same literal syntax the robot uses: int positions are fine.
    return PresetLadder({
        {"ALLIANCE", {0, 150, 300, 450, 600, 750, 900}},
        {"MEDIUM", {0, 180, 360, 540, 720, 900}},
        {"CENTER", {0, 225, 450, 675, 900}},
    });
}

void testStartsAtTableZeroLevelZero() {
    const PresetLadder ladder = robotLadder();
    CHECK(ladder.tableCount() == 3);
    CHECK(ladder.tableIndex() == 0);
    CHECK(std::strcmp(ladder.tableName(), "ALLIANCE") == 0);
    CHECK(ladder.level() == 0);
    CHECK(ladder.maxLevel() == 6);
    CHECK(ladder.levelPosition() == 0.0);
}

void testUpDownStopAtTheEnds() {
    PresetLadder ladder = robotLadder();
    CHECK(!ladder.down()); // already at 0
    CHECK(ladder.level() == 0);
    for (int i = 1; i <= 6; ++i) {
        CHECK(ladder.up());
        CHECK(ladder.level() == i);
    }
    CHECK(!ladder.up()); // at the top; never wraps
    CHECK(ladder.level() == 6);
    CHECK(ladder.levelPosition() == 900.0);
    CHECK(ladder.down());
    CHECK(ladder.level() == 5);
    CHECK(ladder.levelPosition() == 750.0);
}

void testNextTableWrapsAndCapsLevel() {
    PresetLadder ladder = robotLadder();
    ladder.setLevel(6);
    ladder.nextTable(); // MEDIUM has levels 0-5
    CHECK(ladder.tableIndex() == 1);
    CHECK(std::strcmp(ladder.tableName(), "MEDIUM") == 0);
    CHECK(ladder.level() == 5);
    CHECK(ladder.levelPosition() == 900.0);
    ladder.nextTable(); // CENTER has levels 0-4
    CHECK(ladder.level() == 4);
    ladder.nextTable(); // back to ALLIANCE; the cap doesn't undo itself
    CHECK(ladder.tableIndex() == 0);
    CHECK(ladder.level() == 4);
    CHECK(ladder.levelPosition() == 600.0);

    // A level that fits carries over unchanged, moving to the new table's
    // position for it.
    ladder.setLevel(2);
    ladder.nextTable();
    CHECK(ladder.level() == 2);
    CHECK(ladder.levelPosition() == 360.0);
}

void testSelectTableWraps() {
    PresetLadder ladder = robotLadder();
    ladder.selectTable(2);
    CHECK(ladder.tableIndex() == 2);
    ladder.selectTable(5); // 5 % 3
    CHECK(ladder.tableIndex() == 2);
    ladder.selectTable(3);
    CHECK(ladder.tableIndex() == 0);
    ladder.setLevel(6);
    ladder.selectTable(2);
    CHECK(ladder.level() == 4);
}

void testSetLevelClamps() {
    PresetLadder ladder = robotLadder();
    ladder.setLevel(3);
    CHECK(ladder.level() == 3);
    ladder.setLevel(-3);
    CHECK(ladder.level() == 0);
    ladder.setLevel(99);
    CHECK(ladder.level() == 6);
}

void testLevelPositionClamps() {
    const PresetLadder ladder = robotLadder();
    CHECK(ladder.levelPosition(2) == 300.0);
    CHECK(ladder.levelPosition(-1) == 0.0);
    CHECK(ladder.levelPosition(42) == 900.0);
}

void testMidpointBelow() {
    PresetLadder ladder = robotLadder();
    CHECK(ladder.midpointBelow(1) == 75.0);
    CHECK(ladder.midpointBelow(2) == 225.0);
    CHECK(ladder.midpointBelow(6) == 825.0);
    CHECK(ladder.midpointBelow(0) == 0.0);   // nothing below level 0
    CHECK(ladder.midpointBelow(-2) == 0.0);  // clamped to level 0
    CHECK(ladder.midpointBelow(9) == 825.0); // clamped to the top level

    // Uneven spacing: always halfway to the level actually below.
    PresetLadder uneven({{"UNEVEN", {10.0, 20.0, 100.0}}});
    CHECK(uneven.midpointBelow(1) == 15.0);
    CHECK(uneven.midpointBelow(2) == 60.0);
}

void testSanitizesBadTables() {
    // No tables at all: one unnamed table with a single level at 0.
    PresetLadder empty({});
    CHECK(empty.tableCount() == 1);
    CHECK(std::strcmp(empty.tableName(), "") == 0);
    CHECK(empty.maxLevel() == 0);
    CHECK(empty.levelPosition() == 0.0);
    CHECK(!empty.up());
    CHECK(!empty.down());
    CHECK(empty.midpointBelow(1) == 0.0);
    empty.nextTable();
    CHECK(empty.tableIndex() == 0);

    // A table with no positions gets a single 0.0; a null name reads "".
    PresetLadder holes({{"A", {1.0, 2.0}}, {nullptr, {}}});
    CHECK(holes.tableCount() == 2);
    holes.setLevel(1);
    holes.nextTable();
    CHECK(std::strcmp(holes.tableName(), "") == 0);
    CHECK(holes.maxLevel() == 0);
    CHECK(holes.level() == 0);
    CHECK(holes.levelPosition() == 0.0);
}

} // namespace

// --- The reference: src/robot_macros.cpp's scoring modes, verbatim ---
//
// ScoringMode, kModes, currentMode(), maxLevel() and halfwayBelow() are copied
// unchanged; `state` keeps only the two fields they use. The operations in
// testMatchesOriginalModeLogic() are the exact statements update() and
// advancePhase() applied to them.
namespace original {

struct ScoringMode {
    const char* name; // shown on the controller; 8 characters max
    std::vector<double> levelDeg;
};

const std::array<ScoringMode, 3> kModes{{
    {"ALLIANCE", {0, 150, 300, 450, 600, 750, 900}},
    {"MEDIUM", {0, 180, 360, 540, 720, 900}},
    {"CENTER", {0, 225, 450, 675, 900}},
}};

struct State {
    std::size_t mode = 0; // index into kModes
    int level = 0;
};

State state;

const ScoringMode& currentMode() { return kModes[state.mode]; }

int maxLevel() { return static_cast<int>(currentMode().levelDeg.size()) - 1; }

/// Halfway between `level` and the level below it.
double halfwayBelow(int level) {
    const std::vector<double>& levels = currentMode().levelDeg;
    return (levels[level] + levels[level - 1]) / 2.0;
}

} // namespace original

namespace {

void testMatchesOriginalModeLogic() {
    std::mt19937_64 rng(0x1add37);
    std::uniform_int_distribution<int> op(0, 5);
    PresetLadder ladder = robotLadder();
    original::state = {};
    for (int step = 0; step < 200000; ++step) {
        switch (op(rng)) {
            case 0: // RIGHT: next scoring mode
                original::state.mode = (original::state.mode + 1) % original::kModes.size();
                original::state.level = std::min(original::state.level, original::maxLevel());
                ladder.nextTable();
                break;
            case 1: // L1 with the claw deployed
                if (original::state.level < original::maxLevel()) ++original::state.level;
                ladder.up();
                break;
            case 2: // L2
                if (original::state.level > 0) --original::state.level;
                ladder.down();
                break;
            case 3: // R1 or Y: back to level 0
                original::state.level = 0;
                ladder.setLevel(0);
                break;
            case 4: // a score above level 0 finishing: up a level
                original::state.level = std::min(original::state.level + 1, original::maxLevel());
                ladder.up();
                break;
            case 5: // nothing pressed
                break;
        }
        CHECK(ladder.tableIndex() == original::state.mode);
        CHECK(ladder.level() == original::state.level);
        CHECK(ladder.maxLevel() == original::maxLevel());
        CHECK(std::strcmp(ladder.tableName(), original::currentMode().name) == 0);
        // The three lift targets liftTargetDeg() could pick.
        CHECK(sameBits(ladder.levelPosition(),
                       original::currentMode().levelDeg[original::state.level]));
        CHECK(sameBits(ladder.midpointBelow(1), original::halfwayBelow(1)));
        if (original::state.level > 0) {
            CHECK(sameBits(ladder.midpointBelow(ladder.level()),
                           original::halfwayBelow(original::state.level)));
        }
    }
}

} // namespace

int main() {
    testStartsAtTableZeroLevelZero();
    testUpDownStopAtTheEnds();
    testNextTableWrapsAndCapsLevel();
    testSelectTableWraps();
    testSetLevelClamps();
    testLevelPositionClamps();
    testMidpointBelow();
    testSanitizesBadTables();
    testMatchesOriginalModeLogic();
    std::puts("preset_ladder_test: all assertions passed");
    return 0;
}
