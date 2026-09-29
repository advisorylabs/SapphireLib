/**
 * \file sapphirelib/mechanism/preset_ladder.hpp
 *
 * Named tables of preset positions, stepped through a level at a time: "L1
 * raises the lift a notch; RIGHT switches between the alliance-goal and
 * center-goal heights". Holds only indices and numbers; feeding a position
 * to a mechanism is your code's job. Pure; see
 * tests/mechanism/preset_ladder_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>
#include <vector>

namespace sapphirelib::mechanism {

struct PresetTable {
    /// Shown on screens. Keep it short (a controller line is 15 columns).
    const char* name;

    /// Each level's position, level 0 first. Spacing needn't be even.
    std::vector<double> positions;
};

/// The current table and level. Level changes are clamped to the current
/// table and never wrap. Not thread-safe: use it from one task (your
/// per-tick function), like the rest of your driver-control state.
///
///   PresetLadder ladder({
///       {"ALLIANCE", {0, 150, 300, 450}},
///       {"CENTER", {0, 225, 450}},
///   });
///   if (controller.pressed(Button::l1)) ladder.up();
///   if (controller.pressed(Button::right)) ladder.nextTable();
///   lift.setTarget(ladder.levelPosition());
///
/// Constructing it allocates (the tables are copied in), so build it once —
/// at namespace scope or as a static — never per tick.
class PresetLadder {
public:
    /// Needs at least one table: an empty list gets one unnamed table at 0.0,
    /// and a table with no positions gets a single 0.0. Starts at table 0,
    /// level 0.
    explicit PresetLadder(std::vector<PresetTable> tables);

    std::size_t tableCount() const;
    std::size_t tableIndex() const;

    /// The current table's name ("" if it has none).
    const char* tableName() const;

    /// The next table, wrapping to the first. The level carries over, capped
    /// at the new table's top level.
    void nextTable();

    /// Table `index` (wrapped into range), with the same level rule as
    /// nextTable().
    void selectTable(std::size_t index);

    int level() const;

    /// The current table's top level (positions.size() - 1).
    int maxLevel() const;

    /// Clamped to [0, maxLevel()].
    void setLevel(int level);

    /// One level up or down, stopping at the ends. True if it moved.
    bool up();
    bool down();

    /// The current level's position.
    double levelPosition() const;

    /// Level `level`'s position in the current table (level clamped).
    double levelPosition(int level) const;

    /// Halfway between `level` and the level below it,
    /// (levelPosition(level) + levelPosition(level - 1)) / 2 — e.g. "dip
    /// halfway to the level below to score". `level` is clamped first. Level
    /// 0 has nothing below it and returns levelPosition(0).
    double midpointBelow(int level) const;

private:
    const std::vector<double>& positions() const;
    int clampLevel(int level) const;

    std::vector<PresetTable> tables_;
    std::size_t table_ = 0;
    int level_ = 0;
};

} // namespace sapphirelib::mechanism
