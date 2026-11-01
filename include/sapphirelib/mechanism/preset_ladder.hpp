#pragma once

#include <cstddef>
#include <vector>

namespace sapphirelib::mechanism {

/**
 * @brief A named list of preset positions
 */
struct PresetTable {
    /** name shown on screens. Keep it short; a controller line is 15 columns */
    const char* name;

    /** each level's position, level 0 first */
    std::vector<double> positions;

    /**
     * the number level 1 goes by, for a table without the lowest levels the others have: with 2,
     * its levels are numbered 0, 2, 3 and up. Level 0 is always 0. Below 1 counts as 1
     */
    int firstLevelNumber = 1;
};

/**
 * @brief Tables of preset positions, stepped through a level at a time
 *
 * Level changes are clamped to the current table and never wrap. It only holds the numbers;
 * sending a position to a mechanism is up to you. Build it once, not per tick, since it copies the
 * tables
 *
 * @note not thread-safe, so use it from one task
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::mechanism::PresetLadder ladder({
 *     {"ALLIANCE", {0, 150, 300, 450}},
 *     {"CENTER", {0, 225, 450}},
 * });
 *
 * // every tick
 * if (master.pressed(Button::l1)) ladder.up();
 * if (master.pressed(Button::right)) ladder.nextTable();
 * lift.setTarget(ladder.levelPosition());
 * @endcode
 */
class PresetLadder {
public:
    /**
     * @brief Construct a new PresetLadder. Starts at table 0, level 0
     *
     * @param tables the tables. An empty list gets one table at 0, and a table with no positions
     * gets a single 0
     */
    explicit PresetLadder(std::vector<PresetTable> tables);

    /**
     * @brief Get the number of tables
     */
    std::size_t tableCount() const;

    /**
     * @brief Get the current table's index
     */
    std::size_t tableIndex() const;

    /**
     * @brief Get the current table's name, or "" if it has none
     */
    const char* tableName() const;

    /**
     * @brief Switch to the next table, wrapping to the first
     *
     * The level's number (see levelNumber()) carries over, so tables numbered from different
     * levels agree on which one is level 2. A number the new table doesn't have goes to the
     * closest one it does: its level 1, or its top level
     */
    void nextTable();

    /**
     * @brief Switch to a table, keeping the level like nextTable()
     *
     * @param index the table's index. Wrapped into range
     */
    void selectTable(std::size_t index);

    /**
     * @brief Get the current level
     */
    int level() const;

    /**
     * @brief Get the current table's top level
     */
    int maxLevel() const;

    /**
     * @brief Get the number the current level goes by, for screens: the level itself, but for a
     * table with a firstLevelNumber above 1
     */
    int levelNumber() const;

    /**
     * @brief Get the number a level goes by in the current table
     *
     * @param level the level. Clamped to range
     */
    int levelNumber(int level) const;

    /**
     * @brief Set the level
     *
     * @param level the new level. Clamped to [0, maxLevel()]
     */
    void setLevel(int level);

    /**
     * @brief Set the level by the number it goes by (see levelNumber()), for a level picked by
     * the number a screen shows
     *
     * A number the current table doesn't have goes to the closest one it does, as nextTable()
     * does: its level 1 (never level 0 for a number above 0), or its top level. 0 and below is
     * level 0
     *
     * @param number the level's number
     */
    void setLevelNumber(int number);

    /**
     * @brief Go up one level, stopping at the top
     *
     * @return true the level changed
     */
    bool up();

    /**
     * @brief Go down one level, stopping at 0
     *
     * @return true the level changed
     */
    bool down();

    /**
     * @brief Get the current level's position
     */
    double levelPosition() const;

    /**
     * @brief Get a level's position in the current table
     *
     * @param level the level. Clamped to range
     */
    double levelPosition(int level) const;

    /**
     * @brief Get the position halfway between a level and the one below it
     *
     * For "dip halfway to the level below to score"
     *
     * @param level the level. Clamped to range. Level 0 returns levelPosition(0)
     */
    double midpointBelow(int level) const;

private:
    const std::vector<double>& positions() const;
    int clampLevel(int level) const;

    std::vector<PresetTable> tables_;
    std::size_t table_ = 0;
    int level_ = 0;
};

} // namespace sapphirelib::mechanism
