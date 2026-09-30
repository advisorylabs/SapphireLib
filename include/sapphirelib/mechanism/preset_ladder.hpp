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
     * The level carries over, capped at the new table's top level
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
     * @brief Set the level
     *
     * @param level the new level. Clamped to [0, maxLevel()]
     */
    void setLevel(int level);

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
