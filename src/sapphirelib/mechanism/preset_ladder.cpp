#include "sapphirelib/mechanism/preset_ladder.hpp"

#include <algorithm>
#include <utility>

namespace sapphirelib::mechanism {

PresetLadder::PresetLadder(std::vector<PresetTable> tables) : tables_(std::move(tables)) {
    // cleaned up once here, so no query ever has to handle an empty table
    if (tables_.empty()) tables_.push_back({.name = "", .positions = {}});
    for (PresetTable& table : tables_) {
        if (table.positions.empty()) table.positions.push_back(0.0);
        table.firstLevelNumber = std::max(table.firstLevelNumber, 1);
    }
}

std::size_t PresetLadder::tableCount() const { return tables_.size(); }

std::size_t PresetLadder::tableIndex() const { return table_; }

const char* PresetLadder::tableName() const {
    const char* name = tables_[table_].name;
    return name != nullptr ? name : "";
}

void PresetLadder::nextTable() { selectTable(table_ + 1); }

void PresetLadder::selectTable(std::size_t index) {
    const int number = levelNumber();
    table_ = index % tables_.size();
    // the level's number carries over, capped if the new table has fewer levels
    setLevelNumber(number);
}

int PresetLadder::level() const { return level_; }

int PresetLadder::maxLevel() const { return static_cast<int>(positions().size()) - 1; }

int PresetLadder::levelNumber() const { return levelNumber(level_); }

int PresetLadder::levelNumber(int level) const {
    level = clampLevel(level);
    if (level == 0) return 0;
    return level + tables_[table_].firstLevelNumber - 1;
}

void PresetLadder::setLevel(int level) { level_ = clampLevel(level); }

void PresetLadder::setLevelNumber(int number) {
    // never down to 0 from a number above it
    if (number <= 0) {
        level_ = 0;
    } else {
        level_ = clampLevel(std::max(number - tables_[table_].firstLevelNumber + 1, 1));
    }
}

bool PresetLadder::up() {
    if (level_ >= maxLevel()) return false;
    ++level_;
    return true;
}

bool PresetLadder::down() {
    if (level_ <= 0) return false;
    --level_;
    return true;
}

double PresetLadder::levelPosition() const { return positions()[level_]; }

double PresetLadder::levelPosition(int level) const { return positions()[clampLevel(level)]; }

double PresetLadder::midpointBelow(int level) const {
    level = clampLevel(level);
    if (level == 0) return positions()[0];
    return (positions()[level] + positions()[level - 1]) / 2.0;
}

const std::vector<double>& PresetLadder::positions() const { return tables_[table_].positions; }

int PresetLadder::clampLevel(int level) const { return std::clamp(level, 0, maxLevel()); }

} // namespace sapphirelib::mechanism
