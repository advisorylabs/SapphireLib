#include "sapphirelib/mechanism/preset_ladder.hpp"

#include <algorithm>
#include <utility>

namespace sapphirelib::mechanism {

PresetLadder::PresetLadder(std::vector<PresetTable> tables) : tables_(std::move(tables)) {
    // Sanitized once here so no query below ever has to handle an empty
    // table (or no tables) — an out-of-range index in a per-tick function is
    // a crash mid-match, not an error message.
    if (tables_.empty()) tables_.push_back({.name = "", .positions = {}});
    for (PresetTable& table : tables_) {
        if (table.positions.empty()) table.positions.push_back(0.0);
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
    table_ = index % tables_.size();
    // The level carries over (moving to the new table's position for it),
    // capped if the new table has fewer levels.
    level_ = std::min(level_, maxLevel());
}

int PresetLadder::level() const { return level_; }

int PresetLadder::maxLevel() const { return static_cast<int>(positions().size()) - 1; }

void PresetLadder::setLevel(int level) { level_ = clampLevel(level); }

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
