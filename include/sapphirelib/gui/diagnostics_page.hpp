/**
 * \file sapphirelib/gui/diagnostics_page.hpp
 *
 * SapphireLib's default sensor-diagnostics page: re-checks every registered
 * diag::SensorCheck on a live loop (not just at startup — a sensor working
 * fine when the robot powered on but knocked loose mid-match is exactly the
 * kind of thing this should catch too) and shows the failures.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <vector>

#include "sapphirelib/diag/sensor_check.hpp"
#include "sapphirelib/gui/gui.hpp"
#include "sapphirelib/gui/page.hpp"

namespace sapphirelib::gui {

class DiagnosticsPage : public Page {
public:
    /// `gui`, if given, gets a red header banner via Gui::showWarning()
    /// whenever any check is failing (cleared automatically once they all
    /// pass again) — so a bad sensor is visible from any tab, not just this
    /// one.
    explicit DiagnosticsPage(std::vector<diag::SensorCheck> checks, Gui* gui = nullptr);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

    /// True: this page keeps polling even when another tab is showing, so
    /// the screen-wide warning banner it raises through Gui::showWarning()
    /// stays honest no matter which tab the driver is looking at. It
    /// throttles itself to a fixed poll interval to pay for that — see
    /// update().
    bool updatesWhenHidden() const override;

private:
    struct Row {
        diag::SensorCheck check;
        lv_obj_t* label = nullptr;

        /// The verdict whose color is currently applied to `label`, so
        /// update() can skip restyling a row that hasn't changed. build()
        /// establishes the invariant by painting every row failing-colored
        /// before the first check runs.
        bool ok = false;
    };

    std::vector<Row> rows_;
    Gui* gui_;

    /// pros::millis() at the last actual poll — see kPollIntervalMs. 0 means
    /// "never polled", which always polls.
    std::uint32_t lastPollMs_ = 0;
};

} // namespace sapphirelib::gui
