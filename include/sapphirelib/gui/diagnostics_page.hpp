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

private:
    struct Row {
        diag::SensorCheck check;
        lv_obj_t* label = nullptr;
    };

    std::vector<Row> rows_;
    Gui* gui_;
};

} // namespace sapphirelib::gui
