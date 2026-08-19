#include "sapphirelib/gui/diagnostics_page.hpp"

#include <cstdio>

namespace sapphirelib::gui {

namespace {

constexpr std::uint32_t kOkColor = 0x4ade80;
constexpr std::uint32_t kFailColor = 0xf87171;

} // namespace

DiagnosticsPage::DiagnosticsPage(std::vector<diag::SensorCheck> checks, Gui* gui) : gui_(gui) {
    rows_.reserve(checks.size());
    for (auto& check : checks) rows_.push_back(Row{std::move(check), nullptr});
}

const char* DiagnosticsPage::title() const { return "Diag"; }

void DiagnosticsPage::build(lv_obj_t* container) {
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(container, 4, 0);

    for (Row& row : rows_) row.label = lv_label_create(container);

    update();
}

void DiagnosticsPage::update() {
    bool anyFailing = false;
    char buf[80];

    for (Row& row : rows_) {
        const diag::CheckResult result = diag::runCheck(row.check);
        if (result.ok) {
            std::snprintf(buf, sizeof(buf), "OK  %s (port %d)", result.label.c_str(),
                          static_cast<int>(result.port));
            lv_obj_set_style_text_color(row.label, lv_color_hex(kOkColor), 0);
        } else {
            anyFailing = true;
            std::snprintf(buf, sizeof(buf), "FAIL  %s (port %d): %s", result.label.c_str(),
                          static_cast<int>(result.port), result.detail.c_str());
            lv_obj_set_style_text_color(row.label, lv_color_hex(kFailColor), 0);
        }
        lv_label_set_text(row.label, buf);
    }

    if (gui_) {
        if (anyFailing) {
            gui_->showWarning("Sensor problem - check Diag tab");
        } else {
            gui_->clearWarning();
        }
    }
}

} // namespace sapphirelib::gui
