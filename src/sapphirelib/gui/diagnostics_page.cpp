#include "sapphirelib/gui/diagnostics_page.hpp"

#include <cstdio>
#include <string>

#include "pros/rtos.hpp"
#include "sapphirelib/telemetry/event.hpp"

namespace sapphirelib::gui {

namespace {

constexpr std::uint32_t kOkColor = 0x4ade80;
constexpr std::uint32_t kFailColor = 0xf87171;

// how often the port registry is re-read. This page updates even while hidden, and each pass
// costs a registry lookup and a few string allocations per check. Cables come loose on a
// human timescale, so 250ms still catches it within a quarter second
constexpr std::uint32_t kPollIntervalMs = 250;

// commas split an event's fields, so they can't appear inside one
std::string withoutCommas(std::string text) {
    for (char& c : text) {
        if (c == ',') c = ';';
    }
    return text;
}

// log a device event for one check's verdict: missing, lost, or back
void logDevice(const char* what, const diag::CheckResult& result) {
    telemetry::event("device", "%s,port=%d,label=%s%s%s", what, static_cast<int>(result.port),
                     withoutCommas(result.label).c_str(), result.ok ? "" : ",found=",
                     result.ok ? "" : withoutCommas(result.detail).c_str());
}

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

    for (Row& row : rows_) {
        row.label = lv_label_create(container);
        // start every row failing-colored, matching Row::ok's initial false, so update() only
        // restyles on a change. The update() below repaints the passing rows before any drawing
        lv_obj_set_style_text_color(row.label, lv_color_hex(kFailColor), 0);
    }

    update();
}

bool DiagnosticsPage::updatesWhenHidden() const { return true; }

void DiagnosticsPage::update() {
    const std::uint32_t now = pros::millis();
    if (lastPollMs_ != 0 && now - lastPollMs_ < kPollIntervalMs) return;
    lastPollMs_ = now;

    bool anyFailing = false;
    char buf[80];

    for (Row& row : rows_) {
        const diag::CheckResult result = diag::runCheck(row.check);
        if (!row.polled) {
            if (!result.ok) logDevice("missing", result);
        } else if (row.ok != result.ok) {
            logDevice(result.ok ? "back" : "lost", result);
        }
        row.polled = true;
        if (result.ok) {
            std::snprintf(buf, sizeof(buf), "OK  %s (port %d)", result.label.c_str(),
                          static_cast<int>(result.port));
        } else {
            anyFailing = true;
            std::snprintf(buf, sizeof(buf), "FAIL  %s (port %d): %s", result.label.c_str(),
                          static_cast<int>(result.port), result.detail.c_str());
        }
        // restyling a label redraws it like retexting does, so only recolor when the verdict flips
        if (row.ok != result.ok) {
            row.ok = result.ok;
            lv_obj_set_style_text_color(row.label, lv_color_hex(result.ok ? kOkColor : kFailColor),
                                        0);
        }
        setLabelText(row.label, buf);
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
