#include "sapphirelib/gui/diagnostics_page.hpp"

#include <cstdio>
#include <string>

#include "pros/rtos.hpp"
#include "sapphirelib/telemetry/event.hpp"

namespace sapphirelib::gui {

namespace {

constexpr std::uint32_t kOkColor = 0x4ade80;
constexpr std::uint32_t kFailColor = 0xf87171;

// How often the port registry is actually re-read. This page is the one
// default page that keeps updating while hidden (it owns the screen-wide
// warning banner), and each pass costs a registry lookup plus a couple of
// heap-allocated std::strings *per check* — nine of them on a typical
// chassis. Cables come loose on a human timescale, so polling at Gui's full
// refresh rate bought nothing; 250ms still catches a mid-match failure
// within a quarter second.
constexpr std::uint32_t kPollIntervalMs = 250;

/// Commas split an event's key=value fields, so they can't appear inside
/// one.
std::string withoutCommas(std::string text) {
    for (char& c : text) {
        if (c == ',') c = ';';
    }
    return text;
}

/// A `device` telemetry event for one check's verdict: missing/lost/back.
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
        // Matches Row::ok's initial false, so update()'s "only restyle when
        // the verdict flips" check starts from a true statement about what's
        // actually on screen. The update() call below repaints anything
        // that's passing before this is ever drawn.
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
        // Restyling a label invalidates it just like retexting it does, so
        // the color is only reapplied when the check's verdict actually
        // flips — which, for a healthy robot, is never.
        if (row.ok != result.ok) {
            row.ok = result.ok;
            lv_obj_set_style_text_color(row.label,
                                        lv_color_hex(result.ok ? kOkColor : kFailColor), 0);
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
