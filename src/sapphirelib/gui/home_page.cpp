#include "sapphirelib/gui/home_page.hpp"

#include <cstdio>
#include <utility>

#include "pros/misc.hpp"
#include "sapphirelib/telemetry/logger.hpp"

namespace sapphirelib::gui {

namespace {

// big enough to hit with a thumb in the pits, small enough to sit beside the SD line
constexpr std::int32_t kRecordButtonW = 110;
constexpr std::int32_t kRecordButtonH = 34;

// why a recording is running, as the SD line says it
const char* reasonSuffix(telemetry::RecordingReason reason) {
    switch (reason) {
        case telemetry::RecordingReason::manual: return " (button)";
        case telemetry::RecordingReason::competition: return " (match)";
        default: return "";
    }
}

} // namespace

HomePage::HomePage(sensors::Imu* imu) : imu_(imu) {}

void HomePage::setTelemetry(telemetry::Logger* logger) {
    // only stored here. The row is created in build(), or by the next update() on the GUI's task,
    // since creating widgets from another task could race LVGL
    logger_.store(logger, std::memory_order_release);
}

void HomePage::addRow(std::function<std::string()> text) {
    if (container_ != nullptr || !text) return; // built already: see the doc comment
    rows_.push_back(Row{.text = std::move(text)});
}

const char* HomePage::title() const { return "Home"; }

void HomePage::build(lv_obj_t* container) {
    container_ = container;
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(container, 6, 0);

    batteryLabel_ = lv_label_create(container);
    statusLabel_ = lv_label_create(container);
    if (imu_) headingLabel_ = lv_label_create(container);
    if (logger_.load(std::memory_order_acquire) != nullptr) addTelemetryRow();
    for (Row& row : rows_) row.label = lv_label_create(container);

    update();
}

void HomePage::addTelemetryRow() {
    // the SD line and its button side by side, in a row with no box of its own
    lv_obj_t* row = lv_obj_create(container_);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 12, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* button = lv_button_create(row);
    lv_obj_set_size(button, kRecordButtonW, kRecordButtonH);
    recordLabel_ = lv_label_create(button);
    lv_obj_center(recordLabel_);
    lv_obj_add_event_cb(button, &HomePage::recordClicked, LV_EVENT_CLICKED, this);

    telemetryLabel_ = lv_label_create(row);
    // rows added with addRow() go under this one; move it up if they were built first
    if (!rows_.empty() && rows_.front().label != nullptr) {
        lv_obj_move_to_index(row, lv_obj_get_index(rows_.front().label));
    }
}

void HomePage::recordClicked(lv_event_t* e) {
    auto* page = static_cast<HomePage*>(lv_event_get_user_data(e));
    telemetry::Logger* logger = page->logger_.load(std::memory_order_acquire);
    if (logger == nullptr) return;
    // the logger acts within a tick; update() relabels the button once it has
    if (logger->recording()) {
        logger->stopRecording();
    } else {
        logger->startRecording();
    }
}

void HomePage::update() {
    char buf[48];

    // the battery changes over minutes, so this is almost always the same string, and
    // setLabelText() keeps that from redrawing the label every tick
    std::snprintf(buf, sizeof(buf), "Battery: %.0f%%", pros::battery::get_capacity());
    setLabelText(batteryLabel_, buf);

    const char* stateText = pros::competition::is_autonomous() ? "Autonomous"
                            : pros::competition::is_disabled() ? "Disabled"
                                                               : "Driver Control";
    std::snprintf(buf, sizeof(buf), "%s%s", stateText,
                  pros::competition::is_connected() ? " (field connected)" : "");
    setLabelText(statusLabel_, buf);

    if (imu_) {
        std::snprintf(buf, sizeof(buf), "Heading: %.1f deg", imu_->getHeadingDeg());
        setLabelText(headingLabel_, buf);
    }

    telemetry::Logger* logger = logger_.load(std::memory_order_acquire);
    if (logger != nullptr && telemetryLabel_ == nullptr && container_ != nullptr) {
        // setTelemetry() came after build()
        addTelemetryRow();
    }
    if (telemetryLabel_ != nullptr) {
        const telemetry::LoggerStatus status =
            logger != nullptr ? logger->status() : telemetry::LoggerStatus{};
        const char* text = "SD: off";
        switch (status.state) {
            case telemetry::LoggerState::logging:
                // a missing log folder is what to fix before the next match; the file name can wait
                if (status.inRootFolder) {
                    text = "SD: no folder, root";
                } else {
                    std::snprintf(buf, sizeof(buf), "SD: logging SL%06ld%s",
                                  static_cast<long>(status.fileIndex), reasonSuffix(status.reason));
                    text = buf;
                }
                break;
            case telemetry::LoggerState::idle:
                // idle means the file is closed, so this is also "safe to pull the card"
                text = status.cardInserted ? "SD: ready, not logging" : "SD: no card";
                break;
            case telemetry::LoggerState::waitingForCard: text = "SD: waiting for card"; break;
            case telemetry::LoggerState::faulted: text = "SD: FAULT"; break;
            default: break;
        }
        setLabelText(telemetryLabel_, text);
        setLabelText(recordLabel_, status.recording ? "Stop log" : "Start log");
    }

    for (Row& row : rows_) {
        if (row.label != nullptr) setLabelText(row.label, row.text().c_str());
    }
}

} // namespace sapphirelib::gui
