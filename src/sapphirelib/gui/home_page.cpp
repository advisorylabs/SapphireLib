#include "sapphirelib/gui/home_page.hpp"

#include <cstdio>

#include "pros/misc.hpp"
#include "sapphirelib/telemetry/logger.hpp"

namespace sapphirelib::gui {

HomePage::HomePage(sensors::Imu* imu) : imu_(imu) {}

void HomePage::setTelemetry(const telemetry::Logger* logger) {
    // Only stored: the row itself is created in build(), or — if the page
    // was built already — by the next update(), which runs on the GUI's own
    // task. Creating widgets from here could race LVGL mid-render.
    logger_.store(logger, std::memory_order_release);
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
    if (logger_.load(std::memory_order_acquire) != nullptr) {
        telemetryLabel_ = lv_label_create(container);
    }

    update();
}

void HomePage::update() {
    char buf[48];

    // Battery capacity is reported in whole percent and changes over
    // minutes, so this formats to the same string almost every tick —
    // setLabelText() is what keeps that from re-invalidating the label 20
    // times a second.
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

    const telemetry::Logger* logger = logger_.load(std::memory_order_acquire);
    if (logger != nullptr && telemetryLabel_ == nullptr && container_ != nullptr) {
        // setTelemetry() came after build(): the row goes last, just where
        // build() would have put it.
        telemetryLabel_ = lv_label_create(container_);
    }
    if (telemetryLabel_ != nullptr) {
        const telemetry::LoggerStatus status =
            logger != nullptr ? logger->status() : telemetry::LoggerStatus{};
        const char* text = "SD: off";
        switch (status.state) {
            case telemetry::LoggerState::logging:
                // A missing log folder is what to fix before the next match;
                // the file name can wait until it's fixed.
                if (status.inRootFolder) {
                    text = "SD: no folder, root";
                } else {
                    std::snprintf(buf, sizeof(buf), "SD: logging SL%06ld",
                                  static_cast<long>(status.fileIndex));
                    text = buf;
                }
                break;
            case telemetry::LoggerState::waitingForCard: text = "SD: waiting for card"; break;
            case telemetry::LoggerState::faulted: text = "SD: FAULT"; break;
            default: break;
        }
        setLabelText(telemetryLabel_, text);
    }
}

} // namespace sapphirelib::gui
