#include "sapphirelib/gui/home_page.hpp"

#include <cstdio>

#include "pros/misc.hpp"
#include "sapphirelib/telemetry/logger.hpp"

namespace sapphirelib::gui {

HomePage::HomePage(sensors::Imu* imu) : imu_(imu) {}

void HomePage::setTelemetry(const telemetry::Logger* logger) {
    // only stored here. The row is created in build(), or by the next update() on the GUI's task,
    // since creating widgets from another task could race LVGL
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

    const telemetry::Logger* logger = logger_.load(std::memory_order_acquire);
    if (logger != nullptr && telemetryLabel_ == nullptr && container_ != nullptr) {
        // setTelemetry() came after build(): add the row last, where build() would have
        telemetryLabel_ = lv_label_create(container_);
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
