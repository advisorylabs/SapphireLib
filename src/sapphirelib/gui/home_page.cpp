#include "sapphirelib/gui/home_page.hpp"

#include <cstdio>

#include "pros/misc.hpp"

namespace sapphirelib::gui {

HomePage::HomePage(sensors::Imu* imu) : imu_(imu) {}

const char* HomePage::title() const { return "Home"; }

void HomePage::build(lv_obj_t* container) {
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(container, 6, 0);

    batteryLabel_ = lv_label_create(container);
    statusLabel_ = lv_label_create(container);
    if (imu_) headingLabel_ = lv_label_create(container);

    update();
}

void HomePage::update() {
    char buf[48];

    std::snprintf(buf, sizeof(buf), "Battery: %.0f%%", pros::battery::get_capacity());
    lv_label_set_text(batteryLabel_, buf);

    const char* stateText = pros::competition::is_autonomous() ? "Autonomous"
                             : pros::competition::is_disabled() ? "Disabled"
                                                                 : "Driver Control";
    std::snprintf(buf, sizeof(buf), "%s%s", stateText,
                  pros::competition::is_connected() ? " (field connected)" : "");
    lv_label_set_text(statusLabel_, buf);

    if (imu_) {
        std::snprintf(buf, sizeof(buf), "Heading: %.1f deg", imu_->getHeadingDeg());
        lv_label_set_text(headingLabel_, buf);
    }
}

} // namespace sapphirelib::gui
