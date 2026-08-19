/**
 * \file sapphirelib/gui/home_page.hpp
 *
 * SapphireLib's default landing page: battery, competition connection/mode
 * status, and (if given an IMU) current heading.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::gui {

class HomePage : public Page {
public:
    /// `imu`, if given, adds a heading readout row. Pass nullptr to omit
    /// it (e.g. before you've wired up a drivetrain).
    explicit HomePage(sensors::Imu* imu = nullptr);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    sensors::Imu* imu_;
    lv_obj_t* batteryLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* headingLabel_ = nullptr;
};

} // namespace sapphirelib::gui
