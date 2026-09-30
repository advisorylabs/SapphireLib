#pragma once

#include <atomic>

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace sapphirelib::gui {

/**
 * @brief Landing page: battery, competition status, heading, and SD logging status
 *
 * @b Example
 * @code {.cpp}
 * auto home = std::make_unique<sapphirelib::gui::HomePage>(&drivetrain().imu());
 * home->setTelemetry(&logger());
 * gui.addPage(std::move(home));
 * @endcode
 */
class HomePage : public Page {
public:
    /**
     * @brief Construct a new HomePage
     *
     * @param imu adds a heading row. nullptr (the default) leaves it out
     */
    explicit HomePage(sensors::Imu* imu = nullptr);

    /**
     * @brief Show the SD logger's state, so a missing or pulled card is noticed in the pits
     *
     * "SD: logging SL000042", "SD: waiting for card", "SD: no folder, root", "SD: FAULT", or
     * "SD: off"
     *
     * @param logger the logger. Must outlive the page. nullptr shows "SD: off"
     */
    void setTelemetry(const telemetry::Logger* logger);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    sensors::Imu* imu_;
    // set from whichever task wires telemetry up, read on the GUI's
    std::atomic<const telemetry::Logger*> logger_{nullptr};
    lv_obj_t* container_ = nullptr;
    lv_obj_t* batteryLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* headingLabel_ = nullptr;
    lv_obj_t* telemetryLabel_ = nullptr;
};

} // namespace sapphirelib::gui
