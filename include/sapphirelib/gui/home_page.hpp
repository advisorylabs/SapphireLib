/**
 * \file sapphirelib/gui/home_page.hpp
 *
 * SapphireLib's default landing page: battery, competition connection/mode
 * status, (if given an IMU) current heading, and (if given a
 * telemetry::Logger) whether the SD card is logging.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace sapphirelib::gui {

class HomePage : public Page {
public:
    /// `imu`, if given, adds a heading readout row. Pass nullptr to omit
    /// it (e.g. before you've wired up a drivetrain).
    explicit HomePage(sensors::Imu* imu = nullptr);

    /// Adds an "SD:" row showing `logger`'s state — "SD: logging SL000042",
    /// "SD: waiting for card", "SD: no folder, root" (the log folder is
    /// missing, so files land in the card's root), "SD: FAULT", or "SD: off"
    /// (not started) — so a missing or pulled card is noticed in the pits,
    /// not after the match whose data it lost. Call it before or after the
    /// page is added to Gui; nullptr turns the row back to "SD: off".
    /// `logger` must outlive the page.
    void setTelemetry(const telemetry::Logger* logger);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    sensors::Imu* imu_;
    // Set from whichever task wires telemetry up; read on the GUI's.
    std::atomic<const telemetry::Logger*> logger_{nullptr};
    lv_obj_t* container_ = nullptr;
    lv_obj_t* batteryLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* headingLabel_ = nullptr;
    lv_obj_t* telemetryLabel_ = nullptr;
};

} // namespace sapphirelib::gui
