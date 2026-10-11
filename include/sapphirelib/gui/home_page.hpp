#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace sapphirelib::gui {

/**
 * @brief Landing page: battery, competition status, heading, and SD logging status, plus any on/off
 * buttons you add (see addToggle())
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

    /**
     * @brief Add an on/off button for a mode of your own, like a debug mode
     *
     * It reads "<name>: ON" or "<name>: OFF" and is highlighted while on. The state is polled
     * every refresh, so the button shows the truth even if something else changes it
     *
     * @note call it before Gui::addPage(). Both functions run on the GUI's task, so read and write
     * something that's safe from there, like an atomic
     *
     * @param name what the button switches. Keep it short
     * @param isOn reads the state
     * @param setOn sets it; a tap calls it with the opposite of isOn()
     *
     * @b Example
     * @code {.cpp}
     * home->addToggle("Debug", [] { return debugMode.load(); },
     *                 [](bool on) { debugMode.store(on); });
     * @endcode
     */
    void addToggle(std::string name, std::function<bool()> isOn, std::function<void(bool)> setOn);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Toggle {
        std::string name;
        std::function<bool()> isOn;
        std::function<void(bool)> setOn;
        lv_obj_t* button = nullptr;
        lv_obj_t* label = nullptr;
    };

    static void toggleClicked(lv_event_t* e);

    sensors::Imu* imu_;
    // unique_ptr so each Toggle stays put: its address is the button's event user data
    std::vector<std::unique_ptr<Toggle>> toggles_;
    // set from whichever task wires telemetry up, read on the GUI's
    std::atomic<const telemetry::Logger*> logger_{nullptr};
    lv_obj_t* container_ = nullptr;
    lv_obj_t* batteryLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* headingLabel_ = nullptr;
    lv_obj_t* telemetryLabel_ = nullptr;
};

} // namespace sapphirelib::gui
