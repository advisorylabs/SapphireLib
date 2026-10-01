#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace sapphirelib::gui {

/**
 * @brief Landing page: battery, competition status, heading, SD logging with a start/stop
 * button, and any rows of your own
 *
 * @b Example
 * @code {.cpp}
 * auto home = std::make_unique<sapphirelib::gui::HomePage>(&drivetrain().imu());
 * home->setTelemetry(&logger());
 * home->addRow([] { return std::string("Auton: ") + selectedAuton(); });
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
     * @brief Show the SD logger's state, with a button to start and stop recording
     *
     * "SD: logging SL000042 (button)", "SD: ready, not logging", "SD: no card", "SD: waiting for
     * card", "SD: no folder, root", "SD: FAULT", or "SD: off". The button reads "Start log" or
     * "Stop log" (Logger::startRecording() and stopRecording()); a match records on its own when
     * the logger's recordUnderCompetition is set
     *
     * @param logger the logger. Must outlive the page. nullptr shows "SD: off" and no button
     */
    void setTelemetry(telemetry::Logger* logger);

    /**
     * @brief Add a line of your own under the built-in ones, refreshed with the page
     *
     * Call before the Gui builds the page (before Gui::addPage()). The text function runs on the
     * GUI's task while the page is showing, so keep it quick
     *
     * @param text returns the line's text
     */
    void addRow(std::function<std::string()> text);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Row {
        std::function<std::string()> text;
        lv_obj_t* label = nullptr;
    };

    void addTelemetryRow();
    static void recordClicked(lv_event_t* e);

    sensors::Imu* imu_;
    // set from whichever task wires telemetry up, read on the GUI's
    std::atomic<telemetry::Logger*> logger_{nullptr};
    std::vector<Row> rows_;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* batteryLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* headingLabel_ = nullptr;
    lv_obj_t* telemetryLabel_ = nullptr;
    lv_obj_t* recordLabel_ = nullptr;
};

} // namespace sapphirelib::gui
