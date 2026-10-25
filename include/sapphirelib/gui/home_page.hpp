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
 * @brief Landing page: battery, competition status, heading, SD logging with a start/stop
 * button, and any rows, on/off buttons and plain buttons of your own
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

    /**
     * @brief Add an on/off button for a mode of your own, like a debug mode
     *
     * It reads "<name>: ON" or "<name>: OFF" and is highlighted red while on. The state is polled
     * every refresh, so the button shows the truth even if something else changes it. Buttons stack
     * down the page's top right corner, in the order added; there's room for three
     *
     * @note call it before Gui::addPage(); once the page is built it's ignored, like addRow().
     * Both functions run on the GUI's task, so read and write something that's safe from there,
     * like an atomic
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

    /**
     * @brief Add a button that does something when tapped, like putting a FaceScreen up
     *
     * Buttons stack up from the page's bottom right corner, in the order added, under the toggles
     * at the top. There's room for one or two, depending on how far down the page's rows reach
     *
     * @note call it before Gui::addPage(), like addToggle(). onTap runs on the GUI's task
     *
     * @param label the button's text. Keep it short
     * @param onTap what a tap does
     *
     * @b Example
     * @code {.cpp}
     * home->addButton("Face", [] { face.show(); });
     * @endcode
     */
    void addButton(std::string label, std::function<void()> onTap);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Row {
        std::function<std::string()> text;
        lv_obj_t* label = nullptr;
    };

    struct Toggle {
        std::string name;
        std::function<bool()> isOn;
        std::function<void(bool)> setOn;
        lv_obj_t* button = nullptr;
        lv_obj_t* label = nullptr;
    };

    struct Button {
        std::string label;
        std::function<void()> onTap;
    };

    void addTelemetryRow();
    static void recordClicked(lv_event_t* e);
    static void toggleClicked(lv_event_t* e);
    static void buttonClicked(lv_event_t* e);

    sensors::Imu* imu_;
    // unique_ptr so each Toggle stays put: its address is the button's event user data
    std::vector<std::unique_ptr<Toggle>> toggles_;
    // the same, for each Button
    std::vector<std::unique_ptr<Button>> buttons_;
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
