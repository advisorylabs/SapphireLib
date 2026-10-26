#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "sapphirelib/gui/page.hpp"

namespace sapphirelib::gui {

/**
 * @brief Autonomous routine picker
 *
 * The driver taps a routine on the brain screen before the match, or steps through them with
 * next() and previous() (the controller's arrows while disabled), and autonomous() runs it. With
 * rememberSelection(), the pick is saved on the SD card and comes back after a restart
 *
 * @b Example
 * @code {.cpp}
 * auto selector = std::make_unique<sapphirelib::gui::AutonSelectorPage>();
 * sapphirelib::gui::AutonSelectorPage* autons = selector.get();
 * autons->addRoutine("Left side", leftSide);
 * autons->addRoutine("Skills", skills);
 * autons->rememberSelection();
 * gui.addPage(std::move(selector));
 *
 * void autonomous() { autons->run(); }
 * @endcode
 */
class AutonSelectorPage : public Page {
public:
    AutonSelectorPage() = default;

    /**
     * @brief Add a routine. The first one added is selected by default
     *
     * @param name the name shown on its button
     * @param routine the routine
     */
    void addRoutine(std::string name, std::function<void()> routine);

    /**
     * @brief Run the selected routine. Does nothing if there are none
     */
    void run() const;

    /**
     * @brief Get the selected routine's name, or "" if there are none. Safe from any task
     */
    const std::string& selectedName() const;

    /**
     * @brief Pick the next routine, wrapping around to the first. Safe from any task
     */
    void next();

    /**
     * @brief Pick the previous routine, wrapping around to the last. Safe from any task
     */
    void previous();

    /**
     * @brief Save every pick to a file on the SD card, and pick the routine it names now
     *
     * Call it once, after adding the routines and before adding the page to the Gui. A missing
     * card or file, or a name no routine has any more, leaves the first routine picked. Each pick
     * is written from the GUI's own task
     *
     * @param path the file. "/usd/AUTON.TXT" by default
     */
    void rememberSelection(std::string path = "/usd/AUTON.TXT");

    /**
     * @brief Show a line above the routines, refreshed a few times a second: whatever is worth a
     * look before a match (the localizer's state, the battery). Call it before adding the page to
     * the Gui
     *
     * @param text what the line says. Called on the GUI's task, so keep it quick
     */
    void setStatus(std::function<std::string()> text);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;
    // a controller pick has to reach the buttons and the card with another tab showing
    bool updatesWhenHidden() const override { return true; }

private:
    struct Routine {
        std::string name;
        std::function<void()> callback;
        lv_obj_t* button = nullptr;
    };

    lv_obj_t* container_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* list_ = nullptr;
    std::vector<std::unique_ptr<Routine>> routines_;

    // written by the GUI when a button is tapped, or by next() and previous() from any task; read
    // by the autonomous task
    std::atomic<std::size_t> selectedIndex_{0};

    // a pick the buttons don't show yet, and one the card doesn't have yet: update() catches up
    std::atomic<bool> highlightStale_{false};
    std::atomic<bool> saveDue_{false};
    std::string path_; // rememberSelection()'s file, "" for none

    std::function<std::string()> status_;
    std::uint32_t lastStatusMs_ = 0;

    void pick(std::size_t index);
    void select(std::size_t index);
    void highlight();
    void addButton(Routine& routine);
    static void buttonClicked(lv_event_t* e);
};

} // namespace sapphirelib::gui
