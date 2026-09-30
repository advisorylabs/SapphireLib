#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "sapphirelib/gui/page.hpp"

namespace sapphirelib::gui {

/**
 * @brief Autonomous routine picker
 *
 * The driver taps a routine on the brain screen before the match, and autonomous() runs it
 *
 * @b Example
 * @code {.cpp}
 * auto selector = std::make_unique<sapphirelib::gui::AutonSelectorPage>();
 * sapphirelib::gui::AutonSelectorPage* autons = selector.get();
 * autons->addRoutine("Left side", leftSide);
 * autons->addRoutine("Skills", skills);
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

    const char* title() const override;
    void build(lv_obj_t* container) override;

private:
    struct Routine {
        std::string name;
        std::function<void()> callback;
        lv_obj_t* button = nullptr;
    };

    lv_obj_t* container_ = nullptr;
    lv_obj_t* list_ = nullptr;
    std::vector<std::unique_ptr<Routine>> routines_;

    // written by the GUI when a button is tapped, read by the autonomous task
    std::atomic<std::size_t> selectedIndex_{0};

    void select(std::size_t index);
    static void buttonClicked(lv_event_t* e);
};

} // namespace sapphirelib::gui
