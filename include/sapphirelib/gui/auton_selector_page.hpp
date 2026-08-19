/**
 * \file sapphirelib/gui/auton_selector_page.hpp
 *
 * SapphireLib's default autonomous-routine picker.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "sapphirelib/gui/page.hpp"

namespace sapphirelib::gui {

/// Register named routines with addRoutine(); the driver taps one on the
/// brain screen to select it before a match, and autonomous() calls run()
/// to execute whichever was selected.
class AutonSelectorPage : public Page {
public:
    AutonSelectorPage() = default;

    /// Registers a selectable routine. Safe to call before or after
    /// build() — if called after, the button is added immediately. The
    /// first registered routine is selected by default, so run() always
    /// has something to execute even if the driver forgets to pick.
    void addRoutine(std::string name, std::function<void()> routine);

    /// Runs whichever routine is currently selected. No-op if none are
    /// registered.
    void run() const;

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
    std::size_t selectedIndex_ = 0;

    void select(std::size_t index);
    static void buttonClicked(lv_event_t* e);
};

} // namespace sapphirelib::gui
