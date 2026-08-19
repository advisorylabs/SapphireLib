/**
 * \file sapphirelib/gui/gui.hpp
 *
 * Branded, tab-based UI for the V5 brain screen — SapphireLib's default
 * replacement for LLEMU.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "liblvgl/lvgl.h"
#include "sapphirelib/gui/page.hpp"

namespace sapphirelib::gui {

/// Entirely opt-in: sapphirelib::initialize() never touches the screen, so
/// a team that wants their own UI (or none) just never constructs a Gui —
/// nothing else in the library depends on it. Add SapphireLib's default
/// pages (HomePage, AutonSelectorPage, OdometryPage) and/or your own Page
/// subclasses via addPage(); a handful of tabs fit comfortably across the
/// screen.
class Gui {
public:
    /// `brandText` is shown in a persistent header above the tabs, e.g.
    /// "SapphireLib - 96671H". Builds the header and an empty tabview
    /// immediately on lv_screen_active() — call addPage() to populate it,
    /// then start() to bring it to life. Only one Gui should exist at a
    /// time (it takes over the whole active screen).
    explicit Gui(const char* brandText);

    /// Registers a page: adds it as a new tab and builds its widgets
    /// immediately (see Page::build()). Takes ownership. Safe to call
    /// before or after start(), and safe to call more than once.
    void addPage(std::unique_ptr<Page> page);

    /// Starts refreshing every registered page's update() on an LVGL timer
    /// — not a separate PROS task. LVGL isn't thread-safe: every callback
    /// that touches a widget has to run from the same context LVGL's own
    /// display task already drives its timers from, which is exactly what
    /// lv_timer_create() guarantees and a raw pros::Task wouldn't. Call
    /// once; addPage() still works for pages added after start().
    void start(std::uint32_t periodMs = 50);

    /// Replaces the header with a warning banner (red background, `text`
    /// in place of the brand text) — visible on every tab, not just
    /// whichever page noticed the problem. Meant for things a driver
    /// shouldn't be able to miss, like a diag::DiagnosticsPage finding a
    /// sensor in the wrong port. Safe to call repeatedly (e.g. every
    /// update() tick) — it doesn't flicker if the text hasn't changed.
    void showWarning(const std::string& text);

    /// Restores the header to the normal brand text/color.
    void clearWarning();

private:
    lv_obj_t* header_;
    lv_obj_t* headerLabel_;
    std::string brandText_;
    bool warningActive_ = false;

    lv_obj_t* tabview_;
    std::vector<std::unique_ptr<Page>> pages_;
    lv_timer_t* timer_ = nullptr;

    void tick();
    static void timerTrampoline(lv_timer_t* timer);
};

} // namespace sapphirelib::gui
