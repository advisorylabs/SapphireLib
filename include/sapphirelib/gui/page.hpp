/**
 * \file sapphirelib/gui/page.hpp
 *
 * The extension point for SapphireLib's brain-screen GUI: implement this to
 * add a tab to Gui, whether it's one of SapphireLib's own default pages
 * (HomePage, AutonSelectorPage, OdometryPage) or a team's custom one.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "liblvgl/lvgl.h"

namespace sapphirelib::gui {

/// One tab's worth of brain-screen UI. Subclass this for anything Gui
/// should show — a couple of custom pages fit comfortably alongside
/// SapphireLib's defaults on the tab bar.
class Page {
public:
    virtual ~Page() = default;

    /// Short label shown on the page's tab. Keep it short — there's not
    /// much horizontal room across a handful of tabs on a 480px-wide
    /// screen.
    virtual const char* title() const = 0;

    /// Builds this page's widgets inside `container` (the tab's content
    /// area, already sized and positioned by the tabview). Called exactly
    /// once, when the page is registered with Gui::addPage(), regardless of
    /// whether the page is the active tab yet — LVGL keeps every tab's
    /// content alive, not just the visible one.
    virtual void build(lv_obj_t* container) = 0;

    /// Called on Gui's refresh timer (see Gui::start()) for every
    /// registered page, active tab or not — keep this cheap (label text
    /// updates, not expensive computation), since it runs whether or not
    /// the page is currently visible.
    virtual void update() {}
};

} // namespace sapphirelib::gui
