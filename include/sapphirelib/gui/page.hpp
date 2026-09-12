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

/// Sets a label's text only if it actually differs from what's already
/// there. lv_label_set_text() unconditionally reallocates the label's text
/// buffer and invalidates the object, so LVGL redraws that region on its
/// next refresh even when the new text is byte-identical to the old. A page
/// that reformats the same numbers every tick (which is most of them) would
/// otherwise keep the whole screen in a permanent redraw loop; the V5's
/// 480x240 ARGB8888 surface makes that expensive enough to feel. Prefer this
/// over lv_label_set_text() everywhere in a Page::update().
void setLabelText(lv_obj_t* label, const char* text);

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

    /// Called on Gui's refresh timer (see Gui::start()) while this page is
    /// the active tab — plus once more on the tick it becomes active, so it
    /// never renders a frame of stale data. Keep it cheap (label text
    /// updates, not expensive computation).
    virtual void update() {}

    /// Override to return true if this page must keep running update() even
    /// while a different tab is showing. Off by default: a hidden page's
    /// widgets aren't on screen, so refreshing them is pure cost, and with
    /// several tabs registered that dominates the GUI's steady-state load.
    ///
    /// Say true only for a page whose update() has a side effect beyond its
    /// own widgets — DiagnosticsPage is the built-in example, since it
    /// raises Gui::showWarning()'s screen-wide banner and a sensor coming
    /// loose shouldn't go unnoticed just because the driver is looking at
    /// another tab. A page that says true should throttle itself (see
    /// DiagnosticsPage's poll interval) rather than lean on Gui's timer
    /// period.
    virtual bool updatesWhenHidden() const { return false; }
};

} // namespace sapphirelib::gui
