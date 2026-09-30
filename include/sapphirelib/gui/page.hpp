#pragma once

#include "liblvgl/lvgl.h"

namespace sapphirelib::gui {

/**
 * @brief Set a label's text, only if it changed
 *
 * lv_label_set_text() redraws even when the text is the same, and a page that reformats the same
 * numbers every tick would keep the screen redrawing forever. Use this in every Page::update()
 *
 * @param label the label
 * @param text the new text
 */
void setLabelText(lv_obj_t* label, const char* text);

/**
 * @brief One tab of brain screen UI. Subclass it for your own pages
 *
 * @b Example
 * @code {.cpp}
 * class LiftPage : public sapphirelib::gui::Page {
 * public:
 *     const char* title() const override { return "Lift"; }
 *     void build(lv_obj_t* container) override { label_ = lv_label_create(container); }
 *     void update() override {
 *         char text[32];
 *         std::snprintf(text, sizeof(text), "lift: %.0f", lift.position());
 *         sapphirelib::gui::setLabelText(label_, text);
 *     }
 *
 * private:
 *     lv_obj_t* label_ = nullptr;
 * };
 * @endcode
 */
class Page {
public:
    virtual ~Page() = default;

    /**
     * @brief Get the tab's label. Keep it short
     */
    virtual const char* title() const = 0;

    /**
     * @brief Build the page's widgets. Called once, when the page is added
     *
     * @param container the tab's content area
     */
    virtual void build(lv_obj_t* container) = 0;

    /**
     * @brief Refresh the page. Called on the Gui's timer while the tab is showing
     *
     * Keep it cheap: label updates, not heavy work
     */
    virtual void update() {}

    /**
     * @brief Whether update() must keep running while another tab is showing. false by default
     *
     * Only for pages whose update() does something beyond their own widgets, like DiagnosticsPage
     * raising the warning banner. Such a page should throttle itself
     */
    virtual bool updatesWhenHidden() const { return false; }

    /**
     * @brief Whether the page is running a routine that drives the robot, like a calibration spin
     *
     * @note called from other tasks, so read an atomic here, never a widget
     */
    virtual bool isBusy() const { return false; }
};

} // namespace sapphirelib::gui
