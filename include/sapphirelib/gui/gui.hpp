#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "liblvgl/lvgl.h"
#include "sapphirelib/gui/page.hpp"

namespace sapphirelib::gui {

/**
 * @brief Tab-based UI for the brain screen, with a branded header
 *
 * Optional: sapphirelib::initialize() never touches the screen. Add SapphireLib's pages (HomePage,
 * AutonSelectorPage, OdometryPage, ...) or your own with addPage()
 *
 * @b Example
 * @code {.cpp}
 * void initialize() {
 *     // first, so the header shows while the IMU calibrates
 *     static sapphirelib::gui::Gui gui("SapphireLib - 1234A");
 *     gui.addPage(std::make_unique<sapphirelib::gui::HomePage>(&drivetrain().imu()));
 *     gui.addPage(std::make_unique<sapphirelib::gui::OdometryPage>(odometry()));
 *     gui.start();
 * }
 * @endcode
 */
class Gui {
public:
    /**
     * @brief Construct a new Gui. Only one should exist, since it takes over the whole screen
     *
     * @param brandText text shown in the header above the tabs, e.g. "SapphireLib - 96671H"
     */
    explicit Gui(const char* brandText);

    /**
     * @brief Add a page as a new tab, building its widgets right away
     *
     * Safe before or after start()
     *
     * @param page the page. The Gui takes ownership
     */
    void addPage(std::unique_ptr<Page> page);

    /**
     * @brief Start refreshing pages on an LVGL timer
     *
     * An LVGL timer, not a PROS task, since LVGL isn't thread-safe. Only the visible tab refreshes
     * (plus pages that opt in with Page::updatesWhenHidden()). Only the first call starts the timer
     *
     * @param periodMs refresh period, in milliseconds. 50 by default
     */
    void start(std::uint32_t periodMs = 50);

    /**
     * @brief Replace the header with a red warning banner, visible on every tab
     *
     * For things a driver shouldn't miss, like a sensor in the wrong port. Safe to call every tick
     *
     * @param text the warning
     */
    void showWarning(const std::string& text);

    /**
     * @brief Restore the normal header
     */
    void clearWarning();

    /**
     * @brief Whether any page is running a routine that drives the robot (see Page::isBusy())
     *
     * Driver control should skip commanding the drivetrain while this is true. Safe from any task,
     * once the pages are added
     *
     * @b Example
     * @code {.cpp}
     * if (!gui.anyPageBusy()) drivetrain().arcade(throttle, turn);
     * @endcode
     */
    bool anyPageBusy() const;

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
