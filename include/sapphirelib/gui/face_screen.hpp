#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

#include "liblvgl/lvgl.h"
#include "sapphirelib/gui/face_math.hpp"
#include "sapphirelib/sensors/bump_detector.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::gui {

/**
 * @brief A face over the whole brain screen: two eyes, two eyebrows and a bowtie on black,
 * nothing else
 *
 * The eyes blink every few seconds, squint while squinting() says so (a lift on its way up, say),
 * go wide with the brows raised when the IMU feels a bump (see sensors::BumpDetector), and wink
 * each time winks() goes up (a count of scores, say), the bowtie spinning a full turn. It
 * covers everything, the Gui's header and tabs included, and comes up on its own for a match:
 * autonomous and driver control under competition control, not while disabled, so the auton
 * selector is there before the match. show() puts it up any other time
 *
 * A tap anywhere takes it down. One that does during a match keeps it down until the next period
 * (autonomous to driver control), when it comes back up
 *
 * Optional, and separate from Gui: it draws on LVGL's top layer, above any Gui
 *
 * @b Example
 * @code {.cpp}
 * static sapphirelib::gui::FaceScreen face({
 *     .squinting = [] { return liftRising(); },
 *     .winks = [] { return scoreCount(); },
 *     .imu = &drivetrain().imu(),
 * });
 * face.start();
 * home->addButton("Face", [] { face.show(); });
 * @endcode
 */
class FaceScreen {
public:
    /**
     * @brief What the face watches and how it looks
     */
    struct Config {
        /**
         * squint while this returns true. Polled every frame on the GUI's task, so keep it quick.
         * Empty never squints
         */
        std::function<bool()> squinting;

        /**
         * a count of things worth a wink: the face winks on each frame it's gone up since the
         * last one. Polled every frame on the GUI's task while the face is up, so keep it quick;
         * what it counted while the face was down doesn't wink once it's back up. Empty never
         * winks
         */
        std::function<std::uint32_t()> winks;

        /** watched for bumps, which make the face look surprised. nullptr for none */
        sensors::Imu* imu = nullptr;

        /** what counts as a bump */
        sensors::BumpConfig bump;

        /** come up on its own during autonomous and driver control under competition control */
        bool showDuringMatch = true;

        /** the face's color (eyes, brows, mouth and bowtie), 0xRRGGBB. SapphireLib's header red
         * by default */
        std::uint32_t colorHex = 0xf87171;

        /** frame period, in milliseconds. 20 by default */
        std::uint32_t periodMs = 20;
    };

    /**
     * @brief Construct a new FaceScreen. Touches nothing until start()
     *
     * @param config what it watches and how it looks
     */
    explicit FaceScreen(Config config);

    /**
     * @brief Build the face, hidden, and start its frame timer
     *
     * Call once from initialize(), where the Gui's pages are built. Only the first call does
     * anything
     */
    void start();

    /**
     * @brief Put the face up now, outside a match too, until the screen is tapped. Safe from any
     * task; it comes up on the next frame
     */
    void show();

    /**
     * @brief Whether the face is up. Safe from any task
     */
    bool showing() const;

private:
    // which part of a match is running, under competition control
    enum class Period { none, autonomous, driver };

    static Period currentPeriod();
    void tick();
    void draw(const FaceShape& face);
    static void timerTrampoline(lv_timer_t* timer);
    static void tapped(lv_event_t* e);
    static void drawBowtie(lv_event_t* e);

    Config config_;
    FaceAnimator animator_;
    sensors::BumpDetector bumps_;

    // show()'s request, from any task. A tap clears it
    std::atomic<bool> requested_{false};
    std::atomic<bool> showing_{false};

    // GUI task only. A tap during a match keeps the face down until the period changes
    Period period_ = Period::none;
    bool dismissed_ = false;
    // Config::winks' count as of the last frame the face was up
    std::uint32_t winksSeen_ = 0;

    lv_obj_t* root_ = nullptr;
    lv_obj_t* eyes_[2] = {};
    lv_obj_t* brows_[2] = {};
    lv_obj_t* mouth_ = nullptr;
    // drawn by drawBowtie(): LVGL has no triangle object, and the wings turn
    lv_obj_t* bowtie_ = nullptr;
    // members because lv_line_set_points() keeps the pointer, not a copy
    lv_point_precise_t browPoints_[2][2] = {};
    // what's on the screen now, so a frame that changes nothing redraws nothing
    FaceShape drawn_;
    bool drawnValid_ = false;
    lv_timer_t* timer_ = nullptr;
};

} // namespace sapphirelib::gui
