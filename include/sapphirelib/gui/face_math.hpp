#pragma once

#include <cstdint>

#include "sapphirelib/util/random.hpp"

namespace sapphirelib::gui {

/**
 * @brief One eye: a filled circle while open, flattening into a pill as it blinks or squints
 */
struct EyeShape {
    /** center x, in pixels from the screen's left */
    std::int32_t centerX = 0;

    /** center y, in pixels from the screen's top */
    std::int32_t centerY = 0;

    /** width, in pixels */
    std::int32_t widthPx = 0;

    /** height, in pixels */
    std::int32_t heightPx = 0;

    bool operator==(const EyeShape&) const = default;
};

/**
 * @brief One eyebrow: a straight bar from its outer end (toward the screen's edge) to its inner
 * end (toward the middle), in pixels from the screen's top left
 */
struct BrowShape {
    std::int32_t outerX = 0;
    std::int32_t outerY = 0;
    std::int32_t innerX = 0;
    std::int32_t innerY = 0;

    /** how thick the bar is, in pixels */
    std::int32_t thicknessPx = 0;

    bool operator==(const BrowShape&) const = default;
};

/**
 * @brief Everything FaceScreen draws. Index 0 is on the screen's left, 1 on its right
 */
struct FaceShape {
    EyeShape eyes[2];
    BrowShape brows[2];

    bool operator==(const FaceShape&) const = default;
};

/**
 * @brief The face's moods and timing: open eyes that blink every few seconds (now and then
 * twice), a squint, and a surprised look after a bump
 *
 * Pure: no LVGL, no clock, no sensors. Each frame, pass the time and what's going on, and draw the
 * FaceShape that comes back. Moods ease in and out over a few frames instead of snapping. A
 * surprise beats a squint, and no blink starts while surprised
 *
 * @note not thread-safe
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::gui::FaceAnimator face(480, 272);
 * face.reset(now);
 * // every frame
 * const sapphirelib::gui::FaceShape shape = face.update(now, liftRising, bumpedThisFrame);
 * @endcode
 */
class FaceAnimator {
public:
    /**
     * @brief Construct a new FaceAnimator
     *
     * @param widthPx screen width, in pixels. The face is laid out for the V5 brain's 480x272 and
     * scaled to fit anything else
     * @param heightPx screen height, in pixels
     * @param seed for the blink timing. 1 by default; each seed blinks on its own schedule
     */
    FaceAnimator(std::int32_t widthPx, std::int32_t heightPx, std::uint32_t seed = 1);

    /**
     * @brief Start over: eyes open, no squint or surprise, the next blink a few seconds away
     *
     * @param nowMs the current time, in milliseconds
     */
    void reset(std::uint32_t nowMs);

    /**
     * @brief Advance to now and get the face to draw
     *
     * @param nowMs the current time, in milliseconds
     * @param squinting squint while this is true
     * @param bumped true on the frame something bumped the robot: the face looks surprised for a
     * moment, and another bump meanwhile keeps it surprised
     * @return FaceShape the face to draw
     */
    FaceShape update(std::uint32_t nowMs, bool squinting, bool bumped);

    /**
     * @brief How shut a blink has the eyes, 0 (open) to 1 (shut)
     */
    double blink() const;

    /**
     * @brief How far the squint has eased in, 0 (none) to 1 (full)
     */
    double squint() const;

    /**
     * @brief How far the surprise has eased in, 0 (none) to 1 (full)
     */
    double surprise() const;

private:
    double blinkAt(std::uint32_t nowMs) const;
    void waitForNextBlink(std::uint32_t nowMs);
    FaceShape shape() const;

    std::int32_t widthPx_;
    std::int32_t heightPx_;
    Rng rng_;
    std::uint32_t lastMs_ = 0;

    // between blinks: when the wait started and how long it is
    std::uint32_t waitStartMs_ = 0;
    std::uint32_t waitMs_ = 0;
    // the blink at the end of the wait running now gets a second one right behind it
    bool doubleBlinkNext_ = false;
    bool blinking_ = false;
    std::uint32_t blinkStartMs_ = 0;

    // surprised from a bump until kSurpriseHoldMs after the latest one
    bool surprised_ = false;
    std::uint32_t bumpMs_ = 0;

    double blink_ = 0.0;
    double squint_ = 0.0;
    double surprise_ = 0.0;
};

} // namespace sapphirelib::gui
