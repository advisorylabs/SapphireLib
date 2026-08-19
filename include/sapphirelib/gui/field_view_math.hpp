/**
 * \file sapphirelib/gui/field_view_math.hpp
 *
 * Pixel-mapping math for drawing a field/pose view on the brain screen —
 * pure math, no PROS/LVGL dependency, so it can be unit-tested on a desktop
 * compiler. OdometryPage wraps this with the actual widget updates.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

namespace sapphirelib::gui {

struct ScreenPoint {
    std::int32_t x = 0;
    std::int32_t y = 0;
};

/// Maps a field-frame position (`xIn`, `yIn`, in the same inches/convention
/// as odom::Pose — origin at a fixed field corner, x right, y "upfield")
/// into a `viewWidthPx` x `viewHeightPx` pixel view showing a
/// `fieldWidthIn` x `fieldHeightIn` field. The field's origin is placed at
/// the view's bottom-left corner and y is flipped, since screen y increases
/// downward while field y increases upfield.
ScreenPoint fieldToScreen(double xIn, double yIn, double fieldWidthIn, double fieldHeightIn,
                          std::int32_t viewWidthPx, std::int32_t viewHeightPx);

/// The far endpoint of a heading-indicator line of `lengthPx`, drawn from
/// (`originX`, `originY`) pointing in `headingDeg`'s direction (0-360,
/// clockwise-positive, matching odom::Pose::headingDeg) — 0 points toward
/// the top of the view (screen -y), matching field +y being "upfield".
ScreenPoint headingIndicatorEndpoint(std::int32_t originX, std::int32_t originY, double headingDeg,
                                     std::int32_t lengthPx);

} // namespace sapphirelib::gui
