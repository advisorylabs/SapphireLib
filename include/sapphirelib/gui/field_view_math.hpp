#pragma once

#include <cstdint>

namespace sapphirelib::gui {

/**
 * @brief A point on the screen, in pixels
 */
struct ScreenPoint {
    std::int32_t x = 0;
    std::int32_t y = 0;
};

/**
 * @brief Map a field position into a pixel view of the field
 *
 * The field's origin goes at the view's bottom left, with y flipped since screen y grows downward
 *
 * @param xIn field x, in inches
 * @param yIn field y, in inches
 * @param fieldWidthIn field width, in inches
 * @param fieldHeightIn field height, in inches
 * @param viewWidthPx view width, in pixels
 * @param viewHeightPx view height, in pixels
 * @return ScreenPoint the position in the view
 */
ScreenPoint fieldToScreen(double xIn, double yIn, double fieldWidthIn, double fieldHeightIn,
                          std::int32_t viewWidthPx, std::int32_t viewHeightPx);

/**
 * @brief Get the end of a heading line drawn from a point
 *
 * @param originX line start x, in pixels
 * @param originY line start y, in pixels
 * @param headingDeg heading, 0-360 degrees clockwise. 0 points up the screen
 * @param lengthPx line length, in pixels
 * @return ScreenPoint the line's end
 */
ScreenPoint headingIndicatorEndpoint(std::int32_t originX, std::int32_t originY, double headingDeg,
                                     std::int32_t lengthPx);

} // namespace sapphirelib::gui
