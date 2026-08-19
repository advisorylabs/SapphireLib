#include "sapphirelib/gui/field_view_math.hpp"

#include <cmath>

namespace sapphirelib::gui {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
} // namespace

ScreenPoint fieldToScreen(double xIn, double yIn, double fieldWidthIn, double fieldHeightIn,
                          std::int32_t viewWidthPx, std::int32_t viewHeightPx) {
    const double xPx = (xIn / fieldWidthIn) * static_cast<double>(viewWidthPx);
    const double yPx = static_cast<double>(viewHeightPx) - (yIn / fieldHeightIn) * static_cast<double>(viewHeightPx);
    return ScreenPoint{static_cast<std::int32_t>(std::lround(xPx)),
                        static_cast<std::int32_t>(std::lround(yPx))};
}

ScreenPoint headingIndicatorEndpoint(std::int32_t originX, std::int32_t originY, double headingDeg,
                                     std::int32_t lengthPx) {
    const double headingRad = headingDeg * kDegToRad;
    const double dxPx = static_cast<double>(lengthPx) * std::sin(headingRad);
    const double dyPx = -static_cast<double>(lengthPx) * std::cos(headingRad);
    return ScreenPoint{originX + static_cast<std::int32_t>(std::lround(dxPx)),
                        originY + static_cast<std::int32_t>(std::lround(dyPx))};
}

} // namespace sapphirelib::gui
