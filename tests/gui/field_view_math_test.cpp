// Host-side unit test for sapphirelib::gui::fieldToScreen and
// headingIndicatorEndpoint — no PROS/LVGL dependencies, so it builds and
// runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/gui/field_view_math_test.cpp \
//       src/sapphirelib/gui/field_view_math.cpp \
//       -o field_view_math_test && ./field_view_math_test

#include <cassert>
#include <cstdio>

#include "sapphirelib/gui/field_view_math.hpp"

using sapphirelib::gui::fieldToScreen;
using sapphirelib::gui::headingIndicatorEndpoint;
using sapphirelib::gui::ScreenPoint;

namespace {

void expectPoint(ScreenPoint actual, std::int32_t expectedX, std::int32_t expectedY, const char* label) {
    if (actual.x != expectedX || actual.y != expectedY) {
        std::printf("FAIL %s: got (%d, %d), expected (%d, %d)\n", label, static_cast<int>(actual.x),
                    static_cast<int>(actual.y), static_cast<int>(expectedX), static_cast<int>(expectedY));
        assert(false);
    }
}

void testFieldOriginIsBottomLeft() {
    // Field (0,0) should land at the view's bottom-left pixel.
    expectPoint(fieldToScreen(0.0, 0.0, 144.0, 144.0, 150, 150), 0, 150, "origin");
}

void testFieldFarCornerIsTopRight() {
    // Field (144,144) — the far corner — should land at the view's
    // top-right pixel: field +y is upfield, screen +y is downward, so this
    // is where the y-flip matters most.
    expectPoint(fieldToScreen(144.0, 144.0, 144.0, 144.0, 150, 150), 150, 0, "far corner");
}

void testFieldCenter() {
    expectPoint(fieldToScreen(72.0, 72.0, 144.0, 144.0, 150, 150), 75, 75, "center");
}

void testHeadingZeroPointsUp() {
    // Heading 0 (facing +y / "upfield") should point toward the top of the
    // view, i.e. decreasing screen y.
    expectPoint(headingIndicatorEndpoint(50, 50, 0.0, 20), 50, 30, "heading 0");
}

void testHeadingEastPointsRight() {
    // Heading 90 (facing +x / "east") should point right, i.e. increasing
    // screen x, with no vertical change.
    expectPoint(headingIndicatorEndpoint(50, 50, 90.0, 20), 70, 50, "heading 90");
}

void testHeadingSouthPointsDown() {
    expectPoint(headingIndicatorEndpoint(50, 50, 180.0, 20), 50, 70, "heading 180");
}

} // namespace

int main() {
    testFieldOriginIsBottomLeft();
    testFieldFarCornerIsTopRight();
    testFieldCenter();
    testHeadingZeroPointsUp();
    testHeadingEastPointsRight();
    testHeadingSouthPointsDown();
    std::puts("field_view_math_test: all assertions passed");
    return 0;
}
