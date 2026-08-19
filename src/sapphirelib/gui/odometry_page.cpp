#include "sapphirelib/gui/odometry_page.hpp"

#include <cstdio>

#include "sapphirelib/gui/field_view_math.hpp"

namespace sapphirelib::gui {

namespace {

constexpr std::int32_t kFieldViewSizePx = 170;
constexpr std::int32_t kRobotDotSizePx = 8;
constexpr std::int32_t kHeadingLineLengthPx = 16;

} // namespace

OdometryPage::OdometryPage(const odom::Odometry& odometry, double fieldWidthIn, double fieldHeightIn)
    : odometry_(odometry), fieldWidthIn_(fieldWidthIn), fieldHeightIn_(fieldHeightIn) {}

const char* OdometryPage::title() const { return "Odom"; }

void OdometryPage::build(lv_obj_t* container) {
    poseLabel_ = lv_label_create(container);
    lv_obj_set_pos(poseLabel_, 4, 2);

    fieldViewWidthPx_ = kFieldViewSizePx;
    fieldViewHeightPx_ = kFieldViewSizePx;

    fieldView_ = lv_obj_create(container);
    lv_obj_set_size(fieldView_, fieldViewWidthPx_, fieldViewHeightPx_);
    lv_obj_set_pos(fieldView_, 4, 24);
    lv_obj_set_style_bg_color(fieldView_, lv_color_hex(0x0f172a), 0);
    lv_obj_set_style_border_color(fieldView_, lv_color_hex(0x334155), 0);
    lv_obj_set_style_border_width(fieldView_, 2, 0);
    lv_obj_set_style_radius(fieldView_, 4, 0);

    robotDot_ = lv_obj_create(fieldView_);
    lv_obj_set_size(robotDot_, kRobotDotSizePx, kRobotDotSizePx);
    lv_obj_set_style_radius(robotDot_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(robotDot_, lv_color_hex(0x22d3ee), 0);
    lv_obj_set_style_border_width(robotDot_, 0, 0);

    headingLine_ = lv_line_create(fieldView_);
    lv_obj_set_style_line_color(headingLine_, lv_color_hex(0x22d3ee), 0);
    lv_obj_set_style_line_width(headingLine_, 2, 0);
    lv_obj_set_style_line_rounded(headingLine_, true, 0);

    update();
}

void OdometryPage::update() {
    const odom::Pose pose = odometry_.getPose();

    char buf[64];
    std::snprintf(buf, sizeof(buf), "X: %.1f  Y: %.1f  H: %.1f deg", pose.xIn, pose.yIn, pose.headingDeg);
    lv_label_set_text(poseLabel_, buf);

    const ScreenPoint dot = fieldToScreen(pose.xIn, pose.yIn, fieldWidthIn_, fieldHeightIn_,
                                          fieldViewWidthPx_, fieldViewHeightPx_);
    lv_obj_set_pos(robotDot_, dot.x - kRobotDotSizePx / 2, dot.y - kRobotDotSizePx / 2);

    const ScreenPoint tip = headingIndicatorEndpoint(dot.x, dot.y, pose.headingDeg, kHeadingLineLengthPx);
    lv_point_precise_t points[2] = {
        {static_cast<lv_value_precise_t>(dot.x), static_cast<lv_value_precise_t>(dot.y)},
        {static_cast<lv_value_precise_t>(tip.x), static_cast<lv_value_precise_t>(tip.y)},
    };
    lv_line_set_points(headingLine_, points, 2);
}

} // namespace sapphirelib::gui
