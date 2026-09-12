#include "sapphirelib/gui/odometry_page.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "pros/rtos.hpp"
#include "sapphirelib/gui/field_view_math.hpp"
#include "sapphirelib/odom/odometry_math.hpp"

namespace sapphirelib::gui {

namespace {

// Sized to fit the tab content area outright rather than overflow it. The
// page gets 480x179 (see PidTunerPage's kReadoutX comment for where that
// number comes from), and the field view starts 24px down under the pose
// readout — so at the old 170px it ran to y=194 and the bottom ~15px of the
// field, the half the robot usually starts on, was only reachable by
// scrolling the tab. 150px squares it away with room to spare, which also
// lets the container stop being a scroll area at all.
constexpr std::int32_t kFieldViewSizePx = 150;
constexpr std::int32_t kRobotDotSizePx = 8;
constexpr std::int32_t kHeadingLineLengthPx = 16;

// Calibration controls sit to the right of the field view rather than below
// it — the tab content area is only ~179px tall on the V5 brain screen
// (240px display minus the header and tab bar), barely enough for the field
// view alone, and touchscreen scrolling isn't reliable enough to rely on to
// reach anything stacked underneath.
constexpr std::int32_t kCalibrateColumnX = 4 + kFieldViewSizePx + 10;
constexpr std::int32_t kCalibrateColumnWidthPx = 480 - kCalibrateColumnX - 4;
constexpr std::int32_t kCalibrateButtonY = 24;
constexpr std::int32_t kCalibrateButtonHeightPx = 30;
constexpr std::int32_t kCalibrateStatusY = kCalibrateButtonY + kCalibrateButtonHeightPx + 10;

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kCalibrationLoopDelayMs = 10;

} // namespace

OdometryPage::OdometryPage(odom::Odometry& odometry, double fieldWidthIn, double fieldHeightIn)
    : odometry_(odometry), fieldWidthIn_(fieldWidthIn), fieldHeightIn_(fieldHeightIn) {}

void OdometryPage::enableOffsetCalibration(sensors::Imu& imu, const odom::TrackingWheel* verticalWheel,
                                           const odom::TrackingWheel* horizontalWheel,
                                           std::function<void(double)> setSpin, double spinPower,
                                           double turns) {
    calibImu_ = &imu;
    calibVertical_ = verticalWheel;
    calibHorizontal_ = horizontalWheel;
    calibSetSpin_ = std::move(setSpin);
    calibSpinPower_ = spinPower;
    calibTurns_ = turns;
}

bool OdometryPage::isCalibrating() const { return calibrating_.load(); }

const char* OdometryPage::title() const { return "Odom"; }

void OdometryPage::build(lv_obj_t* container) {
    // Exact positions, no theme padding to offset them — and nothing here
    // overflows, so the tab doesn't need to scroll. Dropping the scroll flag
    // also stops LVGL recomputing this container's scroll extents every time
    // the robot dot moves.
    lv_obj_set_style_pad_all(container, 0, 0);
    lv_obj_remove_flag(container, LV_OBJ_FLAG_SCROLLABLE);

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
    // A generic lv_obj picks up the theme's card styling, shadow included.
    // Nothing here needs one, and a shadow under a container whose child
    // moves every tick means recomputing it on every redraw.
    lv_obj_set_style_shadow_width(fieldView_, 0, 0);
    // The field view is a fixed viewport onto the field, not a scroll area.
    // fieldToScreen() doesn't clamp, so a pose at or past a field edge puts
    // the robot dot partly outside these bounds — which, on a scrollable
    // container, makes LVGL recompute the scroll extents every time the dot
    // moves. Clipping is the behavior we want anyway.
    lv_obj_remove_flag(fieldView_, LV_OBJ_FLAG_SCROLLABLE);

    robotDot_ = lv_obj_create(fieldView_);
    lv_obj_set_size(robotDot_, kRobotDotSizePx, kRobotDotSizePx);
    lv_obj_set_style_radius(robotDot_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(robotDot_, lv_color_hex(0x22d3ee), 0);
    lv_obj_set_style_border_width(robotDot_, 0, 0);
    lv_obj_set_style_shadow_width(robotDot_, 0, 0);

    headingLine_ = lv_line_create(fieldView_);
    lv_obj_set_style_line_color(headingLine_, lv_color_hex(0x22d3ee), 0);
    lv_obj_set_style_line_width(headingLine_, 2, 0);
    lv_obj_set_style_line_rounded(headingLine_, true, 0);

    calibrateButton_ = lv_button_create(container);
    lv_obj_set_pos(calibrateButton_, kCalibrateColumnX, kCalibrateButtonY);
    lv_obj_set_size(calibrateButton_, kCalibrateColumnWidthPx, kCalibrateButtonHeightPx);
    lv_obj_t* calibrateLabel = lv_label_create(calibrateButton_);
    lv_label_set_text(calibrateLabel, "Calibrate Offsets");
    lv_obj_center(calibrateLabel);
    lv_obj_add_event_cb(calibrateButton_, &OdometryPage::calibrateClicked, LV_EVENT_CLICKED, this);

    calibStatusLabel_ = lv_label_create(container);
    lv_obj_set_pos(calibStatusLabel_, kCalibrateColumnX, kCalibrateStatusY);
    lv_obj_set_width(calibStatusLabel_, kCalibrateColumnWidthPx);
    lv_label_set_long_mode(calibStatusLabel_, LV_LABEL_LONG_WRAP);
    lv_label_set_text(calibStatusLabel_, calibSetSpin_ ? "Spin the bot free, then tap Calibrate" : "");

    update();
}

void OdometryPage::update() {
    const odom::Pose pose = odometry_.getPose();

    char buf[64];
    std::snprintf(buf, sizeof(buf), "X: %.1f  Y: %.1f  H: %.1f deg", pose.xIn, pose.yIn, pose.headingDeg);
    setLabelText(poseLabel_, buf);

    const ScreenPoint dot = fieldToScreen(pose.xIn, pose.yIn, fieldWidthIn_, fieldHeightIn_,
                                          fieldViewWidthPx_, fieldViewHeightPx_);
    const ScreenPoint tip = headingIndicatorEndpoint(dot.x, dot.y, pose.headingDeg, kHeadingLineLengthPx);

    // Both the dot and the line are repositioned only when they'd actually
    // land on a different pixel. The field view maps 144in of field across
    // 150px, so a stationary robot's odometry noise moves it a fraction of a
    // pixel — and lv_obj_set_pos()/lv_line_set_points() invalidate
    // unconditionally, which would have LVGL redrawing this view (and the
    // field rectangle behind it) every single tick for no visible change.
    const bool dotMoved = dot.x != lastDotX_ || dot.y != lastDotY_;
    const bool tipMoved = tip.x != lastTipX_ || tip.y != lastTipY_;

    if (dotMoved) {
        lv_obj_set_pos(robotDot_, dot.x - kRobotDotSizePx / 2, dot.y - kRobotDotSizePx / 2);
    }
    if (dotMoved || tipMoved) {
        // headingPoints_ is a member on purpose — lv_line_set_points() keeps
        // the pointer, not a copy. See its declaration.
        headingPoints_[0] = {static_cast<lv_value_precise_t>(dot.x),
                             static_cast<lv_value_precise_t>(dot.y)};
        headingPoints_[1] = {static_cast<lv_value_precise_t>(tip.x),
                             static_cast<lv_value_precise_t>(tip.y)};
        lv_line_set_points(headingLine_, headingPoints_, 2);
    }

    lastDotX_ = dot.x;
    lastDotY_ = dot.y;
    lastTipX_ = tip.x;
    lastTipY_ = tip.y;

    if (!calibStatusLabel_) return;

    if (calibrating_.load()) {
        setLabelText(calibStatusLabel_, "Calibrating... keep the bot clear and spinning free");
    } else if (calibResultsReady_.load()) {
        char calibBuf[64];
        if (calibVertical_ && calibHorizontal_) {
            std::snprintf(calibBuf, sizeof(calibBuf), "Vertical: %.2fin  Horizontal: %.2fin (applied)",
                          calibVerticalOffsetIn_.load(), calibHorizontalOffsetIn_.load());
        } else if (calibVertical_) {
            std::snprintf(calibBuf, sizeof(calibBuf), "Vertical offset: %.2fin (applied)",
                          calibVerticalOffsetIn_.load());
        } else {
            std::snprintf(calibBuf, sizeof(calibBuf), "Horizontal offset: %.2fin (applied)",
                          calibHorizontalOffsetIn_.load());
        }
        setLabelText(calibStatusLabel_, calibBuf);
    }
}

void OdometryPage::runCalibration() {
    if (!calibSetSpin_ || (!calibVertical_ && !calibHorizontal_) || calibrating_.load()) return;

    calibrating_.store(true);
    calibResultsReady_.store(false);

    // Runs on a background task, not this LVGL-owned callback, so the
    // multi-second calibration spin doesn't freeze the whole screen — same
    // pattern as PidTunerPage's Run Test/Auto-Tune. Only ever touches
    // atomics here, never an LVGL widget directly (LVGL isn't thread-safe);
    // update() reflects calibrating_/calibResultsReady_ from the correct
    // context.
    pros::Task([this] {
        const double startVerticalIn = calibVertical_ ? calibVertical_->getDistanceIn() : 0.0;
        const double startHorizontalIn = calibHorizontal_ ? calibHorizontal_->getDistanceIn() : 0.0;

        // getCumulativeHeadingDeg() must be polled regularly (see its class
        // comment) or an in-between multi-turn spin could wrap past 180
        // degrees between reads and get misdetected as a much smaller turn
        // the other way — this loop's period is comfortably fast enough.
        const double startCumulativeDeg = calibImu_->getCumulativeHeadingDeg();
        const double targetDeg = std::fabs(calibTurns_) * 360.0;
        const double spinPower = calibTurns_ < 0.0 ? -calibSpinPower_ : calibSpinPower_;

        calibSetSpin_(spinPower);
        while (std::fabs(calibImu_->getCumulativeHeadingDeg() - startCumulativeDeg) < targetDeg) {
            pros::delay(kCalibrationLoopDelayMs);
        }
        calibSetSpin_(0.0);

        const double rotatedRadians =
            (calibImu_->getCumulativeHeadingDeg() - startCumulativeDeg) * (kPi / 180.0);

        odom::OdometryConfig config = odometry_.getConfig();
        if (calibVertical_) {
            const double wheelDistanceIn = calibVertical_->getDistanceIn() - startVerticalIn;
            const double offsetIn = odom::calibrateTrackingWheelOffsetIn(wheelDistanceIn, rotatedRadians);
            calibVerticalOffsetIn_.store(offsetIn);
            config.verticalOffsetIn = offsetIn;
        }
        if (calibHorizontal_) {
            const double wheelDistanceIn = calibHorizontal_->getDistanceIn() - startHorizontalIn;
            const double offsetIn = odom::calibrateTrackingWheelOffsetIn(wheelDistanceIn, rotatedRadians);
            calibHorizontalOffsetIn_.store(offsetIn);
            config.horizontalOffsetIn = offsetIn;
        }
        // Applied straight to the live Odometry so the calibrated offsets
        // take effect immediately — no rebuild/redeploy needed.
        odometry_.setConfig(config);

        calibResultsReady_.store(true);
        calibrating_.store(false);
    });
}

void OdometryPage::calibrateClicked(lv_event_t* e) {
    static_cast<OdometryPage*>(lv_event_get_user_data(e))->runCalibration();
}

} // namespace sapphirelib::gui
