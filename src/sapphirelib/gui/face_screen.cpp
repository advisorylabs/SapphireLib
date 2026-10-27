#include "sapphirelib/gui/face_screen.hpp"

#include <algorithm>
#include <cstdlib>
#include <utility>

#include "pros/misc.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/clock.hpp"

namespace sapphirelib::gui {

namespace {

constexpr std::uint32_t kBackgroundColor = 0x000000;

// a plain object with no theme styling: no padding, border, shadow, or scrolling, and not
// clickable, so a tap on an eye or a brow lands on the face behind it
void stripStyle(lv_obj_t* obj) {
    lv_obj_remove_style_all(obj);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

} // namespace

FaceScreen::FaceScreen(Config config)
    : config_(std::move(config)), animator_(0, 0), bumps_(config_.bump) {}

void FaceScreen::start() {
    if (timer_) return;

    const std::int32_t widthPx = LV_HOR_RES;
    const std::int32_t heightPx = LV_VER_RES;
    // a different blink schedule each run; nothing depends on it
    animator_ = FaceAnimator(widthPx, heightPx, millis());

    // on the top layer, over every screen and the Gui's header and tabs
    root_ = lv_obj_create(lv_layer_top());
    stripStyle(root_);
    lv_obj_add_flag(root_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(root_, widthPx, heightPx);
    lv_obj_set_pos(root_, 0, 0);
    lv_obj_set_style_bg_color(root_, lv_color_hex(kBackgroundColor), 0);
    lv_obj_set_style_bg_opa(root_, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(root_, &FaceScreen::tapped, LV_EVENT_CLICKED, this);
    lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);

    for (int side = 0; side < 2; ++side) {
        eyes_[side] = lv_obj_create(root_);
        stripStyle(eyes_[side]);
        lv_obj_set_style_bg_color(eyes_[side], lv_color_hex(config_.colorHex), 0);
        lv_obj_set_style_bg_opa(eyes_[side], LV_OPA_COVER, 0);
        // a circle while square, a pill while flattened by a blink or a squint
        lv_obj_set_style_radius(eyes_[side], LV_RADIUS_CIRCLE, 0);

        brows_[side] = lv_line_create(root_);
        stripStyle(brows_[side]);
        lv_obj_set_style_line_color(brows_[side], lv_color_hex(config_.colorHex), 0);
        // square ends, so a thick line is a bar
        lv_obj_set_style_line_rounded(brows_[side], false, 0);
    }

    mouth_ = lv_obj_create(root_);
    stripStyle(mouth_);
    lv_obj_set_style_bg_color(mouth_, lv_color_hex(config_.colorHex), 0);
    lv_obj_set_style_bg_opa(mouth_, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(mouth_, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_flag(mouth_, LV_OBJ_FLAG_HIDDEN);

    timer_ = lv_timer_create(&FaceScreen::timerTrampoline, config_.periodMs, this);
}

void FaceScreen::show() { requested_.store(true); }

bool FaceScreen::showing() const { return showing_.load(); }

FaceScreen::Period FaceScreen::currentPeriod() {
    const std::uint8_t status = pros::competition::get_status();
    // with no field or switch plugged in, opcontrol runs on its own and is no match
    if ((status & COMPETITION_CONNECTED) == 0 || (status & COMPETITION_DISABLED) != 0) {
        return Period::none;
    }
    return (status & COMPETITION_AUTONOMOUS) != 0 ? Period::autonomous : Period::driver;
}

void FaceScreen::timerTrampoline(lv_timer_t* timer) {
    static_cast<FaceScreen*>(lv_timer_get_user_data(timer))->tick();
}

void FaceScreen::tapped(lv_event_t* e) {
    auto* face = static_cast<FaceScreen*>(lv_event_get_user_data(e));
    face->requested_.store(false);
    // during a match, down until the next period; outside one, nothing else would bring it back
    face->dismissed_ = true;
}

void FaceScreen::tick() {
    const std::uint32_t now = millis();

    const Period period = currentPeriod();
    if (period != period_) {
        period_ = period;
        dismissed_ = false;
    }
    const bool wanted =
        requested_.load() || (config_.showDuringMatch && period != Period::none && !dismissed_);

    if (wanted != showing_.load()) {
        showing_.store(wanted);
        if (wanted) {
            // fresh: eyes open, and no bump measured against an average from before it was hidden
            animator_.reset(now);
            bumps_.reset();
            // nor a wink for something counted while it was down
            winksSeen_ = config_.winks ? config_.winks() : 0;
            drawnValid_ = false;
            lv_obj_remove_flag(root_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (!wanted) return;

    bool bumped = false;
    if (config_.imu != nullptr) {
        const pros::imu_accel_s_t accel = config_.imu->getAccelG();
        bumped = bumps_.update(accel.x, accel.y, accel.z, now);
        // for tuning BumpConfig::thresholdG: how hard each bump that counted was
        if (bumped) telemetry::event("face", "bump,g=%.2f", bumps_.lastJoltG());
    }
    if (config_.winks) {
        const std::uint32_t winks = config_.winks();
        if (winks != winksSeen_) {
            winksSeen_ = winks;
            animator_.startWink(now);
        }
    }
    const bool squinting = config_.squinting && config_.squinting();
    draw(animator_.update(now, squinting, bumped));
}

void FaceScreen::draw(const FaceShape& face) {
    // only what changed: LVGL redraws on every set_pos() and set_size(), moved or not, and most
    // frames the face holds still between blinks
    for (int side = 0; side < 2; ++side) {
        const EyeShape& eye = face.eyes[side];
        if (!drawnValid_ || !(eye == drawn_.eyes[side])) {
            lv_obj_set_size(eyes_[side], eye.widthPx, eye.heightPx);
            lv_obj_set_pos(eyes_[side], eye.centerX - eye.widthPx / 2,
                           eye.centerY - eye.heightPx / 2);
        }

        const BrowShape& brow = face.brows[side];
        if (!drawnValid_ || !(brow == drawn_.brows[side])) {
            // the line object only as big as the bar, plus its thickness all round for the
            // corners, so a brow moving redraws around the brow and not the whole screen
            const std::int32_t marginPx = brow.thicknessPx;
            const std::int32_t left = std::min(brow.outerX, brow.innerX) - marginPx;
            const std::int32_t top = std::min(brow.outerY, brow.innerY) - marginPx;
            lv_obj_set_pos(brows_[side], left, top);
            lv_obj_set_size(brows_[side], std::abs(brow.outerX - brow.innerX) + 2 * marginPx,
                            std::abs(brow.outerY - brow.innerY) + 2 * marginPx);
            lv_obj_set_style_line_width(brows_[side], brow.thicknessPx, 0);
            browPoints_[side][0] = {static_cast<lv_value_precise_t>(brow.outerX - left),
                                    static_cast<lv_value_precise_t>(brow.outerY - top)};
            browPoints_[side][1] = {static_cast<lv_value_precise_t>(brow.innerX - left),
                                    static_cast<lv_value_precise_t>(brow.innerY - top)};
            lv_line_set_points(brows_[side], browPoints_[side], 2);
        }
    }

    const MouthShape& mouth = face.mouth;
    if (!drawnValid_ || !(mouth == drawn_.mouth)) {
        // hidden while not surprised, rather than a 0x0 object
        if (mouth.sizePx <= 0) {
            lv_obj_add_flag(mouth_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_set_size(mouth_, mouth.sizePx, mouth.sizePx);
            lv_obj_set_pos(mouth_, mouth.centerX - mouth.sizePx / 2,
                           mouth.centerY - mouth.sizePx / 2);
            lv_obj_remove_flag(mouth_, LV_OBJ_FLAG_HIDDEN);
        }
    }
    drawn_ = face;
    drawnValid_ = true;
}

} // namespace sapphirelib::gui
