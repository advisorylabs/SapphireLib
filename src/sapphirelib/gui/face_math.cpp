#include "sapphirelib/gui/face_math.hpp"

#include <algorithm>
#include <cmath>

#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::gui {

namespace {

constexpr double kPi = 3.14159265358979323846;

// the layout, in pixels on the V5 brain's 480x272 screen; other screens scale it to fit
constexpr double kLayoutWidthPx = 480.0;
constexpr double kLayoutHeightPx = 272.0;
// eye centers: each this far from the middle, and this far below the screen's center
constexpr double kEyeSpacingPx = 112.0;
constexpr double kEyeDropPx = 14.0;
constexpr double kEyeSizePx = 104.0;
// brows: centered over each eye, this high above its center
constexpr double kBrowLengthPx = 112.0;
constexpr double kBrowThicknessPx = 18.0;
constexpr double kBrowRaisePx = 86.0;

// blinks: a wait picked between these, then shut and open again
constexpr std::uint32_t kBlinkMinWaitMs = 2500;
constexpr std::uint32_t kBlinkMaxWaitMs = 6000;
constexpr std::uint32_t kBlinkCloseMs = 60;
constexpr std::uint32_t kBlinkShutMs = 30;
constexpr std::uint32_t kBlinkOpenMs = 100;
constexpr std::uint32_t kBlinkMs = kBlinkCloseMs + kBlinkShutMs + kBlinkOpenMs;
// how open a shut eye still is, of its open height, so it reads as a closed lid and not as gone
constexpr double kShutHeight = 0.08;
// now and then a blink gets a second one this soon after
constexpr double kDoubleBlinkChance = 0.2;
constexpr std::uint32_t kDoubleBlinkGapMs = 120;
// no blink starts while at least this surprised: wide eyes stay wide
constexpr double kNoBlinkSurprise = 0.3;

// squint: eyes narrowed to this of their height, brows lowered and their inner ends tipped down
constexpr double kSquintHeight = 0.32;
constexpr double kSquintBrowDropPx = 30.0;
constexpr double kSquintBrowTiltDeg = 14.0;
constexpr double kSquintEaseMs = 120.0;

// surprise: eyes this much bigger, brows raised with their inner ends tipped up. Fast in, held
// after the bump, slower out
constexpr double kSurpriseEyeScale = 1.22;
constexpr double kSurpriseBrowLiftPx = 26.0;
constexpr double kSurpriseBrowTiltDeg = -8.0;
constexpr std::uint32_t kSurpriseHoldMs = 1200;
constexpr double kSurpriseRiseMs = 60.0;
constexpr double kSurpriseFallMs = 300.0;

// wink: the eye on the screen's right shuts, holds, and opens again, slower than a blink so it
// reads as meant, and its brow dips toward it
constexpr int kWinkEye = 1;
constexpr std::uint32_t kWinkCloseMs = 80;
constexpr std::uint32_t kWinkShutMs = 280;
constexpr std::uint32_t kWinkOpenMs = 160;
constexpr std::uint32_t kWinkMs = kWinkCloseMs + kWinkShutMs + kWinkOpenMs;
constexpr double kWinkBrowDropPx = 22.0;

// frames further apart than this ease as if they weren't, so a stalled frame never jumps a mood
// all the way
constexpr std::uint32_t kMaxStepMs = 100;

// one frame's easing: from `value` toward `target` by `step`, never past it
double approach(double value, double target, double step) {
    return value < target ? std::min(target, value + step) : std::max(target, value - step);
}

std::int32_t px(double value) { return static_cast<std::int32_t>(std::lround(value)); }

} // namespace

FaceAnimator::FaceAnimator(std::int32_t widthPx, std::int32_t heightPx, std::uint32_t seed)
    : widthPx_(widthPx), heightPx_(heightPx), rng_(seed) {
    reset(0);
}

void FaceAnimator::reset(std::uint32_t nowMs) {
    lastMs_ = nowMs;
    blinking_ = false;
    surprised_ = false;
    winking_ = false;
    blink_ = 0.0;
    wink_ = 0.0;
    squint_ = 0.0;
    surprise_ = 0.0;
    doubleBlinkNext_ = false;
    waitForNextBlink(nowMs);
}

FaceShape FaceAnimator::update(std::uint32_t nowMs, bool squinting, bool bumped) {
    const double stepMs = std::min(elapsedMs(lastMs_, nowMs), kMaxStepMs);
    lastMs_ = nowMs;

    if (bumped) {
        surprised_ = true;
        bumpMs_ = nowMs;
    }
    if (surprised_ && elapsedMs(bumpMs_, nowMs) >= kSurpriseHoldMs) surprised_ = false;
    surprise_ = approach(surprise_, surprised_ ? 1.0 : 0.0,
                         stepMs / (surprised_ ? kSurpriseRiseMs : kSurpriseFallMs));
    squint_ = approach(squint_, squinting ? 1.0 : 0.0, stepMs / kSquintEaseMs);

    if (winking_ && elapsedMs(winkStartMs_, nowMs) >= kWinkMs) winking_ = false;

    if (blinking_) {
        if (elapsedMs(blinkStartMs_, nowMs) >= kBlinkMs) {
            blinking_ = false;
            waitForNextBlink(nowMs);
        }
    } else if (elapsedMs(waitStartMs_, nowMs) >= waitMs_ && surprise_ < kNoBlinkSurprise &&
               !winking_) {
        // a blink that came due while surprised or winking waits for that to end, then goes, so
        // the open eye stays open through a wink
        blinking_ = true;
        blinkStartMs_ = nowMs;
    }
    blink_ = blinkAt(nowMs);
    wink_ = winkAt(nowMs);
    return shape();
}

void FaceAnimator::startWink(std::uint32_t nowMs) {
    const std::uint32_t t = elapsedMs(winkStartMs_, nowMs);
    if (!winking_ || t >= kWinkMs) {
        winking_ = true;
        winkStartMs_ = nowMs;
    } else if (t > kWinkCloseMs) {
        // shut or opening again: back to just shut, so the eye holds shut a little longer
        // (elapsedMs() is unsigned arithmetic, so this is right even this close to boot)
        winkStartMs_ = nowMs - kWinkCloseMs;
    }
    // still closing: it's on its way to shut anyway
}

double FaceAnimator::blink() const { return blink_; }

double FaceAnimator::wink() const { return wink_; }

double FaceAnimator::squint() const { return squint_; }

double FaceAnimator::surprise() const { return surprise_; }

double FaceAnimator::blinkAt(std::uint32_t nowMs) const {
    if (!blinking_) return 0.0;
    const double t = elapsedMs(blinkStartMs_, nowMs);
    if (t < kBlinkCloseMs) return t / kBlinkCloseMs;
    if (t < kBlinkCloseMs + kBlinkShutMs) return 1.0;
    if (t < kBlinkMs) return 1.0 - (t - kBlinkCloseMs - kBlinkShutMs) / kBlinkOpenMs;
    return 0.0;
}

double FaceAnimator::winkAt(std::uint32_t nowMs) const {
    if (!winking_) return 0.0;
    const double t = elapsedMs(winkStartMs_, nowMs);
    if (t < kWinkCloseMs) return t / kWinkCloseMs;
    if (t < kWinkCloseMs + kWinkShutMs) return 1.0;
    if (t < kWinkMs) return 1.0 - (t - kWinkCloseMs - kWinkShutMs) / kWinkOpenMs;
    return 0.0;
}

void FaceAnimator::waitForNextBlink(std::uint32_t nowMs) {
    waitStartMs_ = nowMs;
    if (doubleBlinkNext_) {
        // the blink that just ended gets its second one
        doubleBlinkNext_ = false;
        waitMs_ = kDoubleBlinkGapMs;
        return;
    }
    waitMs_ = kBlinkMinWaitMs +
              static_cast<std::uint32_t>(rng_.uniform() * (kBlinkMaxWaitMs - kBlinkMinWaitMs));
    doubleBlinkNext_ = rng_.uniform() < kDoubleBlinkChance;
}

FaceShape FaceAnimator::shape() const {
    const double scale = std::min(widthPx_ / kLayoutWidthPx, heightPx_ / kLayoutHeightPx);
    const double middleX = widthPx_ / 2.0;
    const double eyeY = heightPx_ / 2.0 + kEyeDropPx * scale;

    // a surprise beats a squint, so a surprised squint is just surprised
    const double surprise = surprise_;
    const double squint = squint_ * (1.0 - surprise);

    const double eyeSizePx = kEyeSizePx * scale * (1.0 + (kSurpriseEyeScale - 1.0) * surprise);
    const double openHeightPx = eyeSizePx * (1.0 - (1.0 - kSquintHeight) * squint);

    const double browY = eyeY - kBrowRaisePx * scale + kSquintBrowDropPx * scale * squint -
                         kSurpriseBrowLiftPx * scale * surprise;
    const double tiltRad =
        (kSquintBrowTiltDeg * squint + kSurpriseBrowTiltDeg * surprise) * (kPi / 180.0);
    const double halfLengthPx = kBrowLengthPx * scale / 2.0;
    const double halfDx = halfLengthPx * std::cos(tiltRad);
    // positive tilt puts the inner end lower, and screen y grows downward
    const double halfDy = halfLengthPx * std::sin(tiltRad);

    FaceShape face;
    for (int side = 0; side < 2; ++side) {
        // -1 the left eye, +1 the right: outward is this way
        const double outward = side == 0 ? -1.0 : 1.0;
        const double eyeX = middleX + outward * kEyeSpacingPx * scale;
        // the winking eye is as shut as the more shut of its blink and its wink
        const double wink = side == kWinkEye ? wink_ : 0.0;
        const double eyeHeightPx =
            openHeightPx * (1.0 - (1.0 - kShutHeight) * std::max(blink_, wink));
        const double sideBrowY = browY + kWinkBrowDropPx * scale * wink;
        face.eyes[side] = EyeShape{.centerX = px(eyeX),
                                   .centerY = px(eyeY),
                                   .widthPx = px(eyeSizePx),
                                   .heightPx = std::max<std::int32_t>(1, px(eyeHeightPx))};
        face.brows[side] =
            BrowShape{.outerX = px(eyeX + outward * halfDx),
                      .outerY = px(sideBrowY - halfDy),
                      .innerX = px(eyeX - outward * halfDx),
                      .innerY = px(sideBrowY + halfDy),
                      .thicknessPx = std::max<std::int32_t>(1, px(kBrowThicknessPx * scale))};
    }
    return face;
}

} // namespace sapphirelib::gui
