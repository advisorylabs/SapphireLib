/*
 * Host-side unit test for sapphirelib::gui::FaceAnimator, no PROS/LVGL
 * dependencies, so it builds and runs with a normal desktop compiler.
 *
 * (A block comment, not //, so the backslash-continued build line doesn't
 * trip -Wcomment.)
 *
 * Build & run:
 *   g++ -std=c++20 -Iinclude tests/gui/face_math_test.cpp \
 *       src/sapphirelib/gui/face_math.cpp \
 *       -o face_math_test && ./face_math_test
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "sapphirelib/gui/face_math.hpp"

using sapphirelib::gui::BrowShape;
using sapphirelib::gui::EyeShape;
using sapphirelib::gui::FaceAnimator;
using sapphirelib::gui::FaceShape;

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                       \
            std::exit(1);                                                                          \
        }                                                                                          \
    } while (0)

namespace {

constexpr std::uint32_t kFrameMs = 20;
// no blink comes sooner than this after a reset, so tests of the moods run
// inside it and never catch one half shut
constexpr std::uint32_t kFirstBlinkMs = 2500;
// the open eye on the brain's screen
constexpr std::int32_t kEyePx = 104;

bool shut(const EyeShape& eye) { return eye.heightPx * 5 < eye.widthPx; }

bool flat(const BrowShape& brow) { return brow.innerY == brow.outerY; }

std::int32_t browMiddleY(const BrowShape& brow) { return (brow.innerY + brow.outerY) / 2; }

/// Runs frames until `untilMs`, with the same inputs every frame, and returns
/// the last face.
FaceShape run(FaceAnimator& face, std::uint32_t& nowMs, std::uint32_t untilMs, bool squinting,
              bool bumpFirstFrame = false) {
    FaceShape shape = face.update(nowMs, squinting, bumpFirstFrame);
    while (nowMs < untilMs) {
        nowMs += kFrameMs;
        shape = face.update(nowMs, squinting, false);
    }
    return shape;
}

void testOpenAtRest() {
    FaceAnimator face(480, 272);
    std::uint32_t now = 0;
    const FaceShape shape = run(face, now, 1000, false);
    for (int side = 0; side < 2; ++side) {
        CHECK(shape.eyes[side].widthPx == kEyePx);
        CHECK(shape.eyes[side].heightPx == kEyePx);
        CHECK(flat(shape.brows[side]));
        // the brow sits above the eye, over it
        CHECK(shape.brows[side].innerY < shape.eyes[side].centerY - kEyePx / 2);
    }
    // mirrored about the middle, left eye on the left
    CHECK(shape.eyes[0].centerX < 240 && shape.eyes[1].centerX > 240);
    CHECK(shape.eyes[0].centerX + shape.eyes[1].centerX == 480);
    CHECK(shape.eyes[0].centerY == shape.eyes[1].centerY);
    // each brow's inner end is the one toward the middle
    CHECK(shape.brows[0].innerX > shape.brows[0].outerX);
    CHECK(shape.brows[1].innerX < shape.brows[1].outerX);
    // and it all fits on the screen
    CHECK(shape.eyes[0].centerX - kEyePx / 2 > 0 && shape.eyes[1].centerX + kEyePx / 2 < 480);
    CHECK(shape.brows[0].outerY - shape.brows[0].thicknessPx > 0);
    CHECK(shape.eyes[0].centerY + kEyePx / 2 < 272);
}

void testBlinksNowAndThen() {
    FaceAnimator face(480, 272, 7);
    std::uint32_t now = 0;
    face.reset(now);
    int blinks = 0;
    bool wasShut = false;
    std::uint32_t firstShutMs = 0;
    for (; now <= 30000; now += kFrameMs) {
        const FaceShape shape = face.update(now, false, false);
        // both eyes together
        CHECK(shape.eyes[0].widthPx == shape.eyes[1].widthPx);
        CHECK(shape.eyes[0].heightPx == shape.eyes[1].heightPx);
        const bool isShut = shut(shape.eyes[0]);
        if (isShut && !wasShut) {
            if (blinks == 0) firstShutMs = now;
            ++blinks;
        }
        wasShut = isShut;
        // blinking moves the eyes only
        CHECK(flat(shape.brows[0]) && flat(shape.brows[1]));
    }
    CHECK(firstShutMs >= kFirstBlinkMs);
    // every 2.5 to 6s, now and then twice: blinking, but not all the time
    CHECK(blinks >= 4 && blinks <= 20);
}

void testBlinkOpensAgain() {
    FaceAnimator face(480, 272, 3);
    std::uint32_t now = 0;
    face.reset(now);
    // find a blink, then check the eyes are fully open within 250ms of it
    while (!shut(face.update(now, false, false).eyes[0])) now += kFrameMs;
    const std::uint32_t shutMs = now;
    bool reopened = false;
    while (now < shutMs + 250) {
        now += kFrameMs;
        if (face.update(now, false, false).eyes[0].heightPx == kEyePx) reopened = true;
    }
    CHECK(reopened);
}

void testSquint() {
    FaceAnimator face(480, 272);
    std::uint32_t now = 0;
    const FaceShape open = run(face, now, 200, false);
    const FaceShape squinting = run(face, now, 800, true);
    CHECK(face.squint() == 1.0);
    for (int side = 0; side < 2; ++side) {
        // narrowed to about a third, the same width
        CHECK(squinting.eyes[side].widthPx == kEyePx);
        CHECK(squinting.eyes[side].heightPx >= 31 && squinting.eyes[side].heightPx <= 35);
        // brows lower, inner ends tipped down
        CHECK(browMiddleY(squinting.brows[side]) > browMiddleY(open.brows[side]));
        CHECK(squinting.brows[side].innerY > squinting.brows[side].outerY);
        // still clear of the narrowed eye
        CHECK(squinting.brows[side].innerY + squinting.brows[side].thicknessPx <
              squinting.eyes[side].centerY - squinting.eyes[side].heightPx / 2);
    }
    // and eases back
    const FaceShape after = run(face, now, 1400, false);
    CHECK(after == open);
}

void testSquintEasesIn() {
    FaceAnimator face(480, 272);
    std::uint32_t now = 0;
    run(face, now, 200, false);
    now += kFrameMs;
    const FaceShape first = face.update(now, true, false);
    // one frame in, partway: not snapped shut
    CHECK(first.eyes[0].heightPx < kEyePx && first.eyes[0].heightPx > 60);
}

void testSurprise() {
    FaceAnimator face(480, 272);
    std::uint32_t now = 0;
    const FaceShape open = run(face, now, 200, false);
    const FaceShape surprised = run(face, now, 500, false, /*bumpFirstFrame=*/true);
    CHECK(face.surprise() == 1.0);
    for (int side = 0; side < 2; ++side) {
        // bigger, still round
        CHECK(surprised.eyes[side].widthPx > kEyePx * 1.2);
        CHECK(surprised.eyes[side].heightPx == surprised.eyes[side].widthPx);
        // brows up, inner ends tipped up
        CHECK(browMiddleY(surprised.brows[side]) < browMiddleY(open.brows[side]));
        CHECK(surprised.brows[side].innerY < surprised.brows[side].outerY);
        CHECK(surprised.brows[side].innerY - surprised.brows[side].thicknessPx > 0);
    }
    // held a moment after the bump, then back to open
    run(face, now, 1200, false);
    CHECK(face.surprise() == 1.0);
    const FaceShape after = run(face, now, 1900, false);
    CHECK(face.surprise() == 0.0);
    CHECK(after == open);
}

void testSurpriseBeatsSquint() {
    FaceAnimator face(480, 272);
    std::uint32_t now = 0;
    run(face, now, 600, true);
    const FaceShape shape = run(face, now, 900, true, /*bumpFirstFrame=*/true);
    CHECK(shape.eyes[0].heightPx == shape.eyes[0].widthPx);
    CHECK(shape.brows[0].innerY < shape.brows[0].outerY);
    // and the squint is back once the surprise fades, still squinting
    const FaceShape after = run(face, now, 2400, true);
    CHECK(after.eyes[0].heightPx < 40);
}

void testNoBlinkWhileSurprised() {
    FaceAnimator face(480, 272, 11);
    std::uint32_t now = 0;
    face.reset(now);
    // bumped every second for 20s: never a blink
    for (; now <= 20000; now += kFrameMs) {
        const FaceShape shape = face.update(now, false, now % 1000 == 0);
        if (now >= 100) CHECK(shape.eyes[0].heightPx == shape.eyes[0].widthPx);
    }
    // and blinking again once left alone
    bool blinked = false;
    for (; now <= 30000; now += kFrameMs) {
        if (shut(face.update(now, false, false).eyes[0])) blinked = true;
    }
    CHECK(blinked);
}

void testSameSeedSameFace() {
    FaceAnimator a(480, 272, 42);
    FaceAnimator b(480, 272, 42);
    for (std::uint32_t now = 0; now <= 20000; now += kFrameMs) {
        const bool squinting = (now / 3000) % 2 == 1;
        const bool bumped = now % 7000 == 0;
        CHECK(a.update(now, squinting, bumped) == b.update(now, squinting, bumped));
    }
}

void testScalesToTheScreen() {
    FaceAnimator face(240, 136);
    std::uint32_t now = 0;
    const FaceShape shape = run(face, now, 500, false);
    CHECK(shape.eyes[0].widthPx == kEyePx / 2);
    CHECK(shape.eyes[0].centerX + shape.eyes[1].centerX == 240);
}

void testClockWrap() {
    FaceAnimator face(480, 272, 5);
    std::uint32_t now = 0xFFFFF000u;
    face.reset(now);
    bool blinked = false;
    // 10s straight across the wrap
    for (int frame = 0; frame < 500; ++frame) {
        if (shut(face.update(now, false, false).eyes[0])) blinked = true;
        now += kFrameMs;
    }
    CHECK(blinked);
}

void testStalledFrameDoesNotSnap() {
    FaceAnimator face(480, 272);
    std::uint32_t now = 0;
    run(face, now, 200, false);
    // the next frame comes a whole second late, squinting: it eases one
    // capped step, not all the way
    now += 1000;
    face.update(now, true, false);
    CHECK(face.squint() < 1.0);
}

} // namespace

int main() {
    testOpenAtRest();
    testBlinksNowAndThen();
    testBlinkOpensAgain();
    testSquint();
    testSquintEasesIn();
    testSurprise();
    testSurpriseBeatsSquint();
    testNoBlinkWhileSurprised();
    testSameSeedSameFace();
    testScalesToTheScreen();
    testClockWrap();
    testStalledFrameDoesNotSnap();
    std::printf("face_math_test: all passed\n");
    return 0;
}
