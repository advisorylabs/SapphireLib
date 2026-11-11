/**
 * \file vision.cpp
 *
 * The AI Vision sensor, and PinSight: what the sensor's AI model sees of a
 * yellow/yellow Pin, moved into the approach's target (see vision.hpp). Everything meant to be
 * tuned is in the constants at the top; the port and the mount are in
 * config.hpp.
 *
 * Team 96671H: Hitmen
 */

#include "robot/vision.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <utility>

#include "pros/ai_vision.hpp"
#include "pros/error.h"
#include "robot/config.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/telemetry/logger.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"

namespace robot::vision {

namespace {

using sapphirelib::motion::CameraMount;
using sapphirelib::motion::FieldPoint;
using sapphirelib::odom::Pose;

// --- What the camera looks for ---
// The sensor's own AI model of Override's game elements (its AI
// classifications): every detection comes labeled with what it is, so the Pin
// is a detection of the yellow/yellow Pin's class, and no Red/Yellow,
// Blue/Yellow or Red/Blue Pin, Cup, or anything else yellow is ever taken for
// it. The sensor names its classes (get_class_name()); the Pin's is the one
// whose name, letters only and in lower case, is one of kPinClassNames
// ("Yellow/Yellow Pin", "YellowPin"; the brain's device screen shows the
// competition model's as "YellowPin"). Each run onto a Pin logs the class it
// took (vision,pin_class), or every class the sensor has when none is the Pin
// (vision,class events, and the terminal). If the yellow/yellow Pin's class is
// named something else, add it here (letters only, lower case), or set
// kPinClassIdOverride to its number. The sensor has to be running the
// competition model: VEXcode's AI Vision Utility sets that.
constexpr std::array<const char*, 2> kPinClassNames{"yellowyellowpin", "yellowpin"};
constexpr std::int32_t kPinClassIdOverride = -1;
// How sure the model has to be that it's the Pin (its score). TODO: check the
// scale (the vision channel's score column): VEXcode's is 0-100.
constexpr double kMinScore = 40.0;
// The sensor may not report its class names for a while after it powers up:
// until it does, look again at most this often.
constexpr std::uint32_t kClassLookupRetryMs = 500;

// A standing Pin is 6.5in tall and 1.6in across. A detection counts as one if
// it's at least kMinPinHeightPx tall (a Pin about 9ft off, in a 240 pixel high
// image) and kMinPinTallness times taller than wide: a yellow/yellow Pin
// knocked over is still the Pin's class, but the claw can't grab it lying down.
constexpr int kMinPinHeightPx = 12;
constexpr double kMinPinTallness = 1.3;

// Which detection is the Pin. Before it's been seen, the tallest one (the
// nearest Pin, since they're all the same height) within kAcquireGateDeg of
// where the target is: that's only odometry's guess, and the 3.5in odometry is
// off by the second Pin is 10 degrees from 20in. After, the one nearest where
// the last frame saw the Pin, within kTrackGateDeg: the Pin itself, followed
// frame to frame, so another yellow Pin coming into view (a wall group's, the
// stack on the goal) is never taken for it, even while the target is still
// swinging over onto the Pin. kLostFrames frames in a row without it and it
// looks again as it did at first, around the target, which is on the Pin by
// then.
constexpr double kAcquireGateDeg = 25.0;
constexpr double kTrackGateDeg = 8.0;
constexpr std::uint32_t kLostFrames = 4;

// And at first, at least kMinPinHeightFraction as tall as a Pin (kPinHeightIn)
// would be in the image at the target's distance: a Pin across the field (a
// neutral goal's, a wall group's) is a third that, however near its bearing.
// Only a lower bound: something taller is nearer, or a stack. TODO: this takes
// the camera to see a Pin whole from a third of the way in (about 20in), as a
// lens up to about 9in high and looking level does: the vision channel's h_px
// says how tall the Pin really shows.
constexpr double kPinHeightIn = 6.5;
constexpr double kMinPinHeightFraction = 0.55;

// The target eases onto the latest frame that shows the Pin with this time
// constant, every tick: about 40% of the way over one of the sensor's frames
// (about 33ms), so a frame's noise is averaged over a few without lagging a
// turning robot by much.
constexpr double kSmoothingMs = 60.0;

// How old a frame is when the program reads it (the sensor's exposure and
// processing): the pose this long ago is where the robot saw it from, which
// matters while it turns. TODO: measure. Drive at a Pin turning, and see what
// lag lines the logged bearings (channel "vision") up with the Pin.
constexpr std::uint32_t kFrameLatencyMs = 50;

// The sensor's image: 320 pixels across, and how wide it sees. TODO: check
// the 74 degrees on the robot: a Pin standing at the image's edge is 37
// degrees off the camera's axis.
constexpr double kImageWidthPx = 320.0;
constexpr double kHorizontalFovDeg = 74.0;

CameraMount mount() {
    return CameraMount{.imageWidthPx = kImageWidthPx,
                       .horizontalFovDeg = kHorizontalFovDeg,
                       .forwardIn = kVisionForwardIn,
                       .rightIn = kVisionRightIn,
                       .yawDeg = kVisionYawDeg};
}

// Built on first use, at runtime, like every PROS object here.
pros::AIVision& sensor() {
    static pros::AIVision instance(ports::kVision);
    return instance;
}

// The "vision" channel, once attachTelemetry() has made it.
sapphirelib::telemetry::Channel* visionLog = nullptr;

// --- The Pin's class ---

// Whether a class name is the yellow/yellow Pin's: its letters, lower case,
// are one of kPinClassNames. "Red/Yellow Pin" is "redyellowpin", so it isn't.
bool isPinClassName(const char* name) {
    char letters[48] = {};
    std::size_t n = 0;
    for (const char* c = name; *c != '\0' && n + 1 < sizeof(letters); ++c) {
        if (std::isalpha(static_cast<unsigned char>(*c))) {
            letters[n++] = static_cast<char>(std::tolower(static_cast<unsigned char>(*c)));
        }
    }
    return std::any_of(kPinClassNames.begin(), kPinClassNames.end(),
                       [&letters](const char* known) { return std::strcmp(letters, known) == 0; });
}

// The Pin's class number once found (-1 until then), and when it last looked.
std::atomic<std::int32_t> pinClass{-1};
std::atomic<bool> lookedUp{false};
std::atomic<std::uint32_t> lastLookupMs{0};
std::atomic<bool> namesLogged{false};

// Class `id`'s name, into `name` (kClassNameSize chars), "" for an id the
// model has no class at. Every id up to AIVISION_MAX_CLASSNAME_COUNT is asked:
// PROS's get_class_name() returns 1 whatever the sensor says (kernel 4.2.2),
// so get_class_name(-1, ...) isn't the count of classes its header promises,
// and taking it as one only ever looked at class 0.
constexpr std::size_t kClassNameSize = 32;
void readClassName(pros::AIVision& camera, std::int32_t id, char* name) {
    std::memset(name, 0, kClassNameSize);
    if (camera.get_class_name(id, name) == PROS_ERR) name[0] = '\0';
    name[kClassNameSize - 1] = '\0';
}

// Every class the sensor names, into the log (vision,class events and the
// terminal): what to put in kPinClassNames when none is the Pin's.
void logClassNames() {
    pros::AIVision& camera = sensor();
    char name[kClassNameSize];
    for (std::int32_t id = 0; id < AIVISION_MAX_CLASSNAME_COUNT; ++id) {
        readClassName(camera, id, name);
        if (name[0] == '\0') continue;
        // no commas in an event's fields
        for (char& c : name) {
            if (c == ',') c = ' ';
        }
        SAPPHIRELIB_LOG_INFO("vision", "AI class %d: %s", static_cast<int>(id), name);
        sapphirelib::telemetry::event("vision", "class,id=%d,name=%s", static_cast<int>(id), name);
    }
}

// The yellow/yellow Pin's class number: kPinClassIdOverride, or the one the
// sensor names as it (see kPinClassNames), looked up the first time and again
// at most every kClassLookupRetryMs until found. The first time the sensor
// reports any names, every one goes in the log. -1 while there's none.
std::int32_t pinClassId() {
    if (kPinClassIdOverride >= 0) return kPinClassIdOverride;
    const std::int32_t known = pinClass.load();
    if (known >= 0) return known;
    const std::uint32_t now = sapphirelib::millis();
    if (lookedUp.load() && now - lastLookupMs.load() < kClassLookupRetryMs) return -1;
    lookedUp.store(true);
    lastLookupMs.store(now);

    pros::AIVision& camera = sensor();
    char name[kClassNameSize];
    bool named = false;
    std::int32_t found = -1;
    for (std::int32_t id = 0; id < AIVISION_MAX_CLASSNAME_COUNT; ++id) {
        readClassName(camera, id, name);
        if (name[0] == '\0') continue;
        named = true;
        if (found < 0 && isPinClassName(name)) found = id;
    }
    // no names yet: likely too soon after power-up, so look again later
    if (!named) return -1;
    if (!namesLogged.exchange(true)) logClassNames();
    if (found >= 0) pinClass.store(found);
    return found;
}

// --- Frames ---

// One standing yellow/yellow Pin in a frame
struct Blob {
    double centerPx = 0.0;
    double heightPx = 0.0;
    double score = 0.0;
};

// What one read of the sensor found
struct Frame {
    std::array<Blob, AIVISION_MAX_OBJECT_COUNT> pins{};
    std::size_t pinCount = 0;
    int detections = 0;        // every detection of the Pin's class, standing, sure or not
    int objects = 0;           // every detection of any kind
    std::uint32_t classes = 0; // bit n set: the AI model saw something of class n
    std::uint32_t key = 0;     // changes when the frame does
};

// FNV-1a, a step: enough to tell one frame's detections from the next's
std::uint32_t mix(std::uint32_t key, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        key = (key ^ ((value >> (8 * i)) & 0xffu)) * 16777619u;
    }
    return key;
}

// The sensor's latest frame: its detections of class `pinClassId`, and how
// many of anything. False with no sensor answering.
bool readFrame(Frame& frame, std::int32_t pinClassId) {
    pros::AIVision& camera = sensor();
    const std::int32_t count = camera.get_object_count();
    if (count == PROS_ERR || count < 0) return false;
    frame.key = 2166136261u;
    const std::int32_t n = std::min<std::int32_t>(count, AIVISION_MAX_OBJECT_COUNT);
    frame.objects = n;
    for (std::int32_t i = 0; i < n; ++i) {
        const pros::AIVision::Object object = camera.get_object(static_cast<std::uint32_t>(i));
        // a copy: the sensor's structs are packed
        const pros::aivision_object_element_s_t e = object.object.element;
        // every detection keys the frame, so a frame with no Pin in it is still
        // a new frame (and a vision row) when anything in it changes
        frame.key = mix(frame.key, (static_cast<std::uint32_t>(object.id) << 8) | object.type);
        frame.key = mix(frame.key, (static_cast<std::uint32_t>(e.xoffset) << 16) | e.yoffset);
        frame.key = mix(frame.key, (static_cast<std::uint32_t>(e.width) << 16) | e.height);
        if (!pros::AIVision::is_type(object, pros::AivisionDetectType::object)) continue;
        if (object.id < 32) frame.classes |= 1u << object.id;
        if (static_cast<std::int32_t>(object.id) != pinClassId) continue;
        frame.detections++;
        if (e.score < kMinScore || e.height < kMinPinHeightPx ||
            e.height < kMinPinTallness * e.width) {
            continue;
        }
        frame.pins[frame.pinCount++] = Blob{.centerPx = e.xoffset + e.width / 2.0,
                                           .heightPx = static_cast<double>(e.height),
                                           .score = static_cast<double>(e.score)};
    }
    return true;
}

} // namespace

void initialize() {
    pros::AIVision& camera = sensor();
    // The AI model's game elements only: no colors, codes or AprilTags. Fails
    // (errno ENODEV) with no AI Vision sensor on the port.
    if (camera.enable_detection_types(pros::AivisionModeType::objects) == PROS_ERR) {
        SAPPHIRELIB_LOG_WARN("vision",
                             "no AI Vision sensor on port %u: the runs onto the Pins steer by "
                             "odometry alone",
                             static_cast<unsigned>(ports::kVision));
        sapphirelib::telemetry::event("vision", "no_sensor,port=%u",
                                      static_cast<unsigned>(ports::kVision));
        return;
    }
    camera.disable_detection_types(pros::AivisionModeType::colors, pros::AivisionModeType::tags);
    const std::int32_t id = pinClassId();
    if (id >= 0) {
        SAPPHIRELIB_LOG_INFO("vision", "AI Vision on port %u: the yellow/yellow Pin is class %d",
                             static_cast<unsigned>(ports::kVision), static_cast<int>(id));
    } else {
        // likely just too soon after power-up: PinSight looks again
        SAPPHIRELIB_LOG_WARN("vision",
                             "AI Vision on port %u names no yellow/yellow Pin class yet: looking "
                             "again in autonomous",
                             static_cast<unsigned>(ports::kVision));
    }
}

void attachTelemetry(sapphirelib::telemetry::Logger& logger) {
    // Every frame a PinSight looks at: whether it found the Pin (1) or not (0),
    // the Pin's column (px, -1 none) and bearing off the camera's axis (deg),
    // the bearing it looked for it at (expect), the target before this frame
    // moved it (pin_x, pin_y, odometry's frame), how many standing yellow/yellow
    // Pins the model was sure of (blobs) and how many detections of that class
    // at all (pins), and the Pin's height in the image (h_px) and the model's
    // score for it; then every detection of anything (objects) and which AI
    // classes it saw (classes, bit n for class n). About 30 rows a second
    // while an approach runs.
    visionLog = &logger.channel("vision",
                                {"seen", "px", "bearing", "expect", "pin_x", "pin_y", "blobs",
                                 "pins", "h_px", "score", "objects", "classes"},
                                {.capacity = 128, .decimals = 2});
}

PinSight::PinSight(FieldPoint pin) : plan_(pin), pinX_(pin.xIn), pinY_(pin.yIn) {}

FieldPoint PinSight::pin() const { return FieldPoint{.xIn = pinX_.load(), .yIn = pinY_.load()}; }

double PinSight::movedIn() const {
    const FieldPoint now = pin();
    return std::hypot(now.xIn - plan_.xIn, now.yIn - plan_.yIn);
}

Pose PinSight::poseAt(std::uint32_t ms) const {
    // the entry nearest that time: they're 10ms apart, a tick each
    const Stamped* best = nullptr;
    std::uint32_t bestOffMs = 0;
    for (std::size_t i = 0; i < historyCount_; ++i) {
        const Stamped& entry = history_[i];
        const std::int32_t off = static_cast<std::int32_t>(entry.ms - ms);
        const std::uint32_t offMs = static_cast<std::uint32_t>(off < 0 ? -off : off);
        if (best == nullptr || offMs < bestOffMs) {
            best = &entry;
            bestOffMs = offMs;
        }
    }
    return best != nullptr ? best->pose : Pose{};
}

void PinSight::refine(const Pose& pose, FieldPoint& target) {
    const std::uint32_t now = sapphirelib::millis();
    const std::uint32_t sinceMs = historyCount_ > 0 ? now - lastRefineMs_ : 0;
    lastRefineMs_ = now;
    history_[historyNext_] = Stamped{.ms = now, .pose = pose};
    historyNext_ = (historyNext_ + 1) % kHistory;
    historyCount_ = std::min(historyCount_ + 1, kHistory);

    FieldPoint pin = this->pin();
    // the approach drives at this sight's Pin, moved or not
    target = pin;

    // the yellow/yellow Pin's class: without one, there's nothing to look for
    const std::int32_t pinClassNow = pinClassId();
    Frame frame;
    if (pinClassNow < 0 || !readFrame(frame, pinClassNow)) {
        if (!warned_) {
            warned_ = true;
            // a sensor that answers but names no such class, or none answering at all
            const std::int32_t answering = sensor().get_object_count();
            if (pinClassNow < 0 && answering != PROS_ERR && answering >= 0) {
                SAPPHIRELIB_LOG_WARN("vision", "AI Vision on port %u names no yellow/yellow Pin "
                                               "class: steering by odometry",
                                     static_cast<unsigned>(ports::kVision));
                sapphirelib::telemetry::event("vision", "no_pin_class,port=%u",
                                              static_cast<unsigned>(ports::kVision));
                // the classes it does name, in this run's log: the startup
                // list comes before any recording
                logClassNames();
            } else {
                SAPPHIRELIB_LOG_WARN("vision", "no AI Vision sensor answering on port %u: "
                                               "steering by odometry",
                                     static_cast<unsigned>(ports::kVision));
                sapphirelib::telemetry::event("vision", "no_frame,port=%u",
                                              static_cast<unsigned>(ports::kVision));
            }
        }
        return;
    }
    if (!classLogged_) {
        // which class this run looks for, and what the sensor was detecting
        // (AivisionModeType bits: 1 tags, 2 colors, 4 AI objects), in its own log
        classLogged_ = true;
        pros::AIVision& camera = sensor();
        char name[kClassNameSize];
        readClassName(camera, pinClassNow, name);
        for (char& c : name) {
            if (c == ',') c = ' ';
        }
        const std::int32_t modes = camera.get_enabled_detection_types();
        sapphirelib::telemetry::event("vision", "pin_class,id=%d,name=%s,modes=%d",
                                      static_cast<int>(pinClassNow), name, static_cast<int>(modes));
        // initialize() turns the AI model's detections on, but that can come
        // before the sensor is up: make sure of it for every run
        const auto objectsBit = static_cast<std::int32_t>(pros::AivisionModeType::objects);
        if (modes == PROS_ERR || (modes & objectsBit) == 0) {
            camera.enable_detection_types(pros::AivisionModeType::objects);
        }
    }
    const CameraMount camera = mount();
    // A new frame: look for the Pin in it. The same frame as last tick (the
    // sensor takes one about every 33ms) keeps what the last one showed.
    if (frame.key != lastFrameKey_) {
        lastFrameKey_ = frame.key;
        sighting_.reset();
        // where the robot was when the sensor took it
        const Pose seenFrom = poseAt(now - kFrameLatencyMs);
        // following the Pin from the last frame that showed it, or looking for it around the target
        const bool tracking = seenPin_.has_value() && missedFrames_ < kLostFrames;
        const std::optional<double> expected =
            sapphirelib::motion::cameraBearingToDeg(camera, seenFrom, tracking ? *seenPin_ : pin);
        const double gate = tracking ? kTrackGateDeg : kAcquireGateDeg;
        // looking: how tall a Pin as far off as the target would show
        const std::optional<double> depth =
            sapphirelib::motion::cameraDepthIn(camera, seenFrom, pin);
        const double minHeightPx =
            tracking || !depth ? 0.0
                               : kMinPinHeightFraction * sapphirelib::motion::cameraFocalPx(camera) *
                                     kPinHeightIn / *depth;
        const Blob* best = nullptr;
        double bestBearing = 0.0;
        double bestOff = gate;
        for (std::size_t i = 0; expected && i < frame.pinCount; ++i) {
            const Blob& blob = frame.pins[i];
            const double bearing = sapphirelib::motion::cameraBearingDeg(camera, blob.centerPx);
            const double off = std::fabs(bearing - *expected);
            if (off > gate || blob.heightPx < minHeightPx) continue;
            // tracking: nearest the last sighting; looking: the tallest, the nearest Pin
            const bool better = best == nullptr ||
                                (tracking ? off < bestOff : blob.heightPx > best->heightPx);
            if (better) {
                best = &blob;
                bestBearing = bearing;
                bestOff = off;
            }
        }
        if (best != nullptr) {
            sighting_ = Sighting{.bearingDeg = bestBearing, .seenFrom = seenFrom};
            seenPin_ = sapphirelib::motion::onCameraRay(camera, seenFrom, bestBearing, pin);
            missedFrames_ = 0;
            frames_.fetch_add(1);
            if (!tracking) {
                locked_.store(true);
                sapphirelib::telemetry::event(
                    "vision",
                    "lock,px=%.0f,bearing_deg=%.1f,off_deg=%.1f,range_in=%.1f,h_px=%.0f,"
                    "min_h_px=%.0f,score=%.0f",
                    best->centerPx, bestBearing, bestBearing - *expected,
                    std::hypot(pin.xIn - pose.xIn, pin.yIn - pose.yIn), best->heightPx,
                    minHeightPx, best->score);
            }
        } else if (seenPin_) {
            missedFrames_++;
        }
        if (visionLog != nullptr) {
            // the Pin, or with none taken, the tallest sure Pin the gates
            // turned down: where it was in the image says why
            const Blob* shown = best;
            if (shown == nullptr) {
                for (std::size_t i = 0; i < frame.pinCount; ++i) {
                    if (shown == nullptr || frame.pins[i].heightPx > shown->heightPx) {
                        shown = &frame.pins[i];
                    }
                }
            }
            const double shownBearing =
                best != nullptr    ? bestBearing
                : shown != nullptr ? sapphirelib::motion::cameraBearingDeg(camera, shown->centerPx)
                                   : 0.0;
            visionLog->record({best != nullptr ? 1.0 : 0.0,
                               shown != nullptr ? shown->centerPx : -1.0, shownBearing,
                               expected.value_or(0.0), pin.xIn, pin.yIn,
                               static_cast<double>(frame.pinCount),
                               static_cast<double>(frame.detections),
                               shown != nullptr ? shown->heightPx : 0.0,
                               shown != nullptr ? shown->score : 0.0,
                               static_cast<double>(frame.objects),
                               static_cast<double>(frame.classes)});
        }
    }
    // Every tick, swing the target toward the line the latest sighting showed it on, keeping its
    // range from the lens
    if (sighting_) {
        const std::optional<double> nowDeg =
            sapphirelib::motion::cameraBearingToDeg(camera, sighting_->seenFrom, pin);
        if (nowDeg) {
            const double pull =
                1.0 - std::exp(-static_cast<double>(std::min<std::uint32_t>(sinceMs, 100)) /
                               kSmoothingMs);
            pin = sapphirelib::motion::onCameraRay(
                camera, sighting_->seenFrom, *nowDeg + pull * (sighting_->bearingDeg - *nowDeg), pin);
            pinX_.store(pin.xIn);
            pinY_.store(pin.yIn);
            target = pin;
        }
    }
}

sapphirelib::motion::ApproachRefiner steerBy(std::shared_ptr<PinSight> sight) {
    return [sight = std::move(sight)](const Pose& pose, FieldPoint& target) {
        sight->refine(pose, target);
    };
}

} // namespace robot::vision
