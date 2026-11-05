/**
 * \file leds.cpp
 *
 * The LED strands' modes, see leds.hpp. The macros say when autoClaw and a
 * score start and stop, and when the lift goes up; this turns that into
 * HitLib's mode stack, where Idle sits at the bottom from start() on and the
 * others go on and off over it.
 *
 * LiftGauge, Goal, MemoryReset, Showcase, Strobe and Dark aren't in the
 * Pattern Studio design: LiftGauge draws from the lift's level at runtime,
 * which a design can't, and the rest sit with it. start() attaches each strand's
 * design profile with them added after its own modes, so the design's mode
 * indices still hold, and exporting the design again keeps them.
 *
 * Team 96671H: Hitmen
 */

#include "robot/leds.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <mutex>
#include <vector>

#include "hitlib_studio.hpp"
#include "pros/rtos.hpp"

namespace robot::leds {

namespace {

namespace toggleleft = hitlib::profiles::toggleleft;
namespace toggleright = hitlib::profiles::toggleright;

using hitlib::LedStrand;

// --- LiftGauge ---
// How long it shows after each time the lift goes up.
constexpr std::uint32_t kLiftGaugeMs = 1000;
// Over Idle's 10 and under AutoClaw's 20 (and Scoring's 30), so either of
// those hides it.
constexpr std::uint8_t kLiftGaugePriority = 15;
// The gauge fills from the start of each strand, its first strip in the
// design's chain (the Toggle strip, then on into the Brace). If it fills from
// the top down on the robot, make this true.
constexpr bool kLiftGaugeFromFarEnd = true;
// Each level's section is solid: the goal's bottom kGaugeGreenLevels green,
// the next kGaugeOrangeLevels orange, and the rest, up to kGaugeRedLevels,
// red. ALLIANCE's six are green, green, orange, orange, red, red. A goal with
// fewer levels loses red ones first, but keeps one, so its top level is
// always red: MEDIUM's five are green, green, orange, orange, red, and
// CENTER's four green, green, orange, red. One with more has more orange.
constexpr int kGaugeGreenLevels = 2;
constexpr int kGaugeOrangeLevels = 2;
constexpr int kGaugeRedLevels = 2;
constexpr std::uint32_t kGaugeGreen = 0x00FF00;
constexpr std::uint32_t kGaugeOrange = 0xFF4000; // more green reads yellow on these
constexpr std::uint32_t kGaugeRed = 0xFF0000;

// --- MemoryReset ---
// After L1+L2 forgets the lift's memory, the strands blink purple: on, off,
// on, each this long (7 of the design's 25ms refresh ticks).
constexpr std::uint32_t kMemoryResetBlinkMs = 175;
constexpr std::uint32_t kMemoryResetMs = 3 * kMemoryResetBlinkMs;
// Over AutoClaw's 20 (and Idle's and LiftGauge's) but under Scoring's 30.
constexpr std::uint8_t kMemoryResetPriority = 25;
constexpr std::uint32_t kMemoryResetColor = 0xA000FF; // purple

// --- Goal ---
// For a second after RIGHT picks a goal: ALLIANCE solid blue split into
// segments that swap with red, MEDIUM solid orange, CENTER solid green.
constexpr std::uint32_t kGoalMs = 1000;
// Over AutoClaw's 20 and LiftGauge's 15 (which RIGHT onto a taller level
// starts at the same moment), under MemoryReset's 25 and Scoring's 30.
constexpr std::uint8_t kGoalPriority = 24;
constexpr std::uint32_t kAllianceColor = 0x0000FF;      // blue
constexpr std::uint32_t kAllianceSplitColor = 0xFF0000; // red
// spliceMask() takes the points it splits the strand at, one fewer than the
// segments: 5 makes 6, every other one red, swapping every
// kAllianceSwapMs as AutoClaw's split does.
constexpr std::uint8_t kAllianceSplits = 5;
constexpr std::uint32_t kAllianceSwapMs = 200;
constexpr std::uint32_t kMediumColor = kGaugeOrange;
constexpr std::uint32_t kCenterColor = kGaugeGreen;

// --- Showcase, Strobe and Dark ---
// The driver's own: DOWN plays Showcase once through, UP turns Strobe on and
// off, and X turns every LED off and back on (Dark). Over everything the
// robot shows by itself, Dark over all of it.
constexpr std::uint8_t kShowcasePriority = 200;
constexpr std::uint8_t kStrobePriority = 220;
constexpr std::uint8_t kDarkPriority = 255;
// Strobe: a white burst one refresh tick long, then dark for three, about 10
// times a second.
constexpr std::uint32_t kStrobeColor = 0xFFFFFF;
constexpr std::uint32_t kStrobeOnMs = 25;
constexpr std::uint32_t kStrobeOffMs = 75;

// hitlib::studio::group hands one mode index on to every strand, so every
// strand's profile has to number its modes the same way. Pattern Studio
// numbers them in the order the design lists them: if one of these fails,
// list them in the same order on every strand.
static_assert(toggleleft::mode::idle == toggleright::mode::idle,
              "every strand's Idle must be the same mode index");
static_assert(toggleleft::mode::autoclaw == toggleright::mode::autoclaw,
              "every strand's AutoClaw must be the same mode index");
static_assert(toggleleft::mode::scoring == toggleright::mode::scoring,
              "every strand's Scoring must be the same mode index");
// LiftGauge and the rest go after the design's modes, at the same indices on
// every strand.
constexpr std::size_t kDesignModeCount = std::size(toggleleft::modeTable);
static_assert(std::size(toggleright::modeTable) == kDesignModeCount,
              "every strand's design must have as many modes, for LiftGauge's index");

constexpr std::uint8_t kIdleMode = toggleleft::mode::idle;
constexpr std::uint8_t kAutoClawMode = toggleleft::mode::autoclaw;
constexpr std::uint8_t kScoringMode = toggleleft::mode::scoring;
constexpr std::uint8_t kLiftGaugeMode = kDesignModeCount;
constexpr std::uint8_t kMemoryResetMode = kDesignModeCount + 1;
constexpr std::uint8_t kShowcaseMode = kDesignModeCount + 2;
constexpr std::uint8_t kStrobeMode = kDesignModeCount + 3;
constexpr std::uint8_t kDarkMode = kDesignModeCount + 4;
constexpr std::uint8_t kGoalMode = kDesignModeCount + 5;
constexpr std::size_t kExtraModeCount = 6;

// What each mode was last told, so a call that changes nothing does nothing.
// Told from opcontrol's task, setAutoClaw()'s, the routine's and disabled()'s,
// so each change holds ledMutex() from the check through HitLib's calls.
bool autoClawShown = false;
bool scoringShown = false;
bool strobeShown = false;
bool darkShown = false;
// Showcase is timed, so it ends by itself: whether it was started, and when,
// so a second DOWN can tell whether it's still playing.
bool showcaseStarted = false;
std::uint32_t showcaseStartMs = 0;

// Built on first use, at runtime, like every PROS object here that needs the
// kernel running.
pros::Mutex& ledMutex() {
    static pros::Mutex instance;
    return instance;
}

/// With ledMutex() held.
void showLocked(bool& shown, std::uint8_t mode, bool on) {
    if (shown == on) return;
    shown = on;
    // Idle stays on the stack underneath, so taking this off shows it again.
    if (on) {
        hitlib::studio::group.activateMode(mode);
    } else {
        hitlib::studio::group.deactivateMode(mode);
    }
}

void show(bool& shown, std::uint8_t mode, bool on) {
    std::lock_guard<pros::Mutex> lock(ledMutex());
    showLocked(shown, mode, on);
}

// What setLiftGauge() last said, as one value so the LED task never reads a
// level from one call and a level count from another: the count in the high
// byte, the level in the low one.
std::atomic<std::uint16_t> gaugeWanted{0};

std::uint16_t packGauge(int level, int levels) {
    const int count = std::clamp(levels, 0, 255);
    return static_cast<std::uint16_t>(count << 8 | std::clamp(level, 0, count));
}

// What each strand last drew, the same way: only the LED task touches these.
// Index 0 is ToggleLeft, 1 ToggleRight (start()).
std::array<std::uint16_t, 2> gaugeDrawn{};

/// A section's color, solid, from the bands at kGaugeGreenLevels: section 0
/// is the goal's lowest level, `sections - 1` its top one.
std::uint32_t sectionColor(int section, int sections) {
    const int red =
        std::clamp(sections - kGaugeGreenLevels - kGaugeOrangeLevels, 1, kGaugeRedLevels);
    const int green = std::min(kGaugeGreenLevels, sections - red);
    if (section >= sections - red) return kGaugeRed;
    if (section < green) return kGaugeGreen;
    return kGaugeOrange;
}

/// The gauge on one strand: the strand split into `levels` equal sections
/// (as equal as its LEDs allow), the first `level` of them each lit solid in
/// its own color, and the rest dark.
void drawGauge(LedStrand& s, std::uint16_t packed) {
    const int levels = packed >> 8;
    const int lit = packed & 0xFF;
    const int length = s.getLength();
    std::vector<LedStrand::SpliceRegion> regions;
    regions.reserve(static_cast<std::size_t>(lit));
    for (int i = 0; i < lit; ++i) {
        const int from = i * length / levels;
        const int to = (i + 1) * length / levels;
        LedStrand::SpliceRegion region{};
        region.start = static_cast<std::uint8_t>(kLiftGaugeFromFarEnd ? length - to : from);
        region.width = static_cast<std::uint8_t>(to - from);
        region.kind = LedStrand::SpliceRegionAnimKind::SOLID;
        region.color = sectionColor(i, levels);
        regions.push_back(region);
    }
    // The unlit sections show the base: black, over whatever the last mode
    // left running.
    s.setColor(0x000000);
    if (regions.empty()) {
        s.clearSpliceMask();
    } else {
        s.spliceMaskCustom(regions);
    }
}

// LiftGauge's callbacks for the strand in `Slot` (gaugeDrawn's index). Its
// onActivate draws it from scratch, since whichever mode showed before
// replaced the base and the mask; its onTick redraws it only when the lift's
// level has changed since.
template <std::size_t Slot>
void gaugeActivate(LedStrand& s) {
    gaugeDrawn[Slot] = gaugeWanted.load();
    drawGauge(s, gaugeDrawn[Slot]);
}

template <std::size_t Slot>
void gaugeTick(LedStrand& s) {
    const std::uint16_t wanted = gaugeWanted.load();
    if (wanted == gaugeDrawn[Slot]) return;
    gaugeDrawn[Slot] = wanted;
    drawGauge(s, wanted);
}

// Every mode below replaces the whole strand, so each clears the splice mask
// whichever mode showed before may have left (AutoClaw's, LiftGauge's), or
// sets its own.

// The goal showGoal() last said (0 ALLIANCE, 1 MEDIUM, 2 CENTER), and what
// each strand last drew, as LiftGauge's are: a second RIGHT inside Goal's
// second changes what it shows without Goal starting over, so its onTick
// redraws it.
std::atomic<int> goalWanted{0};
std::array<int, 2> goalDrawn{};

void drawGoal(LedStrand& s, int goal) {
    switch (goal) {
        case 0:
            s.setColor(kAllianceColor);
            s.spliceMask(kAllianceSplits, false, true, kAllianceSwapMs, kAllianceSplitColor);
            return;
        case 1:
            s.setColor(kMediumColor);
            break;
        default:
            s.setColor(kCenterColor);
            break;
    }
    s.clearSpliceMask();
}

template <std::size_t Slot>
void goalActivate(LedStrand& s) {
    goalDrawn[Slot] = goalWanted.load();
    drawGoal(s, goalDrawn[Slot]);
}

template <std::size_t Slot>
void goalTick(LedStrand& s) {
    const int wanted = goalWanted.load();
    if (wanted == goalDrawn[Slot]) return;
    goalDrawn[Slot] = wanted;
    drawGoal(s, wanted);
}

// MemoryReset: the whole strand blinking purple, starting lit.
void memoryResetActivate(LedStrand& s) {
    s.flash(kMemoryResetColor, kMemoryResetBlinkMs, kMemoryResetBlinkMs);
    s.clearSpliceMask();
}

void strobeActivate(LedStrand& s) {
    s.flash(kStrobeColor, kStrobeOnMs, kStrobeOffMs);
    s.clearSpliceMask();
}

void darkActivate(LedStrand& s) {
    s.off();
    s.clearSpliceMask();
}

// --- Showcase: one pass through these, about 15 seconds, then back to
// whatever the robot was showing. Every phase plays once (plays = 1), so the
// last, the strand dark, holds until the mode times out rather than looping.
using Speed = LedStrand::Speed;
using Kind = LedStrand::TransitionKind;
using Origin = LedStrand::SpreadOrigin;

constexpr std::uint32_t kRed = 0xFF0000;
constexpr std::uint32_t kBlue = 0x0020FF;
constexpr std::uint32_t kWhite = 0xFFFFFF;
constexpr std::uint32_t kGold = 0xFFB000;
constexpr std::uint32_t kCyan = 0x00FFCC;

// A white comet sweeping end to end and back.
void showScanner(LedStrand& s) {
    s.pulse(kWhite, 5, Speed(1), 0x000000, false, true);
    s.clearSpliceMask();
}

// Red flowing into blue, breathing out from the middle and back in.
void showBreathe(LedStrand& s) {
    s.flow(kRed, kBlue, Speed::everyMs(30));
    s.spreadMask(Speed(1), Origin::CENTER, false, true);
}

// Red, white and blue racing stripes.
void showStripes(LedStrand& s) {
    s.bitscroll({{kRed, 4}, {kWhite, 2}, {kBlue, 4}}, Speed(1), false, 0x000000, false, 2);
    s.clearSpliceMask();
}

// A fast rainbow chopped into blocks that blink in turn.
void showRainbowChop(LedStrand& s) {
    s.rainbow(Speed(2));
    s.spliceMask(5, false, true, 150);
}

// Dense confetti in every color.
void showConfetti(LedStrand& s) {
    s.twinkle({kRed, kBlue, kWhite, kMemoryResetColor, kGold, kCyan}, 60, 40, 0x000000, 2);
    s.clearSpliceMask();
}

// Police lights: each half of the strand (near enough each of its two strips)
// swapping red and blue.
void showPolice(LedStrand& s) {
    s.setColor(kRed);
    s.spliceMask(1, false, true, 125, kBlue);
}

// A white strobe hit...
void showStrobeHit(LedStrand& s) {
    s.flash(kWhite, kStrobeOnMs, 50);
    s.clearSpliceMask();
}

// ...slamming out from the middle into solid red...
void showSlam(LedStrand& s) {
    s.setColor(kRed);
    s.clearSpliceMask();
}

// ...and the curtain: dark closing in from both ends to the middle.
void showCurtain(LedStrand& s) {
    s.off();
    s.clearSpliceMask();
}

// {how long, what, plays, how it wipes in over the phase before}. A
// transition's fields: kind, ms, spread origin, spread inward/from the far
// end, twinkle bit width, twinkle fade step.
constexpr hitlib::Sequencer::Phase kShowcasePhases[] = {
    {1500, showScanner, 1, {}},
    {2000, showBreathe, 1, {Kind::SPREAD, 300, Origin::CENTER, false, 0, 0}},
    {2500, showStripes, 1, {Kind::TWINKLE, 400, Origin::EDGE, false, 2, 0}},
    {2500, showRainbowChop, 1, {Kind::SPREAD, 400, Origin::EDGE, false, 0, 0}},
    {2000, showConfetti, 1, {Kind::TWINKLE, 500, Origin::EDGE, false, 1, 32}},
    {1800, showPolice, 1, {}},
    {800, showStrobeHit, 1, {}},
    {1200, showSlam, 1, {Kind::SPREAD, 200, Origin::CENTER, false, 0, 0}},
    {1000, showCurtain, 1, {Kind::SPREAD, 900, Origin::CENTER, true, 0, 0}},
};

constexpr std::uint32_t phasesMs() {
    std::uint32_t total = 0;
    for (const hitlib::Sequencer::Phase& phase : kShowcasePhases) total += phase.durationMs;
    return total;
}

// How long the mode stays on: every phase, plus slack for each phase ending on
// the refresh tick after its time is up rather than on it, so the curtain
// always finishes closing before the robot's own modes come back.
constexpr std::uint32_t kShowcaseMs = phasesMs() + 400;

// One Sequencer runs on both strands, each at its own place in it.
hitlib::Sequencer showcaseSeq(kShowcasePhases,
                              static_cast<std::uint8_t>(std::size(kShowcasePhases)));

void showcaseActivate(LedStrand& s) { showcaseSeq.start(s); }
void showcaseTick(LedStrand& s) { showcaseSeq.update(s); }

/// Whether Showcase is still on, with ledMutex() held.
bool showcasePlaying() {
    return showcaseStarted && pros::millis() - showcaseStartMs < kShowcaseMs;
}

// One strand's profile: its design's modes, then LiftGauge and the rest. A
// strand keeps a pointer to its profile, so these live for the whole program.
struct RobotProfile {
    std::array<hitlib::ProfileMode, kDesignModeCount + kExtraModeCount> modes{};
    hitlib::Profile profile{};
};

std::array<RobotProfile, 2> robotProfiles;

template <std::size_t Slot>
void attachWithExtras(LedStrand& strand, const hitlib::Profile& design) {
    RobotProfile& p = robotProfiles[Slot];
    std::copy_n(design.modes, kDesignModeCount, p.modes.begin());
    p.modes[kLiftGaugeMode] = {"LiftGauge", kLiftGaugePriority, gaugeActivate<Slot>,
                               gaugeTick<Slot>};
    p.modes[kMemoryResetMode] = {"MemoryReset", kMemoryResetPriority, memoryResetActivate,
                                 nullptr};
    p.modes[kShowcaseMode] = {"Showcase", kShowcasePriority, showcaseActivate, showcaseTick};
    p.modes[kStrobeMode] = {"Strobe", kStrobePriority, strobeActivate, nullptr};
    p.modes[kDarkMode] = {"Dark", kDarkPriority, darkActivate, nullptr};
    p.modes[kGoalMode] = {"Goal", kGoalPriority, goalActivate<Slot>, goalTick<Slot>};
    p.profile = {design.name, p.modes.data(), static_cast<std::uint8_t>(p.modes.size())};
    strand.attachProfile(&p.profile);
}

} // namespace

void start() {
    hitlib::studio::begin();
    // begin() attached each strand's design profile; these are the same with
    // LiftGauge and the rest added. Attaching one empties the strand's mode
    // stack, so Idle goes back on after.
    attachWithExtras<0>(toggleleft::strand, toggleleft::profile);
    attachWithExtras<1>(toggleright::strand, toggleright::profile);
    // Idle by name keeps it on Idle if the design adds a mode ahead of it.
    hitlib::studio::group.activateMode(kIdleMode);
}

void showAutoClaw(bool on) { show(autoClawShown, kAutoClawMode, on); }

void showScoring(bool on) { show(scoringShown, kScoringMode, on); }

void setLiftGauge(int level, int levels) { gaugeWanted.store(packGauge(level, levels)); }

void showLiftGauge() {
    // A timed mode already on has its deadline pushed out instead.
    hitlib::studio::group.activateModeTimed(kLiftGaugeMode, kLiftGaugeMs);
}

void showMemoryReset() {
    // A second L1+L2 while it's still on pushes its deadline out instead.
    hitlib::studio::group.activateModeTimed(kMemoryResetMode, kMemoryResetMs);
}

void showGoal(int goal) {
    // Stored first, so Goal's onActivate draws this goal.
    goalWanted.store(goal);
    // A timed mode already on has its deadline pushed out instead.
    hitlib::studio::group.activateModeTimed(kGoalMode, kGoalMs);
}

void toggleShowcase() {
    std::lock_guard<pros::Mutex> lock(ledMutex());
    if (showcasePlaying()) {
        showcaseStarted = false;
        hitlib::studio::group.deactivateMode(kShowcaseMode);
        return;
    }
    showcaseStarted = true;
    showcaseStartMs = pros::millis();
    hitlib::studio::group.activateModeTimed(kShowcaseMode, kShowcaseMs);
}

void toggleStrobe() {
    std::lock_guard<pros::Mutex> lock(ledMutex());
    showLocked(strobeShown, kStrobeMode, !strobeShown);
}

void toggleLeds() {
    std::lock_guard<pros::Mutex> lock(ledMutex());
    showLocked(darkShown, kDarkMode, !darkShown);
    // Turning them off ends a showcase: it'd otherwise carry on its clock
    // unseen, and come back partway through.
    if (darkShown && showcasePlaying()) {
        showcaseStarted = false;
        hitlib::studio::group.deactivateMode(kShowcaseMode);
    }
}

void showIdle() {
    showAutoClaw(false);
    showScoring(false);
    hitlib::studio::group.deactivateMode(kLiftGaugeMode);
    hitlib::studio::group.deactivateMode(kMemoryResetMode);
    hitlib::studio::group.deactivateMode(kGoalMode);
}

} // namespace robot::leds
