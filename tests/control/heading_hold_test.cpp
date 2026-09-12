// Host-side unit test for sapphirelib::advanceHeldHeadingDeg — no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/control/heading_hold_test.cpp \
//       src/sapphirelib/control/heading_hold.cpp src/sapphirelib/util/angle.cpp \
//       -o heading_hold_test && ./heading_hold_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/control/heading_hold.hpp"
#include "sapphirelib/util/angle.hpp"

using sapphirelib::advanceHeldHeadingDeg;
using sapphirelib::HeadingHoldConfig;
using sapphirelib::wrapDegrees180;

namespace {

// A deadband of 0 keeps most cases below reading as plain rate x time; the
// deadband gets its own tests.
constexpr HeadingHoldConfig kPlain{.slewDegPerSec = 180.0, .deadband = 0.0, .maxLeadDeg = 0.0};

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-9) {
        std::printf("FAIL %s: got %f, expected %f\n", label, actual, expected);
        assert(false);
    }
}

void expectTrue(bool condition, const char* label) {
    if (!condition) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

void testCenteredStickHoldsTheHeading() {
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, /*turnInput=*/0.0, /*dtS=*/0.02, kPlain), 90.0,
               "centered stick doesn't move the target");
}

void testFullStickSweepsAtTheSlewRate() {
    // Rate x time, so a tenth of a second of full stick is a tenth of
    // slewDegPerSec. (Timesteps here stay well under the implausible-dt
    // guard that testImplausibleTimestepsAdvanceNothing covers.)
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 1.0, 0.1, kPlain), 18.0, "full stick for 100ms");
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, -1.0, 0.1, kPlain), -18.0,
               "full stick the other way");
    // And a realistic 20ms driver tick.
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 1.0, 0.02, kPlain), 3.6, "one 20ms tick");
}

void testPartialStickIsProportional() {
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 0.5, 0.1, kPlain), 9.0, "half stick");
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 0.25, 0.1, kPlain), 4.5, "quarter stick");
}

void testSweepAccumulatesAcrossTicks() {
    // Fifty 20ms driver ticks of full stick is a second's worth of slew,
    // accumulated the way the real loop does it rather than in one step.
    double held = 0.0;
    for (int tick = 0; tick < 50; ++tick) {
        held = advanceHeldHeadingDeg(held, /*currentHeadingDeg=*/held, 1.0, 0.02, kPlain);
    }
    expectNear(held, 180.0, "50 ticks of 20ms");
}

void testDeadbandIgnoresStickSlop() {
    // A V5 stick at rest still reports a count or two; without this the held
    // heading would creep all match with nobody touching the controls.
    const HeadingHoldConfig config{.slewDegPerSec = 180.0, .deadband = 0.05, .maxLeadDeg = 0.0};
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, 0.01, 0.1, config), 90.0, "well inside deadband");
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, -0.04, 0.1, config), 90.0, "inside, negative");
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, 0.05, 0.1, config), 90.0, "exactly at the edge");
}

void testDeadbandRescalesRatherThanJumping() {
    // Just past the deadband commands a rate just above zero, not
    // deadband x slew. Half the *remaining* travel is half the slew rate.
    const HeadingHoldConfig config{.slewDegPerSec = 180.0, .deadband = 0.2, .maxLeadDeg = 0.0};
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 0.2001, 0.1, config), 18.0 * (0.0001 / 0.8),
               "just past the deadband barely moves");
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 0.6, 0.1, config), 9.0, "midway past the deadband");
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 1.0, 0.1, config), 18.0,
               "full stick still full rate");
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, -0.6, 0.1, config), -9.0, "and symmetric");
}

void testImplausibleTimestepsAdvanceNothing() {
    // Same guard PID::update() applies: a scheduling hiccup isn't elapsed
    // control time, and integrating a stick across it would swing the held
    // heading by an arbitrary amount.
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, 1.0, /*dtS=*/0.0, kPlain), 90.0, "zero dt");
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, 1.0, /*dtS=*/-0.02, kPlain), 90.0, "negative dt");
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, 1.0, /*dtS=*/5.0, kPlain), 90.0, "implausible dt");

    // The cutoff sits at half a second, which a driver loop never reaches —
    // HolonomicDrivetrain re-adopts the live heading after a quarter of one,
    // so its own calls stay well inside this.
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 1.0, /*dtS=*/0.5, kPlain), 90.0, "half a second");
    expectNear(advanceHeldHeadingDeg(90.0, 90.0, 1.0, /*dtS=*/0.51, kPlain), 90.0,
               "just past the cutoff");
}

void testLeadIsCappedSoTheTargetCannotRunAway() {
    // The chassis is stuck at 0 while the driver holds full stick. Without a
    // cap the target would be 180 degrees away after a second and the
    // chassis would keep spinning long after the stick was released.
    const HeadingHoldConfig config{.slewDegPerSec = 180.0, .deadband = 0.0, .maxLeadDeg = 20.0};
    double held = 0.0;
    for (int tick = 0; tick < 50; ++tick) {
        held = advanceHeldHeadingDeg(held, /*currentHeadingDeg=*/0.0, 1.0, 0.02, config);
    }
    expectNear(held, 20.0, "target parks at the lead cap");
}

void testChassisCatchingUpLetsTheTargetAdvanceAgain() {
    // The cap tracks the chassis rather than freezing: as the chassis turns,
    // the same held stick keeps pulling the target along ahead of it.
    const HeadingHoldConfig config{.slewDegPerSec = 180.0, .deadband = 0.0, .maxLeadDeg = 20.0};
    double held = 0.0;
    double chassis = 0.0;
    for (int tick = 0; tick < 50; ++tick) {
        // Chassis chases at half the commanded slew, so it falls behind and
        // the lead saturates early on, then stays saturated.
        chassis += 90.0 * 0.02;
        held = advanceHeldHeadingDeg(held, chassis, 1.0, 0.02, config);
    }
    expectNear(chassis, 90.0, "chassis turned 90 degrees");
    expectNear(held, chassis + 20.0, "target stayed one lead-cap ahead");
}

void testLeadCapPullsTheTargetBackWhenTheChassisIsShoved() {
    // Nothing to do with the stick: a collision leaves the chassis a long
    // way off the held heading, and banking that whole error would have it
    // spin back hard. The cap bounds what gets banked.
    const HeadingHoldConfig config{.slewDegPerSec = 180.0, .deadband = 0.0, .maxLeadDeg = 20.0};
    const double held = advanceHeldHeadingDeg(/*heldHeadingDeg=*/90.0, /*currentHeadingDeg=*/10.0,
                                              0.0, 0.02, config);
    expectNear(held, 30.0, "target follows the shove to within the cap");
}

void testLeadCapWorksAcrossTheSeam() {
    // A held heading of 5 against a live heading of 355 is 10 degrees of
    // lead, not 350 — the difference between a nudge and a full spin.
    const HeadingHoldConfig config{.slewDegPerSec = 180.0, .deadband = 0.0, .maxLeadDeg = 20.0};

    const double justInside = advanceHeldHeadingDeg(5.0, 355.0, 0.0, 0.02, config);
    expectNear(wrapDegrees180(justInside - 355.0), 10.0, "10 degrees of lead survives the seam");

    // 60 degrees past the seam is outside the cap and gets pulled back to it.
    const double wellOutside = advanceHeldHeadingDeg(55.0, 355.0, 0.0, 0.02, config);
    expectNear(wrapDegrees180(wellOutside - 355.0), 20.0, "capped across the seam");
}

void testDisabledLeadCapLetsTheTargetRunFree() {
    // maxLeadDeg <= 0 is documented as "no cap" — only sensible when the
    // slew rate is already slow enough that the chassis keeps up.
    const HeadingHoldConfig config{.slewDegPerSec = 180.0, .deadband = 0.0, .maxLeadDeg = 0.0};
    expectNear(advanceHeldHeadingDeg(0.0, 0.0, 1.0, 0.5, config), 90.0, "no cap applied");
}

void testHeldHeadingStaysNearTheChassis() {
    // The property the drivetrain relies on: whatever the stick does, the
    // held heading tracks the chassis within the cap, so it can never wind
    // up an unbounded debt.
    const HeadingHoldConfig config{.slewDegPerSec = 360.0, .deadband = 0.05, .maxLeadDeg = 15.0};
    double held = 0.0;
    double chassis = 0.0;
    const double inputs[] = {1.0, 1.0, -1.0, 0.0, 0.5, -0.9, 0.0, 1.0};
    for (int tick = 0; tick < 200; ++tick) {
        const double input = inputs[tick % 8];
        held = advanceHeldHeadingDeg(held, chassis, input, 0.02, config);
        chassis += 40.0 * 0.02 * (input >= 0.0 ? 1.0 : -1.0);
        expectTrue(std::fabs(wrapDegrees180(held - chassis)) <= 15.0 + 1e-9,
                   "held heading stays within the lead cap");
    }
}

} // namespace

int main() {
    testCenteredStickHoldsTheHeading();
    testFullStickSweepsAtTheSlewRate();
    testPartialStickIsProportional();
    testSweepAccumulatesAcrossTicks();
    testDeadbandIgnoresStickSlop();
    testDeadbandRescalesRatherThanJumping();
    testImplausibleTimestepsAdvanceNothing();
    testLeadIsCappedSoTheTargetCannotRunAway();
    testChassisCatchingUpLetsTheTargetAdvanceAgain();
    testLeadCapPullsTheTargetBackWhenTheChassisIsShoved();
    testLeadCapWorksAcrossTheSeam();
    testDisabledLeadCapLetsTheTargetRunFree();
    testHeldHeadingStaysNearTheChassis();
    std::puts("heading_hold_test: all assertions passed");
    return 0;
}
