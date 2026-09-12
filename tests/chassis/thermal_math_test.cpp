// Host-side unit test for sapphirelib::chassis::thermalPowerFraction and
// centerThermalCorrection — no PROS/embedded dependencies, so it builds and
// runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/chassis/thermal_math_test.cpp \
//       src/sapphirelib/chassis/thermal_math.cpp \
//       -o thermal_math_test && ./thermal_math_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "sapphirelib/chassis/thermal_math.hpp"

using sapphirelib::chassis::CenterCorrection;
using sapphirelib::chassis::centerThermalCorrection;
using sapphirelib::chassis::CornerValues;
using sapphirelib::chassis::thermalPowerFraction;

namespace {

constexpr double kCool = 1.0;
constexpr double kGain = 1.0;
constexpr double kMaxVolts = 6.0;

// Most cases below want to exercise the arithmetic rather than the cap, so
// they pass this instead of a realistic ceiling.
constexpr double kUncapped = 100.0;

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

CornerValues allCorners(double value) { return CornerValues{value, value, value, value}; }

// --- thermalPowerFraction ---

void testCoolMotorIsUnderated() {
    expectNear(thermalPowerFraction(20.0), 1.0, "room temperature");
    expectNear(thermalPowerFraction(50.0), 1.0, "warm but under the limit");
    expectNear(thermalPowerFraction(54.0), 1.0, "just under the first step");
}

void testDeratingStepsHitPublishedValues() {
    // Each ramp is centred on the published step, so the step temperature
    // itself lands on the published fraction.
    expectNear(thermalPowerFraction(55.0), 0.75, "midway into the 55C ramp");
    expectNear(thermalPowerFraction(56.0), 0.5, "55C step settled");
    expectNear(thermalPowerFraction(60.0), 0.375, "midway into the 60C ramp");
    expectNear(thermalPowerFraction(61.0), 0.25, "60C step settled");
    expectNear(thermalPowerFraction(66.0), 0.125, "65C step settled");
}

void testShutdownAboveSeventy() {
    expectNear(thermalPowerFraction(71.0), 0.0, "shutdown");
    expectNear(thermalPowerFraction(90.0), 0.0, "well past shutdown");
}

void testCurveIsMonotonicAndContinuous() {
    // No jumps: a step function here would make the correction surge every
    // time a dithering reading crossed a threshold.
    double previous = thermalPowerFraction(40.0);
    for (double t = 40.0; t <= 80.0; t += 0.1) {
        const double fraction = thermalPowerFraction(t);
        expectTrue(fraction <= previous + 1e-12, "monotonically non-increasing");
        expectTrue(std::fabs(fraction - previous) < 0.05, "no discontinuous jump");
        previous = fraction;
    }
}

void testNonFiniteReadingReadsAsCool() {
    // PROS reports a motor that isn't answering as PROS_ERR_F (infinity).
    // That must not look like a hot motor, or an unplugged cable would
    // manufacture a correction out of nothing.
    expectNear(thermalPowerFraction(std::numeric_limits<double>::infinity()), 1.0, "infinity");
    expectNear(thermalPowerFraction(-std::numeric_limits<double>::infinity()), 1.0, "-infinity");
    expectNear(thermalPowerFraction(std::numeric_limits<double>::quiet_NaN()), 1.0, "NaN");
}

// --- centerThermalCorrection ---

void testCoolChassisNeedsNoCorrection() {
    // Driving hard on cool motors: nothing is being lost, so nothing is
    // added. This is what makes the feature inert on a healthy robot.
    const CenterCorrection correction =
        centerThermalCorrection(allCorners(12.0), allCorners(kCool), kCool, kGain, kMaxVolts);
    expectNear(correction.commonVolts, 0.0, "cool chassis, no common correction");
    expectNear(correction.differentialVolts, 0.0, "cool chassis, no differential correction");
}

void testEvenlyDeratedCornersDrivingForward() {
    // mixHolonomic(t, 0, 0) puts the same voltage on all four corners. Each
    // delivers half, so the chassis is short 4 x 4V = 16V of forward thrust,
    // split between the two center wheels.
    const CenterCorrection correction =
        centerThermalCorrection(allCorners(8.0), allCorners(0.5), kCool, kGain, kUncapped);
    expectNear(correction.commonVolts, 8.0, "even derating boosts forward thrust");
    // Symmetric derating twists nothing.
    expectNear(correction.differentialVolts, 0.0, "even derating adds no twist");
}

void testEvenlyDeratedCornersTurning() {
    // mixHolonomic(0, 0, r) -> (+r, -r, +r, -r). The forward components
    // cancel; the yaw components reinforce.
    const CornerValues turning{6.0, -6.0, 6.0, -6.0};
    const CenterCorrection correction =
        centerThermalCorrection(turning, allCorners(0.5), kCool, kGain, kUncapped);
    expectNear(correction.commonVolts, 0.0, "a turn needs no forward correction");
    expectNear(correction.differentialVolts, 6.0, "even derating boosts the turn");
}

void testOneHotCornerWhileStrafing() {
    // The case this exists for. mixHolonomic(0, s, 0) -> (+s, -s, -s, +s),
    // whose forward components cancel exactly while every corner is healthy.
    // Derate the front right alone and they stop cancelling: the chassis
    // creeps forward and twists across what should be a pure sideways move.
    const CornerValues strafing{10.0, -10.0, -10.0, 10.0};
    const CornerValues frontRightHot{kCool, 0.5, kCool, kCool};

    // Front right is short by -10 * 0.5 = -5V of forward thrust; no other
    // corner is short at all. So the corners produce +5V of forward thrust
    // nobody asked for, and the center wheels push back with half each.
    const CenterCorrection correction =
        centerThermalCorrection(strafing, frontRightHot, kCool, kGain, kUncapped);
    expectNear(correction.commonVolts, -2.5, "cancels the forward drift");
    expectNear(correction.differentialVolts, 2.5, "cancels the twist");
}

void testStrafeDriftMatchesTheDriftItCancels() {
    // Independent check of the above, from the other direction: work out
    // what the corners actually deliver, and confirm the correction is the
    // negative of that error halved across the two center wheels.
    const CornerValues commanded{10.0, -10.0, -10.0, 10.0};
    const CornerValues surviving{1.0, 0.5, 1.0, 1.0};

    const double intendedForward =
        commanded.frontLeft + commanded.frontRight + commanded.backLeft + commanded.backRight;
    const double actualForward =
        commanded.frontLeft * surviving.frontLeft + commanded.frontRight * surviving.frontRight +
        commanded.backLeft * surviving.backLeft + commanded.backRight * surviving.backRight;
    expectNear(intendedForward, 0.0, "a strafe asks for no forward motion");
    expectTrue(std::fabs(actualForward) > 1e-9, "a hot corner produces forward motion anyway");

    const CenterCorrection correction =
        centerThermalCorrection(commanded, surviving, kCool, kGain, kUncapped);
    expectNear(correction.commonVolts, (intendedForward - actualForward) / 2.0,
               "correction is exactly the halved error");
}

void testWhichCornersAreHotDecidesWhetherAStrafeWanders() {
    // A strafe drives one diagonal forward and the other backward, so which
    // corners are hot decides both how far the chassis wanders and which
    // way. An average over the four cannot tell these cases apart — which is
    // why the fractions are tracked per corner.
    const CornerValues strafing{10.0, -10.0, -10.0, 10.0};

    // Front right and back left share a diagonal, so adding back left to the
    // front-right case above doubles that same drift rather than offsetting
    // it: -2.5V of correction becomes -5V.
    const CenterCorrection sameDiagonal = centerThermalCorrection(
        strafing, CornerValues{kCool, 0.5, 0.5, kCool}, kCool, kGain, kUncapped);
    expectNear(sameDiagonal.commonVolts, -5.0, "same diagonal: drift doubles");

    // The other diagonal drifts the opposite way, so a hot corner on each
    // leaves the strafe straight — it only loses speed.
    const CenterCorrection oppositeDiagonals = centerThermalCorrection(
        strafing, CornerValues{0.5, 0.5, kCool, kCool}, kCool, kGain, kUncapped);
    expectNear(oppositeDiagonals.commonVolts, 0.0, "one corner on each diagonal: no net drift");

    // And derating the *other* diagonal alone pushes the chassis the other
    // way, which the correction follows in sign.
    const CenterCorrection otherDiagonal = centerThermalCorrection(
        strafing, CornerValues{0.5, kCool, kCool, 0.5}, kCool, kGain, kUncapped);
    expectNear(otherDiagonal.commonVolts, 5.0, "other diagonal: drift reverses");
}

void testGainScalesTheAttempt() {
    const CenterCorrection half = centerThermalCorrection(allCorners(8.0), allCorners(0.5), kCool,
                                                          /*gain=*/0.5, kUncapped);
    expectNear(half.commonVolts, 4.0, "half gain");

    const CenterCorrection off = centerThermalCorrection(allCorners(8.0), allCorners(0.5), kCool,
                                                         /*gain=*/0.0, kMaxVolts);
    expectNear(off.commonVolts, 0.0, "zero gain disables");
    expectNear(off.differentialVolts, 0.0, "zero gain disables the differential too");
}

void testMaxVoltsCapsEachComponent() {
    const CenterCorrection capped =
        centerThermalCorrection(allCorners(12.0), allCorners(0.0), kCool, kGain, kMaxVolts);
    expectNear(capped.commonVolts, kMaxVolts, "common capped");

    // Caps in the negative direction too — drift correction runs backwards.
    const CornerValues strafing{12.0, -12.0, -12.0, 12.0};
    const CenterCorrection negative = centerThermalCorrection(
        strafing, CornerValues{kCool, 0.0, kCool, kCool}, kCool, kGain, /*maxVolts=*/3.0);
    expectNear(negative.commonVolts, -3.0, "negative common capped");

    const CenterCorrection disabled =
        centerThermalCorrection(allCorners(12.0), allCorners(0.0), kCool, kGain, /*maxVolts=*/0.0);
    expectNear(disabled.commonVolts, 0.0, "a cap of 0 disables");
}

void testHotCenterMotorsBackOff() {
    const CenterCorrection halfDerated = centerThermalCorrection(
        allCorners(8.0), allCorners(0.5), /*centerPowerFraction=*/0.5, kGain, kUncapped);
    expectNear(halfDerated.commonVolts, 4.0, "center motors half derated -> half the correction");

    const CenterCorrection shutDown = centerThermalCorrection(
        allCorners(8.0), allCorners(0.5), /*centerPowerFraction=*/0.0, kGain, kUncapped);
    expectNear(shutDown.commonVolts, 0.0, "center motors shut down -> no correction");
}

void testCorrectionStaysWithinTheCap() {
    const CornerValues patterns[] = {
        allCorners(12.0),
        {12.0, -12.0, -12.0, 12.0},
        {12.0, -12.0, 12.0, -12.0},
        {12.0, 0.0, -6.0, 3.0},
    };
    for (const CornerValues& commanded : patterns) {
        for (double fraction = 0.0; fraction <= 1.0; fraction += 0.05) {
            const CenterCorrection correction = centerThermalCorrection(
                commanded, CornerValues{fraction, 1.0, 0.5, fraction}, kCool, kGain, kMaxVolts);
            expectTrue(std::fabs(correction.commonVolts) <= kMaxVolts + 1e-12, "common within cap");
            expectTrue(std::fabs(correction.differentialVolts) <= kMaxVolts + 1e-12,
                       "differential within cap");
        }
    }
}

} // namespace

int main() {
    testCoolMotorIsUnderated();
    testDeratingStepsHitPublishedValues();
    testShutdownAboveSeventy();
    testCurveIsMonotonicAndContinuous();
    testNonFiniteReadingReadsAsCool();

    testCoolChassisNeedsNoCorrection();
    testEvenlyDeratedCornersDrivingForward();
    testEvenlyDeratedCornersTurning();
    testOneHotCornerWhileStrafing();
    testStrafeDriftMatchesTheDriftItCancels();
    testWhichCornersAreHotDecidesWhetherAStrafeWanders();
    testGainScalesTheAttempt();
    testMaxVoltsCapsEachComponent();
    testHotCenterMotorsBackOff();
    testCorrectionStaysWithinTheCap();

    std::puts("thermal_math_test: all assertions passed");
    return 0;
}
