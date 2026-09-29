// Host-side unit test for sapphirelib::tuning's system identification math
// — no PROS/embedded dependencies, so it builds and runs with a normal
// desktop compiler.
//
// Every test drives a simulated axis with known kS/kV/kA and delay through
// the same ramps and steps tuning::runCharacterization() uses on the robot,
// then checks the fit recovers what the simulation was built with. The
// mechanism tests do the same for a lift and an arm, with gravity, driven the
// way tuning::runMechanismCharacterization() drives them.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/tuning/characterization_math_test.cpp \
//       src/sapphirelib/tuning/characterization_math.cpp \
//       src/sapphirelib/control/feedforward.cpp \
//       src/sapphirelib/mechanism/position_control.cpp \
//       src/sapphirelib/control/pid.cpp \
//       -o characterization_math_test && ./characterization_math_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <random>
#include <vector>

#include "sapphirelib/tuning/characterization_math.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::tuning::AxisCharacterization;
using sapphirelib::tuning::CharacterizationData;
using sapphirelib::tuning::CharacterizationRun;
using sapphirelib::tuning::CharacterizationSample;
using sapphirelib::tuning::characterizeAxis;
using sapphirelib::tuning::characterizeMechanism;
using sapphirelib::tuning::estimateMechanismDelayS;
using sapphirelib::tuning::estimateResponseDelayS;
using sapphirelib::tuning::fitFeedforward;
using sapphirelib::tuning::fitMechanism;
using sapphirelib::tuning::GravityKind;
using sapphirelib::tuning::GravityShape;
using sapphirelib::tuning::MechanismCharacterization;
using sapphirelib::tuning::MechanismModel;

namespace {

constexpr double kTickS = 0.01;
constexpr int kSubsteps = 20;

/// A forward-axis-sized plant: about 78 in/s at 12V.
constexpr MotorFeedforward kForwardTruth{.kS = 1.1, .kV = 0.14, .kA = 0.035};

/// A turn-axis-sized plant: about 238 deg/s at 12V.
constexpr MotorFeedforward kTurnTruth{.kS = 1.3, .kV = 0.045, .kA = 0.006};

/// An elevator lift, in degrees of its rotation sensor: 2V to hold it up,
/// about 560 deg/s at 12V going up.
const MechanismModel kLiftTruth{.motion = {.kS = 0.6, .kV = 0.017, .kA = 0.002},
                                .kG = 2.0,
                                .gravity = {.kind = GravityKind::constant}};

/// An arm on a rotation sensor at its pivot, level at a reading of 0.
const MechanismModel kArmTruth{.motion = {.kS = 0.4, .kV = 0.05, .kA = 0.006},
                               .kG = 1.5,
                               .gravity = {.kind = GravityKind::cosine}};

double signOf(double value) { return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0); }

void expectWithin(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %f, expected %f +- %f\n", label, actual, expected, tolerance);
        assert(false);
    }
}

void expectRelative(double actual, double expected, double fraction, const char* label) {
    expectWithin(actual, expected, std::fabs(expected) * fraction, label);
}

void expectTrue(bool condition, const char* label) {
    if (!condition) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

/// Simulates `ticks` 10ms ticks of an axis obeying `truth`, runner-style:
/// read position, then command. Each command reaches the axis
/// `delayTicks` ticks later. Static friction holds the axis still until the
/// applied voltage exceeds kS. `sign` flips the measurement, to model a
/// sensor mounted backwards.
CharacterizationRun simulate(const MotorFeedforward& truth, int delayTicks,
                             const std::function<double(double)>& voltsAt, int ticks,
                             double noise, std::mt19937& rng, double sign = 1.0) {
    std::normal_distribution<double> jitter(0.0, noise > 0.0 ? noise : 1.0);
    std::vector<double> commands;
    CharacterizationRun run;
    double x = 0.0;
    double v = 0.0;

    for (int i = 0; i < ticks; ++i) {
        const double volts = voltsAt(i * kTickS);
        const double reading = sign * x + (noise > 0.0 ? jitter(rng) : 0.0);
        run.push_back(CharacterizationSample{
            .timeMs = static_cast<std::uint32_t>(i * 10), .volts = volts, .position = reading});
        commands.push_back(volts);

        const double applied = i - delayTicks >= 0 ? commands[i - delayTicks] : 0.0;
        const double h = kTickS / kSubsteps;
        for (int s = 0; s < kSubsteps; ++s) {
            const double friction = std::fabs(v) > 1e-3
                                        ? truth.kS * signOf(v)
                                        : signOf(applied) * std::min(std::fabs(applied), truth.kS);
            v += (applied - friction - truth.kV * v) / truth.kA * h;
            x += v * h;
        }
    }
    return run;
}

/// The runner's schedule: a ramp each way, then a step each way, each with a
/// 100ms pre-roll at 0V.
CharacterizationData collect(const MotorFeedforward& truth, int delayTicks, double noise,
                             unsigned seed) {
    std::mt19937 rng(seed);
    const auto ramp = [](double direction) {
        return [direction](double t) {
            return t < 0.1 ? 0.0 : direction * std::min(4.0 * (t - 0.1), 8.0);
        };
    };
    const auto step = [](double direction) {
        return [direction](double t) { return t < 0.1 ? 0.0 : direction * 6.0; };
    };

    CharacterizationData data;
    data.ramps.push_back(simulate(truth, delayTicks, ramp(1.0), 220, noise, rng));
    data.ramps.push_back(simulate(truth, delayTicks, ramp(-1.0), 220, noise, rng));
    data.steps.push_back(simulate(truth, delayTicks, step(1.0), 100, noise, rng));
    data.steps.push_back(simulate(truth, delayTicks, step(-1.0), 100, noise, rng));
    return data;
}

/// One segment of a mechanism run, runner-style: `holdTicks` ticks held on
/// the brake (logged as NaN volts, not moving), then `voltsAt(seconds since
/// the hold ended)` until the reading passes `limit` in `direction` (the
/// runner stops there without commanding that tick) or `maxTicks`. Each
/// command reaches the mechanism `delayTicks` ticks later — the brake too, so
/// it keeps holding that long after the first command.
CharacterizationRun simulateMechanism(const MechanismModel& truth, int delayTicks, double start,
                                      int holdTicks, const std::function<double(double)>& voltsAt,
                                      double direction, double limit, int maxTicks, double noise,
                                      std::mt19937& rng) {
    std::normal_distribution<double> jitter(0.0, noise > 0.0 ? noise : 1.0);
    std::vector<double> commands; // NaN = hold
    CharacterizationRun run;
    double x = start;
    double v = 0.0;

    for (int i = 0; i < maxTicks; ++i) {
        const double reading = x + (noise > 0.0 ? jitter(rng) : 0.0);
        if (i >= holdTicks && (direction > 0.0 ? reading >= limit : reading <= limit)) break;
        const double volts = i < holdTicks ? std::numeric_limits<double>::quiet_NaN()
                                           : voltsAt((i - holdTicks) * kTickS);
        run.push_back(CharacterizationSample{
            .timeMs = static_cast<std::uint32_t>(i * 10), .volts = volts, .position = reading});
        commands.push_back(volts);

        const double applied = i - delayTicks >= 0 ? commands[i - delayTicks]
                                                   : std::numeric_limits<double>::quiet_NaN();
        if (std::isnan(applied)) {
            v = 0.0; // the brake holds it where it is
            continue;
        }
        const double h = kTickS / kSubsteps;
        for (int s = 0; s < kSubsteps; ++s) {
            const double load = applied - truth.gravityVolts(x);
            const double friction = std::fabs(v) > 1e-3
                                        ? truth.motion.kS * signOf(v)
                                        : signOf(load) * std::min(std::fabs(load), truth.motion.kS);
            v += (load - friction - truth.motion.kV * v) / truth.motion.kA * h;
            x += v * h;
        }
    }
    return run;
}

/// The mechanism runner's schedule between `lower` and `upper`: ramp up, ramp
/// down, step up, step down, each from rest at the far end of the last one.
CharacterizationData collectMechanism(const MechanismModel& truth, int delayTicks, double lower,
                                      double upper, double noise, unsigned seed) {
    std::mt19937 rng(seed);
    const auto ramp = [](double direction) {
        return [direction](double t) { return direction * std::min(4.0 * t, 10.0); };
    };
    const auto step = [](double direction, double volts) {
        return [direction, volts](double) { return direction * volts; };
    };
    CharacterizationData data;
    data.ramps.push_back(
        simulateMechanism(truth, delayTicks, lower, 10, ramp(1.0), 1.0, upper, 360, noise, rng));
    data.ramps.push_back(
        simulateMechanism(truth, delayTicks, upper, 10, ramp(-1.0), -1.0, lower, 360, noise, rng));
    data.steps.push_back(simulateMechanism(truth, delayTicks, lower, 10, step(1.0, 8.0), 1.0, upper,
                                           260, noise, rng));
    data.steps.push_back(simulateMechanism(truth, delayTicks, upper, 10, step(-1.0, 6.0), -1.0,
                                           lower, 260, noise, rng));
    return data;
}

void expectRecoversMechanism(const MechanismCharacterization& result, const MechanismModel& truth,
                             double delayS, const char* label) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s: ok", label);
    expectTrue(result.ok, buf);
    std::snprintf(buf, sizeof(buf), "%s: kS", label);
    expectWithin(result.fit.model.motion.kS, truth.motion.kS, 0.12, buf);
    std::snprintf(buf, sizeof(buf), "%s: kG", label);
    expectWithin(result.fit.model.kG, truth.kG, 0.12, buf);
    std::snprintf(buf, sizeof(buf), "%s: kV", label);
    expectRelative(result.fit.model.motion.kV, truth.motion.kV, 0.04, buf);
    std::snprintf(buf, sizeof(buf), "%s: kA", label);
    expectRelative(result.fit.model.motion.kA, truth.motion.kA, 0.12, buf);
    std::snprintf(buf, sizeof(buf), "%s: delay", label);
    expectWithin(result.delayS, delayS, 0.012, buf);
    std::snprintf(buf, sizeof(buf), "%s: R2", label);
    expectTrue(result.fit.rSquared > 0.95, buf);
}

void expectRecovers(const AxisCharacterization& result, const MotorFeedforward& truth,
                    double delayS, const char* label) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s: ok", label);
    expectTrue(result.ok, buf);
    std::snprintf(buf, sizeof(buf), "%s: kS", label);
    expectRelative(result.fit.model.kS, truth.kS, 0.12, buf);
    std::snprintf(buf, sizeof(buf), "%s: kV", label);
    expectRelative(result.fit.model.kV, truth.kV, 0.03, buf);
    std::snprintf(buf, sizeof(buf), "%s: kA", label);
    expectRelative(result.fit.model.kA, truth.kA, 0.08, buf);
    std::snprintf(buf, sizeof(buf), "%s: delay", label);
    expectWithin(result.delayS, delayS, 0.01, buf);
    std::snprintf(buf, sizeof(buf), "%s: R2", label);
    expectTrue(result.fit.rSquared > 0.95, buf);
}

void testRecoversACleanAxis() {
    expectRecovers(characterizeAxis(collect(kForwardTruth, 0, 0.0, 1), 2.0), kForwardTruth, 0.0,
                   "clean, no delay");
}

void testRecoversAnAxisWithDelay() {
    expectRecovers(characterizeAxis(collect(kForwardTruth, 3, 0.0, 1), 2.0), kForwardTruth, 0.03,
                   "clean, 30ms delay");
}

void testRecoversANoisyAxisWithDelay() {
    // Several seeds, since a single lucky noise draw proves little.
    for (unsigned seed = 1; seed <= 5; ++seed) {
        expectRecovers(characterizeAxis(collect(kForwardTruth, 3, 0.005, seed), 2.0),
                       kForwardTruth, 0.03, "0.005in noise, 30ms delay");
    }
}

void testRecoversATurnSizedAxis() {
    expectRecovers(characterizeAxis(collect(kTurnTruth, 4, 0.0, 1), 5.0), kTurnTruth, 0.04,
                   "turn axis, 40ms delay");
}

void testIgnoringDelayWouldHaveBiasedTheFit() {
    // The reason characterizeAxis() bothers with a second pass: fitting
    // delayed data as if it weren't. This pins down that the unshifted fit
    // really is worse, so the second pass is doing real work.
    const CharacterizationData data = collect(kForwardTruth, 6, 0.0, 1);
    std::vector<CharacterizationRun> all = data.ramps;
    all.insert(all.end(), data.steps.begin(), data.steps.end());

    const double naiveError =
        std::fabs(fitFeedforward(all, 2.0, 5, 0).model.kS - kForwardTruth.kS);
    const double shiftedError =
        std::fabs(fitFeedforward(all, 2.0, 5, 6).model.kS - kForwardTruth.kS);
    expectTrue(shiftedError < naiveError, "shifting by the delay improves kS");
}

void testStationaryAxisFails() {
    // Voltage too low to break friction: nothing moves, nothing to fit.
    std::mt19937 rng(1);
    CharacterizationData data;
    const auto weak = [](double t) { return t < 0.1 ? 0.0 : 0.8; };
    data.ramps.push_back(simulate(kForwardTruth, 0, weak, 200, 0.001, rng));
    data.steps.push_back(simulate(kForwardTruth, 0, weak, 100, 0.001, rng));

    const AxisCharacterization result = characterizeAxis(data, 2.0);
    expectTrue(!result.ok, "stationary axis is rejected");
    expectTrue(result.fit.samplesUsed < 20, "and reports why");
}

void testBackwardsSensorFails() {
    // Positive volts making the reading shrink can't be a physical axis; the
    // fit must refuse rather than hand back negative gains.
    std::mt19937 rng(1);
    CharacterizationData data;
    const auto ramp = [](double t) { return t < 0.1 ? 0.0 : std::min(4.0 * (t - 0.1), 8.0); };
    const auto step = [](double t) { return t < 0.1 ? 0.0 : 6.0; };
    data.ramps.push_back(simulate(kForwardTruth, 0, ramp, 220, 0.0, rng, -1.0));
    data.steps.push_back(simulate(kForwardTruth, 0, step, 100, 0.0, rng, -1.0));

    expectTrue(!characterizeAxis(data, 2.0).ok, "reversed sensor is rejected");
}

void testDelayNeedsAStep() {
    const CharacterizationRun idle(50, CharacterizationSample{});
    expectTrue(estimateResponseDelayS(idle, kForwardTruth) < 0.0, "no step, no delay estimate");
    expectTrue(estimateResponseDelayS(idle, MotorFeedforward{}) < 0.0, "invalid model");
}

void testRecoversALiftWithGravity() {
    expectRecoversMechanism(
        characterizeMechanism(collectMechanism(kLiftTruth, 0, 20.0, 700.0, 0.0, 1),
                              kLiftTruth.gravity, 5.0),
        kLiftTruth, 0.0, "lift, no delay");
    expectRecoversMechanism(
        characterizeMechanism(collectMechanism(kLiftTruth, 3, 20.0, 700.0, 0.0, 1),
                              kLiftTruth.gravity, 5.0),
        kLiftTruth, 0.03, "lift, 30ms delay");
    for (unsigned seed = 1; seed <= 4; ++seed) {
        expectRecoversMechanism(
            characterizeMechanism(collectMechanism(kLiftTruth, 2, 20.0, 700.0, 0.05, seed),
                                  kLiftTruth.gravity, 5.0),
            kLiftTruth, 0.02, "lift, 0.05deg noise, 20ms delay");
    }
}

void testRecoversAnArmWithCosineGravity() {
    expectRecoversMechanism(
        characterizeMechanism(collectMechanism(kArmTruth, 2, -60.0, 60.0, 0.02, 3),
                              kArmTruth.gravity, 3.0),
        kArmTruth, 0.02, "arm, 20ms delay");
}

void testIgnoringGravityBiasesTheFit() {
    // Why the gravity term exists: a lift fitted as if it were a drive axis
    // can't explain needing more volts going up than coming down, and lands
    // on a worse model.
    const CharacterizationData data = collectMechanism(kLiftTruth, 0, 20.0, 700.0, 0.0, 1);
    std::vector<CharacterizationRun> all = data.ramps;
    all.insert(all.end(), data.steps.begin(), data.steps.end());
    const auto withGravity = fitMechanism(all, kLiftTruth.gravity, 5.0);
    const auto without = fitMechanism(all, GravityShape{}, 5.0);
    expectTrue(without.model.kG == 0.0, "no gravity term, no kG");
    expectTrue(withGravity.rSquared > without.rSquared + 0.01, "gravity explains the data better");
    expectTrue(std::fabs(withGravity.model.motion.kS - kLiftTruth.motion.kS) <
                   std::fabs(without.model.motion.kS - kLiftTruth.motion.kS),
               "and gets friction right");
}

void testGravityFeedforwardMatchesTheModel() {
    MechanismModel arm = kArmTruth;
    arm.gravity.horizontalPosition = 30.0;
    arm.gravity.armDegreesPerUnit = 0.5;
    const auto feedforward = arm.gravityFeedforward();
    for (const double position : {-100.0, 0.0, 30.0, 95.0, 210.0}) {
        expectWithin(feedforward.volts(position), arm.gravityVolts(position), 1e-12,
                     "cosine feedforward cancels the model's gravity");
    }
    const auto lift = kLiftTruth.gravityFeedforward();
    expectWithin(lift.constantVolts, kLiftTruth.kG, 0.0, "a lift's kG is its constant volts");
    expectWithin(lift.cosineVolts, 0.0, 0.0, "and it has no cosine term");
    expectWithin(MechanismModel{}.gravityFeedforward().constantVolts, 0.0, 0.0, "none is none");
}

void testHeldSamplesAreSkipped() {
    // A run that is nothing but held samples has nothing to fit and no step,
    // however much the position reading wanders.
    CharacterizationRun held;
    for (int i = 0; i < 100; ++i) {
        held.push_back(CharacterizationSample{.timeMs = static_cast<std::uint32_t>(i * 10),
                                              .volts = std::numeric_limits<double>::quiet_NaN(),
                                              .position = i * 0.5});
    }
    expectTrue(fitMechanism({held}, kLiftTruth.gravity, 5.0).samplesUsed == 0,
               "held intervals are never fitted");
    expectTrue(estimateMechanismDelayS(held, kLiftTruth) < 0.0, "a hold is not a step");
}

// --- The analyzer's golden case --------------------------------------------
//
// tools/analyzer/js/model.js ports this file's math to JavaScript, and
// tools/analyzer/test/model.test.js checks it against the numbers below on
// the very same data. The data is built with nothing but + - * / and
// comparisons (no RNG, no libm), so both languages generate it bit for bit;
// only the fit's final log() can differ in its last bit. Keep goldenRun() in
// step with goldenRun() in that test.

/// Deterministic "noise": a fixed pseudo-random sequence of small integers,
/// scaled.
double goldenNoise(int i, int seed) { return (((i * 7919 + seed * 104729) % 97) - 48) * 1e-4; }

/// simulateMechanism(), deterministic, with a constant gravity load `kG`
/// (0 for a drive axis) and hold ticks marked by NaN.
CharacterizationRun goldenRun(double kS, double kV, double kA, double kG, int delayTicks,
                              int holdTicks, double direction, double volts, bool ramp,
                              double start, double limit, int maxTicks, int seed) {
    std::vector<double> commands;
    CharacterizationRun run;
    double x = start;
    double v = 0.0;
    for (int i = 0; i < maxTicks; ++i) {
        const double reading = x + goldenNoise(i, seed);
        if (i >= holdTicks && (direction > 0.0 ? reading >= limit : reading <= limit)) break;
        double command = std::numeric_limits<double>::quiet_NaN();
        if (i >= holdTicks) {
            const double t = (i - holdTicks) * 0.01;
            command = direction * (ramp ? std::min(4.0 * t, volts) : volts);
        }
        run.push_back(CharacterizationSample{
            .timeMs = static_cast<std::uint32_t>(i * 10), .volts = command, .position = reading});
        commands.push_back(command);
        const double applied = i - delayTicks >= 0 ? commands[i - delayTicks]
                                                   : std::numeric_limits<double>::quiet_NaN();
        if (std::isnan(applied)) {
            v = 0.0;
            continue;
        }
        for (int s = 0; s < 20; ++s) {
            const double load = applied - kG;
            const double friction =
                std::fabs(v) > 1e-3 ? kS * signOf(v) : signOf(load) * std::min(std::fabs(load), kS);
            v += (load - friction - kV * v) / kA * 0.0005;
            x += v * 0.0005;
        }
    }
    return run;
}

CharacterizationData goldenData(double kS, double kV, double kA, double kG, int delayTicks,
                                int holdTicks, double lower, double upper) {
    CharacterizationData data;
    data.ramps.push_back(
        goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, 1.0, 8.0, true, lower, upper, 300, 1));
    data.ramps.push_back(
        goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, -1.0, 8.0, true, upper, lower, 300, 2));
    data.steps.push_back(
        goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, 1.0, 6.0, false, lower, upper, 200, 3));
    data.steps.push_back(
        goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, -1.0, 6.0, false, upper, lower, 200, 4));
    return data;
}

// What this file's math produces on the golden data (printed with
// SAPPHIRELIB_PRINT_GOLDEN=1). tools/analyzer/test/model.test.js holds the
// same numbers.
constexpr int kGoldenDriveSamples = 820;
constexpr double kGoldenDriveKs = 1.0763486609024009;
constexpr double kGoldenDriveKv = 0.14048089905604738;
constexpr double kGoldenDriveKa = 0.035163511411799429;
constexpr double kGoldenDriveR2 = 0.99998910846074862;
constexpr double kGoldenDriveDelay = 0.031434814419046525;
constexpr int kGoldenLiftSamples = 776;
constexpr double kGoldenLiftKs = 0.56109754370967291;
constexpr double kGoldenLiftKv = 0.017076500587205532;
constexpr double kGoldenLiftKa = 0.0021459253802502522;
constexpr double kGoldenLiftKg = 2.011164031791643;
constexpr double kGoldenLiftR2 = 0.9998817832001935;
constexpr double kGoldenLiftDelay = 0.018001926627715409;

void testGoldenCaseMatchesTheAnalyzer() {
    // A drive axis: no gravity, 0V pre-roll (never held), 30ms of delay.
    const AxisCharacterization drive =
        characterizeAxis(goldenData(1.1, 0.14, 0.035, 0.0, 3, 0, -1000.0, 1000.0), 2.0);
    // A lift: constant gravity, held for 10 ticks before each segment.
    const MechanismCharacterization lift =
        characterizeMechanism(goldenData(0.6, 0.017, 0.002, 2.0, 2, 10, 20.0, 700.0),
                              GravityShape{.kind = GravityKind::constant}, 5.0);

    if (std::getenv("SAPPHIRELIB_PRINT_GOLDEN") != nullptr) {
        std::printf("drive %d %.17g %.17g %.17g %.17g %d %.17g\n", drive.ok, drive.fit.model.kS,
                    drive.fit.model.kV, drive.fit.model.kA, drive.fit.rSquared,
                    drive.fit.samplesUsed, drive.delayS);
        std::printf("lift %d %.17g %.17g %.17g %.17g %.17g %d %.17g\n", lift.ok,
                    lift.fit.model.motion.kS, lift.fit.model.motion.kV, lift.fit.model.motion.kA,
                    lift.fit.model.kG, lift.fit.rSquared, lift.fit.samplesUsed, lift.delayS);
    }

    const auto same = [](double actual, double expected, const char* label) {
        expectWithin(actual, expected, std::fabs(expected) * 1e-12, label);
    };
    expectTrue(drive.ok && drive.fit.samplesUsed == kGoldenDriveSamples, "golden drive: ok, rows");
    same(drive.fit.model.kS, kGoldenDriveKs, "golden drive kS");
    same(drive.fit.model.kV, kGoldenDriveKv, "golden drive kV");
    same(drive.fit.model.kA, kGoldenDriveKa, "golden drive kA");
    same(drive.fit.rSquared, kGoldenDriveR2, "golden drive R2");
    same(drive.delayS, kGoldenDriveDelay, "golden drive delay");
    expectTrue(lift.ok && lift.fit.samplesUsed == kGoldenLiftSamples, "golden lift: ok, rows");
    same(lift.fit.model.motion.kS, kGoldenLiftKs, "golden lift kS");
    same(lift.fit.model.motion.kV, kGoldenLiftKv, "golden lift kV");
    same(lift.fit.model.motion.kA, kGoldenLiftKa, "golden lift kA");
    same(lift.fit.model.kG, kGoldenLiftKg, "golden lift kG");
    same(lift.fit.rSquared, kGoldenLiftR2, "golden lift R2");
    same(lift.delayS, kGoldenLiftDelay, "golden lift delay");
}

} // namespace

int main() {
    testRecoversACleanAxis();
    testRecoversAnAxisWithDelay();
    testRecoversANoisyAxisWithDelay();
    testRecoversATurnSizedAxis();
    testIgnoringDelayWouldHaveBiasedTheFit();
    testStationaryAxisFails();
    testBackwardsSensorFails();
    testDelayNeedsAStep();
    testRecoversALiftWithGravity();
    testRecoversAnArmWithCosineGravity();
    testIgnoringGravityBiasesTheFit();
    testGravityFeedforwardMatchesTheModel();
    testHeldSamplesAreSkipped();
    testGoldenCaseMatchesTheAnalyzer();
    std::puts("characterization_math_test: all assertions passed");
    return 0;
}
