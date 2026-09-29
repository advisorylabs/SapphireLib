// Host-side end-to-end test for Auto-Tune: the real characterization runners
// (src/sapphirelib/tuning/characterization_runner.cpp) drive a simulated axis
// through a fake clock, the real fit identifies it, the real design picks
// gains, and those gains close the loop on the same simulated axis — the
// whole chain PidTunerPage runs on the robot, minus the robot. The fake clock
// is util/clock.hpp's seam: time only moves when the runner (or the closed
// loop below) sleeps, and moving it moves the axis.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/tuning/characterization_runner_test.cpp src/sapphirelib/tuning/characterization_runner.cpp src/sapphirelib/tuning/characterization_math.cpp src/sapphirelib/tuning/gain_design.cpp src/sapphirelib/control/feedforward.cpp src/sapphirelib/mechanism/position_control.cpp src/sapphirelib/control/pid.cpp -o characterization_runner_test && ./characterization_runner_test
// clang-format on

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/mechanism/position_control.hpp"
#include "sapphirelib/tuning/characterization_math.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"
#include "sapphirelib/tuning/gain_design.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::PID;
using sapphirelib::mechanism::computePositionCommand;
using sapphirelib::mechanism::PositionConfig;
using sapphirelib::tuning::AxisCharacterization;
using sapphirelib::tuning::characterizeAxis;
using sapphirelib::tuning::characterizeMechanism;
using sapphirelib::tuning::CharacterizationConfig;
using sapphirelib::tuning::CharacterizationData;
using sapphirelib::tuning::designPositionGains;
using sapphirelib::tuning::GainDesign;
using sapphirelib::tuning::GravityKind;
using sapphirelib::tuning::MechanismCharacterization;
using sapphirelib::tuning::MechanismCharacterizationConfig;
using sapphirelib::tuning::MechanismModel;
using sapphirelib::tuning::ResponseSpec;
using sapphirelib::tuning::runCharacterization;
using sapphirelib::tuning::runMechanismCharacterization;

namespace {

void expectWithin(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %f, expected %f +- %f\n", label, actual, expected, tolerance);
        assert(false);
    }
}

void expectTrue(bool condition, const char* label) {
    if (!condition) {
        std::printf("FAIL %s\n", label);
        assert(false);
    }
}

double signOf(double value) { return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0); }

// --- A simulated axis on a fake clock ------------------------------------------

/// kA·a = u − kS·sign(v) − kG·g(x) − kV·v, with static friction, a command
/// latency, a brake that freezes it in place, and sensor noise. Integrated in
/// 1ms steps as the fake clock advances.
struct SimAxis {
    MechanismModel truth;
    std::uint32_t latencyMs = 0;
    double noise = 0.0;
    double x = 0.0;
    double v = 0.0;

    /// What reaches the motors, and when: each command takes effect
    /// latencyMs after it was sent.
    struct Input {
        std::uint32_t atMs;
        bool held;
        double volts;
    };
    std::deque<Input> pending{};
    bool held = false;
    double volts = 0.0;

    std::mt19937 rng{1};
    std::vector<double> commandLog{}; // every actuate(), in order ("hold" = NaN)
    int measures = 0;

    double measure() {
        ++measures;
        std::normal_distribution<double> jitter(0.0, noise > 0.0 ? noise : 1.0);
        return x + (noise > 0.0 ? jitter(rng) : 0.0);
    }
    void actuate(double u);
    void hold();
    void step1ms();
};

std::uint32_t fakeMs = 0;
SimAxis* sim = nullptr;

void SimAxis::actuate(double u) {
    commandLog.push_back(u);
    pending.push_back(Input{fakeMs + latencyMs, false, u});
}

void SimAxis::hold() {
    commandLog.push_back(std::numeric_limits<double>::quiet_NaN());
    pending.push_back(Input{fakeMs + latencyMs, true, 0.0});
}

void SimAxis::step1ms() {
    while (!pending.empty() && pending.front().atMs <= fakeMs) {
        held = pending.front().held;
        volts = pending.front().volts;
        pending.pop_front();
    }
    if (held) {
        v = 0.0; // the brake holds it where it is
        return;
    }
    constexpr int kSubsteps = 4;
    const double h = 0.001 / kSubsteps;
    for (int s = 0; s < kSubsteps; ++s) {
        const double load = volts - truth.gravityVolts(x);
        const double friction = std::fabs(v) > 1e-3
                                    ? truth.motion.kS * signOf(v)
                                    : signOf(load) * std::min(std::fabs(load), truth.motion.kS);
        v += (load - friction - truth.motion.kV * v) / truth.motion.kA * h;
        x += v * h;
    }
}

} // namespace

// The clock seam (util/clock.hpp), faked.
namespace sapphirelib {
std::uint32_t millis() { return fakeMs; }
std::uint64_t micros() { return static_cast<std::uint64_t>(fakeMs) * 1000; }
void delayMs(std::uint32_t ms) {
    for (std::uint32_t i = 0; i < ms; ++i) {
        if (sim != nullptr) sim->step1ms();
        ++fakeMs;
    }
}
} // namespace sapphirelib

namespace {

CharacterizationConfig axisConfig(SimAxis& axis) {
    CharacterizationConfig config;
    config.actuate = [&axis](double u) { axis.actuate(u); };
    config.measure = [&axis] { return axis.measure(); };
    return config;
}

/// Closes a 10ms position loop on `axis` the way the drivetrains do (PID on
/// the error, output clamped to ±12V) from rest at 0 to `target`. Returns the
/// peak and the time it last left a ±`band` window around the target.
struct StepResult {
    double peak;
    double settleS;
    double finalError;
};

StepResult closeDriveLoop(SimAxis& axis, sapphirelib::PIDGains gains, double target, double band) {
    PID pid(PID::Config{.gains = gains, .outputLimit = 12.0, .nominalDtS = 0.01});
    axis.x = 0.0;
    axis.v = 0.0;
    double peak = 0.0;
    double lastOutsideS = 0.0;
    for (int i = 0; i < 300; ++i) {
        const double position = axis.x;
        peak = std::max(peak, position);
        if (std::fabs(target - position) > band) lastOutsideS = i * 0.01;
        axis.actuate(pid.update(target - position, 0.0));
        sapphirelib::delayMs(10);
    }
    return StepResult{peak, lastOutsideS + 0.01, std::fabs(target - axis.x)};
}

// --- Tests -----------------------------------------------------------------------

/// About 78 in/s at 12V, 30ms from command to motion.
const MechanismModel kDriveTruth{.motion = {.kS = 1.1, .kV = 0.14, .kA = 0.035}};

/// An elevator lift in sensor degrees: 2V holds it up.
const MechanismModel kLiftTruth{.motion = {.kS = 0.6, .kV = 0.017, .kA = 0.002},
                                .kG = 2.0,
                                .gravity = {.kind = GravityKind::constant}};

void testDriveAxisEndToEnd() {
    SimAxis axis{.truth = kDriveTruth, .latencyMs = 30, .noise = 0.01};
    sim = &axis;
    fakeMs = 5000;

    CharacterizationConfig config = axisConfig(axis);
    config.maxTravel = 30.0;
    const CharacterizationData data = runCharacterization(config);
    expectTrue(!data.aborted && data.ramps.size() == 2 && data.steps.size() == 2,
               "drive: all four segments ran");
    expectTrue(std::fabs(axis.x) < 30.0, "drive: alternating directions keep it near home");

    const AxisCharacterization result = characterizeAxis(data, config.minSpeed);
    expectTrue(result.ok, "drive: fit");
    expectWithin(result.fit.model.kV, kDriveTruth.motion.kV, 0.04 * kDriveTruth.motion.kV,
                 "drive: kV");
    expectWithin(result.fit.model.kA, kDriveTruth.motion.kA, 0.12 * kDriveTruth.motion.kA,
                 "drive: kA");
    expectWithin(result.fit.model.kS, kDriveTruth.motion.kS, 0.2, "drive: kS");
    expectWithin(result.delayS, 0.03, 0.015, "drive: delay");

    const ResponseSpec spec{.settleTimeS = 0.6, .dampingRatio = 1.0};
    const GainDesign design = designPositionGains(result.fit.model, spec, result.delayS);
    expectTrue(design.ok, "drive: design");

    // The designed gains on the axis that was measured, friction and all: no
    // real overshoot, settled within 1in (driveDistance()'s default exit)
    // about when the design said. Friction can stop it anywhere in the
    // static band, which the design reports.
    const StepResult step = closeDriveLoop(axis, design.gains, 12.0, 1.0);
    expectTrue(step.peak < 12.0 * 1.05, "drive: no real overshoot");
    expectTrue(step.settleS < design.settleTimeS * 1.4, "drive: settles about on time");
    expectTrue(step.finalError <= design.staticErrorBound + 0.05,
               "drive: comes to rest inside the friction band the design reports");
    sim = nullptr;
}

void testLiftEndToEnd() {
    SimAxis lift{.truth = kLiftTruth, .latencyMs = 20, .noise = 0.05, .x = 5.0};
    sim = &lift;
    fakeMs = 9000;

    MechanismCharacterizationConfig config;
    config.axis = axisConfig(lift);
    config.axis.minSpeed = 5.0;
    config.axis.rampMaxVolts = 10.0;
    config.axis.stepVolts = 8.0;
    config.lowerLimit = 40.0;
    config.upperLimit = 640.0;
    config.hold = [&lift] { lift.hold(); };
    config.gravity = {.kind = GravityKind::constant};

    double highest = lift.x;
    config.axis.shouldAbort = [&] {
        highest = std::max(highest, lift.x);
        return false;
    };
    const CharacterizationData data = runMechanismCharacterization(config);
    expectTrue(!data.aborted && data.ramps.size() == 2 && data.steps.size() == 2,
               "lift: all four segments ran");
    expectTrue(highest < 700.0, "lift: never ran far past its upper limit");
    expectTrue(std::isnan(lift.commandLog.back()), "lift: ends held, not at 0V");

    const MechanismCharacterization result =
        characterizeMechanism(data, config.gravity, config.axis.minSpeed);
    expectTrue(result.ok, "lift: fit");
    expectWithin(result.fit.model.kG, kLiftTruth.kG, 0.15, "lift: kG");
    expectWithin(result.fit.model.motion.kS, kLiftTruth.motion.kS, 0.15, "lift: kS");
    expectWithin(result.fit.model.motion.kV, kLiftTruth.motion.kV, 0.05 * kLiftTruth.motion.kV,
                 "lift: kV");
    expectWithin(result.fit.model.motion.kA, kLiftTruth.motion.kA, 0.15 * kLiftTruth.motion.kA,
                 "lift: kA");
    expectWithin(result.delayS, 0.02, 0.015, "lift: delay");

    // What PidTunerPage + the robot's onMeasured do with it: design gains
    // from the motion model, cancel gravity with the fitted feedforward, and
    // run PositionMechanism's control law at the opcontrol tick.
    const ResponseSpec spec{.settleTimeS = 0.5, .dampingRatio = 1.0};
    const GainDesign design =
        designPositionGains(result.fit.model.motion, spec, result.delayS);
    expectTrue(design.ok, "lift: design");
    const PositionConfig position{
        .pid = {.gains = design.gains,
                .outputLimit = 12.0,
                .derivativeOnMeasurement = true,
                .nominalDtS = 0.02},
        .gravity = result.fit.model.gravityFeedforward(),
    };
    PID pid(position.pid);

    // Hold at one level, then step up and back down, as a driver would.
    lift.x = 100.0;
    lift.v = 0.0;
    lift.pending.clear();
    lift.held = false;
    for (const double target : {100.0, 400.0, 150.0}) {
        double peakOvershoot = 0.0;
        double lastOutsideS = 0.0;
        const double from = lift.x;
        for (int i = 0; i < 100; ++i) {
            const double reading = lift.x;
            const double past = (target - from) >= 0.0 ? reading - target : target - reading;
            peakOvershoot = std::max(peakOvershoot, past);
            if (std::fabs(target - reading) > 15.0) lastOutsideS = i * 0.02;
            lift.actuate(computePositionCommand(pid, position, target, reading).volts);
            sapphirelib::delayMs(20);
        }
        char label[64];
        std::snprintf(label, sizeof(label), "lift: holds %.0f without sagging", target);
        // With gravity cancelled, only static friction can leave it short.
        expectTrue(std::fabs(target - lift.x) <= design.staticErrorBound + 1.0, label);
        std::snprintf(label, sizeof(label), "lift: reaches %.0f within 15deg promptly", target);
        expectTrue(lastOutsideS < 0.9, label);
        std::snprintf(label, sizeof(label), "lift: no real overshoot at %.0f", target);
        expectTrue(peakOvershoot < 0.05 * std::fabs(target - from) + 2.0, label);
    }
    sim = nullptr;
}

void testAbortStopsTheRunAndStillFinishes() {
    SimAxis axis{.truth = kDriveTruth};
    sim = &axis;
    fakeMs = 1000;

    int starts = 0;
    int finishes = 0;
    std::size_t commandsAtStart = 0;
    CharacterizationConfig config = axisConfig(axis);
    config.maxTravel = 30.0;
    // Stopped partway through the first ramp.
    config.shouldAbort = [] { return fakeMs > 1000 + 400; };
    config.start = [&] {
        ++starts;
        commandsAtStart = axis.commandLog.size();
    };
    config.finish = [&] {
        ++finishes;
        expectTrue(axis.commandLog.back() == 0.0, "abort: 0V was the last command before finish");
    };
    const CharacterizationData data = runCharacterization(config);
    expectTrue(data.aborted, "abort: reported");
    expectTrue(data.ramps.size() == 1 && data.steps.empty(), "abort: no segments after it");
    expectTrue(starts == 1 && finishes == 1, "abort: start and finish each ran once");
    expectTrue(commandsAtStart == 0, "start ran before the first command");
    expectTrue(fakeMs < 1000 + 600, "abort: returned promptly");
    sim = nullptr;
}

void testMechanismAbortEndsHeld() {
    SimAxis lift{.truth = kLiftTruth, .x = 5.0};
    sim = &lift;
    fakeMs = 1000;
    int finishes = 0;
    MechanismCharacterizationConfig config;
    config.axis = axisConfig(lift);
    config.axis.shouldAbort = [] { return fakeMs > 1000 + 700; };
    config.axis.finish = [&] { ++finishes; };
    config.lowerLimit = 40.0;
    config.upperLimit = 640.0;
    config.hold = [&lift] { lift.hold(); };
    const CharacterizationData data = runMechanismCharacterization(config);
    expectTrue(data.aborted && finishes == 1, "mechanism abort: reported, finished once");
    expectTrue(std::isnan(lift.commandLog.back()), "mechanism abort: left held");
    sim = nullptr;
}

void testLostSensorEndsTheSegment() {
    // A sensor that stops answering mid-segment must not leave the runner
    // driving blind until the duration cap.
    SimAxis axis{.truth = kDriveTruth};
    sim = &axis;
    fakeMs = 1000;
    CharacterizationConfig config = axisConfig(axis);
    config.measure = [&axis] {
        return fakeMs > 1000 + 500 ? std::numeric_limits<double>::quiet_NaN() : axis.measure();
    };
    const CharacterizationData data = runCharacterization(config);
    expectTrue(!data.ramps.empty(), "lost sensor: the first ramp started");
    const auto& first = data.ramps.front();
    expectTrue(first.back().timeMs < 500, "lost sensor: the segment ended when it did");
    for (const auto& run : data.steps) {
        expectTrue(run.empty(), "lost sensor: later segments end at once");
    }
    expectTrue(axis.commandLog.back() == 0.0, "lost sensor: left at 0V");
    sim = nullptr;
}

} // namespace

int main() {
    testDriveAxisEndToEnd();
    testLiftEndToEnd();
    testAbortStopsTheRunAndStillFinishes();
    testMechanismAbortEndsHeld();
    testLostSensorEndsTheSegment();
    std::puts("characterization_runner_test: all assertions passed");
    return 0;
}
