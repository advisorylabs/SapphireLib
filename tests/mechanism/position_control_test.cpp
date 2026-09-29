/*
 * Host-side unit test for sapphirelib::mechanism's position control law
 * (computePositionCommand, GravityFeedforward, positionLawName) — no
 * PROS/embedded dependencies, so it builds and runs with a normal desktop
 * compiler.
 *
 * The centerpiece is a bit-exactness check against a verbatim copy of the
 * lift loop from the robot's original driver macros (driveLift() in
 * src/robot_macros.cpp), which this law replaced: over random targets and
 * sensor readings, with dropouts, both must send the identical millivolts and
 * brake calls and leave their PIDs in identical states.
 *
 * (A block comment, not //, so the backslash-continued build line doesn't
 * trip -Wcomment.)
 *
 * Build & run:
 *   g++ -std=c++20 -Iinclude tests/mechanism/position_control_test.cpp \
 *       src/sapphirelib/mechanism/position_control.cpp \
 *       src/sapphirelib/control/pid.cpp \
 *       -o position_control_test && ./position_control_test
 */

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>

#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/mechanism/position_control.hpp"

using sapphirelib::PID;
using sapphirelib::PidObserver;
using sapphirelib::PidStep;
using sapphirelib::mechanism::computePositionCommand;
using sapphirelib::mechanism::GravityFeedforward;
using sapphirelib::mechanism::PositionCommand;
using sapphirelib::mechanism::PositionConfig;
using sapphirelib::mechanism::PositionLaw;
using sapphirelib::mechanism::positionLawName;

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                       \
            std::exit(1);                                                                          \
        }                                                                                          \
    } while (0)

// pros/error.h's value, spelled out so this test needs no PROS header.
#define PROS_ERR (INT32_MAX)

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool sameStep(const PidStep& a, const PidStep& b) {
    return sameBits(a.target, b.target) && sameBits(a.measurement, b.measurement) &&
           sameBits(a.error, b.error) && sameBits(a.pTerm, b.pTerm) && sameBits(a.iTerm, b.iTerm) &&
           sameBits(a.dTerm, b.dTerm) && sameBits(a.rawOutput, b.rawOutput) &&
           sameBits(a.output, b.output) && sameBits(a.dtS, b.dtS) && a.flags == b.flags;
}

/// Counts how often a PID is updated and reset.
struct CountingObserver : PidObserver {
    int updates = 0;
    int resets = 0;
    void onPidUpdate(const PID&, const PidStep&) override { ++updates; }
    void onPidReset(const PID&) override { ++resets; }
};

// --- Fakes for the two PROS devices driveLift() touches ---

struct MotorCommand {
    bool brake = false;
    std::int32_t millivolts = 0;
    bool operator==(const MotorCommand&) const = default;
};

struct FakeMotorGroup {
    MotorCommand last;
    int commands = 0;
    std::int32_t move_voltage(std::int32_t millivolts) {
        last = {.brake = false, .millivolts = millivolts};
        ++commands;
        return 1;
    }
    std::int32_t brake() {
        last = {.brake = true, .millivolts = 0};
        ++commands;
        return 1;
    }
};

struct FakeRotation {
    std::int32_t centidegrees = 0;
    std::int32_t get_position() const { return centidegrees; }
};

} // namespace

// --- The reference: src/robot_macros.cpp's lift loop, verbatim ---
//
// liftPositionDeg() and driveLift() are copied unchanged from the original
// file. Only the declarations around them differ: the devices are the fakes
// above, and the constants are variables rather than constexpr so each
// scenario below can swap in its own — the first scenario is the robot's real
// values.
namespace original {

sapphirelib::PIDGains kLiftGains{.kP = 0.2, .kI = 0.0, .kD = 0.01};
double kLiftGravityVolts = 0.0;
double kLiftSeatVolts = 0.0;
double kLiftRestDeg = 2.0;

FakeMotorGroup lift;
FakeRotation liftSensor;

sapphirelib::PID liftPid(sapphirelib::PID::Config{
    .gains = kLiftGains,
    .outputLimit = 12.7,
    // Level changes step the target, which would otherwise kick the output.
    .derivativeOnMeasurement = true,
    .nominalDtS = 0.02, // update() runs every 20ms opcontrol tick
});

/// NaN if the rotation sensor isn't answering.
double liftPositionDeg() {
    const std::int32_t centidegrees = liftSensor.get_position();
    if (centidegrees == PROS_ERR) return std::numeric_limits<double>::quiet_NaN();
    return centidegrees / 100.0;
}

void driveLift(double targetDeg) {
    const double positionDeg = liftPositionDeg();
    if (std::isnan(positionDeg)) {
        // Chasing a missing reading could run the lift into either end, so
        // hold it where it is on the motors' own encoders instead.
        lift.brake();
        liftPid.reset();
        return;
    }
    double volts;
    if (targetDeg > 0.0) {
        volts = liftPid.update(targetDeg, positionDeg) + kLiftGravityVolts;
    } else if (positionDeg > kLiftRestDeg) {
        // Heading for level 0: see kLiftSeatVolts.
        volts = std::min(liftPid.update(targetDeg, positionDeg), -kLiftSeatVolts);
    } else {
        volts = 0.0;
        liftPid.reset();
    }
    lift.move_voltage(static_cast<std::int32_t>(std::clamp(volts, -12.0, 12.0) * 1000.0));
}

} // namespace original

namespace {

/// The new side: the law plus the two lines PositionMechanism::step() uses to
/// turn its command into a device call.
struct PortedLift {
    PositionConfig config;
    PID pid;
    FakeMotorGroup motors;

    explicit PortedLift(PositionConfig c) : config(c), pid(c.pid) {}

    PositionLaw drive(double target, double position) {
        const PositionCommand command = computePositionCommand(pid, config, target, position);
        if (command.brake) {
            motors.brake();
        } else {
            motors.move_voltage(command.millivolts());
        }
        return command.law;
    }
};

struct Scenario {
    const char* name;
    PID::Config pid;
    double gravityVolts;
    double seatVolts;
    double restDeg;
};

// The robot's own lift config first, then variations that exercise the parts
// of the law (and of PID) the real values leave at zero.
const Scenario kScenarios[] = {
    {.name = "robot lift",
     .pid = {.gains = {.kP = 0.2, .kI = 0.0, .kD = 0.01},
             .outputLimit = 12.7,
             .derivativeOnMeasurement = true,
             .nominalDtS = 0.02},
     .gravityVolts = 0.0,
     .seatVolts = 0.0,
     .restDeg = 2.0},
    {.name = "gravity + seat + kI",
     .pid = {.gains = {.kP = 0.12, .kI = 0.5, .kD = 0.004},
             .outputLimit = 12.7,
             .derivativeOnMeasurement = true,
             .nominalDtS = 0.02},
     .gravityVolts = 0.8,
     .seatVolts = 1.5,
     .restDeg = 2.0},
    {.name = "no output limit, wide rest band",
     .pid = {.gains = {.kP = 0.05, .kI = 2.0, .kD = 0.002},
             .integralLimit = 40.0,
             .derivativeOnMeasurement = true,
             .nominalDtS = 0.01},
     .gravityVolts = 1.7,
     .seatVolts = 3.0,
     .restDeg = 5.0},
    {.name = "slew + derivative on error",
     .pid = {.gains = {.kP = 0.3, .kI = 0.1, .kD = 0.02},
             .outputLimit = 10.0,
             .slewRate = 0.75,
             .derivativeOnMeasurement = false,
             .nominalDtS = 0.02},
     .gravityVolts = -0.4,
     .seatVolts = 0.0,
     .restDeg = 0.5},
};

PositionConfig portedConfig(const Scenario& scenario) {
    return PositionConfig{
        .pid = scenario.pid,
        .gravity = {.constantVolts = scenario.gravityVolts},
        .seat = {.enabled = true,
                 .floor = 0.0,
                 .seatVolts = scenario.seatVolts,
                 .restBand = scenario.restDeg},
        .tolerance = 15.0,
    };
}

void testBitExactAgainstOriginalDriveLift() {
    std::mt19937_64 rng(0x5eed1f7);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double ladder[] = {0.0,   75.0,  90.0,  112.5, 150.0, 180.0, 225.0, 300.0,
                             337.5, 360.0, 450.0, 540.0, 600.0, 675.0, 720.0, 900.0};
    long ticks = 0;
    long lawCounts[6] = {};

    for (const Scenario& scenario : kScenarios) {
        original::kLiftGains = scenario.pid.gains;
        original::kLiftGravityVolts = scenario.gravityVolts;
        original::kLiftSeatVolts = scenario.seatVolts;
        original::kLiftRestDeg = scenario.restDeg;

        for (int trace = 0; trace < 150; ++trace) {
            original::liftPid = PID(scenario.pid);
            original::lift = FakeMotorGroup{};
            PortedLift ported(portedConfig(scenario));

            double target = 0.0;
            int targetHold = 0;
            std::int32_t centidegrees = static_cast<std::int32_t>(unit(rng) * 90000.0);
            int dropout = 0;

            for (int tick = 0; tick < 2000; ++tick) {
                // Target: held for a while, like a lift level, then changed.
                if (targetHold-- <= 0) {
                    targetHold = static_cast<int>(unit(rng) * 100.0);
                    const double pick = unit(rng);
                    if (pick < 0.35) {
                        target = ladder[static_cast<int>(unit(rng) * 16.0) % 16];
                    } else if (pick < 0.45) {
                        target = 0.0;
                    } else if (pick < 0.50) {
                        target = (unit(rng) < 0.5 ? -1.0 : 1.0) * 1e-9;
                    } else if (pick < 0.55) {
                        target = -10.0 * unit(rng);
                    } else {
                        target = -20.0 + 970.0 * unit(rng);
                    }
                }

                // Reading: a random walk with jumps, readings right at the
                // rest-band edge, and bursts of PROS_ERR dropouts.
                const double event = unit(rng);
                if (dropout > 0) {
                    --dropout;
                } else if (event < 0.03) {
                    dropout = 1 + static_cast<int>(unit(rng) * 10.0);
                } else if (event < 0.08) {
                    centidegrees = static_cast<std::int32_t>(-2000.0 + 102000.0 * unit(rng));
                } else if (event < 0.13) {
                    const std::int32_t edge = static_cast<std::int32_t>(scenario.restDeg * 100.0);
                    const std::int32_t edges[] = {0, 1, -1, edge - 1, edge, edge + 1};
                    centidegrees = edges[static_cast<int>(unit(rng) * 6.0) % 6];
                } else {
                    centidegrees += static_cast<std::int32_t>((unit(rng) - 0.5) * 600.0);
                }
                const std::int32_t reading = dropout > 0 ? PROS_ERR : centidegrees;

                // Now and then, a loop restart resets both (as the gap reset
                // does), mid-motion.
                if (unit(rng) < 0.005) {
                    original::liftPid.reset();
                    ported.pid.reset();
                }

                original::liftSensor.centidegrees = reading;
                original::driveLift(target);
                const double position = original::liftPositionDeg(); // same conversion
                const PositionLaw law = ported.drive(target, position);
                ++lawCounts[static_cast<int>(law)];

                // One device call each, and the same one.
                CHECK(original::lift.commands == tick + 1);
                CHECK(ported.motors.commands == tick + 1);
                CHECK(original::lift.last == ported.motors.last);
                // The same PID step recorded...
                CHECK(sameStep(original::liftPid.lastStep(), ported.pid.lastStep()));
                // ...and the same hidden state (integral, previous
                // error/measurement/output, first-step flag): copies of the
                // two PIDs answer an arbitrary next input identically.
                PID originalProbe = original::liftPid;
                PID portedProbe = ported.pid;
                const double probeTarget = target + 13.25;
                const double probeMeasurement = std::isnan(position) ? 42.0 : position - 3.5;
                CHECK(sameBits(originalProbe.update(probeTarget, probeMeasurement),
                               portedProbe.update(probeTarget, probeMeasurement)));
                CHECK(sameStep(originalProbe.lastStep(), portedProbe.lastStep()));
                ++ticks;
            }
        }
    }
    // Every branch the original has must actually have been exercised.
    CHECK(lawCounts[static_cast<int>(PositionLaw::track)] > 10000);
    CHECK(lawCounts[static_cast<int>(PositionLaw::seat)] > 10000);
    CHECK(lawCounts[static_cast<int>(PositionLaw::rest)] > 1000);
    CHECK(lawCounts[static_cast<int>(PositionLaw::sensorLost)] > 10000);
    std::printf("  bit-exact vs original driveLift(): %ld ticks (track %ld, seat %ld, rest %ld, "
                "no sensor %ld)\n",
                ticks, lawCounts[0], lawCounts[1], lawCounts[2], lawCounts[3]);
}

void testGravityConstantIsExact() {
    // No cos() rounding may reach a mechanism that didn't ask for it: with
    // cosineVolts 0, volts() is constantVolts to the bit, at any position.
    const double constants[] = {0.0, -0.0, 0.8, -1.25, 3.3};
    const double positions[] = {0.0, 1e-9, 45.0, 90.0, -720.5, 12345.678, kNaN};
    for (double constant : constants) {
        const GravityFeedforward gravity{.constantVolts = constant, .horizontalPosition = 30.0};
        for (double position : positions) CHECK(sameBits(gravity.volts(position), constant));
    }
}

void testGravityCosineArm() {
    const GravityFeedforward arm{.cosineVolts = 2.0};
    CHECK(std::fabs(arm.volts(0.0) - 2.0) < 1e-12);   // level: full holding volts
    CHECK(std::fabs(arm.volts(90.0)) < 1e-12);        // straight up: none
    CHECK(std::fabs(arm.volts(180.0) + 2.0) < 1e-12); // level the other way
    CHECK(std::fabs(arm.volts(60.0) - 1.0) < 1e-12);  // cos 60 = 1/2

    // Horizontal somewhere other than 0, plus a constant on top.
    const GravityFeedforward offset{
        .constantVolts = 0.5, .cosineVolts = 2.0, .horizontalPosition = 90.0};
    CHECK(std::fabs(offset.volts(90.0) - 2.5) < 1e-12);
    CHECK(std::fabs(offset.volts(180.0) - 0.5) < 1e-12);
    CHECK(std::fabs(offset.volts(0.0) - 0.5) < 1e-12);

    // Sensor on the motor side of a 5:1 reduction: 450 sensor degrees is 90
    // arm degrees.
    const GravityFeedforward geared{.cosineVolts = 2.0, .armDegreesPerUnit = 1.0 / 5.0};
    CHECK(std::fabs(geared.volts(450.0)) < 1e-12);
    CHECK(std::fabs(geared.volts(900.0) + 2.0) < 1e-12);
}

PositionConfig seatedConfig() {
    return PositionConfig{
        .pid = {.gains = {.kP = 0.01}, .nominalDtS = 0.02},
        .gravity = {.constantVolts = 1.0},
        .seat = {.enabled = true, .floor = 0.0, .seatVolts = 3.0, .restBand = 2.0},
    };
}

void testTrackAddsGravity() {
    const PositionConfig config = seatedConfig();
    PID pid(config.pid);
    const PositionCommand command = computePositionCommand(pid, config, 100.0, 40.0);
    CHECK(command.law == PositionLaw::track);
    CHECK(!command.brake);
    CHECK(std::fabs(command.volts - (0.01 * 60.0 + 1.0)) < 1e-12);
}

void testSeatDrivesDownAtLeastSeatVolts() {
    const PositionConfig config = seatedConfig();

    // A gentle loop output is overridden by the seat minimum, and no gravity
    // is added while seating.
    PID gentle(config.pid);
    PositionCommand command = computePositionCommand(gentle, config, 0.0, 50.0);
    CHECK(command.law == PositionLaw::seat);
    CHECK(command.volts == -3.0); // min(-0.5, -3)

    // A loop output already stronger than the minimum is kept as is.
    PositionConfig strong = config;
    strong.pid.gains.kP = 0.2;
    PID strongPid(strong.pid);
    command = computePositionCommand(strongPid, strong, 0.0, 50.0);
    CHECK(command.law == PositionLaw::seat);
    CHECK(std::fabs(command.volts - (-10.0)) < 1e-12);

    // A target below the floor seats too; one just above it tracks.
    PID below(config.pid);
    CHECK(computePositionCommand(below, config, -5.0, 50.0).law == PositionLaw::seat);
    PID above(config.pid);
    CHECK(computePositionCommand(above, config, 1e-9, 50.0).law == PositionLaw::track);

    // Seat disabled: a target at the floor is plain tracking.
    PositionConfig noSeat = config;
    noSeat.seat.enabled = false;
    PID plain(noSeat.pid);
    command = computePositionCommand(plain, noSeat, 0.0, 50.0);
    CHECK(command.law == PositionLaw::track);
    CHECK(std::fabs(command.volts - (-0.5 + 1.0)) < 1e-12);
}

void testRestCutsPowerAndResetsLoop() {
    const PositionConfig config = seatedConfig();
    PID pid(config.pid);
    CountingObserver observer;
    pid.setObserver(&observer);

    computePositionCommand(pid, config, 0.0, 30.0); // seating: the PID has state now
    CHECK(observer.updates == 1);

    // Exactly at floor + restBand is resting (the seat needs position > it).
    PositionCommand command = computePositionCommand(pid, config, 0.0, 2.0);
    CHECK(command.law == PositionLaw::rest);
    CHECK(command.volts == 0.0);
    CHECK(!command.brake);
    CHECK(observer.updates == 1); // no loop update while resting
    CHECK(observer.resets == 1);

    // Resting again: nothing left to clear, so no second reset report.
    command = computePositionCommand(pid, config, 0.0, 0.5);
    CHECK(command.law == PositionLaw::rest);
    CHECK(observer.resets == 1);

    // The next loop update starts fresh: no derivative from the old reading.
    computePositionCommand(pid, config, 100.0, 0.5);
    CHECK((pid.lastStep().flags & PidStep::kFirstStep) != 0);
    CHECK(pid.lastStep().dTerm == 0.0);
}

void testSensorLostBrakesAndResets() {
    PositionConfig config = seatedConfig();
    config.pid.gains.kD = 0.05;
    for (double missing : {kNaN, kInf, -kInf}) {
        PID pid(config.pid);
        CountingObserver observer;
        pid.setObserver(&observer);
        computePositionCommand(pid, config, 300.0, 100.0);
        const PositionCommand command = computePositionCommand(pid, config, 300.0, missing);
        CHECK(command.law == PositionLaw::sensorLost);
        CHECK(command.brake);
        CHECK(command.volts == 0.0);
        CHECK(command.millivolts() == 0);
        CHECK(observer.updates == 1); // the missing reading never reached the loop
        CHECK(observer.resets == 1);
        // Back online: a fresh start, not a derivative across the dropout.
        computePositionCommand(pid, config, 300.0, 150.0);
        CHECK((pid.lastStep().flags & PidStep::kFirstStep) != 0);
    }
}

void testClampAtMaxVolts() {
    PositionConfig config{.pid = {.gains = {.kP = 1.0}}, .maxVolts = 8.0};
    PID up(config.pid);
    CHECK(computePositionCommand(up, config, 100.0, 0.0).volts == 8.0);
    PID down(config.pid);
    CHECK(computePositionCommand(down, config, -100.0, 0.0).volts == -8.0);
    PID within(config.pid);
    CHECK(computePositionCommand(within, config, 5.0, 0.0).volts == 5.0);

    // Feedforward is added before the clamp, so it can't push past it either.
    config.gravity.constantVolts = 3.0;
    PID withGravity(config.pid);
    CHECK(computePositionCommand(withGravity, config, 6.0, 0.0).volts == 8.0);
}

void testUpdatesPidAtMostOnce() {
    const PositionConfig config = seatedConfig();
    PID pid(config.pid);
    CountingObserver observer;
    pid.setObserver(&observer);
    const struct {
        double target;
        double position;
        int expectedUpdates;
    } cases[] = {
        {100.0, 50.0, 1}, // track
        {0.0, 50.0, 1},   // seat
        {0.0, 1.0, 0},    // rest
        {100.0, kNaN, 0}, // sensor lost
    };
    for (const auto& c : cases) {
        const int before = observer.updates;
        computePositionCommand(pid, config, c.target, c.position);
        CHECK(observer.updates - before == c.expectedUpdates);
    }
}

void testMillivolts() {
    // Truncation toward zero, exactly static_cast<int32_t>(volts * 1000.0).
    CHECK((PositionCommand{.volts = 12.0}.millivolts()) == 12000);
    CHECK((PositionCommand{.volts = -12.0}.millivolts()) == -12000);
    CHECK((PositionCommand{.volts = 1.23456}.millivolts()) == 1234);
    CHECK((PositionCommand{.volts = -1.23456}.millivolts()) == -1234);
    CHECK((PositionCommand{.volts = 0.0009}.millivolts()) == 0);
    CHECK((PositionCommand{.volts = -0.0009}.millivolts()) == 0);
}

void testLawNames() {
    CHECK(std::strcmp(positionLawName(PositionLaw::track), "track") == 0);
    CHECK(std::strcmp(positionLawName(PositionLaw::seat), "seat") == 0);
    CHECK(std::strcmp(positionLawName(PositionLaw::rest), "rest") == 0);
    CHECK(std::strcmp(positionLawName(PositionLaw::sensorLost), "no sensor") == 0);
    CHECK(std::strcmp(positionLawName(PositionLaw::manual), "manual") == 0);
    CHECK(std::strcmp(positionLawName(PositionLaw::off), "off") == 0);
}

/// A lift on a hard stop at 0: `gravityVolts` of its weight pulls it down,
/// and it accelerates at kAccelPerVolt deg/s² per net volt, with viscous
/// damping. Runs `seconds` of the law at 100Hz and returns the final
/// position; `lastLaw`/`lastVolts` report the final command.
struct SimResult {
    double position;
    PositionLaw lastLaw;
    double lastVolts;
};

SimResult simulateLift(const PositionConfig& config, double gravityVolts, double start,
                       double target, double seconds) {
    constexpr double kDtS = 0.01;
    constexpr double kAccelPerVolt = 200.0;
    constexpr double kDamping = 10.0;
    PID pid(config.pid);
    double position = start;
    double velocity = 0.0;
    PositionCommand command;
    for (int i = 0; i < static_cast<int>(seconds / kDtS); ++i) {
        command = computePositionCommand(pid, config, target, position);
        const double volts = command.brake ? 0.0 : command.millivolts() / 1000.0;
        const double acceleration = kAccelPerVolt * (volts - gravityVolts) - kDamping * velocity;
        velocity += acceleration * kDtS;
        position += velocity * kDtS;
        if (position < 0.0) { // the hard stop
            position = 0.0;
            velocity = 0.0;
        }
    }
    return {.position = position, .lastLaw = command.law, .lastVolts = command.volts};
}

void testSimulatedLiftConvergesWithFeedforward() {
    constexpr double kGravity = 2.0;
    constexpr double kP = 0.1;
    PositionConfig config{
        .pid = {.gains = {.kP = kP, .kD = 0.005},
                .outputLimit = 12.0,
                .derivativeOnMeasurement = true,
                .nominalDtS = 0.01},
        .gravity = {.constantVolts = kGravity},
        .tolerance = 1.0,
    };
    const SimResult held = simulateLift(config, kGravity, 0.0, 300.0, 10.0);
    CHECK(std::fabs(held.position - 300.0) <= config.tolerance);
    CHECK(held.lastLaw == PositionLaw::track);

    // Without the feedforward, a P loop settles short by exactly the error
    // whose kP·error holds the weight: kG / kP.
    config.gravity.constantVolts = 0.0;
    const SimResult sagging = simulateLift(config, kGravity, 0.0, 300.0, 10.0);
    CHECK(std::fabs((300.0 - sagging.position) - kGravity / kP) < 0.1);
}

void testSimulatedLiftSeatsAndRests() {
    constexpr double kGravity = 2.0;
    const PositionConfig config{
        .pid = {.gains = {.kP = 0.1, .kD = 0.005},
                .outputLimit = 12.0,
                .derivativeOnMeasurement = true,
                .nominalDtS = 0.01},
        .gravity = {.constantVolts = kGravity},
        .seat = {.enabled = true, .floor = 0.0, .seatVolts = 2.0, .restBand = 2.0},
    };
    const SimResult seated = simulateLift(config, kGravity, 300.0, 0.0, 5.0);
    CHECK(seated.position <= config.seat.floor + config.seat.restBand);
    CHECK(seated.lastLaw == PositionLaw::rest);
    CHECK(seated.lastVolts == 0.0);
}

} // namespace

int main() {
    testBitExactAgainstOriginalDriveLift();
    testGravityConstantIsExact();
    testGravityCosineArm();
    testTrackAddsGravity();
    testSeatDrivesDownAtLeastSeatVolts();
    testRestCutsPowerAndResetsLoop();
    testSensorLostBrakesAndResets();
    testClampAtMaxVolts();
    testUpdatesPidAtMostOnce();
    testMillivolts();
    testLawNames();
    testSimulatedLiftConvergesWithFeedforward();
    testSimulatedLiftSeatsAndRests();
    std::puts("position_control_test: all assertions passed");
    return 0;
}
