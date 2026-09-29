/**
 * \file tuning.cpp
 *
 * PidTunerPage wiring: the round-trip test motions, the Auto-Tune
 * experiments, and what happens with their results. Every characterization
 * run is also mirrored into the SD log (char.fwd/char.strafe/char.turn/
 * char.lift), so an axis can be refit offline from real runs — the telemetry
 * analyzer's Tune tab (tools/analyzer) does exactly that.
 *
 * Team 96671H — Hitmen
 */

#include "robot/tuning.hpp"

#include <string>
#include <utility>

#include "robot/devices.hpp"
#include "robot/macros.hpp"
#include "robot/telemetry.hpp"
#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/telemetry/characterization_tap.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/log.hpp"

namespace robot {

using sapphirelib::MotorFeedforward;
using sapphirelib::chassis::DriverInputMode;
using sapphirelib::chassis::HolonomicAxisModels;
using sapphirelib::gui::PidTunerPage;
using sapphirelib::motion::LocalOffset;
using sapphirelib::motion::toLocalFrame;
using sapphirelib::odom::Pose;
using sapphirelib::telemetry::Channel;
using sapphirelib::telemetry::ChannelOptions;
using sapphirelib::telemetry::tapCharacterization;
using sapphirelib::telemetry::tapMechanismCharacterization;
using sapphirelib::tuning::AxisCharacterization;
using sapphirelib::tuning::CharacterizationConfig;
using sapphirelib::tuning::MechanismCharacterization;
using sapphirelib::tuning::MechanismCharacterizationConfig;
using sapphirelib::tuning::ResponseSpec;

namespace {

// Round-trip test motions for PidTunerPage — repeated "Run Test" taps don't
// walk the robot off the field, since each one returns to where it started.
void driveTuningTest() {
    drivetrain().driveDistance(24);
    // driveDistance() is relative — it measures from wherever the encoders
    // read when it starts — so coming back is -24, not 0 (0 would settle
    // instantly where it already is).
    drivetrain().driveDistance(-24);
}

void turnTuningTest() {
    drivetrain().turnToHeading(90);
    drivetrain().turnToHeading(0);
}

// --- Auto-Tune for PidTunerPage ---
//
// Auto-Tune measures each axis below (forward, strafe, turn) by driving it
// through short voltage ramps and steps, fits a model of it, and designs
// every controller's gains from those models — see PidTunerPage's class
// comment. It needs clear floor: each translation segment travels up to
// kTranslationTravelIn from where it started (plus coasting), alternating
// direction so the robot ends up roughly where it began, and the turn axis
// spins in place for several seconds.
//
// Each experiment factory is called fresh every time Auto-Tune is tapped
// (not once at startup), so translation is measured along whichever way the
// chassis faces at the start of that run.
//
// TODO: the voltages and travel below are starting points, not measured for
// this robot. If an axis reports "no fit", first check its sensor sign
// (a positive voltage must make the measurement grow), then give it more
// travel or voltage.

constexpr double kTranslationTravelIn = 30.0;
constexpr double kTranslationStepVolts = 6.0;
constexpr double kTurnStepVolts = 6.0;

// How each controller should behave — see tuning::ResponseSpec. All three
// are designed from the same axis measurements; only these specs differ.
constexpr ResponseSpec kDriveResponse{.settleTimeS = 0.6, .dampingRatio = 1.0};
constexpr ResponseSpec kTurnResponse{.settleTimeS = 0.5, .dampingRatio = 1.0};
// Softer on purpose: the driver is steering the target heading, and a hold
// that snaps onto it as hard as an autonomous turn feels twitchy and fights
// them whenever the robot gets bumped.
constexpr ResponseSpec kHeadingHoldResponse{.settleTimeS = 0.9, .dampingRatio = 1.0};
// The lift, designed from its own measurement (with gravity) — see
// macros::liftExperiment(). Its loop runs at the 20ms opcontrol tick, which
// the design allows for.
constexpr ResponseSpec kLiftResponse{.settleTimeS = 0.5, .dampingRatio = 1.0};

// The char.* channels' rows arrive every 10ms (the runner's sample period)
// during a run; 512 rows is 5s of them, far more than the logger's writer
// ever falls behind.
constexpr ChannelOptions kCharacterizationChannel{.capacity = 512};

enum class TranslationAxis { forward, strafe };

CharacterizationConfig translationExperiment(TranslationAxis axis) {
    // Captured by value into `measure` below, so every sample reads distance
    // along *this* run's starting heading — the open-loop voltages don't
    // hold heading, and the live heading can drift slightly over a run.
    const Pose reference = odometry().getPose();

    return CharacterizationConfig{
        .actuate =
            [axis](double volts) {
                if (axis == TranslationAxis::forward) {
                    drivetrain().holonomicVolts(volts, 0.0, 0.0);
                } else {
                    drivetrain().holonomicVolts(0.0, volts, 0.0);
                }
            },
        .measure =
            [reference, axis] {
                const Pose pose = odometry().getPose();
                const LocalOffset local = toLocalFrame(pose.xIn - reference.xIn,
                                                       pose.yIn - reference.yIn, reference.headingDeg);
                return axis == TranslationAxis::forward ? local.forwardIn : local.lateralIn;
            },
        .stepVolts = kTranslationStepVolts,
        .maxTravel = kTranslationTravelIn,
        .minSpeed = 2.0, // in/s
    };
}

CharacterizationConfig turnExperiment() {
    return CharacterizationConfig{
        .actuate = [](double volts) { drivetrain().holonomicVolts(0.0, 0.0, volts); },
        // Cumulative (unwrapped) heading, so a multi-turn spin reads as one
        // continuous position instead of jumping at the 0/360 seam.
        .measure = [] { return drivetrain().imu().getCumulativeHeadingDeg(); },
        .stepVolts = kTurnStepVolts,
        .minSpeed = 5.0, // deg/s
    };
}

/// `experiment`, with every sample it takes also recorded into `channel`
/// (see tapCharacterization() — the run itself is unchanged), and a
/// `tune,start,<axis>` event marking where this axis's rows begin in the log.
/// Called on the tuning task, at the start of each axis's run.
CharacterizationConfig logged(const char* axis, CharacterizationConfig experiment,
                              Channel& channel) {
    sapphirelib::telemetry::event("tune", "start,%s", axis);
    return tapCharacterization(std::move(experiment), channel);
}

// Installs one freshly measured axis on the drivetrain, for
// DriverInputMode::velocity, and logs it over `pros terminal` and to the SD
// log — the only places the strafe model shows up, since no PID tab entry
// uses that axis. Not persisted — copy the logged numbers into a
// setAxisModels() call in initDevices() to keep them.
void installModel(MotorFeedforward HolonomicAxisModels::*axis, const char* name,
                  const AxisCharacterization& result) {
    SAPPHIRELIB_LOG_INFO("tune", "%s: kS=%.4f kV=%.5f kA=%.5f R2=%.3f delay=%.0fms", name,
                         result.fit.model.kS, result.fit.model.kV, result.fit.model.kA,
                         result.fit.rSquared, result.delayS * 1000.0);
    // Next to the char.* rows it was fitted from, so an offline refit can be
    // checked against what the robot got.
    sapphirelib::telemetry::event("tune", "model,%s,kS=%.4f,kV=%.5f,kA=%.5f,r2=%.3f,delay_ms=%.0f",
                                  name, result.fit.model.kS, result.fit.model.kV,
                                  result.fit.model.kA, result.fit.rSquared, result.delayS * 1000.0);
    HolonomicAxisModels models = drivetrain().axisModels();
    models.*axis = result.fit.model;
    drivetrain().setAxisModels(models);
}

// Installs the lift's measured gravity feedforward on the running lift, and
// logs its model the same way installModel() does. The Lift entry's gains are
// designed and set by the page itself. Not persisted: copy kG into
// kLiftGravityVolts in macros.cpp to keep it.
void installLiftModel(const MechanismCharacterization& result) {
    const auto& model = result.fit.model;
    SAPPHIRELIB_LOG_INFO("tune", "Lift: kS=%.4f kG=%.4f kV=%.5f kA=%.5f R2=%.3f delay=%.0fms",
                         model.motion.kS, model.kG, model.motion.kV, model.motion.kA,
                         result.fit.rSquared, result.delayS * 1000.0);
    sapphirelib::telemetry::event(
        "tune", "model,Lift,kS=%.4f,kV=%.5f,kA=%.5f,kG=%.4f,r2=%.3f,delay_ms=%.0f", model.motion.kS,
        model.motion.kV, model.motion.kA, model.kG, result.fit.rSquared, result.delayS * 1000.0);
    macros::liftMechanism().setGravity(model.gravityFeedforward());
}

} // namespace

void registerTuning(PidTunerPage& page) {
    // Each axis's samples, mirrored into the SD log as the run takes them —
    // the same (volts, position) pairs Auto-Tune fits on the robot, so the
    // app can refit offline from real runs. Only the experiment factories
    // are wrapped; tuning/ doesn't know. Created here, during initialize(),
    // because registering a channel takes a mutex.
    Channel& forwardLog = logger().channel("char.fwd", {"volts", "pos"}, kCharacterizationChannel);
    Channel& strafeLog = logger().channel("char.strafe", {"volts", "pos"}, kCharacterizationChannel);
    Channel& turnLog = logger().channel("char.turn", {"volts", "pos"}, kCharacterizationChannel);
    Channel& liftLog = logger().channel("char.lift", {"volts", "pos"}, kCharacterizationChannel);

    page.addAxis(
        "Fwd",
        [channel = &forwardLog] {
            return logged("Fwd", translationExperiment(TranslationAxis::forward), *channel);
        },
        [](const AxisCharacterization& result) {
            installModel(&HolonomicAxisModels::forward, "Fwd", result);
        });
    page.addAxis(
        "Strafe",
        [channel = &strafeLog] {
            return logged("Strafe", translationExperiment(TranslationAxis::strafe), *channel);
        },
        [](const AxisCharacterization& result) {
            installModel(&HolonomicAxisModels::strafe, "Strafe", result);
        });
    page.addAxis(
        "Turn", [channel = &turnLog] { return logged("Turn", turnExperiment(), *channel); },
        [](const AxisCharacterization& result) {
            installModel(&HolonomicAxisModels::turn, "Turn", result);
        });
    // The lift: its own Auto-Tune group, so selecting Lift and tapping
    // Auto-Tune runs only this (and selecting Drive, Turn or Hold only the
    // drivetrain). Held samples are logged as NaN volts.
    page.addMechanismAxis(
        "Lift",
        [channel = &liftLog] {
            sapphirelib::telemetry::event("tune", "start,Lift");
            return tapMechanismCharacterization(macros::liftExperiment(), *channel);
        },
        &installLiftModel);
    page.addController("Drive", drivetrain().drivePID(), &driveTuningTest, "Fwd", kDriveResponse);
    page.addController("Turn", drivetrain().turnPID(), &turnTuningTest, "Turn", kTurnResponse);
    page.addController("Hold", drivetrain().headingHoldPID(), nullptr, "Turn",
                       kHeadingHoldResponse);
    page.addController("Lift", macros::liftMechanism().pid(), &macros::liftTuningTest, "Lift",
                       kLiftResponse);
    // Driver stick mode - see DriverInputMode. Velocity needs Auto-Tune's
    // models; until an axis has one, that axis quietly keeps voltage behavior.
    page.setToggle(
        [] {
            if (drivetrain().driverInputMode() == DriverInputMode::voltage) {
                return std::string("Sticks: Voltage");
            }
            return std::string(drivetrain().axisModels().forward.valid()
                                   ? "Sticks: Velocity"
                                   : "Sticks: Velocity (no model)");
        },
        [] {
            drivetrain().setDriverInputMode(drivetrain().driverInputMode() == DriverInputMode::voltage
                                                ? DriverInputMode::velocity
                                                : DriverInputMode::voltage);
        });
}

} // namespace robot
