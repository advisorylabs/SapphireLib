#include "sapphirelib/chassis/tank_drivetrain.hpp"

#include <cmath>

#include "pros/rtos.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/sensors/imu_scale_math.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/log.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kLoopDelayMs = 10;

constexpr const char* kNoOdometry = "no odometry - call setOdometry() first, or pass one";

// heading error to steer toward a bearing, and which way to drive. Reverses instead of turning
// more than 90 degrees, like driveDistance() with negative inches
struct SteeringError {
    double headingErrorDeg;
    double direction; // +1 forward, -1 reverse
};

SteeringError steerToward(double targetBearingDeg, double currentHeadingDeg) {
    double headingError = wrapDegrees180(targetBearingDeg - currentHeadingDeg);
    double direction = 1.0;
    if (std::fabs(headingError) > 90.0) {
        headingError = wrapDegrees180(headingError - 180.0);
        direction = -1.0;
    }
    return SteeringError{headingError, direction};
}

// log a motion's end event and return its result
motion::MotionResult finishMotion(const char* kind, const motion::MotionResult& result) {
    telemetry::event("motion", "end,%s,reason=%s,error=%.3f,ms=%u", kind,
                     motion::exitReasonName(result.reason), result.finalError,
                     static_cast<unsigned>(result.elapsedMs));
    return result;
}

// for a motion that can't start (no odometry, an empty path): print why, and still log a
// start/end pair so the log shows it was asked for
motion::MotionResult abortMotion(const char* kind, const char* why) {
    SAPPHIRELIB_LOG_ERROR("motion", "%s: %s", kind, why);
    telemetry::event("motion", "start,%s", kind);
    return finishMotion(kind, motion::MotionResult{.reason = motion::ExitReason::aborted});
}

} // namespace

TankDrivetrain::TankDrivetrain(std::initializer_list<std::int8_t> leftPorts,
                                std::initializer_list<std::int8_t> rightPorts, Gearset gearset,
                                std::uint8_t imuPort, DrivetrainConfig config,
                                PID::Config drivePIDConfig, PID::Config turnPIDConfig,
                                double imuHeadingScale)
    : left_(leftPorts, gearset),
      right_(rightPorts, gearset),
      imu_(imuPort, imuHeadingScale),
      config_(config),
      drivePID_(drivePIDConfig),
      turnPID_(turnPIDConfig) {}

sensors::Imu& TankDrivetrain::imu() { return imu_; }

PID& TankDrivetrain::drivePID() { return drivePID_; }

PID& TankDrivetrain::turnPID() { return turnPID_; }

void TankDrivetrain::setOdometry(const odom::Odometry* odometry) { odometry_ = odometry; }

AxisVolts TankDrivetrain::appliedAxisVolts() const {
    return AxisVolts{.forward = appliedForwardVolts_.load(),
                     .strafe = 0.0,
                     .turn = appliedTurnVolts_.load()};
}

void TankDrivetrain::setSideVoltages(double leftVolts, double rightVolts) {
    left_.moveVoltage(leftVolts);
    right_.moveVoltage(rightVolts);

    // every caller mixes left = forward + turn, right = forward - turn, so this gets back exactly
    // what was asked for
    appliedForwardVolts_.store((leftVolts + rightVolts) / 2.0);
    appliedTurnVolts_.store((leftVolts - rightVolts) / 2.0);
}

void TankDrivetrain::tank(double left, double right) { setSideVoltages(left * 12.0, right * 12.0); }

void TankDrivetrain::arcade(double throttle, double turn) {
    tank(throttle + turn, throttle - turn);
}

double TankDrivetrain::rotationHeadingDeg() {
    return sensors::wrapDegrees360(imu_.getCumulativeHeadingDeg());
}

double TankDrivetrain::degreesToInches(double degrees) const {
    const double wheelCircumferenceIn = config_.wheelDiameterIn * kPi;
    return (degrees / 360.0 / config_.externalGearRatio) * wheelCircumferenceIn;
}

double TankDrivetrain::sideAverageDegrees() const {
    return (left_.getPositionDegrees() + right_.getPositionDegrees()) / 2.0;
}

motion::MotionResult TankDrivetrain::driveDistance(double inches, ExitConditions exit) {
    telemetry::event("motion",
                     "start,driveDistance,target_in=%.3f,threshold=%.3f,settle_ms=%u,timeout_ms=%u",
                     inches, exit.errorThreshold, static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));

    // rotation frame, so a setPose() from another task doesn't look like a heading error
    const double startHeading = rotationHeadingDeg();

    // measure from where the encoders read now instead of taring them. A tare would also zero
    // any MotorGroupTrackingWheel on these motors and make odometry jump
    const double startDegrees = sideAverageDegrees();
    drivePID_.reset();

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorIn = 0.0;

    while (true) {
        const double traveledInches = degreesToInches(sideAverageDegrees() - startDegrees);
        const double error = inches - traveledInches;

        const double output = drivePID_.update(inches, traveledInches);

        double correction = 0.0;
        if (config_.headingCorrectionKP != 0.0) {
            const double headingError = wrapDegrees180(startHeading - rotationHeadingDeg());
            correction = config_.headingCorrectionKP * headingError;
        }

        setSideVoltages(output - correction, output + correction);

        finalErrorIn = std::fabs(error);
        reason = tracker.update(std::fabs(error) <= exit.errorThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return finishMotion("driveDistance", motion::MotionResult{.reason = reason,
                                                              .finalError = finalErrorIn,
                                                              .elapsedMs = tracker.elapsedMs()});
}

motion::MotionResult TankDrivetrain::turnToHeading(double headingDeg, ExitConditions exit) {
    telemetry::event("motion",
                     "start,turnToHeading,target_deg=%.3f,threshold=%.3f,settle_ms=%u,"
                     "timeout_ms=%u",
                     headingDeg, exit.errorThreshold, static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));

    turnPID_.reset();

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorDeg = 0.0;

    while (true) {
        const double error = wrapDegrees180(headingDeg - imu_.getHeadingDeg());

        // the PID doesn't know heading wraps at 360, so pass the wrapped error as the target and 0
        // as the measurement. derivativeOnMeasurement must stay false for this
        const double output = turnPID_.update(error, 0.0);

        setSideVoltages(output, -output);

        finalErrorDeg = std::fabs(error);
        reason = tracker.update(std::fabs(error) <= exit.errorThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return finishMotion("turnToHeading", motion::MotionResult{.reason = reason,
                                                              .finalError = finalErrorDeg,
                                                              .elapsedMs = tracker.elapsedMs()});
}

motion::MotionResult TankDrivetrain::moveToPoint(double xIn, double yIn,
                                                 const odom::Odometry& odometry,
                                                 ExitConditions exit) {
    // no hold_deg, unlike the holonomic drivetrain: a tank drive steers to face the point
    telemetry::event("motion",
                     "start,moveToPoint,x=%.3f,y=%.3f,threshold=%.3f,settle_ms=%u,timeout_ms=%u",
                     xIn, yIn, exit.errorThreshold, static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));
    return finishMotion("moveToPoint", approachPoint(xIn, yIn, odometry, exit));
}

motion::MotionResult TankDrivetrain::moveToPoint(double xIn, double yIn, ExitConditions exit) {
    if (odometry_ == nullptr) return abortMotion("moveToPoint", kNoOdometry);
    return moveToPoint(xIn, yIn, *odometry_, exit);
}

motion::MotionResult TankDrivetrain::approachPoint(double xIn, double yIn,
                                                   const odom::Odometry& odometry,
                                                   ExitConditions exit) {
    drivePID_.reset();
    turnPID_.reset();

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorIn = 0.0;

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double dxIn = xIn - pose.xIn;
        const double dyIn = yIn - pose.yIn;
        const double distanceIn = std::hypot(dxIn, dyIn);
        const double targetBearingDeg = std::atan2(dxIn, dyIn) / kPi * 180.0;

        const SteeringError steer = steerToward(targetBearingDeg, pose.headingDeg);
        const double forwardOutput = steer.direction * drivePID_.update(distanceIn, 0.0);
        const double turnOutput = turnPID_.update(steer.headingErrorDeg, 0.0);

        setSideVoltages(forwardOutput + turnOutput, forwardOutput - turnOutput);

        finalErrorIn = distanceIn;
        reason = tracker.update(distanceIn <= exit.errorThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return motion::MotionResult{
        .reason = reason, .finalError = finalErrorIn, .elapsedMs = tracker.elapsedMs()};
}

motion::MotionResult TankDrivetrain::moveToPose(double xIn, double yIn, double headingDeg,
                                                const odom::Odometry& odometry,
                                                motion::PoseExitConditions exit) {
    telemetry::event("motion",
                     "start,moveToPose,x=%.3f,y=%.3f,heading_deg=%.3f,pos_threshold=%.3f,"
                     "heading_threshold=%.3f,settle_ms=%u,timeout_ms=%u",
                     xIn, yIn, headingDeg, exit.positionErrorThresholdIn,
                     exit.headingErrorThresholdDeg, static_cast<unsigned>(exit.settleTimeMs),
                     static_cast<unsigned>(exit.timeoutMs));

    drivePID_.reset();
    turnPID_.reset();

    const double targetHeadingRad = headingDeg * kPi / 180.0;

    motion::ExitTracker tracker(exit.settleTimeMs, exit.timeoutMs, pros::millis());
    motion::ExitReason reason = motion::ExitReason::running;
    double finalErrorIn = 0.0;

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double distanceToTargetIn = std::hypot(xIn - pose.xIn, yIn - pose.yIn);

        // boomerang carrot point: behind the target along its heading, moving toward the target as
        // the chassis closes in, so it curves into the final heading
        const double carrotOffsetIn = distanceToTargetIn * exit.boomerangLeadPct;
        const double carrotXIn = xIn - carrotOffsetIn * std::sin(targetHeadingRad);
        const double carrotYIn = yIn - carrotOffsetIn * std::cos(targetHeadingRad);

        const double dxIn = carrotXIn - pose.xIn;
        const double dyIn = carrotYIn - pose.yIn;
        const double targetBearingDeg = std::atan2(dxIn, dyIn) / kPi * 180.0;

        const SteeringError steer = steerToward(targetBearingDeg, pose.headingDeg);
        const double forwardOutput = steer.direction * drivePID_.update(distanceToTargetIn, 0.0);
        const double turnOutput = turnPID_.update(steer.headingErrorDeg, 0.0);

        setSideVoltages(forwardOutput + turnOutput, forwardOutput - turnOutput);

        const double headingErrorToFinalDeg = std::fabs(wrapDegrees180(headingDeg - pose.headingDeg));
        const bool withinThreshold = distanceToTargetIn <= exit.positionErrorThresholdIn &&
                                     headingErrorToFinalDeg <= exit.headingErrorThresholdDeg;
        finalErrorIn = distanceToTargetIn;
        reason = tracker.update(withinThreshold, pros::millis());
        if (reason != motion::ExitReason::running) break;

        pros::delay(kLoopDelayMs);
    }

    stop();
    return finishMotion("moveToPose", motion::MotionResult{.reason = reason,
                                                           .finalError = finalErrorIn,
                                                           .elapsedMs = tracker.elapsedMs()});
}

motion::MotionResult TankDrivetrain::moveToPose(double xIn, double yIn, double headingDeg,
                                                motion::PoseExitConditions exit) {
    if (odometry_ == nullptr) return abortMotion("moveToPose", kNoOdometry);
    return moveToPose(xIn, yIn, headingDeg, *odometry_, exit);
}

motion::MotionResult TankDrivetrain::followPath(const motion::Path& path,
                                                const odom::Odometry& odometry,
                                                motion::PursuitConfig config) {
    // Path doesn't reject an empty list, and back() below would be undefined on one
    if (path.waypoints().empty()) return abortMotion("followPath", "empty path");

    telemetry::event("motion",
                     "start,followPath,waypoints=%u,lookahead_in=%.3f,cruise_v=%.3f,timeout_ms=%u",
                     static_cast<unsigned>(path.waypoints().size()), config.lookaheadIn,
                     config.cruiseVoltage, static_cast<unsigned>(config.timeoutMs));

    turnPID_.reset();

    std::size_t segmentIndex = 0;
    const motion::Waypoint& finalPoint = path.waypoints().back();
    const std::uint32_t startMs = pros::millis();

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double distToFinalIn = std::hypot(finalPoint.xIn - pose.xIn, finalPoint.yIn - pose.yIn);
        if (distToFinalIn <= config.finalApproachIn) break;

        // check the timeout after the distance, so reaching the final approach on the same tick
        // still gets the settled stop
        const std::uint32_t pursuitMs = pros::millis() - startMs;
        if (config.timeoutMs > 0 && pursuitMs >= config.timeoutMs) {
            stop();
            return finishMotion("followPath",
                                motion::MotionResult{.reason = motion::ExitReason::timedOut,
                                                     .finalError = distToFinalIn,
                                                     .elapsedMs = pursuitMs});
        }

        const motion::LookaheadResult lookahead =
            motion::findLookaheadPoint(pose.xIn, pose.yIn, path, config.lookaheadIn, segmentIndex);
        segmentIndex = lookahead.segmentIndex;

        const double dxIn = lookahead.point.xIn - pose.xIn;
        const double dyIn = lookahead.point.yIn - pose.yIn;
        const double targetBearingDeg = std::atan2(dxIn, dyIn) / kPi * 180.0;
        const double headingError = wrapDegrees180(targetBearingDeg - pose.headingDeg);
        const double turnOutput = turnPID_.update(headingError, 0.0);

        setSideVoltages(config.cruiseVoltage + turnOutput, config.cruiseVoltage - turnOutput);

        pros::delay(kLoopDelayMs);
    }

    motion::MotionResult result =
        approachPoint(finalPoint.xIn, finalPoint.yIn, odometry, config.finalExit);
    // the final approach decides how the path ended, but the time covers the whole path
    result.elapsedMs = pros::millis() - startMs;
    return finishMotion("followPath", result);
}

motion::MotionResult TankDrivetrain::followPath(const motion::Path& path,
                                                motion::PursuitConfig config) {
    if (odometry_ == nullptr) return abortMotion("followPath", kNoOdometry);
    return followPath(path, *odometry_, config);
}

void TankDrivetrain::stop(BrakeMode mode) {
    left_.setBrakeMode(mode);
    right_.setBrakeMode(mode);
    left_.brake();
    right_.brake();

    // braking commands no voltage, so stop reporting the last motion's volts
    appliedForwardVolts_.store(0.0);
    appliedTurnVolts_.store(0.0);
}

} // namespace sapphirelib::chassis
