#include "sapphirelib/chassis/tank_drivetrain.hpp"

#include <cmath>

#include "pros/rtos.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kLoopDelayMs = 10;

/// Heading error to steer toward `targetBearingDeg`, and which direction to
/// drive: reverses instead of turning more than 90 degrees to face the
/// target, same as driveDistance() accepting negative inches. Shared by
/// moveToPoint() and moveToPose(), which only differ in how they compute
/// targetBearingDeg (straight at the point vs. at a boomerang carrot point).
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

} // namespace

TankDrivetrain::TankDrivetrain(std::initializer_list<std::int8_t> leftPorts,
                                std::initializer_list<std::int8_t> rightPorts, Gearset gearset,
                                std::uint8_t imuPort, DrivetrainConfig config,
                                PID::Config drivePIDConfig, PID::Config turnPIDConfig)
    : left_(leftPorts, gearset),
      right_(rightPorts, gearset),
      imu_(imuPort),
      config_(config),
      drivePID_(drivePIDConfig),
      turnPID_(turnPIDConfig) {}

void TankDrivetrain::tank(double left, double right) {
    left_.moveVoltage(left * 12.0);
    right_.moveVoltage(right * 12.0);
}

void TankDrivetrain::arcade(double throttle, double turn) {
    tank(throttle + turn, throttle - turn);
}

double TankDrivetrain::degreesToInches(double degrees) const {
    const double wheelCircumferenceIn = config_.wheelDiameterIn * kPi;
    return (degrees / 360.0 / config_.externalGearRatio) * wheelCircumferenceIn;
}

void TankDrivetrain::driveDistance(double inches, ExitConditions exit) {
    const double startHeading = imu_.get_heading();

    left_.tarePosition();
    right_.tarePosition();
    drivePID_.reset();

    std::uint32_t settledForMs = 0;
    std::uint32_t lastTick = pros::millis();
    const std::uint32_t start = lastTick;

    while (true) {
        const double traveledDegrees =
            (left_.getPositionDegrees() + right_.getPositionDegrees()) / 2.0;
        const double traveledInches = degreesToInches(traveledDegrees);
        const double error = inches - traveledInches;

        const double output = drivePID_.update(inches, traveledInches);

        double correction = 0.0;
        if (config_.headingCorrectionKP != 0.0) {
            const double headingError = wrapDegrees180(startHeading - imu_.get_heading());
            correction = config_.headingCorrectionKP * headingError;
        }

        left_.moveVoltage(output - correction);
        right_.moveVoltage(output + correction);

        const std::uint32_t now = pros::millis();
        if (std::fabs(error) <= exit.errorThreshold) {
            settledForMs += now - lastTick;
            if (settledForMs >= exit.settleTimeMs) break;
        } else {
            settledForMs = 0;
        }
        if (exit.timeoutMs > 0 && (now - start) >= exit.timeoutMs) break;

        lastTick = now;
        pros::delay(kLoopDelayMs);
    }

    stop();
}

void TankDrivetrain::turnToHeading(double headingDeg, ExitConditions exit) {
    turnPID_.reset();

    std::uint32_t settledForMs = 0;
    std::uint32_t lastTick = pros::millis();
    const std::uint32_t start = lastTick;

    while (true) {
        const double error = wrapDegrees180(headingDeg - imu_.get_heading());

        // Feed the pre-wrapped error in as `target` against a fixed
        // `measurement` of 0, since PID doesn't know heading wraps at 360.
        // turnPID's derivativeOnMeasurement should stay false for this to
        // behave as a normal derivative-on-error term.
        const double output = turnPID_.update(error, 0.0);

        left_.moveVoltage(output);
        right_.moveVoltage(-output);

        const std::uint32_t now = pros::millis();
        if (std::fabs(error) <= exit.errorThreshold) {
            settledForMs += now - lastTick;
            if (settledForMs >= exit.settleTimeMs) break;
        } else {
            settledForMs = 0;
        }
        if (exit.timeoutMs > 0 && (now - start) >= exit.timeoutMs) break;

        lastTick = now;
        pros::delay(kLoopDelayMs);
    }

    stop();
}

void TankDrivetrain::moveToPoint(double xIn, double yIn, const odom::Odometry& odometry, ExitConditions exit) {
    drivePID_.reset();
    turnPID_.reset();

    std::uint32_t settledForMs = 0;
    std::uint32_t lastTick = pros::millis();
    const std::uint32_t start = lastTick;

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double dxIn = xIn - pose.xIn;
        const double dyIn = yIn - pose.yIn;
        const double distanceIn = std::hypot(dxIn, dyIn);
        const double targetBearingDeg = std::atan2(dxIn, dyIn) / kPi * 180.0;

        const SteeringError steer = steerToward(targetBearingDeg, pose.headingDeg);
        const double forwardOutput = steer.direction * drivePID_.update(distanceIn, 0.0);
        const double turnOutput = turnPID_.update(steer.headingErrorDeg, 0.0);

        left_.moveVoltage(forwardOutput + turnOutput);
        right_.moveVoltage(forwardOutput - turnOutput);

        const std::uint32_t now = pros::millis();
        if (distanceIn <= exit.errorThreshold) {
            settledForMs += now - lastTick;
            if (settledForMs >= exit.settleTimeMs) break;
        } else {
            settledForMs = 0;
        }
        if (exit.timeoutMs > 0 && (now - start) >= exit.timeoutMs) break;

        lastTick = now;
        pros::delay(kLoopDelayMs);
    }

    stop();
}

void TankDrivetrain::moveToPose(double xIn, double yIn, double headingDeg, const odom::Odometry& odometry,
                                 motion::PoseExitConditions exit) {
    drivePID_.reset();
    turnPID_.reset();

    const double targetHeadingRad = headingDeg * kPi / 180.0;

    std::uint32_t settledForMs = 0;
    std::uint32_t lastTick = pros::millis();
    const std::uint32_t start = lastTick;

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double distanceToTargetIn = std::hypot(xIn - pose.xIn, yIn - pose.yIn);

        // Boomerang carrot point: placed behind the target along its facing
        // direction, receding toward the target itself as the chassis
        // closes in — so it naturally curves into the target heading
        // instead of driving straight in and point-turning at the end.
        const double carrotOffsetIn = distanceToTargetIn * exit.boomerangLeadPct;
        const double carrotXIn = xIn - carrotOffsetIn * std::sin(targetHeadingRad);
        const double carrotYIn = yIn - carrotOffsetIn * std::cos(targetHeadingRad);

        const double dxIn = carrotXIn - pose.xIn;
        const double dyIn = carrotYIn - pose.yIn;
        const double targetBearingDeg = std::atan2(dxIn, dyIn) / kPi * 180.0;

        const SteeringError steer = steerToward(targetBearingDeg, pose.headingDeg);
        const double forwardOutput = steer.direction * drivePID_.update(distanceToTargetIn, 0.0);
        const double turnOutput = turnPID_.update(steer.headingErrorDeg, 0.0);

        left_.moveVoltage(forwardOutput + turnOutput);
        right_.moveVoltage(forwardOutput - turnOutput);

        const double headingErrorToFinalDeg = std::fabs(wrapDegrees180(headingDeg - pose.headingDeg));
        const std::uint32_t now = pros::millis();
        if (distanceToTargetIn <= exit.positionErrorThresholdIn &&
            headingErrorToFinalDeg <= exit.headingErrorThresholdDeg) {
            settledForMs += now - lastTick;
            if (settledForMs >= exit.settleTimeMs) break;
        } else {
            settledForMs = 0;
        }
        if (exit.timeoutMs > 0 && (now - start) >= exit.timeoutMs) break;

        lastTick = now;
        pros::delay(kLoopDelayMs);
    }

    stop();
}

void TankDrivetrain::followPath(const motion::Path& path, const odom::Odometry& odometry,
                                 motion::PursuitConfig config) {
    turnPID_.reset();

    std::size_t segmentIndex = 0;
    const motion::Waypoint& finalPoint = path.waypoints().back();

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double distToFinalIn = std::hypot(finalPoint.xIn - pose.xIn, finalPoint.yIn - pose.yIn);
        if (distToFinalIn <= config.finalApproachIn) break;

        const motion::LookaheadResult lookahead =
            motion::findLookaheadPoint(pose.xIn, pose.yIn, path, config.lookaheadIn, segmentIndex);
        segmentIndex = lookahead.segmentIndex;

        const double dxIn = lookahead.point.xIn - pose.xIn;
        const double dyIn = lookahead.point.yIn - pose.yIn;
        const double targetBearingDeg = std::atan2(dxIn, dyIn) / kPi * 180.0;
        const double headingError = wrapDegrees180(targetBearingDeg - pose.headingDeg);
        const double turnOutput = turnPID_.update(headingError, 0.0);

        left_.moveVoltage(config.cruiseVoltage + turnOutput);
        right_.moveVoltage(config.cruiseVoltage - turnOutput);

        pros::delay(kLoopDelayMs);
    }

    moveToPoint(finalPoint.xIn, finalPoint.yIn, odometry, config.finalExit);
}

void TankDrivetrain::stop(BrakeMode mode) {
    left_.setBrakeMode(mode);
    right_.setBrakeMode(mode);
    left_.brake();
    right_.brake();
}

} // namespace sapphirelib::chassis
