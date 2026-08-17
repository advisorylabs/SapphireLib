#include "sapphirelib/chassis/tank_drivetrain.hpp"

#include <cmath>

#include "pros/rtos.hpp"
#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kLoopDelayMs = 10;

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

void TankDrivetrain::stop(BrakeMode mode) {
    left_.setBrakeMode(mode);
    right_.setBrakeMode(mode);
    left_.brake();
    right_.brake();
}

} // namespace sapphirelib::chassis
