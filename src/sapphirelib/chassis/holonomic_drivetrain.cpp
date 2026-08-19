#include "sapphirelib/chassis/holonomic_drivetrain.hpp"

#include <algorithm>
#include <cmath>

#include "pros/rtos.hpp"
#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kLoopDelayMs = 10;

struct WheelMix {
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
};

/// Standard mecanum/X-drive mixing equations. `throttle`/`strafe`/`turn`
/// share units (either normalized [-1, 1] driver input, or PID output volts
/// — the caller decides); the result is in those same units, unclamped.
WheelMix mixHolonomic(double throttle, double strafe, double turn) {
    return {throttle + strafe + turn, throttle - strafe - turn, throttle - strafe + turn,
            throttle + strafe - turn};
}

} // namespace

HolonomicDrivetrain::HolonomicDrivetrain(std::int8_t frontLeftPort, std::int8_t frontRightPort,
                                          std::int8_t backLeftPort, std::int8_t backRightPort,
                                          Gearset gearset, std::uint8_t imuPort,
                                          DrivetrainConfig config, PID::Config drivePIDConfig,
                                          PID::Config turnPIDConfig)
    : frontLeft_({frontLeftPort}, gearset),
      frontRight_({frontRightPort}, gearset),
      backLeft_({backLeftPort}, gearset),
      backRight_({backRightPort}, gearset),
      imu_(imuPort),
      config_(config),
      drivePID_(drivePIDConfig),
      turnPID_(turnPIDConfig) {
    // Block until calibration finishes so get_heading() (used here and by
    // driveDistance()/turnToHeading()/headingDeg()) is valid as soon as the
    // constructor returns, instead of reading 0 until calibration happens to
    // finish on its own.
    imu_.reset(true);
    fieldHeadingZeroDeg_ = imu_.get_heading();
}

void HolonomicDrivetrain::setWheelVoltages(double frontLeft, double frontRight, double backLeft,
                                            double backRight) {
    frontLeft_.moveVoltage(frontLeft);
    frontRight_.moveVoltage(frontRight);
    backLeft_.moveVoltage(backLeft);
    backRight_.moveVoltage(backRight);
}

void HolonomicDrivetrain::holonomic(double throttle, double strafe, double turn) {
    const WheelMix mix = mixHolonomic(throttle, strafe, turn);

    // Scale the whole mix down (never up) so the largest wheel command never
    // exceeds a normalized magnitude of 1 — preserves the requested
    // direction instead of clipping one wheel and distorting it.
    const double largest = std::max({std::fabs(mix.frontLeft), std::fabs(mix.frontRight),
                                      std::fabs(mix.backLeft), std::fabs(mix.backRight), 1.0});

    setWheelVoltages(mix.frontLeft / largest * 12.0, mix.frontRight / largest * 12.0,
                      mix.backLeft / largest * 12.0, mix.backRight / largest * 12.0);
}

void HolonomicDrivetrain::holonomicFieldCentric(double throttle, double strafe, double turn) {
    // Rotate the field-relative stick vector into the robot's current frame
    // by the heading it has picked up since the last field-heading zero —
    // matches pros::Imu::get_heading()'s clockwise-positive convention.
    const double headingDeltaRad =
        wrapDegrees180(imu_.get_heading() - fieldHeadingZeroDeg_) * (kPi / 180.0);
    const double cosHeading = std::cos(headingDeltaRad);
    const double sinHeading = std::sin(headingDeltaRad);
    const double robotThrottle = throttle * cosHeading + strafe * sinHeading;
    const double robotStrafe = -throttle * sinHeading + strafe * cosHeading;

    holonomic(robotThrottle, robotStrafe, turn);
}

void HolonomicDrivetrain::resetFieldHeading() { fieldHeadingZeroDeg_ = imu_.get_heading(); }

double HolonomicDrivetrain::degreesToInches(double degrees) const {
    const double wheelCircumferenceIn = config_.wheelDiameterIn * kPi;
    return (degrees / 360.0 / config_.externalGearRatio) * wheelCircumferenceIn;
}

void HolonomicDrivetrain::driveDistance(double inches, ExitConditions exit) {
    const double startHeading = imu_.get_heading();

    frontLeft_.tarePosition();
    frontRight_.tarePosition();
    backLeft_.tarePosition();
    backRight_.tarePosition();
    drivePID_.reset();

    std::uint32_t settledForMs = 0;
    std::uint32_t lastTick = pros::millis();
    const std::uint32_t start = lastTick;

    while (true) {
        const double traveledDegrees =
            (frontLeft_.getPositionDegrees() + frontRight_.getPositionDegrees() +
             backLeft_.getPositionDegrees() + backRight_.getPositionDegrees()) /
            4.0;
        const double traveledInches = degreesToInches(traveledDegrees);
        const double error = inches - traveledInches;

        const double output = drivePID_.update(inches, traveledInches);

        double correction = 0.0;
        if (config_.headingCorrectionKP != 0.0) {
            const double headingError = wrapDegrees180(startHeading - imu_.get_heading());
            correction = config_.headingCorrectionKP * headingError;
        }

        const WheelMix mix = mixHolonomic(output, /*strafe=*/0.0, correction);
        setWheelVoltages(mix.frontLeft, mix.frontRight, mix.backLeft, mix.backRight);

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

void HolonomicDrivetrain::turnToHeading(double headingDeg, ExitConditions exit) {
    turnPID_.reset();

    std::uint32_t settledForMs = 0;
    std::uint32_t lastTick = pros::millis();
    const std::uint32_t start = lastTick;

    while (true) {
        const double error = wrapDegrees180(headingDeg - imu_.get_heading());

        // Same target/measurement=0 trick as TankDrivetrain::turnToHeading —
        // see that function's comment for why.
        const double output = turnPID_.update(error, 0.0);

        const WheelMix mix = mixHolonomic(/*throttle=*/0.0, /*strafe=*/0.0, output);
        setWheelVoltages(mix.frontLeft, mix.frontRight, mix.backLeft, mix.backRight);

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

double HolonomicDrivetrain::headingDeg() const { return imu_.get_heading(); }

void HolonomicDrivetrain::stop(BrakeMode mode) {
    frontLeft_.setBrakeMode(mode);
    frontRight_.setBrakeMode(mode);
    backLeft_.setBrakeMode(mode);
    backRight_.setBrakeMode(mode);
    frontLeft_.brake();
    frontRight_.brake();
    backLeft_.brake();
    backRight_.brake();
}

} // namespace sapphirelib::chassis
