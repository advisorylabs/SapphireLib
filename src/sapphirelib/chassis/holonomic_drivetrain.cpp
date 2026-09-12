#include "sapphirelib/chassis/holonomic_drivetrain.hpp"

#include <algorithm>
#include <cmath>

#include "pros/rtos.hpp"
#include "sapphirelib/chassis/thermal_math.hpp"
#include "sapphirelib/motion/pure_pursuit_math.hpp"
#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::chassis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kLoopDelayMs = 10;

/// Full motor voltage. holonomic() takes normalized [-1, 1] axes while the
/// PIDs are configured in volts, so converting between them means dividing
/// by this — same as moveToPose() already does with its turn output.
constexpr double kFullScaleVolts = 12.0;

/// A gap longer than this between heading-hold calls means the driver loop
/// wasn't running — an autonomous routine, a PID tuner test, a calibration
/// spin — and whatever heading was being held belongs to before that, not
/// now. Generous next to a 10-20ms driver loop, short next to any of those
/// interruptions.
constexpr double kHeadingHoldResumeGapS = 0.25;

/// How often motor temperatures are actually re-read. A V5 motor takes tens
/// of seconds to move a degree under load, so polling this near the 100Hz
/// rate setWheelVoltages() runs at would be six device reads per tick for a
/// number that hasn't changed.
constexpr std::uint32_t kThermalPollIntervalMs = 500;

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
                                          PID::Config turnPIDConfig, double imuHeadingScale,
                                          std::optional<AsteriskConfig> asterisk)
    : frontLeft_({frontLeftPort}, gearset),
      frontRight_({frontRightPort}, gearset),
      backLeft_({backLeftPort}, gearset),
      backRight_({backRightPort}, gearset),
      imu_(imuPort, imuHeadingScale),
      config_(config),
      drivePID_(drivePIDConfig),
      turnPID_(turnPIDConfig),
      asterisk_(asterisk) {
    if (asterisk_) {
        const std::initializer_list<std::int8_t> middleLeftPorts{asterisk_->middleLeftPort};
        const std::initializer_list<std::int8_t> middleRightPorts{asterisk_->middleRightPort};
        middleLeft_.emplace(middleLeftPorts, gearset);
        middleRight_.emplace(middleRightPorts, gearset);

        // Center wheels coast whenever they're not actively driving,
        // turning, or correcting drift — see setWheelVoltages().
        middleLeft_->setBrakeMode(BrakeMode::coast);
        middleRight_->setBrakeMode(BrakeMode::coast);
    }

    // sensors::Imu's constructor already blocks until IMU calibration
    // finishes, so getHeadingDeg() (used here and by
    // driveDistance()/turnToHeading()/headingDeg()) is valid as soon as
    // this constructor returns.
    fieldHeadingZeroDeg_ = imu_.getHeadingDeg();
}

sensors::Imu& HolonomicDrivetrain::imu() { return imu_; }

PID& HolonomicDrivetrain::drivePID() { return drivePID_; }

PID& HolonomicDrivetrain::turnPID() { return turnPID_; }

void HolonomicDrivetrain::setDriftSource(const odom::TrackingWheel* verticalWheel,
                                         const odom::Odometry* odometry) {
    driftSource_ = verticalWheel;
    driftOffsetSource_ = odometry;
    if (driftSource_ != nullptr) {
        lastDriftVerticalIn_ = driftSource_->getDistanceIn();
        lastDriftStrafeIn_ = encoderStrafeIn();
        lastDriftHeadingDeg_ = imu_.getCumulativeHeadingDeg();
        lastDriftTickMs_ = pros::millis();
    }
}

double HolonomicDrivetrain::encoderStrafeIn() const {
    return degreesToInches((frontLeft_.getPositionDegrees() - frontRight_.getPositionDegrees() -
                            backLeft_.getPositionDegrees() + backRight_.getPositionDegrees()) /
                           4.0);
}

void HolonomicDrivetrain::refreshThermalFractions() {
    // Only reached with the Asterisk wheels configured (setWheelVoltages()
    // returns before this otherwise). Bailing here when the feature is off
    // keeps six device reads per poll off a chassis that isn't using it —
    // the fractions stay at their 1.0 defaults, which produce no correction.
    if (!asterisk_ || asterisk_->thermalCompensation <= 0.0) return;

    const std::uint32_t now = pros::millis();
    if (lastThermalPollMs_ != 0 && now - lastThermalPollMs_ < kThermalPollIntervalMs) return;
    lastThermalPollMs_ = now;

    thermalFractions_ = CornerValues{
        thermalPowerFraction(frontLeft_.getTemperatureC()),
        thermalPowerFraction(frontRight_.getTemperatureC()),
        thermalPowerFraction(backLeft_.getTemperatureC()),
        thermalPowerFraction(backRight_.getTemperatureC()),
    };

    // Averaged, unlike the corners: this one only ever scales the whole
    // correction down, so which of the two center wheels is hotter doesn't
    // change what the correction should be, only how much of it to ask for.
    centerThermalFraction_ = (thermalPowerFraction(middleLeft_->getTemperatureC()) +
                              thermalPowerFraction(middleRight_->getTemperatureC())) /
                             2.0;
}

void HolonomicDrivetrain::setWheelVoltages(double frontLeft, double frontRight, double backLeft,
                                            double backRight) {
    frontLeft_.moveVoltage(frontLeft);
    frontRight_.moveVoltage(frontRight);
    backLeft_.moveVoltage(backLeft);
    backRight_.moveVoltage(backRight);

    if (!asterisk_) return;

    // Recover the pure throttle/strafe/turn components from the four
    // already-mixed corner voltages (see mixHolonomic()): summing all four
    // cancels strafe and turn, leaving 4x throttle; each cross-combination
    // below cancels two of the three, leaving 4x the remaining one. This
    // lets the center wheels react correctly no matter which call site
    // produced the mix — holonomic() driver input, driveDistance()'s
    // heading-corrected drive, or turnToHeading()'s turn-only mix.
    const double throttleVolts = (frontLeft + frontRight + backLeft + backRight) / 4.0;
    const double strafeVolts = (frontLeft - frontRight - backLeft + backRight) / 4.0;
    const double turnVolts = (frontLeft - frontRight + backLeft - backRight) / 4.0;

    // Feedforward from what the corners are failing to deliver. Computed
    // from this tick's corner voltages rather than cached, because the same
    // derating means different things depending on what's being driven —
    // see refreshThermalFractions(). Purely additive: with every corner cool
    // this is zero and the terms below are untouched.
    refreshThermalFractions();
    const CenterCorrection thermal = centerThermalCorrection(
        CornerValues{frontLeft, frontRight, backLeft, backRight}, thermalFractions_,
        centerThermalFraction_, asterisk_->thermalCompensation,
        asterisk_->maxThermalCorrectionVolts);

    double centerVolts = throttleVolts + thermal.commonVolts;

    if (driftSource_ != nullptr) {
        const std::uint32_t now = pros::millis();
        const double verticalIn = driftSource_->getDistanceIn();
        const double strafeIn = encoderStrafeIn();
        const double headingDeg = imu_.getCumulativeHeadingDeg();
        const double dtS = (now - lastDriftTickMs_) / 1000.0;

        // Only correct while strafing dominates — during forward/back
        // driving the center wheels are just adding power. Skip a zero dt
        // and unusually long gaps (e.g. a paused motion) rather than treat
        // them as a drift spike. The baseline below refreshes on every call
        // either way, so a stale gap never carries into the next strafe.
        const bool strafingDominant = std::fabs(strafeVolts) > std::fabs(throttleVolts);
        if (asterisk_->driftCorrectionKP != 0.0 && strafingDominant && dtS > 1e-3 && dtS < 1.0) {
            const double verticalOffsetIn = driftOffsetSource_ != nullptr
                                                ? driftOffsetSource_->getConfig().verticalOffsetIn
                                                : 0.0;
            const double driftIn =
                strafeDriftIn(verticalIn - lastDriftVerticalIn_, verticalOffsetIn,
                              headingDeg - lastDriftHeadingDeg_, strafeIn - lastDriftStrafeIn_,
                              throttleVolts, strafeVolts);
            centerVolts -= asterisk_->driftCorrectionKP * (driftIn / dtS);
        }

        lastDriftVerticalIn_ = verticalIn;
        lastDriftStrafeIn_ = strafeIn;
        lastDriftHeadingDeg_ = headingDeg;
        lastDriftTickMs_ = now;
    }

    // Differential turn term, on top of the common forward/back term. The
    // left side takes +turn in mixHolonomic(), and the middle ports use the
    // same sign convention as the corner ports, so middle-left matching
    // front-left's sign is what makes these push the turn rather than fight
    // it. Without this the center wheels saw a zero net command during a
    // pure turn and just coasted through it; they now carry their share.
    //
    // This also means driveDistance()'s heading correction — which reaches
    // here as a small turn component riding on top of the drive output —
    // gets the center wheels helping hold the line, not just watching.
    //
    // The thermal term rides along here for the same reason it does on the
    // throttle: a lopsided set of corner temperatures twists the chassis as
    // well as pushing it off line, and cancelling that twist is differential
    // work.
    const double centerTurnVolts =
        turnVolts * asterisk_->turnContribution + thermal.differentialVolts;

    // Clamped per side after summing, since it's the sum that has to fit in
    // a motor's range. holonomic() normalizes its mix so neither side can
    // reach the limit from driver input; this bounds the autonomous paths,
    // whose raw PID volts have always been free to run past it (the corner
    // wheels clamp the same way, inside moveVoltage()).
    const double leftVolts = std::clamp(centerVolts + centerTurnVolts, -12.0, 12.0);
    const double rightVolts = std::clamp(centerVolts - centerTurnVolts, -12.0, 12.0);
    middleLeft_->moveVoltage(leftVolts);
    middleRight_->moveVoltage(rightVolts);
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

double HolonomicDrivetrain::headingHoldTurnVolts(double turnInput) {
    const std::uint32_t now = pros::millis();
    const double currentHeadingDeg = imu_.getHeadingDeg();
    const double dtS = (now - lastHeadingHoldMs_) / 1000.0;
    const bool resuming =
        lastHeadingHoldMs_ == 0 || !(dtS > 0.0) || dtS > kHeadingHoldResumeGapS;
    lastHeadingHoldMs_ = now;

    if (resuming) {
        // Adopt wherever the chassis is pointing rather than steering it
        // back to a heading from before whatever just interrupted driver
        // control. The PID is reset for the same reason: its accumulated
        // integral and last error describe a situation that no longer
        // exists.
        heldHeadingDeg_ = currentHeadingDeg;
        turnPID_.reset();
        return 0.0;
    }

    heldHeadingDeg_ =
        advanceHeldHeadingDeg(heldHeadingDeg_, currentHeadingDeg, turnInput, dtS, headingHold_);

    // Same target/measurement=0 trick as turnToHeading() — see that
    // function's comment. Explicit dt because a driver loop's period is the
    // caller's business and needn't match PID::Config::nominalDtS.
    return turnPID_.update(wrapDegrees180(heldHeadingDeg_ - currentHeadingDeg), 0.0, dtS);
}

void HolonomicDrivetrain::holonomicHeadingHold(double throttle, double strafe, double turnInput) {
    holonomic(throttle, strafe, headingHoldTurnVolts(turnInput) / kFullScaleVolts);
}

void HolonomicDrivetrain::holonomicFieldCentricHeadingHold(double throttle, double strafe,
                                                            double turnInput) {
    holonomicFieldCentric(throttle, strafe, headingHoldTurnVolts(turnInput) / kFullScaleVolts);
}

void HolonomicDrivetrain::setHeadingHold(HeadingHoldConfig config) { headingHold_ = config; }

double HolonomicDrivetrain::heldHeadingDeg() const { return heldHeadingDeg_; }

void HolonomicDrivetrain::holonomicFieldCentric(double throttle, double strafe, double turn) {
    // Rotate the field-relative stick vector into the robot's current frame
    // by the heading it has picked up since the last field-heading zero —
    // matches pros::Imu::get_heading()'s clockwise-positive convention.
    const double headingDeltaRad =
        wrapDegrees180(imu_.getHeadingDeg() - fieldHeadingZeroDeg_) * (kPi / 180.0);
    const double cosHeading = std::cos(headingDeltaRad);
    const double sinHeading = std::sin(headingDeltaRad);
    const double robotThrottle = throttle * cosHeading + strafe * sinHeading;
    const double robotStrafe = -throttle * sinHeading + strafe * cosHeading;

    holonomic(robotThrottle, robotStrafe, turn);
}

void HolonomicDrivetrain::resetFieldHeading() { fieldHeadingZeroDeg_ = imu_.getHeadingDeg(); }

void HolonomicDrivetrain::moveToPoint(double xIn, double yIn, const odom::Odometry& odometry,
                                       ExitConditions exit) {
    moveToPointHolding(xIn, yIn, odometry.getPose().headingDeg, odometry, exit);
}

void HolonomicDrivetrain::moveToPointHolding(double xIn, double yIn, double holdHeadingDeg,
                                              const odom::Odometry& odometry, ExitConditions exit) {
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

        const double outputVolts = drivePID_.update(distanceIn, 0.0);
        const motion::LocalOffset local = motion::toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const double scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;

        // Hold heading the same way moveToPose() does. Translation would
        // still arrive if the chassis yawed (toLocalFrame() uses the live
        // heading); this just keeps it from yawing on the way.
        const double turnOutput =
            turnPID_.update(wrapDegrees180(holdHeadingDeg - pose.headingDeg), 0.0);

        holonomic(local.forwardIn * scale / 12.0, local.lateralIn * scale / 12.0, turnOutput / 12.0);

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

void HolonomicDrivetrain::moveToPose(double xIn, double yIn, double headingDeg,
                                      const odom::Odometry& odometry, motion::PoseExitConditions exit) {
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

        const double outputVolts = drivePID_.update(distanceIn, 0.0);
        const motion::LocalOffset local = motion::toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const double scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;

        const double headingError = wrapDegrees180(headingDeg - pose.headingDeg);
        const double turnOutput = turnPID_.update(headingError, 0.0);

        holonomic(local.forwardIn * scale / 12.0, local.lateralIn * scale / 12.0, turnOutput / 12.0);

        const std::uint32_t now = pros::millis();
        if (distanceIn <= exit.positionErrorThresholdIn &&
            std::fabs(headingError) <= exit.headingErrorThresholdDeg) {
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

void HolonomicDrivetrain::followPath(const motion::Path& path, const odom::Odometry& odometry,
                                      motion::PursuitConfig config) {
    std::size_t segmentIndex = 0;
    const motion::Waypoint& finalPoint = path.waypoints().back();
    const double holdHeadingDeg = odometry.getPose().headingDeg;
    turnPID_.reset();

    while (true) {
        const odom::Pose pose = odometry.getPose();
        const double distToFinalIn = std::hypot(finalPoint.xIn - pose.xIn, finalPoint.yIn - pose.yIn);
        if (distToFinalIn <= config.finalApproachIn) break;

        const motion::LookaheadResult lookahead =
            motion::findLookaheadPoint(pose.xIn, pose.yIn, path, config.lookaheadIn, segmentIndex);
        segmentIndex = lookahead.segmentIndex;

        const double dxIn = lookahead.point.xIn - pose.xIn;
        const double dyIn = lookahead.point.yIn - pose.yIn;
        const double distanceIn = std::hypot(dxIn, dyIn);
        const motion::LocalOffset local = motion::toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const double scale = distanceIn > 1e-6 ? config.cruiseVoltage / distanceIn : 0.0;
        const double turnOutput =
            turnPID_.update(wrapDegrees180(holdHeadingDeg - pose.headingDeg), 0.0);

        holonomic(local.forwardIn * scale / 12.0, local.lateralIn * scale / 12.0, turnOutput / 12.0);

        pros::delay(kLoopDelayMs);
    }

    // Keep holding the path's starting heading through the final approach,
    // rather than re-capturing whatever the chassis yawed to on the way.
    moveToPointHolding(finalPoint.xIn, finalPoint.yIn, holdHeadingDeg, odometry, config.finalExit);
}

double HolonomicDrivetrain::degreesToInches(double degrees) const {
    const double wheelCircumferenceIn = config_.wheelDiameterIn * kPi;
    return (degrees / 360.0 / config_.externalGearRatio) * wheelCircumferenceIn;
}

void HolonomicDrivetrain::driveDistance(double inches, ExitConditions exit) {
    const double startHeading = imu_.getHeadingDeg();

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
            const double headingError = wrapDegrees180(startHeading - imu_.getHeadingDeg());
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
        const double error = wrapDegrees180(headingDeg - imu_.getHeadingDeg());

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

double HolonomicDrivetrain::headingDeg() { return imu_.getHeadingDeg(); }

void HolonomicDrivetrain::stop(BrakeMode mode) {
    frontLeft_.setBrakeMode(mode);
    frontRight_.setBrakeMode(mode);
    backLeft_.setBrakeMode(mode);
    backRight_.setBrakeMode(mode);
    frontLeft_.brake();
    frontRight_.brake();
    backLeft_.brake();
    backRight_.brake();

    // Center wheels keep their own permanent coast brake mode (set at
    // construction) regardless of `mode` — see AsteriskConfig.
    if (asterisk_) {
        middleLeft_->brake();
        middleRight_->brake();
    }
}

} // namespace sapphirelib::chassis
