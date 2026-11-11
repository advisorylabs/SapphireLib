#include "sapphirelib/motion/target_approach.hpp"

#include <algorithm>
#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// the pinhole's focal length, in pixels: the image's half width over the tangent of half its view
double focalPx(const CameraMount& camera) {
    const double halfFovRad = std::clamp(camera.horizontalFovDeg, 1.0, 179.0) * 0.5 * kDegToRad;
    return camera.imageWidthPx * 0.5 / std::tan(halfFovRad);
}

// where the lens is on the field, and which way it looks, in degrees clockwise from +y
struct Lens {
    double xIn;
    double yIn;
    double axisDeg;
};

Lens lensOf(const CameraMount& camera, const odom::Pose& robot) {
    const double h = robot.headingDeg * kDegToRad;
    // forward is (sin h, cos h) and right is (cos h, -sin h): heading 0 faces +y, 90 faces +x
    return Lens{robot.xIn + camera.forwardIn * std::sin(h) + camera.rightIn * std::cos(h),
                robot.yIn + camera.forwardIn * std::cos(h) - camera.rightIn * std::sin(h),
                robot.headingDeg + camera.yawDeg};
}

} // namespace

double cameraBearingDeg(const CameraMount& camera, double columnPx) {
    return std::atan((columnPx - camera.imageWidthPx * 0.5) / focalPx(camera)) * kRadToDeg;
}

double cameraFocalPx(const CameraMount& camera) { return focalPx(camera); }

std::optional<double> cameraDepthIn(const CameraMount& camera, const odom::Pose& robot,
                                    FieldPoint point) {
    const Lens lens = lensOf(camera, robot);
    const double a = lens.axisDeg * kDegToRad;
    const double along =
        (point.xIn - lens.xIn) * std::sin(a) + (point.yIn - lens.yIn) * std::cos(a);
    if (along <= 1e-6) return std::nullopt;
    return along;
}

std::optional<double> cameraBearingToDeg(const CameraMount& camera, const odom::Pose& robot,
                                         FieldPoint point) {
    const Lens lens = lensOf(camera, robot);
    const double a = lens.axisDeg * kDegToRad;
    const double dx = point.xIn - lens.xIn;
    const double dy = point.yIn - lens.yIn;
    const double along = dx * std::sin(a) + dy * std::cos(a);
    const double across = dx * std::cos(a) - dy * std::sin(a);
    if (along <= 1e-6) return std::nullopt;
    return std::atan2(across, along) * kRadToDeg;
}

FieldPoint onCameraRay(const CameraMount& camera, const odom::Pose& robot, double bearingDeg,
                       FieldPoint prior) {
    const Lens lens = lensOf(camera, robot);
    const double rangeIn = std::hypot(prior.xIn - lens.xIn, prior.yIn - lens.yIn);
    const double ray = (lens.axisDeg + bearingDeg) * kDegToRad;
    return FieldPoint{lens.xIn + rangeIn * std::sin(ray), lens.yIn + rangeIn * std::cos(ray)};
}

double approachSpeedInPerS(double remainingIn, double cruiseInPerS, double decelInPerS2) {
    if (!(remainingIn > 0.0)) return 0.0;
    const double cruise = std::max(cruiseInPerS, 0.0);
    if (!(decelInPerS2 > 0.0)) return cruise;
    return std::min(cruise, std::sqrt(2.0 * decelInPerS2 * remainingIn));
}

TargetApproach::TargetApproach(PID& forward, PID& turn) : forward_(forward), turn_(turn) {}

void TargetApproach::begin(const ApproachConfig& config, double startSpeedInPerS,
                           std::uint32_t nowMs) {
    config_ = config;
    tracker_ = ExitTracker(0, config.timeoutMs, nowMs);
    reason_ = ExitReason::running;
    // from the speed it has, never backward
    speedRefInPerS_ = std::max(startSpeedInPerS, 0.0);
    lagIn_ = 0.0;
    aimed_ = false;
    started_ = false;
    lastMs_ = nowMs;
    finalErrorIn_ = 0.0;
    forward_.reset();
    turn_.reset();
}

ApproachTick TargetApproach::update(const odom::Pose& pose, const odom::Velocity& velocity,
                                    FieldPoint target, const MotorFeedforward& forwardModel,
                                    const MotorFeedforward& turnModel, std::uint32_t nowMs) {
    ApproachTick tick;
    const double dtS = started_ ? static_cast<double>(nowMs - lastMs_) / 1000.0 : 0.0;
    started_ = true;
    lastMs_ = nowMs;

    const double h = pose.headingDeg * kDegToRad;
    const double speedInPerS = velocity.xInPerS * std::sin(h) + velocity.yInPerS * std::cos(h);
    const double dx = target.xIn - pose.xIn;
    const double dy = target.yIn - pose.yIn;
    tick.distanceIn = std::hypot(dx, dy);
    tick.remainingIn = tick.distanceIn - config_.endShortIn;

    // Aim at the target until it's nearly in the robot, then hold that heading
    if (!aimed_ || tick.distanceIn > config_.steerMinIn) {
        double aim = std::atan2(dx, dy) * kRadToDeg;
        if (aim < 0.0) aim += 360.0;
        aimDeg_ = aim;
        aimed_ = true;
        tick.steering = true;
    }
    tick.aimDeg = aimDeg_;
    tick.headingErrorDeg = wrapDegrees180(aimDeg_ - pose.headingDeg);

    // The speed reference: toward cruise (or what the end allows) no faster than the limits
    const double goal =
        approachSpeedInPerS(tick.remainingIn, config_.cruiseInPerS, config_.decelInPerS2);
    const double before = speedRefInPerS_;
    if (dtS > 0.0) {
        if (goal > speedRefInPerS_) {
            speedRefInPerS_ = std::min(goal, speedRefInPerS_ + config_.accelInPerS2 * dtS);
        } else {
            speedRefInPerS_ = std::max(goal, speedRefInPerS_ - config_.decelInPerS2 * dtS);
        }
        // how far the robot has fallen behind the reference, capped so a held-up robot catches up
        // at its own pace rather than at full power
        lagIn_ = std::clamp(lagIn_ + (speedRefInPerS_ - speedInPerS) * dtS, -config_.maxLagIn,
                            config_.maxLagIn);
    }
    tick.speedRefInPerS = speedRefInPerS_;
    tick.accelRefInPerS2 = dtS > 0.0 ? (speedRefInPerS_ - before) / dtS : 0.0;
    tick.lagIn = lagIn_;

    tick.forwardVolts = forwardModel.volts(speedRefInPerS_, tick.accelRefInPerS2) +
                        forward_.updateWithRate(lagIn_, 0.0, speedRefInPerS_ - speedInPerS, dtS);
    // D on the robot's turn rate alone, so a target the camera moves doesn't kick it, and the
    // turn's static friction toward the target, eased in (kTurnFrictionRampDeg)
    const double friction =
        turnModel.kS * std::clamp(tick.headingErrorDeg / kTurnFrictionRampDeg, -1.0, 1.0);
    tick.turnVolts = friction + turn_.updateWithRate(tick.headingErrorDeg, 0.0,
                                                     -velocity.headingDegPerS, dtS);

    finalErrorIn_ = std::fabs(tick.remainingIn);
    reason_ = tracker_.update(tick.remainingIn <= config_.endThresholdIn, nowMs);
    return tick;
}

MotionResult TargetApproach::result() const {
    return MotionResult{
        .reason = reason_, .finalError = finalErrorIn_, .elapsedMs = tracker_.elapsedMs()};
}

} // namespace sapphirelib::motion
