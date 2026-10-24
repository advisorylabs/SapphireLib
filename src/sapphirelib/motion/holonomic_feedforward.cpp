#include "sapphirelib/motion/holonomic_feedforward.hpp"

#include <cmath>

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// below this fraction of top speed (about 0.06 in/s on a drive axis) the chassis counts as at
// rest, and friction follows the acceleration instead
constexpr double kMovingFraction = 1e-3;

// an axis's part of a direction, as a fraction of its top speed. 0 without a valid model
double fraction(const MotorFeedforward& model, double value) {
    const double top = model.valid() ? model.maxVelocity(12.0) : 0.0;
    return top > 0.0 ? value / top : 0.0;
}

} // namespace

chassis::AxisVolts frictionVolts(const chassis::HolonomicAxisModels& models, AxisMotion direction) {
    const double f = fraction(models.forward, direction.forward);
    const double s = fraction(models.strafe, direction.strafe);
    const double t = fraction(models.turn, direction.turn);
    const double length = std::sqrt(f * f + s * s + t * t);
    if (!(length > 0.0)) return chassis::AxisVolts{};
    return chassis::AxisVolts{.forward = models.forward.kS * f / length,
                              .strafe = models.strafe.kS * s / length,
                              .turn = models.turn.kS * t / length};
}

chassis::AxisVolts holonomicFeedforward(const chassis::HolonomicAxisModels& models,
                                        AxisMotion velocity, AxisMotion acceleration) {
    const double moving = std::sqrt(std::pow(fraction(models.forward, velocity.forward), 2) +
                                    std::pow(fraction(models.strafe, velocity.strafe), 2) +
                                    std::pow(fraction(models.turn, velocity.turn), 2));
    const chassis::AxisVolts friction =
        frictionVolts(models, moving > kMovingFraction ? velocity : acceleration);
    return chassis::AxisVolts{
        .forward = friction.forward + models.forward.kV * velocity.forward +
                   models.forward.kA * acceleration.forward,
        .strafe = friction.strafe + models.strafe.kV * velocity.strafe +
                  models.strafe.kA * acceleration.strafe,
        .turn = friction.turn + models.turn.kV * velocity.turn + models.turn.kA * acceleration.turn,
    };
}

AxisMotion rotateAxisMotion(AxisMotion motion, double headingOffsetDeg) {
    // toLocalFrame() at heading + offset, written in terms of the components at heading
    const double c = std::cos(headingOffsetDeg * kDegToRad);
    const double s = std::sin(headingOffsetDeg * kDegToRad);
    return AxisMotion{.forward = motion.forward * c + motion.strafe * s,
                      .strafe = motion.strafe * c - motion.forward * s,
                      .turn = motion.turn};
}

chassis::AxisVolts trajectoryFeedforward(const chassis::HolonomicAxisModels& models,
                                         const TrajectorySample& sample, double headingOffsetDeg) {
    const AxisMotion direction = rotateAxisMotion(
        {sample.forwardPerIn, sample.strafePerIn, sample.turnDegPerIn}, headingOffsetDeg);
    const AxisMotion velocity = rotateAxisMotion(
        {sample.forwardInPerS, sample.strafeInPerS, sample.omegaDegPerS}, headingOffsetDeg);
    const AxisMotion acceleration = rotateAxisMotion(
        {sample.forwardAccelInPerS2, sample.strafeAccelInPerS2, sample.turnAccelDegPerS2},
        headingOffsetDeg);
    const chassis::AxisVolts friction = frictionVolts(models, direction);
    return chassis::AxisVolts{
        .forward = friction.forward + models.forward.kV * velocity.forward +
                   models.forward.kA * acceleration.forward,
        .strafe = friction.strafe + models.strafe.kV * velocity.strafe +
                  models.strafe.kA * acceleration.strafe,
        .turn = friction.turn + models.turn.kV * velocity.turn + models.turn.kA * acceleration.turn,
    };
}

} // namespace sapphirelib::motion
