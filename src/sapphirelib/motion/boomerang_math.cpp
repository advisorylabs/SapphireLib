#include "sapphirelib/motion/boomerang_math.hpp"

#include <algorithm>
#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// backing in has to point the robot more than this much nearer its carrot to be picked, in degrees
constexpr double kTieDeg = 1.0;

// where the carrot is for a robot finishing forward (+1) or backing in (-1): behind the target
// along the way it travels at the end, sliding onto the target as the robot closes in
void carrotFor(double direction, double targetXIn, double targetYIn, double targetHeadingDeg,
               double offsetIn, double& xIn, double& yIn) {
    const double headingRad = targetHeadingDeg * kDegToRad;
    xIn = targetXIn - direction * offsetIn * std::sin(headingRad);
    yIn = targetYIn - direction * offsetIn * std::cos(headingRad);
}

// the turn that points the robot's front (forward) or back (backing in) at a point
double turnToward(double direction, const odom::Pose& pose, double xIn, double yIn) {
    const double bearingDeg = std::atan2(xIn - pose.xIn, yIn - pose.yIn) / kDegToRad;
    return wrapDegrees180(bearingDeg + (direction > 0.0 ? 0.0 : 180.0) - pose.headingDeg);
}

} // namespace

SteeringError steerToward(double targetBearingDeg, double currentHeadingDeg) {
    double headingError = wrapDegrees180(targetBearingDeg - currentHeadingDeg);
    double direction = 1.0;
    if (std::fabs(headingError) > 90.0) {
        headingError = wrapDegrees180(headingError - 180.0);
        direction = -1.0;
    }
    return SteeringError{headingError, direction};
}

BoomerangStep boomerangStep(const odom::Pose& pose, double targetXIn, double targetYIn,
                            double targetHeadingDeg, double leadPct, double settleRadiusIn,
                            double turnRadiusIn, const BoomerangStep& previous) {
    BoomerangStep step;
    step.started = true;
    const double dxIn = targetXIn - pose.xIn;
    const double dyIn = targetYIn - pose.yIn;
    step.distanceIn = std::hypot(dxIn, dyIn);
    const double offsetIn = step.distanceIn * leadPct;

    // forward or backing in, chosen once, from whichever points the robot more nearly at its
    // carrot now (forward on a tie, within a degree). Chosen afresh every tick, a robot whose
    // carrot swings past 90 degrees flips to backing in and arrives facing the wrong way
    if (previous.started) {
        step.direction = previous.direction;
    } else {
        double xIn = 0.0;
        double yIn = 0.0;
        carrotFor(1.0, targetXIn, targetYIn, targetHeadingDeg, offsetIn, xIn, yIn);
        const double forwardTurn = std::fabs(turnToward(1.0, pose, xIn, yIn));
        carrotFor(-1.0, targetXIn, targetYIn, targetHeadingDeg, offsetIn, xIn, yIn);
        const double backwardTurn = std::fabs(turnToward(-1.0, pose, xIn, yIn));
        step.direction = backwardTurn < forwardTurn - kTieDeg ? -1.0 : 1.0;
    }

    // once in, until a bump takes it well out again, so it doesn't flap at the edge
    step.settling =
        settleRadiusIn > 0.0 && (previous.settling ? step.distanceIn <= 2.0 * settleRadiusIn
                                                   : step.distanceIn <= settleRadiusIn);
    if (step.settling) {
        // what's left along the way it faces, in the direction it's going: about the distance,
        // so the drive PID's error carries on smoothly
        const double headingRad = pose.headingDeg * kDegToRad;
        const double aheadIn = dxIn * std::sin(headingRad) + dyIn * std::cos(headingRad);
        step.driveErrorIn = step.direction * aheadIn;
        // onto the point itself first, which takes out what the curve left to the side, then,
        // on it, turn in place to the final heading. Latched like settling, so it doesn't flap
        step.turning = previous.turning ? step.distanceIn <= 2.0 * turnRadiusIn
                                        : step.distanceIn <= turnRadiusIn;
        if (step.turning) {
            step.turnErrorDeg = wrapDegrees180(targetHeadingDeg - pose.headingDeg);
        } else {
            step.turnErrorDeg = turnToward(step.direction, pose, targetXIn, targetYIn);
            step.driveScale = std::max(0.0, std::cos(step.turnErrorDeg * kDegToRad));
        }
        return step;
    }

    double carrotXIn = 0.0;
    double carrotYIn = 0.0;
    carrotFor(step.direction, targetXIn, targetYIn, targetHeadingDeg, offsetIn, carrotXIn,
              carrotYIn);
    step.turnErrorDeg = turnToward(step.direction, pose, carrotXIn, carrotYIn);
    step.driveErrorIn = step.distanceIn;
    // facing well away, turn before driving: none of the drive at 90 degrees off and past
    step.driveScale = std::max(0.0, std::cos(step.turnErrorDeg * kDegToRad));
    return step;
}

} // namespace sapphirelib::motion
