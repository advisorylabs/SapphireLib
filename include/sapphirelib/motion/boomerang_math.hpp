#pragma once

#include "sapphirelib/odom/pose.hpp"

namespace sapphirelib::motion {

/**
 * @brief Which way a tank drive turns and drives to head for a bearing
 */
struct SteeringError {
    /** how far to turn, in degrees, clockwise positive */
    double headingErrorDeg = 0.0;
    /** +1 to drive forward, -1 to back up */
    double direction = 1.0;
};

/**
 * @brief Steer a tank drive toward a bearing, backing up instead of turning more than 90 degrees,
 * like driveDistance() with negative inches
 *
 * @param targetBearingDeg the bearing to head for, in degrees, clockwise from +y
 * @param currentHeadingDeg the robot's heading, in degrees
 * @return SteeringError the turn and the direction
 */
SteeringError steerToward(double targetBearingDeg, double currentHeadingDeg);

/**
 * @brief What one tick of TankDrivetrain::moveToPose() closes, and how
 *
 * The motion drives direction * driveScale * (the drive PID's output on driveErrorIn), and turns
 * by the turn PID's output on turnErrorDeg, both with 0 as the measurement
 */
struct BoomerangStep {
    /** distance from the robot to the target, in inches: the exit's position error */
    double distanceIn = 0.0;
    /**
     * what the drive PID closes, in inches: the distance to the target while steering, the
     * distance left along the way the robot faces once settling
     */
    double driveErrorIn = 0.0;
    /** +1 finishing forward, -1 backing in. Chosen on the first tick, kept to the end */
    double direction = 1.0;
    /**
     * how much of the drive PID's output to use: the cosine of the turn while steering, so a
     * robot facing well away turns before it drives (none at 90 degrees off and past); 1 settling
     */
    double driveScale = 1.0;
    /**
     * what the turn PID closes, in degrees: toward the carrot while steering, to the final heading
     * once settling
     */
    double turnErrorDeg = 0.0;
    /** whether it's settling (see boomerangStep()) */
    bool settling = false;
    /** whether, settling, it's on the point and turning in place to the final heading */
    bool turning = false;
    /** false only before the first tick: a default-constructed step */
    bool started = false;
};

/**
 * @brief One tick of the boomerang controller TankDrivetrain::moveToPose() drives with
 *
 * The robot steers toward a carrot point behind the target along the way it will travel at the
 * end, which slides onto the target as the robot closes in, so it curves into the final heading.
 * On the first tick it picks whether to finish forward or back in, whichever points it more nearly
 * at its carrot, and keeps it: re-picked every tick, a carrot swinging past 90 degrees flips the
 * robot to backing in and it arrives facing the wrong way. Backing in, the carrot goes in front of
 * the target instead, so the robot still ends facing the final heading.
 *
 * Within settleRadiusIn the carrot is on top of the target and the bearing to it is noise (a tenth
 * of an inch to the side is a big angle), and a tank can't move sideways anyway: it settles,
 * turning to the final heading and closing the distance left along the way it faces. It stays
 * settling until it's twice the radius away again (a bump)
 *
 * @param pose the robot's pose
 * @param targetXIn target x, in inches
 * @param targetYIn target y, in inches
 * @param targetHeadingDeg the final heading, in degrees
 * @param leadPct the carrot's distance behind the target, as a fraction of the distance left
 * @param settleRadiusIn how close it settles, in inches. 0 never settles: it then steers at the
 * carrot to the end, and nothing turns it to the final heading
 * @param previous the last tick's step: the direction, and whether it was settling.
 * Default-constructed for the first tick
 * @return BoomerangStep the errors for the drive and turn PIDs
 */
BoomerangStep boomerangStep(const odom::Pose& pose, double targetXIn, double targetYIn,
                            double targetHeadingDeg, double leadPct, double settleRadiusIn,
                            double turnRadiusIn, const BoomerangStep& previous);

} // namespace sapphirelib::motion
