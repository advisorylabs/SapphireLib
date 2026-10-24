#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace sapphirelib::motion {

class Trajectory;

/**
 * @brief A point a trajectory passes through, in the same field frame as odom::Pose
 *
 * The heading is where the robot faces on reaching the point. Between two points with headings the
 * robot turns smoothly as it drives; points without one leave the turn spread across them
 */
struct TrajectoryWaypoint {
    /** x position, in inches */
    double xIn = 0.0;
    /** y position, in inches */
    double yIn = 0.0;
    /**
     * heading on reaching this point, in degrees, clockwise from +y. Required on a trajectory's
     * first point (where the robot starts); optional after that
     */
    std::optional<double> headingDeg;
};

/**
 * @brief Speed limits on one axis of a holonomic chassis
 *
 * Forward and strafe in inches, turn in degrees. 0 leaves the axis limited only by the voltage
 * budget and its measured model
 */
struct AxisLimits {
    /** top speed, in units per second. 0 for no cap */
    double maxVelocity = 0.0;
    /** top acceleration (and deceleration), in units per second squared. 0 for no cap */
    double maxAcceleration = 0.0;
};

/**
 * @brief How hard a trajectory may drive the chassis
 *
 * @b Example
 * @code {.cpp}
 * // slow and gentle: 7V of the motors' 12V, and no faster than 30 in/s sideways
 * sapphirelib::motion::TrajectoryConstraints careful{.voltageBudget = 7.0,
 *                                                   .strafe = {.maxVelocity = 30.0}};
 * @endcode
 */
struct TrajectoryConstraints {
    /**
     * the most any wheel may be planned to need, in volts. What's left of 12V is headroom for the
     * tracking feedback and battery sag. 10 by default; clamped to 12
     */
    double voltageBudget = 10.0;

    /** forward/backward limits, in inches */
    AxisLimits forward;
    /** sideways limits, in inches */
    AxisLimits strafe;
    /** rotation limits, in degrees */
    AxisLimits turn;
};

/**
 * @brief One piece of a trajectory, with its own limits
 *
 * A trajectory is a chain of sections driven one after another without stopping between them,
 * unless a section asks to. Every section after the first continues from the previous one's last
 * waypoint, so it lists only the points after it
 *
 * @b Example
 * @code {.cpp}
 * // full speed to the goal, then slower through the intake
 * const std::vector<sapphirelib::motion::TrajectorySection> rush{
 *     {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0}, {.xIn = -40, .yIn = -30}}},
 *     {.waypoints = {{.xIn = -24, .yIn = -12, .headingDeg = 45}},
 *      .constraints = {.voltageBudget = 7.0}},
 * };
 * @endcode
 */
struct TrajectorySection {
    /** the points this section drives through, in order */
    std::vector<TrajectoryWaypoint> waypoints;

    /** how hard this section may drive */
    TrajectoryConstraints constraints;

    /**
     * come to rest at this section's last waypoint before the next section starts. The path may
     * turn sharply there (drive in, stop, back out at an angle). The last section always stops,
     * unless TrajectoryOptions::endSpeedInPerS says otherwise
     */
    bool stopAtEnd = false;
};

/**
 * @brief Where a planned trajectory starts and how it ends, for chaining separately planned
 * trajectories: branching mid-routine on a sensor
 *
 * @b Example
 * @code {.cpp}
 * // the trunk ends at up to 30in/s instead of stopping, and each branch picks up from its end
 * trunk = generateTrajectory(kTrunk, models, {.endSpeedInPerS = 30.0});
 * left = generateTrajectory(kLeftTail, models, {.from = &trunk.trajectory});
 * @endcode
 */
struct TrajectoryOptions {
    /**
     * start where this trajectory ends instead of at rest on the first waypoint: at its last
     * point, facing its last heading, at its last speed and in its direction of travel. The
     * sections then list only the points after that one, and the first needs no heading, as for
     * every section after the first of a chain. The plan fails if it can't carry that speed into
     * its path (see endSpeedInPerS). Read only while planning; nullptr starts at rest, as usual
     */
    const Trajectory* from = nullptr;

    /**
     * the speed to end at, in inches per second, instead of stopping, for a trajectory another
     * one continues from. The most it may be: the plan ends slower if its path or limits need it.
     * The path's last point isn't settled on; follow it with TrajectoryFollowConfig::next. 0 (the
     * default) comes to rest at the end
     */
    double endSpeedInPerS = 0.0;
};

/**
 * @brief How followTrajectory() tracks a trajectory and when it ends
 *
 * @b Example
 * @code {.cpp}
 * // settle within half an inch, and pause the plan when 6 inches behind it
 * drivetrain().followTrajectory(trajectory, {.positionErrorThresholdIn = 0.5, .maxLagIn = 6.0});
 * @endcode
 */
struct TrajectoryFollowConfig {
    /** distance from the final point that counts as arrived, in inches. 1 by default */
    double positionErrorThresholdIn = 1.0;

    /** heading error that counts as arrived, in degrees. 2 by default */
    double headingErrorThresholdDeg = 2.0;

    /** how long both errors must stay within threshold, in milliseconds. 200 by default */
    std::uint32_t settleTimeMs = 200;

    /**
     * how long the motion may run past the trajectory's planned duration, in milliseconds. Time
     * the plan spends paused (see maxLagIn) counts against it, so a robot held in place gives up
     * (MotionResult timedOut) instead of waiting forever. 1500 by default
     */
    std::uint32_t overtimeMs = 1500;

    /**
     * how far behind the plan the robot may fall before the plan waits for it, in inches. Blocked
     * or bumped, the robot then catches up at the planned speed instead of at full power. 4 by
     * default; 0 never pauses
     */
    double maxLagIn = 4.0;

    /**
     * what to drive next, for a trajectory that ends moving (TrajectoryOptions::endSpeedInPerS):
     * called as the plan reaches that end, with the trajectory that's ending, and the robot carries
     * straight on into whichever trajectory it returns, without stopping. That one must continue
     * from the ending one (planned with TrajectoryOptions::from) and outlive the motion. If it
     * ends moving too, this is called again at its end, so one function can choose at every
     * junction of a tree by checking which trajectory is ending.
     *
     * Runs on the motion's task inside the control loop, so it must be quick: read a sensor, pick
     * a plan that's already made. Returning nullptr (or leaving this empty) settles on the ending
     * trajectory's last point from speed, on feedback alone, which overshoots: always return one
     */
    std::function<const Trajectory*(const Trajectory& ending)> next;
};

} // namespace sapphirelib::motion
