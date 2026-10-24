#pragma once

#include <string>
#include <vector>

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/motion/trajectory.hpp"
#include "sapphirelib/motion/trajectory_config.hpp"

namespace sapphirelib::motion {

/**
 * @brief Result of generateTrajectory()
 */
struct TrajectoryResult {
    /** false if no trajectory could be planned; error says why */
    bool ok = false;

    /** what was wrong, e.g. "the Strafe model isn't measured: run Auto-Tune". Empty when ok */
    std::string error;

    /** the plan. Empty unless ok */
    Trajectory trajectory;
};

/**
 * @brief Plan a trajectory through a chain of sections for a holonomic chassis
 *
 * Fits the path (see samplePath()), then plans the fastest speed along it that keeps every
 * section inside its constraints. The binding one is usually the voltage budget: at each point
 * the forward, strafe and turn volts the measured models need (friction shared between the axes,
 * see frictionVolts()) must add up to no more than the budget, since that sum is what the worst
 * wheel gets after mixing. That slows the plan for strafing (a slower axis), for corners, and for
 * turning while driving. Speed starts and ends at 0, and is 0 at every stopAtEnd point; between
 * sections without one it carries straight through
 *
 * With options, a trajectory can instead start where another one ends, at its speed and in its
 * direction of travel (TrajectoryOptions::from), or end moving for another to continue from
 * (endSpeedInPerS): the pieces of a branch, planned separately and driven through without
 * stopping (TrajectoryFollowConfig::next)
 *
 * @note allocates, and takes a few milliseconds for a long path: generate before the motion, not
 * inside a control loop
 *
 * @param sections the chain. See TrajectorySection
 * @param models the chassis's measured axis models (HolonomicDrivetrain::axisModels()). All three
 * must be valid
 * @param options where it starts and how it ends. By default, at rest at both
 * @return TrajectoryResult the trajectory, or why there isn't one
 *
 * @b Example
 * @code {.cpp}
 * const sapphirelib::motion::TrajectoryResult rush =
 *     sapphirelib::motion::generateTrajectory(kRush, drivetrain().axisModels());
 * if (!rush.ok) printf("rush: %s\n", rush.error.c_str());
 * else drivetrain().followTrajectory(rush.trajectory);
 * @endcode
 */
TrajectoryResult generateTrajectory(const std::vector<TrajectorySection>& sections,
                                    const chassis::HolonomicAxisModels& models,
                                    const TrajectoryOptions& options = {});

/**
 * @brief Plan a trajectory through waypoints, all with the same constraints
 *
 * One section; see the chain version above
 *
 * @param waypoints the points, in order. The first needs a heading
 * @param constraints how hard the trajectory may drive
 * @param models the chassis's measured axis models
 * @return TrajectoryResult the trajectory, or why there isn't one
 */
TrajectoryResult generateTrajectory(const std::vector<TrajectoryWaypoint>& waypoints,
                                    const TrajectoryConstraints& constraints,
                                    const chassis::HolonomicAxisModels& models);

/**
 * @brief The range of path accelerations the voltage budget allows at one point
 *
 * The pure core of generateTrajectory(), for tests: each axis needs p + q * a volts for a path
 * acceleration a, and the sum of their sizes must stay within the budget
 *
 * @param p each axis's volts at zero path acceleration
 * @param q each axis's volts per unit of path acceleration
 * @param budgetVolts the voltage budget
 * @param lo set to the lowest allowed acceleration (possibly -infinity)
 * @param hi set to the highest allowed acceleration (possibly infinity)
 * @return true if any acceleration fits the budget
 */
bool voltageAccelRange(const double p[3], const double q[3], double budgetVolts, double& lo,
                       double& hi);

} // namespace sapphirelib::motion
