/**
 * \file robot/paths.hpp
 *
 * Paths from Sapphire Sim's Paths tab (tools/sim), with what the claw and the
 * extender do along them. The tab's Claw and Extender columns put an action
 * on a point, and Copy C++ writes them out after the path's sections, as a
 * list named after the path:
 *
 *   // Sapphire Sim path "rush": 3.42 s planned with the open TUNE.CFG's models
 *   const std::vector<sapphirelib::motion::TrajectorySection> kRush{...};
 *   // what the claw and the extender do along "rush", by point (robot::followPath())
 *   const std::vector<robot::PathAction> kRushActions{
 *       {.waypoint = 3, .action = robot::PathAction::clawOpen},
 *   };
 *
 * A routine in autons.cpp plans the path and follows it with them:
 *
 *   void rushAuton() {
 *       static TrajectoryResult rush; // the motion reads it to the end
 *       if (!planPiece("rush", rush, kRush)) return;
 *       odometry().setPose({.xIn = -48, .yIn = -60, .headingDeg = 0}); // its first point
 *       followPath(rush, kRushActions);
 *   }
 *
 * Each action happens as the plan reaches its point, without the robot
 * slowing down for it (TrajectoryProgress::distanceIn against
 * TrajectoryResult::waypointIn): the extender goes out on the way past, the
 * claw opens on arriving. For the robot to come to rest there first, end a
 * section at the point with Stop at its end.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/motion/trajectory_generator.hpp"

namespace robot {

/// One thing for the claw or the extender to do, at one point of a path.
struct PathAction {
    /// What it does: the Paths tab's Claw and Extender choices, in the same
    /// order as tools/sim/js/pathview.js's ACTIONS (keep the two the same).
    enum Kind : std::uint8_t {
        clawOpen,    // macros::setClawClosed(false)
        clawClose,   // macros::setClawClosed(true)
        extenderOut, // macros::setExtended(true)
        extenderIn,  // macros::setExtended(false)
    };

    /// The point: its number in the Paths tab, its index among all the
    /// path's waypoints, its sections' in order. 0 is where it starts.
    std::size_t waypoint = 0;

    Kind action = clawOpen;
};

/// Follows `plan`'s trajectory, doing each of `actions` as the plan reaches
/// its point, and returns how the motion ended. Blocks until it does. Two
/// actions at one point happen in the order listed.
///
/// An action whose point the plan never reaches (the robot held up until the
/// motion timed out, say) isn't done, so the claw doesn't open short of the
/// goal. One on a point the plan doesn't have is logged and skipped.
///
/// The trajectory runs on a MotionQueue of its own, and the actions on the
/// calling task, the routine's, through macros::setClawClosed() and
/// setExtended(). As with followTrajectory(), `plan` must outlive the motion,
/// even past a competition switch ending autonomous: make it static.
sapphirelib::motion::MotionResult followPath(const sapphirelib::motion::TrajectoryResult& plan,
                                             const std::vector<PathAction>& actions);

} // namespace robot
