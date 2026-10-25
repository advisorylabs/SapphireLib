/**
 * \file paths.cpp
 *
 * followPath(): a trajectory from the Paths tab, with the claw and the
 * extender doing what its points say as the plan reaches them. See paths.hpp.
 *
 * Team 96671H: Hitmen
 */

#include "robot/paths.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "pros/rtos.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"
#include "sapphirelib/motion/motion_queue.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/log.hpp"

namespace robot {

using sapphirelib::motion::MotionResult;
using sapphirelib::motion::TrajectoryProgress;
using sapphirelib::motion::TrajectoryResult;

namespace {

// How often followPath() checks the plan's progress for actions coming due:
// at 60in/s, about a third of an inch.
constexpr std::uint32_t kActionPollMs = 5;
// A point counts as reached this close to it, so one at the very end isn't
// missed by a rounding error between the plan's last sample and the point.
constexpr double kReachedIn = 1e-6;

const char* actionName(PathAction::Kind action) {
    switch (action) {
    case PathAction::clawOpen: return "claw_open";
    case PathAction::clawClose: return "claw_close";
    case PathAction::extenderOut: return "extender_out";
    case PathAction::extenderIn: return "extender_in";
    }
    return "unknown";
}

void apply(PathAction::Kind action) {
    switch (action) {
    case PathAction::clawOpen: macros::setClawClosed(false); break;
    case PathAction::clawClose: macros::setClawClosed(true); break;
    case PathAction::extenderOut: macros::setExtended(true); break;
    case PathAction::extenderIn: macros::setExtended(false); break;
    }
}

// One action, by how far along the path it's due.
struct Due {
    double atIn;
    std::size_t waypoint;
    PathAction::Kind action;
};

} // namespace

MotionResult followPath(const TrajectoryResult& plan, const std::vector<PathAction>& actions) {
    // static: the queue's task outlives this call, and so may its motion, if
    // a competition switch ends autonomous first; so does what it writes
    static sapphirelib::motion::MotionQueue queue;
    static MotionResult result;

    std::vector<Due> due;
    for (const PathAction& action : actions) {
        if (action.waypoint >= plan.waypointIn.size()) {
            SAPPHIRELIB_LOG_WARN("paths", "%s at point %u: the path has only %u points",
                                 actionName(action.action), static_cast<unsigned>(action.waypoint),
                                 static_cast<unsigned>(plan.waypointIn.size()));
            continue;
        }
        due.push_back({plan.waypointIn[action.waypoint], action.waypoint, action.action});
    }
    // in the order the plan reaches them; stable, so two at one point keep theirs
    std::stable_sort(due.begin(), due.end(),
                     [](const Due& a, const Due& b) { return a.atIn < b.atIn; });

    // the number this trajectory's motion will have: until it starts, the
    // progress is still whichever trajectory ran last
    const std::uint32_t motion = drivetrain().trajectoryProgress().motion + 1;
    result = MotionResult{};
    const sapphirelib::motion::Trajectory* trajectory = &plan.trajectory;
    queue.enqueue([trajectory] { result = drivetrain().followTrajectory(*trajectory); });
    queue.run();

    std::size_t next = 0;
    const auto doDue = [&] {
        const TrajectoryProgress progress = drivetrain().trajectoryProgress();
        if (progress.motion != motion) return;
        while (next < due.size() && progress.distanceIn >= due[next].atIn - kReachedIn) {
            apply(due[next].action);
            sapphirelib::telemetry::event("path", "action,%s,point=%u,in=%.1f",
                                          actionName(due[next].action),
                                          static_cast<unsigned>(due[next].waypoint),
                                          due[next].atIn);
            ++next;
        }
    };
    while (queue.isBusy()) {
        doDue();
        pros::delay(kActionPollMs);
    }
    // the motion's last progress, in case it reached a point after the last poll
    doDue();
    for (; next < due.size(); ++next) {
        sapphirelib::telemetry::event("path", "skipped,%s,point=%u,in=%.1f",
                                      actionName(due[next].action),
                                      static_cast<unsigned>(due[next].waypoint), due[next].atIn);
    }
    return result;
}

} // namespace robot
