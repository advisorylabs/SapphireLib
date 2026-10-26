#include "sapphirelib/auton/auton.hpp"

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include "pros/rtos.hpp"
#include "sapphirelib/chassis/competition_watch.hpp"
#include "sapphirelib/motion/motion_queue.hpp"
#include "sapphirelib/motion/trajectory_generator.hpp"
#include "sapphirelib/util/angle.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"

namespace sapphirelib::auton {

namespace {

constexpr double kPi = 3.14159265358979323846;

// how often the waitUntil...() calls check
constexpr std::uint32_t kPollMs = 5;

// how close counts as there, by default and when chaining (see Options::exitError)
constexpr double kExitIn = 1.0;
constexpr double kChainExitIn = 3.0;
constexpr double kExitDeg = 2.0;
constexpr double kChainExitDeg = 5.0;

// a sensor counts as facing a side within this many degrees of it
constexpr double kFacingToleranceDeg = 45.0;

// what use() was given. Set once in initialize(), before any routine runs, so unsynchronized
chassis::HolonomicDrivetrain* holonomic = nullptr;
chassis::TankDrivetrain* tank = nullptr;
odom::Odometry* odometry = nullptr;
localization::MonteCarloLocalizer* localizer = nullptr;
Settings settings;

// runAsync()'s task. Built on first use, at runtime, like every PROS object here
motion::MotionQueue& queue() {
    static motion::MotionQueue instance;
    return instance;
}

// the queue's task, once it has run a block: a motion called from it is part of a block
std::atomic<pros::task_t> blockTask{nullptr};

// the running block's watch, on its own task: a cancelMotion() or a competition mode change
// since runAsync() skips the block's motions that haven't started. Queue task only
const chassis::CompetitionWatch* blockWatch = nullptr;

// every motion takes the next number as it starts, and the latest block's first motion gets
// blockFirst: until then, the waits are still looking at the motion before it
std::atomic<std::uint32_t> motionCount{0};
std::atomic<std::uint32_t> blockFirst{1};

// the odometers as the latest motion started, for waitUntilTraveled() and waitUntilTurned()
std::atomic<double> startTravelledIn{0.0};
std::atomic<double> startTurnedDeg{0.0};

// the latest motion's trajectory, if it's one, and its number on trajectoryProgress(), for
// waitUntilWaypoint()
std::atomic<const motion::TrajectoryResult*> trajectoryPlan{nullptr};
std::atomic<std::uint32_t> trajectoryMotion{0};

bool onBlockTask() {
    const pros::task_t task = blockTask.load();
    return task != nullptr && pros::c::task_get_current() == task;
}

// Gets a motion ready to run: waits out the background motions if it isn't one of them, and
// starts its progress. Or the result to return instead, when it can't run
std::optional<motion::MotionResult> begin(const char* kind) {
    if ((holonomic == nullptr && tank == nullptr) || odometry == nullptr) {
        SAPPHIRELIB_LOG_ERROR("auton", "%s: no drivetrain - call auton::use() in initialize()",
                              kind);
        return motion::MotionResult{.reason = motion::ExitReason::aborted};
    }
    if (onBlockTask()) {
        // the rest of a cancelled block, or one that outlived autonomous: don't start
        if (blockWatch != nullptr && blockWatch->changed()) {
            return motion::MotionResult{.reason = motion::ExitReason::interrupted};
        }
    } else {
        // so two motions never drive at once
        queue().waitUntilDone();
    }

    const odom::Odometry::Snapshot snapshot = odometry->snapshot();
    startTravelledIn.store(snapshot.travelledIn);
    startTurnedDeg.store(snapshot.turnedDeg);
    trajectoryPlan.store(nullptr);
    motionCount.fetch_add(1);
    return std::nullopt;
}

// runs a motion on whichever drivetrain use() was given: both have the same motions
template <class Motion> motion::MotionResult onChassis(Motion&& motion) {
    return holonomic != nullptr ? motion(*holonomic) : motion(*tank);
}

// a holonomic-only motion on a tank drive
motion::MotionResult holonomicOnly(const char* kind) {
    SAPPHIRELIB_LOG_ERROR("auton", "%s: needs a holonomic drivetrain", kind);
    return motion::MotionResult{.reason = motion::ExitReason::aborted};
}

chassis::ExitConditions exitFor(const Options& options, double error, double chainError) {
    chassis::ExitConditions exit{options.exitError > 0.0 ? options.exitError
                                                         : (options.chain ? chainError : error)};
    if (options.timeoutMs > 0) exit.timeoutMs = options.timeoutMs;
    exit.maxVolts = options.maxVolts;
    exit.chain = options.chain;
    return exit;
}

// --- followTrajectory()'s plans ---

bool sameAxis(const MotorFeedforward& a, const MotorFeedforward& b) {
    return a.kS == b.kS && a.kV == b.kV && a.kA == b.kA;
}

bool sameModels(const chassis::HolonomicAxisModels& a, const chassis::HolonomicAxisModels& b) {
    return sameAxis(a.forward, b.forward) && sameAxis(a.strafe, b.strafe) &&
           sameAxis(a.turn, b.turn);
}

bool sameLimits(const motion::AxisLimits& a, const motion::AxisLimits& b) {
    return a.maxVelocity == b.maxVelocity && a.maxAcceleration == b.maxAcceleration;
}

bool sameSections(const std::vector<motion::TrajectorySection>& a,
                  const std::vector<motion::TrajectorySection>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t s = 0; s < a.size(); ++s) {
        const motion::TrajectorySection& x = a[s];
        const motion::TrajectorySection& y = b[s];
        if (x.stopAtEnd != y.stopAtEnd || x.waypoints.size() != y.waypoints.size() ||
            x.constraints.voltageBudget != y.constraints.voltageBudget ||
            !sameLimits(x.constraints.forward, y.constraints.forward) ||
            !sameLimits(x.constraints.strafe, y.constraints.strafe) ||
            !sameLimits(x.constraints.turn, y.constraints.turn)) {
            return false;
        }
        for (std::size_t w = 0; w < x.waypoints.size(); ++w) {
            if (x.waypoints[w].xIn != y.waypoints[w].xIn ||
                x.waypoints[w].yIn != y.waypoints[w].yIn ||
                x.waypoints[w].headingDeg != y.waypoints[w].headingDeg) {
                return false;
            }
        }
    }
    return true;
}

struct Plan {
    std::vector<motion::TrajectorySection> sections;
    chassis::HolonomicAxisModels models;
    motion::TrajectoryResult result;
};

pros::Mutex& planMutex() {
    static pros::Mutex instance;
    return instance;
}

// every plan made, newest last. Never freed: a motion may still be driving an older one
std::vector<std::unique_ptr<Plan>>& plans() {
    static std::vector<std::unique_ptr<Plan>> instance;
    return instance;
}

// the plan for these sections from the drivetrain's models now: made the first time, and again if
// Auto-Tune or TUNE.CFG has changed the models since
const Plan& planFor(const std::vector<motion::TrajectorySection>& sections) {
    const chassis::HolonomicAxisModels models = holonomic->axisModels();
    std::lock_guard<pros::Mutex> lock(planMutex());
    for (auto it = plans().rbegin(); it != plans().rend(); ++it) {
        if (sameModels((*it)->models, models) && sameSections((*it)->sections, sections)) {
            return **it;
        }
    }
    const std::uint64_t startUs = sapphirelib::micros();
    auto plan = std::make_unique<Plan>(
        Plan{sections, models, motion::generateTrajectory(sections, models)});
    const double planMs = (sapphirelib::micros() - startUs) / 1000.0;
    if (plan->result.ok) {
        SAPPHIRELIB_LOG_INFO("auton", "trajectory planned in %.1fms: %.2fs over %.0fin", planMs,
                             plan->result.trajectory.durationS(),
                             plan->result.trajectory.lengthIn());
    } else {
        SAPPHIRELIB_LOG_ERROR("auton", "trajectory: %s", plan->result.error.c_str());
    }
    plans().push_back(std::move(plan));
    return *plans().back();
}

// --- the waits ---

// waits, while background motions run, for done() to come true once the latest block's first
// motion has started
template <class Done> bool waitWhileRunning(const char* kind, Done&& done) {
    // the block would be waiting on itself
    if (onBlockTask()) {
        SAPPHIRELIB_LOG_WARN("auton", "%s: call it from the routine, not inside runAsync()", kind);
        return false;
    }
    if (!queue().isBusy()) {
        SAPPHIRELIB_LOG_WARN("auton", "%s: no runAsync() motion is running", kind);
        return false;
    }
    const std::uint32_t first = blockFirst.load();
    while (queue().isBusy()) {
        if (motionCount.load() >= first && done()) return true;
        pros::delay(kPollMs);
    }
    return false;
}

// --- the wall resets ---

bool resetFacing(double facingDeg, const char* kind) {
    if (localizer == nullptr) {
        SAPPHIRELIB_LOG_ERROR("auton", "%s: no localizer - pass one to auton::use()", kind);
        return false;
    }
    const std::vector<localization::DistanceSensorMount> mounts = localizer->sensorMounts();
    std::optional<std::size_t> sensor;
    double closestDeg = kFacingToleranceDeg;
    for (std::size_t i = 0; i < mounts.size(); ++i) {
        const double offDeg = std::fabs(wrapDegrees180(mounts[i].facingDeg - facingDeg));
        if (offDeg <= closestDeg) {
            closestDeg = offDeg;
            sensor = i;
        }
    }
    if (!sensor) {
        SAPPHIRELIB_LOG_ERROR("auton", "%s: no distance sensor faces that way", kind);
        return false;
    }
    const localization::WallFix fix = localizer->resetFromWall(*sensor, settings.wallReset);
    if (fix.ok) {
        SAPPHIRELIB_LOG_INFO("auton", "%s: %c = %.1f (moved %.1fin)", kind,
                             fix.axis == localization::WallAxis::x ? 'x' : 'y', fix.valueIn,
                             fix.shiftIn);
    } else {
        SAPPHIRELIB_LOG_INFO("auton", "%s: skipped, %s", kind, fix.why);
    }
    return fix.ok;
}

} // namespace

void use(chassis::HolonomicDrivetrain& drivetrain, odom::Odometry& odometryToUse,
         localization::MonteCarloLocalizer* localizerToUse, Settings settingsToUse) {
    holonomic = &drivetrain;
    tank = nullptr;
    odometry = &odometryToUse;
    localizer = localizerToUse;
    settings = settingsToUse;
    drivetrain.setOdometry(&odometryToUse);
}

void use(chassis::TankDrivetrain& drivetrain, odom::Odometry& odometryToUse,
         localization::MonteCarloLocalizer* localizerToUse, Settings settingsToUse) {
    holonomic = nullptr;
    tank = &drivetrain;
    odometry = &odometryToUse;
    localizer = localizerToUse;
    settings = settingsToUse;
    drivetrain.setOdometry(&odometryToUse);
}

void setPose(double xIn, double yIn, double headingDeg) {
    if (odometry == nullptr) {
        SAPPHIRELIB_LOG_ERROR("auton", "setPose: no odometry - call auton::use() in initialize()");
        return;
    }
    odometry->setPose({.xIn = xIn, .yIn = yIn, .headingDeg = headingDeg});
}

odom::Pose pose() { return odometry != nullptr ? odometry->getPose() : odom::Pose{}; }

motion::MotionResult drive(double inches, Options options) {
    if (auto skip = begin("drive")) return *skip;
    const chassis::ExitConditions exit = exitFor(options, kExitIn, kChainExitIn);
    return onChassis([&](auto& drivetrain) { return drivetrain.driveDistance(inches, exit); });
}

motion::MotionResult turnTo(double headingDeg, Options options) {
    return turnTo(headingDeg, motion::TurnDirection::shortest, options);
}

motion::MotionResult turnTo(double headingDeg, motion::TurnDirection direction, Options options) {
    if (auto skip = begin("turnTo")) return *skip;
    const chassis::ExitConditions exit = exitFor(options, kExitDeg, kChainExitDeg);
    return onChassis(
        [&](auto& drivetrain) { return drivetrain.turnToHeading(headingDeg, exit, direction); });
}

motion::MotionResult turnToPoint(double xIn, double yIn, Options options) {
    if (auto skip = begin("turnToPoint")) return *skip;
    // from where the robot is once it's its turn to move: heading 0 is +y, 90 is +x
    const odom::Pose now = odometry->getPose();
    const double bearingDeg = std::atan2(xIn - now.xIn, yIn - now.yIn) * 180.0 / kPi;
    const chassis::ExitConditions exit = exitFor(options, kExitDeg, kChainExitDeg);
    return onChassis([&](auto& drivetrain) {
        return drivetrain.turnToHeading(bearingDeg < 0.0 ? bearingDeg + 360.0 : bearingDeg, exit);
    });
}

motion::MotionResult moveToPoint(double xIn, double yIn, Options options) {
    if (auto skip = begin("moveToPoint")) return *skip;
    const chassis::ExitConditions exit = exitFor(options, kExitIn, kChainExitIn);
    return onChassis(
        [&](auto& drivetrain) { return drivetrain.moveToPoint(xIn, yIn, *odometry, exit); });
}

motion::MotionResult moveToPose(double xIn, double yIn, double headingDeg, Options options) {
    if (auto skip = begin("moveToPose")) return *skip;
    motion::PoseExitConditions exit;
    exit.positionErrorThresholdIn =
        options.exitError > 0.0 ? options.exitError : (options.chain ? kChainExitIn : kExitIn);
    exit.headingErrorThresholdDeg = options.chain ? kChainExitDeg : kExitDeg;
    if (options.timeoutMs > 0) exit.timeoutMs = options.timeoutMs;
    exit.maxVolts = options.maxVolts;
    exit.chain = options.chain;
    return onChassis([&](auto& drivetrain) {
        return drivetrain.moveToPose(xIn, yIn, headingDeg, *odometry, exit);
    });
}

motion::MotionResult strafe(double inches, Options options) {
    if (holonomic == nullptr) return holonomicOnly("strafe");
    if (auto skip = begin("strafe")) return *skip;
    // straight out to the robot's right: (cos h, -sin h) with heading clockwise from +y
    const odom::Pose now = odometry->getPose();
    const double headingRad = now.headingDeg * kPi / 180.0;
    const double xIn = now.xIn + inches * std::cos(headingRad);
    const double yIn = now.yIn - inches * std::sin(headingRad);
    return holonomic->moveToPoint(xIn, yIn, *odometry, exitFor(options, kExitIn, kChainExitIn));
}

motion::MotionResult followPath(std::vector<motion::Waypoint> points, PathOptions options) {
    if (auto skip = begin("followPath")) return *skip;
    const motion::Path path(std::move(points));
    chassis::ExitConditions finalExit{options.chain ? kChainExitIn : kExitIn};
    finalExit.maxVolts = options.maxVolts;
    finalExit.chain = options.chain;
    const motion::PursuitConfig config{.lookaheadIn = options.lookaheadIn,
                                       .cruiseVoltage = options.maxVolts,
                                       .finalExit = finalExit,
                                       .timeoutMs = options.timeoutMs};
    return onChassis(
        [&](auto& drivetrain) { return drivetrain.followPath(path, *odometry, config); });
}

motion::MotionResult followTrajectory(const std::vector<motion::TrajectorySection>& sections,
                                      motion::TrajectoryFollowConfig config) {
    if (holonomic == nullptr) return holonomicOnly("followTrajectory");
    if (auto skip = begin("followTrajectory")) return *skip;
    const Plan& plan = planFor(sections);
    if (!plan.result.ok) return motion::MotionResult{.reason = motion::ExitReason::aborted};
    // the number this trajectory's motion will have: until it starts, the progress is still
    // whichever trajectory ran last
    trajectoryMotion.store(holonomic->trajectoryProgress().motion + 1);
    trajectoryPlan.store(&plan.result);
    return holonomic->followTrajectory(plan.result.trajectory, *odometry, std::move(config));
}

bool planTrajectory(const std::vector<motion::TrajectorySection>& sections) {
    if (holonomic == nullptr) {
        holonomicOnly("planTrajectory");
        return false;
    }
    return planFor(sections).result.ok;
}

void runAsync(std::function<void()> motions) {
    // runAsync() inside runAsync(): the block's own task just runs them
    if (onBlockTask()) {
        motions();
        return;
    }
    // one block at a time, so the waits know whose motions they're watching
    queue().waitUntilDone();
    blockFirst.store(motionCount.load() + 1);
    // as of now, on the routine's task: a mode change or cancel from here on skips the block
    const chassis::CompetitionWatch watch;
    queue().enqueue([motions = std::move(motions), watch] {
        blockTask.store(pros::c::task_get_current());
        blockWatch = &watch;
        if (!watch.changed()) motions();
        blockWatch = nullptr;
    });
    queue().run();
}

bool isMotionRunning() { return queue().isBusy(); }

void waitForMotion() {
    if (!onBlockTask()) queue().waitUntilDone();
}

void cancelMotion() {
    queue().clear();
    chassis::CompetitionWatch::cancelMotions();
}

bool waitUntilTraveled(double inches) {
    return waitWhileRunning("waitUntilTraveled", [inches] {
        return odometry->snapshot().travelledIn - startTravelledIn.load() >= inches;
    });
}

bool waitUntilTurned(double degrees) {
    return waitWhileRunning("waitUntilTurned", [degrees] {
        return odometry->snapshot().turnedDeg - startTurnedDeg.load() >= degrees;
    });
}

bool waitUntilNear(double xIn, double yIn, double radiusIn) {
    return waitWhileRunning("waitUntilNear", [xIn, yIn, radiusIn] {
        const odom::Pose now = odometry->getPose();
        return std::hypot(xIn - now.xIn, yIn - now.yIn) <= radiusIn;
    });
}

bool waitUntilWaypoint(std::size_t waypoint) {
    if (holonomic == nullptr) {
        holonomicOnly("waitUntilWaypoint");
        return false;
    }
    bool warned = false;
    return waitWhileRunning("waitUntilWaypoint", [waypoint, &warned] {
        const motion::TrajectoryResult* plan = trajectoryPlan.load();
        if (plan == nullptr) return false;
        if (waypoint >= plan->waypointIn.size()) {
            if (!warned) {
                SAPPHIRELIB_LOG_WARN("auton", "waitUntilWaypoint: point %u, but the path has %u",
                                     static_cast<unsigned>(waypoint),
                                     static_cast<unsigned>(plan->waypointIn.size()));
                warned = true;
            }
            return false;
        }
        const motion::TrajectoryProgress progress = holonomic->trajectoryProgress();
        // a point at the very end counts despite rounding between the plan's last sample and it
        return progress.motion == trajectoryMotion.load() &&
               progress.distanceIn >= plan->waypointIn[waypoint] - 1e-6;
    });
}

bool resetFront() { return resetFacing(0.0, "resetFront"); }

bool resetBack() { return resetFacing(180.0, "resetBack"); }

bool resetLeft() { return resetFacing(270.0, "resetLeft"); }

bool resetRight() { return resetFacing(90.0, "resetRight"); }

void setMcl(bool on) {
    if (localizer == nullptr) {
        SAPPHIRELIB_LOG_ERROR("auton", "setMcl: no localizer - pass one to auton::use()");
        return;
    }
    localizer->setCorrectionEnabled(on);
}

} // namespace sapphirelib::auton
