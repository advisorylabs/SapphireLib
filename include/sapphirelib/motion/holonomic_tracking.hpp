#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/motion/trajectory.hpp"
#include "sapphirelib/motion/trajectory_config.hpp"
#include "sapphirelib/odom/pose.hpp"

namespace sapphirelib::motion {

/**
 * @brief How far a robot is from a trajectory's reference, in the robot's frame
 *
 * Each error is the reference minus the robot, so a positive error is a push the same way
 */
struct TrackingErrors {
    /** how far the reference is ahead of the robot, in inches */
    double forwardIn = 0.0;
    /** how far the reference is to the robot's right, in inches */
    double strafeIn = 0.0;
    /** reference heading minus the robot's, in degrees, wrapped to (-180, 180] */
    double headingDeg = 0.0;
    /** reference velocity minus the robot's, ahead of the robot, in inches per second */
    double forwardRateInPerS = 0.0;
    /** reference velocity minus the robot's, to the robot's right, in inches per second */
    double strafeRateInPerS = 0.0;
    /** reference turn rate minus the robot's, in degrees per second */
    double headingRateDegPerS = 0.0;
    /** how far the robot is behind the reference along the path's direction, in inches */
    double alongTrackIn = 0.0;
    /** how far the robot is to the right of the path's direction, in inches */
    double crossTrackIn = 0.0;
};

/**
 * @brief Work out how far the robot is from the reference
 *
 * @param reference the planned state
 * @param pose the robot's pose (odometry's, with any localizer correction)
 * @param velocity the robot's measured field velocity (Odometry::Snapshot::rawVelocity)
 * @return TrackingErrors the errors, in the robot's frame
 */
TrackingErrors trackingErrors(const TrajectorySample& reference, const odom::Pose& pose,
                              const odom::Velocity& velocity);

/**
 * @brief One tick of a trajectory's clock
 */
struct ClockStep {
    /** the plan's time after this tick, in seconds */
    double tS = 0.0;
    /** whether the clock held still because the robot was too far behind */
    bool paused = false;
};

/**
 * @brief Advance the plan's clock one tick, holding it while the robot falls too far behind
 *
 * @param tS the plan's time, in seconds
 * @param dtS time since the last tick, in seconds
 * @param alongTrackIn how far the robot is behind the reference, in inches (TrackingErrors)
 * @param maxLagIn how far behind it may fall before the clock holds, in inches. 0 never holds
 * @return ClockStep the new time, and whether it held
 */
ClockStep advanceTrajectoryClock(double tS, double dtS, double alongTrackIn, double maxLagIn);

/**
 * @brief What became of a chain of trajectories on one tick (see TrajectoryFollowConfig::next)
 */
enum class ChainEvent : std::uint8_t {
    /** nothing: no junction this tick */
    none,
    /** the plan carried on into the next trajectory */
    continued,
    /** a trajectory ended moving and next gave nothing to carry on into: settling from speed */
    noNext,
    /**
     * next gave a trajectory that doesn't start where the ending one ends (not planned with
     * TrajectoryOptions::from): refused, and settling from speed
     */
    mismatched,
};

/**
 * @brief Everything one tick of followTrajectory() worked out, for telemetry and tools
 */
struct TrackingTick {
    /**
     * the plan's clock, in seconds. Stops while paused, so it can lag the time since the start.
     * Across a chain it runs on from one trajectory into the next, so it counts from the first
     * one's start, not reference.tS
     */
    double referenceTimeS = 0.0;
    /** where the plan had the robot this tick. At rest once holding */
    TrajectorySample reference;
    /** the feedforward volts per axis, before wheel mixing. 0 once holding */
    chassis::AxisVolts feedforward;
    /** feedforward plus the tracking PIDs: what was commanded, before wheel mixing */
    chassis::AxisVolts volts;
    /** how far the robot was from the reference */
    TrackingErrors errors;
    /** whether the clock held this tick, waiting for the robot */
    bool paused = false;
    /** whether the plan has ended and the robot is settling on its last point */
    bool holding = false;
    /**
     * the section the reference is in. Across a chain, counted on from the trajectories before:
     * the second trajectory's first section follows the first one's last
     */
    std::size_t section = 0;
    /** which trajectory of a chain the reference is on: 0 for the first */
    std::size_t piece = 0;
    /** what became of the chain this tick */
    ChainEvent chain = ChainEvent::none;
};

/**
 * @brief Receives a trajectory and every tick of following it. See
 * HolonomicDrivetrain::setTrajectoryObserver()
 */
class TrajectoryObserver {
public:
    virtual ~TrajectoryObserver() = default;

    /**
     * @brief Called once as a trajectory starts, on the motion's task
     *
     * @note runs inside a control loop, so it must not block or allocate
     *
     * @param trajectory the plan about to be followed
     */
    virtual void onTrajectoryStart(const Trajectory& /*trajectory*/) {}

    /**
     * @brief Called as the plan carries on into the next trajectory of a chain
     * (TrajectoryFollowConfig::next), on the motion's task, just before that tick's
     * onTrackingTick()
     *
     * @note runs inside a control loop, so it must not block or allocate
     *
     * @param trajectory the trajectory the plan carries on into
     * @param startTimeS when it starts on the plan's clock (TrackingTick::referenceTimeS), in
     * seconds: its own times are from 0
     * @param firstSection the number its first section has in TrackingTick::section
     */
    virtual void onTrajectoryContinue(const Trajectory& /*trajectory*/, double /*startTimeS*/,
                                      std::size_t /*firstSection*/) {}

    /**
     * @brief Called every tick of the motion, on the motion's task
     *
     * @note runs inside a control loop, so it must not block or allocate
     *
     * @param tick what the tick worked out
     */
    virtual void onTrackingTick(const TrackingTick& tick) = 0;
};

/**
 * @brief How far along a trajectory the follower is
 *
 * @b Example
 * @code {.cpp}
 * // start the intake as the robot enters the trajectory's second section
 * const std::uint32_t rush = drivetrain().trajectoryProgress().motion + 1; // the next to start
 * queue.enqueue([] { drivetrain().followTrajectory(rushPlan.trajectory); });
 * queue.run();
 * sapphirelib::waitUntil([rush] {
 *     const auto p = drivetrain().trajectoryProgress();
 *     return p.motion == rush && p.section >= 1;
 * }, 3000);
 * @endcode
 */
struct TrajectoryProgress {
    /** whether a trajectory is being followed */
    bool running = false;
    /**
     * the section the reference is in. A section starts at its first point. Across a chain,
     * counted on from the trajectories before (see TrackingTick::section)
     */
    std::size_t section = 0;
    /** how far along the path the reference is, in inches, from the start of a chain */
    double distanceIn = 0.0;
    /** the plan's clock, in seconds, from the start of a chain */
    double timeS = 0.0;
    /** whether the plan is waiting for the robot */
    bool paused = false;
    /** which trajectory of a chain the plan is on: 0 for the first, 1 once it carries on */
    std::size_t piece = 0;
    /**
     * which followTrajectory() call this is from: they're numbered from 1 as they start, 0 before
     * any has. Until a queued motion's first tick the progress is still the last one's, so a
     * routine waiting on its own trajectory checks this too (HolonomicDrivetrain fills it in)
     */
    std::uint32_t motion = 0;
};

/**
 * @brief followTrajectory()'s control loop, without the hardware
 *
 * Each tick: advance the plan's clock (holding it while the robot is more than maxLagIn behind),
 * sample the reference, and command trajectoryFeedforward() plus a PD per axis on the robot-frame
 * position error, with the velocity error as the D term (PID::updateWithRate()). After the plan
 * ends it holds the last point on the PIDs alone until both errors settle, or the overtime runs out
 *
 * A trajectory that ends moving carries on into the one TrajectoryFollowConfig::next picks, with
 * the clock running on, the PIDs untouched, and the overtime allowance growing by the new
 * trajectory's duration
 *
 * The PIDs are borrowed for the motion; it resets them in begin()
 */
class TrajectoryFollower {
public:
    /**
     * @brief Construct a new TrajectoryFollower
     *
     * @param forward the forward tracking PID: volts per inch of error
     * @param strafe the strafe tracking PID: volts per inch of error
     * @param turn the turn tracking PID: volts per degree of error
     */
    TrajectoryFollower(PID& forward, PID& strafe, PID& turn);

    /**
     * @brief Start following a trajectory
     *
     * @param trajectory the plan. Must outlive the motion, and must not be empty
     * @param models the chassis's axis models, for the feedforward
     * @param config thresholds, overtime, and maxLagIn
     * @param nowMs the time, in milliseconds
     */
    void begin(const Trajectory& trajectory, const chassis::HolonomicAxisModels& models,
               TrajectoryFollowConfig config, std::uint32_t nowMs);

    /**
     * @brief Run one tick
     *
     * @param pose the robot's pose
     * @param velocity the robot's measured field velocity
     * @param nowMs the time, in milliseconds
     * @return TrackingTick what to command (volts), and why
     */
    TrackingTick update(const odom::Pose& pose, const odom::Velocity& velocity,
                        std::uint32_t nowMs);

    /**
     * @brief Get why the motion should stop: running until it should
     */
    ExitReason reason() const;

    /**
     * @brief Get how the motion went: its reason, the distance to the last point, and how long
     */
    MotionResult result() const;

    /**
     * @brief Get how far along the plan the follower is
     */
    TrajectoryProgress progress() const;

    /**
     * @brief Get the trajectory being followed: the one begin() was given, or the one a chain
     * carried on into
     */
    const Trajectory& trajectory() const;

    /**
     * @brief Get when the trajectory being followed starts on the plan's clock
     * (TrackingTick::referenceTimeS), in seconds. 0 for the first of a chain
     */
    double pieceStartS() const;

    /**
     * @brief Get the number the first section of the trajectory being followed has in
     * TrackingTick::section. 0 for the first of a chain
     */
    std::size_t pieceFirstSection() const;

private:
    PID& forward_;
    PID& strafe_;
    PID& turn_;
    const Trajectory* trajectory_ = nullptr;
    chassis::HolonomicAxisModels models_;
    TrajectoryFollowConfig config_;
    std::optional<ExitTracker> tracker_;
    ExitReason reason_ = ExitReason::running;
    // the plan's clock on the trajectory being followed, from its own start
    double referenceTimeS_ = 0.0;
    double lastAlongTrackIn_ = 0.0;
    std::uint32_t lastMs_ = 0;
    bool started_ = false;
    double finalErrorIn_ = 0.0;
    TrajectoryProgress progress_;
    // a chain: which trajectory, and where it starts in the chain's time, sections and distance
    std::size_t piece_ = 0;
    double pieceStartS_ = 0.0;
    std::size_t pieceFirstSection_ = 0;
    double pieceStartIn_ = 0.0;
    // next gave nothing to carry on into, so the plan ends where it is
    bool chainEnded_ = false;
};

} // namespace sapphirelib::motion
