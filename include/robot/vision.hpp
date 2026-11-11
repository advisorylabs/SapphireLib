/**
 * \file robot/vision.hpp
 *
 * The AI Vision sensor (config.hpp's kVision), and what autonomous can do
 * with it: steer a run onto a yellow Pin. No routine does now: the Triple
 * Pin's runs (autons.cpp's ontoPin()) hand their last third to an approach
 * without a refiner, aimed where the plan has the Pin.
 *
 * Odometry drifts, and by the Triple Pin's second Pin it was 3.5in off and by
 * the third more. A trajectory drives to where odometry says the Pin is; this
 * sees where it is. Part way into a run onto a Pin, the trajectory can hand
 * off (TrajectoryFollowConfig::until) to an approach
 * (sapphirelib/motion/target_approach.hpp) that turns the robot to face the
 * Pin and drives at it at a steady speed, to an end a little past it, and a
 * PinSight moves the approach's target onto the Pin the camera sees, every
 * frame. The robot turns to keep the Pin in the middle of the image (on the
 * claw, for a camera off the centerline, see config.hpp's kVision*); it
 * never strafes for it.
 *
 * What counts as the Pin: a detection the sensor's own AI model labels a
 * yellow/yellow Pin (its AI classifications: the class vision.cpp finds by
 * name, kPinClassNames), sure enough (kMinScore), at least kMinPinHeightPx
 * tall and taller than wide (standing, not knocked over). At first, the
 * tallest such one (the nearest) within a wide gate of where the target is;
 * after that, the one nearest where the last frame saw it, within a narrow
 * gate, so another yellow/yellow Pin (a wall group's, the stack on the goal)
 * is never taken for it. Seen or not, the target stays in odometry's frame,
 * so the approach drives on toward wherever it was last put when the camera
 * loses it.
 *
 * Driver control never reads the sensor: autoClaw there is the claw's own
 * distance sensor, as it always was (macros.hpp).
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "sapphirelib/motion/target_approach.hpp"
#include "sapphirelib/odom/pose.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace robot::vision {

/// Sets the sensor up: its AI model's game elements on, colors and AprilTags
/// off, and finds the yellow/yellow Pin's class, logging every class the
/// sensor names. initDevices() calls it. With nothing on kVision, or no class
/// named as the Pin, it logs a warning and the runs onto the Pins steer by
/// odometry (PinSight looks for the class again, in case the sensor was only
/// slow to report its names).
void initialize();

/// Logs every frame a PinSight looks at (channel "vision", see
/// docs/TELEMETRY_FORMAT.md). Call once from startTelemetry(), before the
/// logger starts. Without it nothing is logged and nothing else changes.
void attachTelemetry(sapphirelib::telemetry::Logger& logger);

/// One run's look at one Pin, from where odometry has it to where the camera
/// sees it. Make one per run, shared with the approach (steerBy()), which
/// calls refine() on the motion's task every tick; pin() and the rest are
/// safe from any task.
class PinSight {
public:
    /// `pin`: where the plan has the Pin, in odometry's frame.
    explicit PinSight(sapphirelib::motion::FieldPoint pin);

    /// The approach's refiner: looks at the camera's latest frame and, if it
    /// shows the Pin, moves `target` toward it (and this sight's pin()).
    void refine(const sapphirelib::odom::Pose& pose, sapphirelib::motion::FieldPoint& target);

    /// Where the Pin is now, as best known: the plan's until the camera has
    /// seen it.
    sapphirelib::motion::FieldPoint pin() const;

    /// How far the camera has moved the Pin from the plan's, in inches.
    double movedIn() const;

    /// Whether the camera has seen the Pin yet.
    bool locked() const { return locked_.load(); }

    /// How many frames showed it the Pin.
    std::uint32_t frames() const { return frames_.load(); }

private:
    // the pose a few ticks back, for a frame the sensor took that long ago
    struct Stamped {
        std::uint32_t ms = 0;
        sapphirelib::odom::Pose pose;
    };
    static constexpr std::size_t kHistory = 16;

    sapphirelib::odom::Pose poseAt(std::uint32_t ms) const;

    const sapphirelib::motion::FieldPoint plan_;
    std::atomic<double> pinX_;
    std::atomic<double> pinY_;
    std::atomic<bool> locked_{false};
    std::atomic<std::uint32_t> frames_{0};
    // the motion's task only, from here down
    Stamped history_[kHistory];
    std::size_t historyNext_ = 0;
    std::size_t historyCount_ = 0;
    // the last frame's yellow detections, so the same frame isn't looked at twice
    std::uint32_t lastFrameKey_ = 0;
    // what the latest frame showed of the Pin: none if it didn't
    struct Sighting {
        double bearingDeg = 0.0;           // off the camera's axis
        sapphirelib::odom::Pose seenFrom;  // where the robot was as the sensor took it
    };
    std::optional<Sighting> sighting_;
    // where the last frame that showed the Pin put it (at the target's range), and how many new
    // frames since have missed it: what the next frame looks for it around
    std::optional<sapphirelib::motion::FieldPoint> seenPin_;
    std::uint32_t missedFrames_ = 0;
    std::uint32_t lastRefineMs_ = 0;
    bool warned_ = false;
    // whether this run has logged the class it looks for (vision,pin_class)
    bool classLogged_ = false;
};

/// The approach's refiner for `sight`: auton::approach(x, y, config,
/// vision::steerBy(sight)). Holds `sight`, so it lives as long as the motion
/// does, even one that outlives the routine that made it.
sapphirelib::motion::ApproachRefiner steerBy(std::shared_ptr<PinSight> sight);

} // namespace robot::vision
