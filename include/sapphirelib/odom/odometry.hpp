/**
 * \file sapphirelib/odom/odometry.hpp
 *
 * Background pose tracker built on IMU heading plus zero, one, or two
 * tracking wheels — see the class comment for how the four supported sensor
 * combinations map onto Odometry::Sensors.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "pros/rtos.hpp"
#include "sapphirelib/odom/odometry_config.hpp"
#include "sapphirelib/odom/pose.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::odom {

/// Supports all four sensor combinations from the roadmap purely through
/// which TrackingWheel pointers are passed in via Sensors:
///   - IMU + drive encoders only: vertical = a MotorGroupTrackingWheel, horizontal = nullptr
///   - IMU + vertical wheel:      vertical = a RotationTrackingWheel,   horizontal = nullptr
///   - IMU + horizontal wheel:    vertical = a MotorGroupTrackingWheel (forward fallback), horizontal = a RotationTrackingWheel
///   - IMU + both wheels:         vertical/horizontal = a RotationTrackingWheel each
/// See computeOdometryDelta() (odometry_math.hpp) for the tracking algorithm
/// itself.
class Odometry {
public:
    struct Sensors {
        /// Required — every config needs a heading source. Pass a
        /// sensors::Imu (not a raw pros::Imu) so heading readings get the
        /// same multi-turn drift correction as the rest of SapphireLib — if
        /// you're also using a TankDrivetrain/HolonomicDrivetrain, share its
        /// imu() rather than constructing a second sensors::Imu on the same
        /// port.
        sensors::Imu* imu;

        /// Forward/back distance source. nullptr means no forward tracking
        /// at all (not a supported config — pass a MotorGroupTrackingWheel
        /// at minimum).
        TrackingWheel* vertical = nullptr;

        /// Left/right distance source. nullptr means no lateral tracking
        /// (expected for the two configs without a horizontal wheel).
        TrackingWheel* horizontal = nullptr;
    };

    /// Holds a pros::Mutex internally (via pros::MutexVar), which is
    /// non-copyable/non-movable, so — like MotorGroup-based classes
    /// elsewhere in SapphireLib — Odometry can only be constructed in
    /// place, never passed by value.
    ///
    /// Starts at `startPose`, heading included: it sets the Imu's field
    /// heading to `startPose.headingDeg` (see setPose()), so construction
    /// re-frames a shared drivetrain Imu too. The default Pose{} makes
    /// wherever the chassis faces now heading 0.
    Odometry(Sensors sensors, OdometryConfig config, Pose startPose = Pose{});

    /// Advances the pose estimate by one update. Call this yourself on your
    /// own loop, or use startTask() to run it on a background PROS task.
    void update();

    /// Thread-safe pose read.
    Pose getPose() const;

    /// Thread-safe pose overwrite — e.g. to seed a known starting
    /// position/heading at the top of an autonomous routine.
    ///
    /// The heading is applied through the Imu (sensors::Imu::setHeadingDeg()),
    /// not just written into the pose, because a pose heading the IMU didn't
    /// agree with would be overwritten by the next update(). With odometry
    /// sharing the drivetrain's imu() (the recommended wiring), that one call
    /// puts the whole robot in the new frame: turnToHeading(270) now means
    /// the field's 270, moveToPose() compares against the same heading the
    /// pose reports, and x/y integrate along the new axes. What it leaves
    /// alone, on purpose: HolonomicDrivetrain's field-centric "forward" and
    /// its driver heading hold, which track physical directions (see
    /// resetFieldHeading()), and every getCumulativeHeadingDeg() reader.
    ///
    /// Travel from before the reset never lands in the new pose, even with
    /// the chassis moving: this takes the wheel and rotation readings the next
    /// update() measures from, and an update() already in flight when it
    /// lands is discarded. The heading is stored wrapped to 0-360.
    void setPose(Pose pose);

    /// Thread-safe config read — e.g. to preserve one axis's offset while
    /// recalibrating the other.
    OdometryConfig getConfig() const;

    /// Thread-safe config overwrite — e.g. to apply a freshly calibrated
    /// tracking wheel offset (see odom::calibrateTrackingWheelOffsetIn() /
    /// gui::OdometryPage::enableOffsetCalibration()) without reconstructing
    /// Odometry. Takes effect on the next update().
    void setConfig(OdometryConfig config);

    /// Starts a background pros::Task that calls update() every `periodMs`.
    /// Only the first call starts one; later calls do nothing (a second task
    /// would integrate every wheel delta twice). The task runs for the rest
    /// of the program, so this Odometry must too — make it static.
    void startTask(std::uint32_t periodMs = 10);

private:
    Sensors sensors_;
    mutable pros::MutexVar<OdometryConfig> config_;
    mutable pros::MutexVar<Pose> pose_;

    /// The previous update's reading in the Imu's *rotation* frame
    /// (getCumulativeHeadingDeg()), which setPose() never shifts. update()
    /// adds the current heading offset to it, so the previous heading is
    /// always expressed in the same frame as the current one — a re-frame
    /// between two updates then isn't mistaken for a turn.
    double lastRotationDeg_;

    /// Bumped by every setPose(), so an update() that read its sensors before
    /// the reset knows to discard its result instead of adding pre-reset
    /// motion to the new pose.
    std::atomic<std::uint32_t> poseGeneration_{0};

    /// The generation the last update() started from — update-task only.
    /// When it differs from poseGeneration_, a setPose() happened since, and
    /// the next update measures from the reset readings below instead of its
    /// own previous ones.
    std::uint32_t seenGeneration_ = 0;

    /// Rotation and wheel readings setPose() took at the moment of the reset.
    std::atomic<double> resetRotationDeg_{0.0};
    std::atomic<double> resetVerticalIn_{0.0};
    std::atomic<double> resetHorizontalIn_{0.0};
    double lastVerticalIn_ = 0.0;
    double lastHorizontalIn_ = 0.0;
    std::unique_ptr<pros::Task> task_;
};

} // namespace sapphirelib::odom
