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
    Odometry(Sensors sensors, OdometryConfig config, Pose startPose = Pose{});

    /// Advances the pose estimate by one update. Call this yourself on your
    /// own loop, or use startTask() to run it on a background PROS task.
    void update();

    /// Thread-safe pose read.
    Pose getPose() const;

    /// Thread-safe pose overwrite — e.g. to seed a known starting
    /// position/heading before autonomous.
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
    /// Call at most once per Odometry instance.
    void startTask(std::uint32_t periodMs = 10);

private:
    Sensors sensors_;
    mutable pros::MutexVar<OdometryConfig> config_;
    mutable pros::MutexVar<Pose> pose_;
    double lastHeadingDeg_;
    double lastVerticalIn_ = 0.0;
    double lastHorizontalIn_ = 0.0;
    std::unique_ptr<pros::Task> task_;
};

} // namespace sapphirelib::odom
