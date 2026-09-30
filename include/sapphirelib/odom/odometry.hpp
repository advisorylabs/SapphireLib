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

/**
 * @brief Tracks the robot's pose from the IMU and tracking wheels
 *
 * Which sensors it uses depends on which tracking wheels you pass in:
 *   - IMU + drive encoders: vertical = MotorGroupTrackingWheel, horizontal = nullptr
 *   - IMU + vertical wheel: vertical = RotationTrackingWheel, horizontal = nullptr
 *   - IMU + horizontal wheel: vertical = MotorGroupTrackingWheel,
 *     horizontal = RotationTrackingWheel
 *   - IMU + both wheels: a RotationTrackingWheel for each
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::odom::Odometry& odometry() {
 *     static sapphirelib::odom::RotationTrackingWheel vertical(11, 2.0);
 *     static sapphirelib::odom::Odometry instance(
 *         {.imu = &drivetrain().imu(), .vertical = &vertical},
 *         {.verticalOffsetIn = 0.5});
 *     return instance;
 * }
 * @endcode
 */
class Odometry {
public:
    /**
     * @brief The sensors odometry reads from
     */
    struct Sensors {
        /**
         * heading source. Required. Share the drivetrain's imu() instead of opening a second one
         * on the same port
         */
        sensors::Imu* imu;

        /** forward/backward distance source. Required; use a MotorGroupTrackingWheel at least */
        TrackingWheel* vertical = nullptr;

        /** left/right distance source. nullptr for no sideways tracking */
        TrackingWheel* horizontal = nullptr;
    };

    /**
     * @brief Construct a new Odometry
     *
     * Sets the IMU's field heading to startPose.headingDeg (see setPose()), which re-frames a
     * shared drivetrain IMU too
     *
     * @param sensors the IMU and tracking wheels
     * @param config tracking wheel offsets
     * @param startPose the starting pose. Pose{} by default, which makes the way the robot faces
     * now heading 0
     */
    Odometry(Sensors sensors, OdometryConfig config, Pose startPose = Pose{});

    /**
     * @brief Update the pose once. Call this on your own loop, or use startTask()
     */
    void update();

    /**
     * @brief Get the pose. Thread-safe
     *
     * @return Pose x and y in inches, heading in degrees
     *
     * @b Example
     * @code {.cpp}
     * sapphirelib::odom::Pose pose = odometry().getPose();
     * printf("x: %f, y: %f, heading: %f\n", pose.xIn, pose.yIn, pose.headingDeg);
     * @endcode
     */
    Pose getPose() const;

    /**
     * @brief Set the pose. Thread-safe
     *
     * The heading is set through the IMU, so with odometry sharing the drivetrain's IMU, the whole
     * robot moves to the new frame: turnToHeading(270) means the field's 270. Field-centric forward
     * and driver heading hold aren't affected, since they track physical directions. Travel from
     * before the reset never ends up in the new pose
     *
     * @param pose the new pose. The heading is wrapped to 0-360
     *
     * @b Example
     * @code {.cpp}
     * void autonomous() {
     *     // the robot starts at (-48, -60), facing 90 degrees
     *     odometry().setPose({-48, -60, 90});
     * }
     * @endcode
     */
    void setPose(Pose pose);

    /**
     * @brief Get the tracking wheel offsets. Thread-safe
     */
    OdometryConfig getConfig() const;

    /**
     * @brief Set the tracking wheel offsets, e.g. after calibrating them. Thread-safe
     *
     * Takes effect on the next update(). See calibrateTrackingWheelOffsetIn()
     *
     * @param config the new offsets
     */
    void setConfig(OdometryConfig config);

    /**
     * @brief Start a task that calls update() every periodMs
     *
     * Only the first call starts a task; a second would count every movement twice. The task runs
     * for the rest of the program, so the Odometry must be static
     *
     * @param periodMs update period, in milliseconds. 10 by default
     */
    void startTask(std::uint32_t periodMs = 10);

private:
    Sensors sensors_;
    mutable pros::MutexVar<OdometryConfig> config_;
    mutable pros::MutexVar<Pose> pose_;

    // the last update's rotation reading (getCumulativeHeadingDeg()), which setPose() never
    // shifts, so a re-frame between two updates isn't mistaken for a turn
    double lastRotationDeg_;

    // bumped by setPose(), so an update that read its sensors before the reset throws its result
    // away
    std::atomic<std::uint32_t> poseGeneration_{0};

    // the generation the last update started from (update task only). When it differs from
    // poseGeneration_, the next update measures from the reset readings below
    std::uint32_t seenGeneration_ = 0;

    // readings setPose() took at the reset
    std::atomic<double> resetRotationDeg_{0.0};
    std::atomic<double> resetVerticalIn_{0.0};
    std::atomic<double> resetHorizontalIn_{0.0};
    double lastVerticalIn_ = 0.0;
    double lastHorizontalIn_ = 0.0;
    std::unique_ptr<pros::Task> task_;
};

} // namespace sapphirelib::odom
