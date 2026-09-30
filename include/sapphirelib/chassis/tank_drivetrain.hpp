#pragma once

#include <atomic>
#include <cstdint>
#include <initializer_list>

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/motion/motion_config.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::chassis {

/**
 * @brief Tank (differential) drivetrain
 *
 * Works with just the IMU and drive encoders. For mecanum or X-drive, see HolonomicDrivetrain
 *
 * Every blocking motion returns a motion::MotionResult and logs a motion start/end event to the
 * running telemetry::Logger, if there is one
 */
class TankDrivetrain {
public:
    /**
     * @brief Construct a new TankDrivetrain
     *
     * @note blocks for about 2-3 seconds while the IMU calibrates, so build it from initialize(),
     * not at namespace scope
     *
     * @param leftPorts left motor ports. Negative reverses a motor
     * @param rightPorts right motor ports
     * @param gearset the drive motors' cartridge
     * @param imuPort IMU port
     * @param config wheel size, gear ratio, and driveDistance() heading correction
     * @param drivePIDConfig PID settings for distance motions
     * @param turnPIDConfig PID settings for turns
     * @param imuHeadingScale correction for the IMU's multi-turn drift. 1 by default; see
     * sensors::calibrateHeadingScale()
     *
     * @b Example
     * @code {.cpp}
     * sapphirelib::chassis::TankDrivetrain& drivetrain() {
     *     static sapphirelib::chassis::TankDrivetrain instance(
     *         {1, -2, 3}, // left motors
     *         {-4, 5, -6}, // right motors
     *         sapphirelib::chassis::Gearset::blue,
     *         10, // IMU port
     *         {.wheelDiameterIn = 3.25, .externalGearRatio = 1.0, .headingCorrectionKP = 0.4},
     *         {.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},   // drive
     *         {.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0}); // turn
     *     return instance;
     * }
     * @endcode
     */
    TankDrivetrain(std::initializer_list<std::int8_t> leftPorts,
                   std::initializer_list<std::int8_t> rightPorts, Gearset gearset,
                   std::uint8_t imuPort, DrivetrainConfig config, PID::Config drivePIDConfig,
                   PID::Config turnPIDConfig, double imuHeadingScale = 1.0);

    /**
     * @brief Drive with tank controls
     *
     * @param left left side, -1 to 1
     * @param right right side, -1 to 1
     *
     * @b Example
     * @code {.cpp}
     * drivetrain().tank(master.get_analog(ANALOG_LEFT_Y) / 127.0,
     *                   master.get_analog(ANALOG_RIGHT_Y) / 127.0);
     * @endcode
     */
    void tank(double left, double right);

    /**
     * @brief Drive with arcade controls
     *
     * @param throttle forward/backward, -1 to 1
     * @param turn rotation, -1 to 1
     *
     * @b Example
     * @code {.cpp}
     * drivetrain().arcade(master.get_analog(ANALOG_LEFT_Y) / 127.0,
     *                     master.get_analog(ANALOG_RIGHT_X) / 127.0);
     * @endcode
     */
    void arcade(double throttle, double turn);

    /**
     * @brief Get the forward and turn voltage last commanded
     *
     * forward = (left + right) / 2, turn = (left - right) / 2, strafe is always 0. Covers driver
     * control and every autonomous motion. Before each motor's own 12V clamp. All zero after stop()
     *
     * @note safe to call from any task. The values are separate atomics, so a read can mix two
     * consecutive ticks
     *
     * @return AxisVolts forward, strafe, and turn volts
     */
    AxisVolts appliedAxisVolts() const;

    /**
     * @brief Drive straight forward or backward a distance
     *
     * Uses the drive encoders, with IMU heading correction. Measures from where the encoders read
     * when it starts, without taring them
     *
     * @param inches distance to drive, in inches. Negative drives backward
     * @param exit when the motion ends. errorThreshold is in inches
     * @return motion::MotionResult how the motion ended
     *
     * @b Example
     * @code {.cpp}
     * // drive forward 24 inches
     * drivetrain().driveDistance(24);
     * // back up 12 inches, giving up after 1.5 seconds
     * drivetrain().driveDistance(-12, {.timeoutMs = 1500});
     * @endcode
     */
    motion::MotionResult driveDistance(double inches, ExitConditions exit = ExitConditions{1.0});

    /**
     * @brief Turn in place to a heading
     *
     * @param headingDeg target field heading, 0-360 degrees
     * @param exit when the motion ends. errorThreshold is in degrees, 2 by default
     * @return motion::MotionResult how the motion ended
     *
     * @b Example
     * @code {.cpp}
     * // turn to face 90 degrees
     * drivetrain().turnToHeading(90);
     * @endcode
     */
    motion::MotionResult turnToHeading(double headingDeg,
                                       ExitConditions exit = ExitConditions{2.0});

    /**
     * @brief Set the odometry used by the motions that don't take one
     *
     * @note set it once in initialize(), before any motion runs. The odometry must outlive the
     * drivetrain
     *
     * @param odometry the odometry. nullptr unsets it
     */
    void setOdometry(const odom::Odometry* odometry);

    /**
     * @brief Drive to a point on the field
     *
     * Turns toward the point while driving, and reverses instead of spinning around when the
     * point is more than 90 degrees behind. Doesn't control the final heading; see moveToPose()
     *
     * @param xIn target x, in inches
     * @param yIn target y, in inches
     * @param odometry where to read the pose from
     * @param exit when the motion ends. errorThreshold is the distance to the point, in inches
     * @return motion::MotionResult how the motion ended
     */
    motion::MotionResult moveToPoint(double xIn, double yIn, const odom::Odometry& odometry,
                                     ExitConditions exit = ExitConditions{1.0});

    /**
     * @brief Drive to a point on the field, using the odometry from setOdometry()
     *
     * @param xIn target x, in inches
     * @param yIn target y, in inches
     * @param exit when the motion ends. errorThreshold is the distance to the point, in inches
     * @return motion::MotionResult how the motion ended. Aborted if no odometry was set
     *
     * @b Example
     * @code {.cpp}
     * // drive to (24, 24), and stop the routine if it didn't get there
     * if (!drivetrain().moveToPoint(24, 24).settled()) return;
     * @endcode
     */
    motion::MotionResult moveToPoint(double xIn, double yIn,
                                     ExitConditions exit = ExitConditions{1.0});

    /**
     * @brief Drive to a pose on the field with the boomerang controller
     *
     * Aims at a carrot point behind the target along its heading (see
     * motion::PoseExitConditions::boomerangLeadPct), so the chassis curves into the final heading
     * instead of turning in place at the end
     *
     * @param xIn target x, in inches
     * @param yIn target y, in inches
     * @param headingDeg target heading, in degrees
     * @param odometry where to read the pose from
     * @param exit when the motion ends
     * @return motion::MotionResult how the motion ended. finalError is the distance to the point
     */
    motion::MotionResult moveToPose(double xIn, double yIn, double headingDeg,
                                    const odom::Odometry& odometry,
                                    motion::PoseExitConditions exit = {});

    /**
     * @brief Drive to a pose on the field, using the odometry from setOdometry()
     *
     * @param xIn target x, in inches
     * @param yIn target y, in inches
     * @param headingDeg target heading, in degrees
     * @param exit when the motion ends
     * @return motion::MotionResult how the motion ended. Aborted if no odometry was set
     *
     * @b Example
     * @code {.cpp}
     * // curve into (24, 48) facing 90 degrees, with a tighter curve than the default
     * drivetrain().moveToPose(24, 48, 90, {.boomerangLeadPct = 0.4});
     * @endcode
     */
    motion::MotionResult moveToPose(double xIn, double yIn, double headingDeg,
                                    motion::PoseExitConditions exit = {});

    /**
     * @brief Follow a path with pure pursuit
     *
     * Steers toward a point lookaheadIn ahead on the path at a constant voltage, then switches to
     * moveToPoint() for the last waypoint so it slows down and settles there. The path may end
     * where it starts, for a lap
     *
     * @param path the waypoints to follow
     * @param odometry where to read the pose from
     * @param config lookahead, speed, and exit settings
     * @return motion::MotionResult how the final approach ended. Timed out if the pursuit ran past
     * config.timeoutMs; aborted for an empty path
     */
    motion::MotionResult followPath(const motion::Path& path, const odom::Odometry& odometry,
                                    motion::PursuitConfig config);

    /**
     * @brief Follow a path with pure pursuit, using the odometry from setOdometry()
     *
     * @param path the waypoints to follow
     * @param config lookahead, speed, and exit settings
     * @return motion::MotionResult how the motion ended. Aborted if no odometry was set
     *
     * @b Example
     * @code {.cpp}
     * const sapphirelib::motion::Path path({{0, 0}, {0, 24}, {24, 48}});
     * drivetrain().followPath(path, {.lookaheadIn = 8.0, .cruiseVoltage = 8.0});
     * @endcode
     */
    motion::MotionResult followPath(const motion::Path& path, motion::PursuitConfig config);

    /**
     * @brief Stop the drivetrain
     *
     * @param mode brake mode to stop with. brake by default
     */
    void stop(BrakeMode mode = BrakeMode::brake);

    /**
     * @brief Get the drivetrain's IMU, to share with an odom::Odometry instead of opening a second
     * one on the same port
     */
    sensors::Imu& imu();

    /**
     * @brief Get the PID used by distance motions, for live tuning (see gui::PidTunerPage)
     */
    PID& drivePID();

    /**
     * @brief Get the PID used by turns
     */
    PID& turnPID();

private:
    MotorGroup left_;
    MotorGroup right_;
    sensors::Imu imu_;
    DrivetrainConfig config_;
    PID drivePID_;
    PID turnPID_;

    // see setOdometry()
    const odom::Odometry* odometry_ = nullptr;

    // see appliedAxisVolts(). Atomics, not a mutex, since PROS deletes competition tasks on every
    // mode change and a mutex held at that moment would stay locked
    std::atomic<double> appliedForwardVolts_{0.0};
    std::atomic<double> appliedTurnVolts_{0.0};

    double degreesToInches(double degrees) const;

    // the IMU's rotation since construction, wrapped to 0-360. setPose() never shifts it, so
    // driveDistance()'s heading correction uses it
    double rotationHeadingDeg();

    // the one place the drive motors get a voltage, so appliedAxisVolts() sees every command
    void setSideVoltages(double leftVolts, double rightVolts);

    // mean of the two sides' encoders, in motor degrees
    double sideAverageDegrees() const;

    // moveToPoint()'s loop. Logs no events of its own, so followPath()'s final approach shows up
    // as part of the path
    motion::MotionResult approachPoint(double xIn, double yIn, const odom::Odometry& odometry,
                                       ExitConditions exit);
};

} // namespace sapphirelib::chassis
