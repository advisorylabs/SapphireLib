#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

#include "pros/rtos.hpp"
#include "sapphirelib/chassis/drift_math.hpp"
#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/chassis/motor_group.hpp"
#include "sapphirelib/chassis/thermal_math.hpp"
#include "sapphirelib/control/heading_hold.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/motion/motion_config.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::chassis {

/**
 * @brief Holonomic (mecanum or X-drive) drivetrain
 *
 * One motor per corner, mixed so the chassis can drive, strafe, and turn independently. Works
 * with just the IMU and drive encoders, and can add a pair of "Asterisk" center wheels (see
 * AsteriskConfig). For a tank drive, see TankDrivetrain
 *
 * Every blocking motion returns a motion::MotionResult and logs a motion start/end event to the
 * running telemetry::Logger, if there is one
 */
class HolonomicDrivetrain {
public:
    /**
     * @brief Construct a new HolonomicDrivetrain
     *
     * @note blocks for about 2-3 seconds while the IMU calibrates, so build it from initialize(),
     * not at namespace scope
     *
     * @param frontLeftPort front left motor port. Negative reverses it
     * @param frontRightPort front right motor port
     * @param backLeftPort back left motor port
     * @param backRightPort back right motor port
     * @param gearset the drive motors' cartridge
     * @param imuPort IMU port
     * @param config wheel size, gear ratio, and driveDistance() heading correction
     * @param drivePIDConfig PID settings for distance motions
     * @param turnPIDConfig PID settings for turns. Also the heading hold PID's starting settings
     * @param imuHeadingScale correction for the IMU's multi-turn drift. 1 by default; see
     * sensors::calibrateHeadingScale()
     * @param asterisk center wheel settings. nullopt (the default) for a 4 motor chassis
     *
     * @b Example
     * @code {.cpp}
     * sapphirelib::chassis::HolonomicDrivetrain& drivetrain() {
     *     static sapphirelib::chassis::HolonomicDrivetrain instance(
     *         1, -2, 3, -4, // front left, front right, back left, back right
     *         sapphirelib::chassis::Gearset::green,
     *         10, // IMU port
     *         {.wheelDiameterIn = 4.0, .externalGearRatio = 1.0, .headingCorrectionKP = 0.4},
     *         {.gains = {.kP = 1.2, .kI = 0.0, .kD = 0.001}, .outputLimit = 12.0},   // drive
     *         {.gains = {.kP = 0.35, .kI = 0.0, .kD = 0.0002}, .outputLimit = 12.0}); // turn
     *     return instance;
     * }
     * @endcode
     */
    HolonomicDrivetrain(std::int8_t frontLeftPort, std::int8_t frontRightPort,
                        std::int8_t backLeftPort, std::int8_t backRightPort, Gearset gearset,
                        std::uint8_t imuPort, DrivetrainConfig config, PID::Config drivePIDConfig,
                        PID::Config turnPIDConfig, double imuHeadingScale = 1.0,
                        std::optional<AsteriskConfig> asterisk = std::nullopt);

    /**
     * @brief Drive with robot-centric holonomic control
     *
     * The mix is scaled so no wheel goes past 12V without changing direction. How each stick
     * becomes volts depends on setDriverInputMode()
     *
     * @param throttle forward/backward, -1 to 1
     * @param strafe left/right, -1 to 1
     * @param turn rotation, -1 to 1
     *
     * @b Example
     * @code {.cpp}
     * void opcontrol() {
     *     while (true) {
     *         double throttle = master.get_analog(ANALOG_LEFT_Y) / 127.0;
     *         double strafe = master.get_analog(ANALOG_LEFT_X) / 127.0;
     *         double turn = master.get_analog(ANALOG_RIGHT_X) / 127.0;
     *         drivetrain().holonomic(throttle, strafe, turn);
     *         pros::delay(10);
     *     }
     * }
     * @endcode
     */
    void holonomic(double throttle, double strafe, double turn);

    /**
     * @brief Drive each axis at a voltage, before wheel mixing
     *
     * Not affected by setDriverInputMode(), so it's for characterization runs, calibration spins,
     * and custom controllers. Scaled like holonomic() so no wheel goes past 12V
     *
     * @param forwardVolts forward/backward, in volts
     * @param strafeVolts left/right, in volts
     * @param turnVolts rotation, in volts
     */
    void holonomicVolts(double forwardVolts, double strafeVolts, double turnVolts);

    /**
     * @brief Get the voltage last commanded on each axis, before wheel mixing
     *
     * Covers driver control, holonomicVolts(), and every autonomous motion. Before each motor's own
     * 12V clamp, so a motion's PID output can read past 12. All zero after stop()
     *
     * @note safe to call from any task. The three values are separate atomics, so a read can mix
     * two consecutive ticks
     *
     * @return AxisVolts forward, strafe, and turn volts
     */
    AxisVolts appliedAxisVolts() const;

    /**
     * @brief Set how driver control turns stick input into motor output
     *
     * Heading hold's turn output always comes from its PID; this only changes the translation
     * sticks there. Safe to call from any task
     *
     * @param mode voltage or velocity
     */
    void setDriverInputMode(DriverInputMode mode);

    /**
     * @brief Get the driver input mode
     */
    DriverInputMode driverInputMode() const;

    /**
     * @brief Set the measured axis models, used by DriverInputMode::velocity
     *
     * Safe to call from any task, e.g. when a tuning run finishes during driver control
     *
     * @param models forward, strafe, and turn models
     *
     * @b Example
     * @code {.cpp}
     * drivetrain().setAxisModels({
     *     .forward = {.kS = 0.97, .kV = 0.20, .kA = 0.045},
     *     .strafe = {.kS = 1.28, .kV = 0.24, .kA = 0.055},
     *     .turn = {.kS = 0.55, .kV = 0.045, .kA = 0.0067},
     * });
     * drivetrain().setDriverInputMode(sapphirelib::chassis::DriverInputMode::velocity);
     * @endcode
     */
    void setAxisModels(HolonomicAxisModels models);

    /**
     * @brief Get the axis models
     */
    HolonomicAxisModels axisModels() const;

    /**
     * @brief Drive with field-centric control
     *
     * throttle and strafe are relative to the field, not the chassis: throttle always drives
     * toward the heading from construction or the last resetFieldHeading()
     *
     * @param throttle forward/backward on the field, -1 to 1
     * @param strafe left/right on the field, -1 to 1
     * @param turn rotation, -1 to 1
     */
    void holonomicFieldCentric(double throttle, double strafe, double turn);

    /**
     * @brief Drive with the turn stick steering a heading instead of a turn rate
     *
     * The turn stick moves a held heading (see HeadingHoldConfig), and headingHoldPID() holds the
     * chassis on it while it drives and strafes. Letting go leaves the chassis pointed somewhere
     * definite, and bumps get corrected. Call it every tick while driving; after a gap (autonomous,
     * a tuner run) it starts from the chassis's current heading
     *
     * @param throttle forward/backward, -1 to 1
     * @param strafe left/right, -1 to 1
     * @param turnInput turn stick, -1 to 1
     */
    void holonomicHeadingHold(double throttle, double strafe, double turnInput);

    /**
     * @brief holonomicHeadingHold() with field-centric translation
     *
     * @param throttle forward/backward on the field, -1 to 1
     * @param strafe left/right on the field, -1 to 1
     * @param turnInput turn stick, -1 to 1
     *
     * @b Example
     * @code {.cpp}
     * drivetrain().holonomicFieldCentricHeadingHold(throttle, strafe, turn);
     * @endcode
     */
    void holonomicFieldCentricHeadingHold(double throttle, double strafe, double turnInput);

    /**
     * @brief Set how the turn stick steers the held heading. Takes effect on the next call
     *
     * @param config heading hold settings
     */
    void setHeadingHold(HeadingHoldConfig config);

    /**
     * @brief Get the heading that heading hold is holding
     *
     * Only meaningful once holonomicHeadingHold() has been called. Stays within
     * HeadingHoldConfig::maxLeadDeg of the chassis
     *
     * @return double held heading, 0-360 degrees, in the same frame as headingDeg()
     */
    double heldHeadingDeg() const;

    /**
     * @brief Make the way the chassis faces now the "forward" for field-centric driving
     *
     * Usually bound to a driver button. Doesn't change the held heading, and Odometry::setPose()
     * doesn't change the field-centric forward; only this does
     *
     * @b Example
     * @code {.cpp}
     * if (master.get_digital_new_press(DIGITAL_A)) drivetrain().resetFieldHeading();
     * @endcode
     */
    void resetFieldHeading();

    /**
     * @brief Set the odometry used by the motions that don't take one
     *
     * @note set it once in initialize(), before any motion runs. The odometry must outlive the
     * drivetrain
     *
     * @param odometry the odometry. nullptr unsets it
     *
     * @b Example
     * @code {.cpp}
     * void initialize() {
     *     odometry().startTask();
     *     drivetrain().setOdometry(&odometry());
     * }
     * @endcode
     */
    void setOdometry(const odom::Odometry* odometry);

    /**
     * @brief Drive to a point on the field
     *
     * Holds the heading the chassis started with. To end at a different heading, use
     * moveToPose() instead of following this with a turn
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
     * @brief Drive to a pose on the field
     *
     * Translates and turns at the same time; a holonomic chassis doesn't need the boomerang
     * controller TankDrivetrain uses
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
     * // drive to (24, 48) facing 90 degrees, with a 3 second timeout
     * drivetrain().moveToPose(24, 48, 90, {.timeoutMs = 3000});
     * @endcode
     */
    motion::MotionResult moveToPose(double xIn, double yIn, double headingDeg,
                                    motion::PoseExitConditions exit = {});

    /**
     * @brief Follow a path with pure pursuit
     *
     * Chases a point lookaheadIn ahead on the path at a constant voltage, then switches to
     * moveToPoint() for the last waypoint so it slows down and settles there. Holds the starting
     * heading the whole way. The path may end where it starts, for a lap
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
     * @brief Drive straight forward or backward a distance
     *
     * Uses the drive encoders, with IMU heading correction. Forward axis only; there's no closed
     * loop strafing yet. Measures from where the encoders read when it starts, without taring them
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
     * @brief Stop the drivetrain
     *
     * @param mode brake mode to stop with. brake by default
     */
    void stop(BrakeMode mode = BrakeMode::brake);

    /**
     * @brief Get the field heading
     *
     * The frame turnToHeading() targets and Odometry::setPose() sets. Not const; see
     * sensors::Imu::getHeadingDeg()
     *
     * @return double heading, 0-360 degrees, clockwise positive
     */
    double headingDeg();

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

    /**
     * @brief Get the PID used by heading hold. Starts with the turn PID's settings
     */
    PID& headingHoldPID();

    /**
     * @brief Set the tracking wheel the Asterisk wheels use to detect drift while strafing
     *
     * Does nothing without an asterisk config. See AsteriskConfig::driftCorrectionKP
     *
     * @param verticalWheel the forward-facing tracking wheel, usually the one odometry uses
     * @param odometry supplies the wheel's offset, read every tick. nullptr treats the wheel as
     * centered
     *
     * @b Example
     * @code {.cpp}
     * drivetrain().setDriftSource(&verticalWheel(), &odometry());
     * @endcode
     */
    void setDriftSource(const odom::TrackingWheel* verticalWheel,
                        const odom::Odometry* odometry = nullptr);

private:
    MotorGroup frontLeft_;
    MotorGroup frontRight_;
    MotorGroup backLeft_;
    MotorGroup backRight_;
    sensors::Imu imu_;
    DrivetrainConfig config_;
    PID drivePID_;
    PID turnPID_;
    PID headingHoldPID_;

    std::atomic<DriverInputMode> driverInputMode_{DriverInputMode::voltage};
    mutable pros::MutexVar<HolonomicAxisModels> axisModels_;

    // see setOdometry()
    const odom::Odometry* odometry_ = nullptr;

    // see appliedAxisVolts(). Atomics, not a mutex, since PROS deletes competition tasks on every
    // mode change and a mutex held at that moment would stay locked
    std::atomic<double> appliedForwardVolts_{0.0};
    std::atomic<double> appliedStrafeVolts_{0.0};
    std::atomic<double> appliedTurnVolts_{0.0};

    // field-centric forward, in the rotation frame so setPose() doesn't move it. See
    // resetFieldHeading()
    double fieldHeadingZeroDeg_;

    // Asterisk center wheels, empty on a 4 motor chassis
    std::optional<AsteriskConfig> asterisk_;
    std::optional<MotorGroup> middleLeft_;
    std::optional<MotorGroup> middleRight_;

    // drift correction state, see setDriftSource()
    const odom::TrackingWheel* driftSource_ = nullptr;
    const odom::Odometry* driftOffsetSource_ = nullptr;
    double lastDriftVerticalIn_ = 0.0;
    double lastDriftStrafeIn_ = 0.0;
    double lastDriftHeadingDeg_ = 0.0;
    std::uint32_t lastDriftTickMs_ = 0;

    // thermal compensation state. Only the power fractions are cached (temperature changes
    // slowly); the correction is worked out every tick from that tick's corner voltages. Per
    // corner, since one hot corner and four evenly hot ones need opposite responses
    CornerValues thermalFractions_;
    double centerThermalFraction_ = 1.0;
    std::uint32_t lastThermalPollMs_ = 0;

    // heading hold state. lastHeadingHoldMs_ of 0 means not holding
    HeadingHoldConfig headingHold_;
    double heldHeadingDeg_ = 0.0; // rotation frame, see heldHeadingDeg()
    std::uint32_t lastHeadingHoldMs_ = 0;

    double degreesToInches(double degrees) const;
    void setWheelVoltages(double frontLeft, double frontRight, double backLeft, double backRight);

    // mean of the four corner encoders, in motor degrees
    double cornerAverageDegrees() const;

    // publish one tick's axis volts for appliedAxisVolts()
    void recordAppliedVolts(double forwardVolts, double strafeVolts, double turnVolts);

    // rotate a field-relative (throttle, strafe) into the chassis's frame
    void fieldToRobot(double& throttle, double& strafe);

    // one stick, -1 to 1, to axis volts for the current DriverInputMode
    double stickVolts(double input, const MotorFeedforward& model) const;

    // sideways travel from the four corner encoders
    double encoderStrafeIn() const;

    // moveToPoint() holding a given heading, so followPath() can keep its starting heading
    // through the final approach. Logs no events of its own
    motion::MotionResult moveToPointHolding(double xIn, double yIn, double holdHeadingDeg,
                                            const odom::Odometry& odometry, ExitConditions exit);

    // advance the held heading one tick and return the heading hold PID's output, in volts
    double headingHoldTurnVolts(double turnInput);

    // the IMU's rotation since construction, wrapped to 0-360. setPose() never shifts it, so
    // it's used for everything that tracks a physical direction or only takes differences
    double rotationHeadingDeg();

    // re-read motor temperatures (at most every kThermalPollIntervalMs) into the power fractions
    void refreshThermalFractions();
};

} // namespace sapphirelib::chassis
