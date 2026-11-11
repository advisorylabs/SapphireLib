#pragma once

#include <cstdint>
#include <functional>
#include <optional>

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/odom/pose.hpp"

/**
 * @brief Driving onto a target a sensor can see: steered by turning to face it, at a steady speed
 * through it, to an end a little past it
 *
 * A trajectory drives to where odometry says a game element is. When odometry has drifted, the
 * element isn't there. An approach drives at a target point instead, turning to face it as it goes
 * (the robot faces the way it drives, so its front, claw and all, lines up on the target), and the
 * point can move under it: a camera that sees the element moves it onto the line it's seen along
 * (ApproachRefiner, onCameraRay()). It never strafes.
 *
 * It holds one speed, cruiseInPerS, through the target and stops only past it (ApproachConfig::
 * endShortIn), so a grab timed off the robot's speed (the claw's lead) sees the speed it assumes.
 * It takes over at whatever speed the robot already has: TrajectoryFollowConfig::until hands a
 * trajectory off to it without stopping.
 *
 * Everything here is plain math, unit-tested on a desktop. chassis::HolonomicDrivetrain::approach()
 * runs it, and auton::approach() is the routine's call.
 */
namespace sapphirelib::motion {

/**
 * @brief A point on the field, in inches, in odom::Pose's frame
 */
struct FieldPoint {
    /** x, in inches */
    double xIn = 0.0;
    /** y, in inches */
    double yIn = 0.0;
};

/**
 * @brief Where a camera sits on the robot, and how wide it sees
 *
 * A pinhole camera: a column of the image is a bearing off the camera's axis. The defaults are a
 * VEX AI Vision Sensor (320 pixels across) looking straight ahead from the tracking center
 */
struct CameraMount {
    /** the image's width, in pixels */
    double imageWidthPx = 320.0;
    /** how wide the image sees, edge to edge, in degrees */
    double horizontalFovDeg = 74.0;
    /** the lens, ahead of the tracking center, in inches (negative is behind) */
    double forwardIn = 0.0;
    /** the lens, right of the tracking center, in inches (negative is left) */
    double rightIn = 0.0;
    /** which way the camera looks, off the robot's front, in degrees: positive is turned right */
    double yawDeg = 0.0;
};

/**
 * @brief The bearing to something the camera sees at a column of its image
 *
 * @param camera the camera
 * @param columnPx the column, in pixels from the image's left edge (the middle of a detection)
 * @return double degrees off the camera's axis: positive is right of it, 0 the image's middle
 */
double cameraBearingDeg(const CameraMount& camera, double columnPx);

/**
 * @brief The camera's focal length, in pixels: something 1in across, 1in ahead of the lens, is this
 * many pixels across in the image (square pixels, so up and down too)
 */
double cameraFocalPx(const CameraMount& camera);

/**
 * @brief How far ahead of the lens a field point is, along the camera's axis: what its size in the
 * image goes by (cameraFocalPx() times its size over this)
 *
 * @return std::optional<double> inches. None for a point behind the lens, or beside it
 */
std::optional<double> cameraDepthIn(const CameraMount& camera, const odom::Pose& robot,
                                    FieldPoint point);

/**
 * @brief Where a field point should show up, as a bearing off the camera's axis
 *
 * @param camera the camera
 * @param robot the robot's pose
 * @param point the point
 * @return std::optional<double> degrees off the camera's axis, positive right. None for a point
 * behind the lens, or beside it
 */
std::optional<double> cameraBearingToDeg(const CameraMount& camera, const odom::Pose& robot,
                                         FieldPoint point);

/**
 * @brief Move a target onto the line the camera sees it along, keeping its distance from the lens
 *
 * The camera measures direction, not range: so the target keeps the range it had (odometry's, or
 * whatever the last update left), and only its direction from the lens changes. For a robot whose
 * pose has drifted sideways of the target, that's most of the error, and the range is what the
 * approach's end past the target (ApproachConfig::endShortIn) has room for.
 *
 * @param camera the camera
 * @param robot the robot's pose, when the camera took the image
 * @param bearingDeg where the camera saw it, off its axis (cameraBearingDeg())
 * @param prior where the target was thought to be
 * @return FieldPoint the target on the camera's line
 */
FieldPoint onCameraRay(const CameraMount& camera, const odom::Pose& robot, double bearingDeg,
                       FieldPoint prior);

/**
 * @brief The fastest the robot may go this far from the end and still stop there
 *
 * @param remainingIn how far it is to the end, in inches. At or below 0, 0
 * @param cruiseInPerS the speed to hold until it has to slow, in in/s
 * @param decelInPerS2 how hard it may slow, in in/s^2. At or below 0, cruise until the end
 * @return double the speed, in in/s
 */
double approachSpeedInPerS(double remainingIn, double cruiseInPerS, double decelInPerS2);

/**
 * how much heading error, in degrees, it takes for TargetApproach to add the turn model's whole kS:
 * a fraction of it below that, in proportion
 */
inline constexpr double kTurnFrictionRampDeg = 1.5;

/**
 * @brief How an approach runs. Every field has a default, so pass only what you change
 *
 * @b Example
 * @code {.cpp}
 * // through a Pin at 18in/s, to 3in past where it seats 5.65in ahead of the tracking center
 * sapphirelib::motion::ApproachConfig onto{.cruiseInPerS = 18.0, .endShortIn = 5.65 - 3.0};
 * @endcode
 */
struct ApproachConfig {
    /** the speed to drive at, through the target and on until it has to slow for the end, in/s */
    double cruiseInPerS = 15.0;

    /** how fast it speeds up to cruiseInPerS, from the speed it starts at, in in/s^2 */
    double accelInPerS2 = 60.0;

    /**
     * how hard it slows: to cruiseInPerS from a faster start, and to a stop at the end, in in/s^2.
     * It holds cruiseInPerS until cruiseInPerS^2 / (2 decelInPerS2) from the end: keep that short of
     * how far the end is past the target, so the robot meets the target at cruise
     */
    double decelInPerS2 = 60.0;

    /**
     * where it ends: the tracking center this far short of the target, in inches, along the line
     * it drives onto the target. Negative ends past the target
     */
    double endShortIn = 0.0;

    /** this close to the end counts as there, in inches */
    double endThresholdIn = 0.5;

    /**
     * closer to the target than this, in inches, it stops re-aiming and holds the heading it has:
     * an inch sideways of a target an inch away is 45 degrees, not worth swinging the robot for
     * when the target is nearly in it. Keep it past endShortIn
     */
    double steerMinIn = 6.0;

    /**
     * how far the robot may fall behind (or run ahead of) its speed reference, in inches, before
     * the correction stops growing: a robot held up then catches up at its own pace, not at full
     * power
     */
    double maxLagIn = 2.0;

    /** give up after this long, in milliseconds. 0 never does */
    std::uint32_t timeoutMs = 3000;

    /** the most volts any wheel gets, 12 for no limit */
    double maxVolts = 12.0;
};

/**
 * @brief Called every tick of an approach with the robot's pose, to move the target: a camera
 * that sees it (onCameraRay()). Leave the target alone to keep it where it was. Runs on the
 * motion's task inside its loop, so it must be quick
 */
using ApproachRefiner = std::function<void(const odom::Pose& pose, FieldPoint& target)>;

/**
 * @brief What an approach did on one tick
 */
struct ApproachTick {
    /** forward volts, before wheel mixing */
    double forwardVolts = 0.0;
    /** turn volts, before wheel mixing */
    double turnVolts = 0.0;
    /** the speed it's aiming for now, in in/s */
    double speedRefInPerS = 0.0;
    /** how fast that's changing, in in/s^2 */
    double accelRefInPerS2 = 0.0;
    /** how far the robot is behind the speed reference, in inches (capped at maxLagIn) */
    double lagIn = 0.0;
    /** the tracking center to the target, in inches */
    double distanceIn = 0.0;
    /** the tracking center to the end, in inches. Negative past it */
    double remainingIn = 0.0;
    /** the heading it steers for, in degrees */
    double aimDeg = 0.0;
    /** aimDeg minus the robot's heading, in degrees, wrapped to (-180, 180] */
    double headingErrorDeg = 0.0;
    /** whether it re-aimed at the target this tick (false once inside steerMinIn) */
    bool steering = false;
};

/**
 * @brief The approach's controller: one call per tick, with the pose and the target
 *
 * Forward: a speed reference that eases from the starting speed to cruiseInPerS and down to a stop
 * at the end (approachSpeedInPerS()), driven by the forward model's feedforward plus a PID on how
 * far the robot has fallen behind that reference (D on the speed error), as the trajectory
 * follower tracks. Turn: a PID on the heading error to the target, plus the turn model's static
 * friction (kS) toward it, eased in over the first kTurnFrictionRampDeg of error so it never
 * chatters around zero: without it a P-only turn sits a few degrees off the target, where its
 * volts just match kS (3.5 degrees in the simulator, on the robot's 1.1V). No strafe
 *
 * @b Example
 * @code {.cpp}
 * TargetApproach approach(drivetrain.trackingForwardPID(), drivetrain.trackingTurnPID());
 * approach.begin(config, startSpeedInPerS, pros::millis());
 * while (approach.reason() == ExitReason::running) {
 *     const auto snap = odometry.snapshot();
 *     const ApproachTick tick = approach.update(snap.pose, snap.rawVelocity, target, models.forward,
 *                                               pros::millis());
 *     drivetrain.holonomicVolts(tick.forwardVolts, 0.0, tick.turnVolts);
 *     pros::delay(10);
 * }
 * @endcode
 */
class TargetApproach {
public:
    /**
     * @brief Construct a new TargetApproach
     *
     * @param forward forward PID: volts per inch the robot lags its speed reference (D: per in/s)
     * @param turn turn PID: volts per degree of heading error
     */
    TargetApproach(PID& forward, PID& turn);

    /**
     * @brief Start: resets both PIDs
     *
     * @param config how it runs
     * @param startSpeedInPerS how fast the robot is already going forward, in in/s (0 from rest)
     * @param nowMs the time, in milliseconds
     */
    void begin(const ApproachConfig& config, double startSpeedInPerS, std::uint32_t nowMs);

    /**
     * @brief Run one tick
     *
     * @param pose the robot's pose
     * @param velocity its field velocity (Odometry::Snapshot::rawVelocity)
     * @param target where the target is now
     * @param forwardModel the forward axis model, for the feedforward
     * @param turnModel the turn axis model, for its static friction (see kTurnFrictionRampDeg)
     * @param nowMs the time, in milliseconds
     * @return ApproachTick what to command, and why
     */
    ApproachTick update(const odom::Pose& pose, const odom::Velocity& velocity, FieldPoint target,
                        const MotorFeedforward& forwardModel, const MotorFeedforward& turnModel,
                        std::uint32_t nowMs);

    /** @brief Why it ended, or running */
    ExitReason reason() const { return reason_; }

    /** @brief How it ended: finalError is the distance to the end, in inches */
    MotionResult result() const;

private:
    PID& forward_;
    PID& turn_;
    ApproachConfig config_;
    ExitTracker tracker_{0, 0, 0};
    ExitReason reason_ = ExitReason::running;
    double speedRefInPerS_ = 0.0;
    double lagIn_ = 0.0;
    double aimDeg_ = 0.0;
    bool aimed_ = false;
    bool started_ = false;
    std::uint32_t lastMs_ = 0;
    double finalErrorIn_ = 0.0;
};

} // namespace sapphirelib::motion
