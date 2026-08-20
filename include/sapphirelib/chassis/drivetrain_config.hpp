/**
 * \file sapphirelib/chassis/drivetrain_config.hpp
 *
 * Config types shared by every drivetrain kinematics (TankDrivetrain,
 * HolonomicDrivetrain, ...).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

namespace sapphirelib::chassis {

struct DrivetrainConfig {
    /// Diameter of the drive wheels, in inches. Required — there's no sane
    /// default across robots.
    double wheelDiameterIn = 0.0;

    /// Motor shaft rotations per wheel rotation (external gear ratio).
    /// 1.0 for direct drive.
    double externalGearRatio = 1.0;

    /// Proportional gain applied to IMU heading drift during driveDistance()
    /// to keep the robot driving straight. 0 disables heading correction.
    double headingCorrectionKP = 0.0;
};

/// Optional 5th/6th "Asterisk" center wheels for a HolonomicDrivetrain: two
/// motors mounted between the four corners, facing straight forward/back
/// (not omni) rather than at the corner 45s. HolonomicDrivetrain drives them
/// off whatever forward/back component is already present in the corner
/// wheels' commanded voltages — so they add power during forward/backward
/// motion and sit idle (coasting) during a pure strafe or turn — plus, while
/// strafing, corrects any forward/back drift the vertical tracking wheel
/// picks up (see HolonomicDrivetrain::setDriftSource()). Omit the
/// drivetrain's `asterisk` constructor argument entirely for a standard
/// 4-motor holonomic chassis.
struct AsteriskConfig {
    /// Ports for the two center motors, same sign convention as the corner
    /// ports (negative = reversed). Both share the drivetrain's corner
    /// gearset.
    std::int8_t middleLeftPort;
    std::int8_t middleRightPort;

    /// Proportional gain applied to the vertical tracking wheel's drift
    /// rate (inches/sec of unwanted forward/back motion) while strafing, to
    /// correct it via the center wheels alone. 0 disables drift correction
    /// (the center wheels will still drive during forward/backward motion —
    /// this only gates the strafe-time correction). Has no effect unless
    /// setDriftSource() has been called with a vertical TrackingWheel.
    double driftCorrectionKP = 0.0;
};

/// Exit conditions for a blocking motion (driveDistance()/turnToHeading()).
struct ExitConditions {
    /// The motion is "settled" once its error stays within this threshold —
    /// inches for driveDistance(), degrees for turnToHeading().
    double errorThreshold;

    /// How long the error must stay within errorThreshold before the motion
    /// exits successfully.
    std::uint32_t settleTimeMs = 200;

    /// Hard cutoff regardless of settling — the failsafe against a stalled
    /// or unreachable target. 0 disables the timeout (not recommended).
    std::uint32_t timeoutMs = 3000;
};

} // namespace sapphirelib::chassis
