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
