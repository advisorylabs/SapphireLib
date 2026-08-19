/**
 * \file sapphirelib/motion/motion_config.hpp
 *
 * Config types shared by moveToPose()/followPath() on every drivetrain.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

#include "sapphirelib/chassis/drivetrain_config.hpp"

namespace sapphirelib::motion {

/// Exit conditions for moveToPose() — settles once *both* the position and
/// heading errors are within their thresholds at the same time.
struct PoseExitConditions {
    double positionErrorThresholdIn;
    double headingErrorThresholdDeg;

    /// How long both errors must stay within threshold before the motion
    /// exits successfully.
    std::uint32_t settleTimeMs = 200;

    /// Hard cutoff regardless of settling. 0 disables the timeout (not
    /// recommended).
    std::uint32_t timeoutMs = 4000;

    /// Boomerang controller lead: how far behind the target (as a fraction
    /// of remaining distance) the carrot point is placed. Only used by
    /// TankDrivetrain::moveToPose() — HolonomicDrivetrain doesn't need the
    /// carrot trick since it can translate and rotate independently. Smaller
    /// = tighter curve into the final heading; larger = wider curve. Must be
    /// > 0.
    double boomerangLeadPct = 0.6;
};

/// Tuning for followPath().
struct PursuitConfig {
    /// How far ahead along the path to aim, in inches. Larger values cut
    /// corners more (smoother, less accurate); smaller values track the
    /// path more tightly (choppier, and can oscillate if too small relative
    /// to the control loop's rate/latency).
    double lookaheadIn;

    /// Cruise voltage while following the path, clamped to +-12. Constant —
    /// followPath() doesn't decelerate mid-path, only on the final
    /// approach (see finalApproachIn).
    double cruiseVoltage = 8.0;

    /// Once within this distance of the path's final waypoint, followPath()
    /// switches from chasing the lookahead point to a direct moveToPoint()
    /// at the final waypoint, so it actually decelerates and settles there
    /// instead of orbiting it.
    double finalApproachIn = 6.0;

    /// Exit conditions for that final moveToPoint() hand-off.
    chassis::ExitConditions finalExit = chassis::ExitConditions{1.0};
};

} // namespace sapphirelib::motion
