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

#include "sapphirelib/control/feedforward.hpp"

namespace sapphirelib::chassis {

/// How driver-control stick input becomes motor output.
enum class DriverInputMode {
    /// Stick position is a fraction of full voltage: half stick is 6V. The
    /// classic feel — direct, but the first few percent of travel does
    /// nothing while the voltage is still too low to overcome friction.
    voltage,

    /// Stick position is a fraction of the axis's top speed: half stick asks
    /// for half of top speed, and the measured model (see
    /// HolonomicAxisModels) works out the voltage for it, friction included.
    /// Falls back to voltage behavior on any axis without a valid model.
    velocity,
};

/// Measured models for a holonomic chassis's three independent axes — see
/// tuning::characterizeAxis(). Translation axes are in inches, turn in
/// degrees, all against the axis voltage before wheel mixing.
struct HolonomicAxisModels {
    MotorFeedforward forward;
    MotorFeedforward strafe;
    MotorFeedforward turn;
};

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
/// off the forward/back and rotation components already present in the
/// corner wheels' commanded voltages — so they add power during
/// forward/backward motion, add torque to every turn (see turnContribution),
/// and sit idle (coasting) only during a pure sideways strafe — plus, while
/// strafing, correct any forward/back drift the vertical tracking wheel
/// picks up (see HolonomicDrivetrain::setDriftSource()). Omit the
/// drivetrain's `asterisk` constructor argument entirely for a standard
/// 4-motor holonomic chassis.
struct AsteriskConfig {
    /// Ports for the two center motors, same sign convention as the corner
    /// ports (negative = reversed). Both share the drivetrain's corner
    /// gearset.
    std::int8_t middleLeftPort;
    std::int8_t middleRightPort;

    /// Proportional gain (volts per inch/sec) applied to the drift rate
    /// while strafing — the forward/back motion the vertical tracking wheel
    /// sees, minus its rotation arc and minus whatever forward component the
    /// command itself asked for (see chassis::strafeDriftIn()) — to correct
    /// it via the center wheels alone. 0 disables drift correction
    /// (the center wheels will still drive during forward/backward motion —
    /// this only gates the strafe-time correction). Has no effect unless
    /// setDriftSource() has been called with a vertical TrackingWheel.
    double driftCorrectionKP = 0.0;

    /// Fraction of the commanded rotation the center wheels contribute to,
    /// driven differentially (left forward, right backward for a positive
    /// turn). 1.0 — the default — gives them the same rotational authority
    /// per volt as the corners; 0 makes them ignore turns and coast through
    /// them, which is what this drivetrain did before this knob existed.
    ///
    /// Contributing is the physically correct thing for them to do: in a
    /// point turn about the chassis center, a wheel sitting on the left or
    /// right flank travels purely fore/aft, which is exactly the direction
    /// a straight-mounted center wheel rolls — so it adds torque rather
    /// than scrubbing sideways against the turn.
    ///
    /// Worth turning down (not off) if the center motors are meaningfully
    /// weaker than the corners — a pair of 5.5W motors against 11W corners,
    /// say — and you find them saturating and dragging on fast turns before
    /// the corners do.
    double turnContribution = 1.0;

    /// How much of what the corner motors fail to deliver the center wheels
    /// try to make up. A V5 motor quietly cuts its own available power as it
    /// heats — roughly half by 55C, off by 70C — and nothing in the command
    /// path tells you. With this above 0, the center wheels work out what
    /// the corners aren't producing and add it back. 1.0 (the default)
    /// attempts the full shortfall, 0.5 half, 0 disables the feature.
    /// Bounded by maxThermalCorrectionVolts, and faded out as the center
    /// motors heat up themselves — see chassis::centerThermalCorrection(),
    /// which is where the arithmetic and its reasoning live.
    ///
    /// The case that matters most isn't the obvious one. Corners heating up
    /// *together* just makes the chassis slower, and the center wheels
    /// driving harder helps with that. But corners rarely heat evenly, and
    /// an *uneven* set is what actually ruins a strafe: the four corners'
    /// forward components are supposed to cancel exactly, so one weak corner
    /// leaves the chassis creeping forward or back — and twisting — across a
    /// strafe meant to be pure sideways. Both of those are fore/aft errors,
    /// which is exactly what a straight-mounted center wheel can cancel.
    ///
    /// Complements driftCorrectionKP rather than replacing it. This is
    /// feedforward — it knows the corner is weak and corrects before the
    /// chassis has moved — while driftCorrectionKP is the reactive loop that
    /// catches whatever is left, plus everything thermal derating isn't (a
    /// dragging bearing, debris under a wheel). Run both; each makes the
    /// other's job smaller.
    ///
    /// What it can't do: add sideways thrust. Nothing mounted fore/aft can,
    /// so a strafe on hot corners still loses speed even once it's holding
    /// its line. And it can only spend command headroom that exists — at
    /// full stick the center wheels are already at 12V with nothing left to
    /// give.
    double thermalCompensation = 1.0;

    /// Ceiling, in volts, on each component of that correction — the
    /// forward/back one and the differential one are capped separately, so
    /// this is the most the feature will add to a center wheel.
    ///
    /// Fully derated corners under full forward command would otherwise ask
    /// for the whole 12V on top of what the center wheels already carry,
    /// which on a pair of 5.5W motors against 11W corners is both more than
    /// they have and a good way to make them the next thing to overheat.
    /// 6.0 leaves half the motor's range available for compensation while
    /// keeping the rest in reserve. Realistic strafe-drift corrections are a
    /// few volts, well under this; it mostly binds on the drive-harder case.
    /// 0 disables the correction as surely as thermalCompensation = 0 does.
    double maxThermalCorrectionVolts = 6.0;
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
