/**
 * \file sapphirelib/chassis/motor_group.hpp
 *
 * Thin wrapper around pros::MotorGroup: exposes voltage/velocity control,
 * position/velocity readouts, gearing, and brake mode through SapphireLib's
 * own types, so chassis and (later) odometry code depend on this instead of
 * PROS motor headers directly.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <initializer_list>

#include "pros/motor_group.hpp"

namespace sapphirelib::chassis {

/// V5 smart motor gearing cartridges, named by their rated free-speed RPM.
enum class Gearset { red /* 100 RPM */, green /* 200 RPM */, blue /* 600 RPM */ };

enum class BrakeMode { coast, brake, hold };

/// Wraps a pros::MotorGroup representing one side of a drivetrain (or any
/// other ganged set of motors).
class MotorGroup {
public:
    MotorGroup(std::initializer_list<std::int8_t> ports, Gearset gearset);

    /// Commands an open-loop voltage, in volts, clamped to +-12.
    void moveVoltage(double volts);

    /// Commands a closed-loop velocity, in RPM, clamped to the gearset's max.
    void moveVelocity(double rpm);

    /// Stops using the currently configured brake mode.
    void brake();

    void setBrakeMode(BrakeMode mode);

    /// Zeroes the position returned by getPositionDegrees().
    void tarePosition();

    /// Average absolute position across the group, in degrees of motor
    /// shaft rotation (0 as of the last tarePosition() call).
    double getPositionDegrees() const;

    /// Average actual velocity across the group, in RPM.
    double getVelocityRPM() const;

    Gearset gearset() const;

    /// The gearset's rated free-speed, in RPM (100 / 200 / 600).
    double maxRPM() const;

private:
    pros::MotorGroup motors_;
    Gearset gearset_;
};

} // namespace sapphirelib::chassis
