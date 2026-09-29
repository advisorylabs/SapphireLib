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

#include <atomic>
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

    /// Zeroes the position returned by getPositionDegrees(). The tare
    /// happens in the motors themselves, not in this object, so it also
    /// zeroes every other reader of the same ports — a
    /// MotorGroupTrackingWheel feeding odometry, say, whose pose would jump.
    /// To measure a distance, record a starting reading and subtract it
    /// instead (as the drivetrains' driveDistance() does).
    void tarePosition();

    /// Average absolute position across the group, in degrees of motor
    /// shaft rotation (0 as of the last tarePosition() call).
    ///
    /// Motors that aren't answering are skipped, same as
    /// getTemperatureC(): PROS reports them as PROS_ERR_F (infinity), and
    /// one infinite entry would make the whole average infinite — enough to
    /// send driveDistance() off at full power until its timeout. If *none* of
    /// the group's motors answered — the whole group when it's one motor,
    /// like each HolonomicDrivetrain corner — it holds the last reading it
    /// got, so an unplugged motor reads as "didn't move" rather than jumping
    /// to 0 from however far it had turned (which, measured from a start
    /// reading, looks like a sudden huge error and drives the chassis off at
    /// full power just the same).
    double getPositionDegrees() const;

    /// Average actual velocity across the group, in RPM. Skips motors that
    /// aren't answering, the same way getPositionDegrees() does.
    double getVelocityRPM() const;

    /// Hottest motor in the group, in degrees Celsius — the max, not the
    /// mean, because the V5 derates each motor on its own temperature, so
    /// the worst one governs what the group can actually deliver.
    ///
    /// Motors that aren't answering are skipped (PROS reports those as
    /// PROS_ERR_F, i.e. infinity). An empty group, or one where nothing
    /// answered, reads 0 — cool, so nothing downstream mistakes a
    /// disconnected cable for an overheating motor. Use diag::SensorCheck to
    /// catch that case.
    double getTemperatureC() const;

    Gearset gearset() const;

    /// The gearset's rated free-speed, in RPM (100 / 200 / 600).
    double maxRPM() const;

private:
    pros::MotorGroup motors_;
    Gearset gearset_;

    /// The last position any motor answered with — see getPositionDegrees().
    /// Atomic because motion loops, odometry and the GUI read it from
    /// different tasks.
    mutable std::atomic<double> lastPositionDeg_{0.0};
};

} // namespace sapphirelib::chassis
