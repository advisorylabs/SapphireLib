#pragma once

#include <atomic>
#include <cstdint>
#include <initializer_list>

#include "pros/motor_group.hpp"

namespace sapphirelib::chassis {

/**
 * @brief V5 motor cartridges, named by their free speed
 */
enum class Gearset { red /* 100 RPM */, green /* 200 RPM */, blue /* 600 RPM */ };

/**
 * @brief What a motor does when stopped
 */
enum class BrakeMode { coast, brake, hold };

/**
 * @brief A group of motors driven together, like one side of a drivetrain
 *
 * @b Example
 * @code {.cpp}
 * // left side on ports 1, 2 (reversed), and 3, with blue cartridges
 * sapphirelib::chassis::MotorGroup left({1, -2, 3}, sapphirelib::chassis::Gearset::blue);
 * left.moveVoltage(6.0);
 * @endcode
 */
class MotorGroup {
public:
    /**
     * @brief Construct a new MotorGroup
     *
     * @param ports motor ports. Negative reverses a motor
     * @param gearset the motors' cartridge
     */
    MotorGroup(std::initializer_list<std::int8_t> ports, Gearset gearset);

    /**
     * @brief Move the motors at a voltage
     *
     * @param volts voltage, clamped to +-12
     */
    void moveVoltage(double volts);

    /**
     * @brief Move the motors at a velocity, using the motors' own velocity control
     *
     * @param rpm velocity in RPM, clamped to the cartridge's top speed
     */
    void moveVelocity(double rpm);

    /**
     * @brief Stop the motors with the current brake mode
     */
    void brake();

    /**
     * @brief Set the brake mode
     *
     * @param mode coast, brake, or hold
     */
    void setBrakeMode(BrakeMode mode);

    /**
     * @brief Zero the position
     *
     * @note this zeroes the motors themselves, so anything else reading the same ports (like a
     * MotorGroupTrackingWheel feeding odometry) jumps too. To measure a distance, subtract a
     * starting reading instead
     */
    void tarePosition();

    /**
     * @brief Get the average position of the motors
     *
     * Motors that don't answer are skipped. If none answer, the last reading is kept, so an
     * unplugged motor reads as "didn't move" instead of jumping to 0
     *
     * @return double position, in degrees of motor rotation
     */
    double getPositionDegrees() const;

    /**
     * @brief Get the average velocity of the motors. Motors that don't answer are skipped
     *
     * @return double velocity, in RPM
     */
    double getVelocityRPM() const;

    /**
     * @brief Get the temperature of the hottest motor
     *
     * The hottest, not the average, since each motor derates on its own temperature. Motors that
     * don't answer are skipped, and a group where none answer reads 0. Use diag::SensorCheck to
     * catch an unplugged motor
     *
     * @return double temperature, in degrees Celsius
     */
    double getTemperatureC() const;

    /**
     * @brief Get the cartridge
     */
    Gearset gearset() const;

    /**
     * @brief Get the cartridge's free speed
     *
     * @return double 100, 200, or 600 RPM
     */
    double maxRPM() const;

private:
    pros::MotorGroup motors_;
    Gearset gearset_;

    // the last position any motor answered with. Atomic since motions, odometry, and the GUI read
    // it from different tasks
    mutable std::atomic<double> lastPositionDeg_{0.0};
};

} // namespace sapphirelib::chassis
