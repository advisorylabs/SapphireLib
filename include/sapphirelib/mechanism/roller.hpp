#pragma once

#include <cstdint>
#include <initializer_list>

#include "pros/motor_group.hpp"
#include "sapphirelib/mechanism/jam_detector.hpp"

namespace sapphirelib::mechanism {

/**
 * @brief An intake, conveyor, or other roller, with optional anti-jam
 *
 * Call spin() every tick, even with 0V, since jam detection only works while it's being fed.
 * Safe to construct at namespace scope
 *
 * @note not thread-safe, so drive it from one task
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::mechanism::Roller intake({-18, 11}, {.enabled = true});
 *
 * // every tick
 * intake.spin(master.held(Button::r1) ? 12.0 : 0.0, master.now());
 * if (intake.jammed()) master.rumble(".");
 * @endcode
 */
class Roller {
public:
    /**
     * @brief Construct a new Roller
     *
     * @param ports motor ports. Negative reverses a motor
     * @param jam anti-jam settings. Off by default
     */
    explicit Roller(std::initializer_list<std::int8_t> ports, JamConfig jam = {});

    /**
     * @brief Spin the roller, or reverse it while clearing a jam
     *
     * @param volts voltage, clamped to +-12. Positive is the motors' forward. NaN sends 0V
     * @param nowMs the current time, in milliseconds
     */
    void spin(double volts, std::uint32_t nowMs);

    /**
     * @brief Stop the roller and forget any jam in progress
     */
    void stop();

    /**
     * @brief Whether the roller is reversing to clear a jam right now
     */
    bool jammed() const;

    /**
     * @brief Get the motors, for brake mode, current, temperature, and so on
     */
    pros::MotorGroup& motors();

private:
    // mean speed of the motors that answer, in RPM, or NaN if none do. Read per motor, since
    // get_actual_velocity_all() would allocate every tick
    double speedRpm() const;

    pros::MotorGroup motors_;
    JamDetector jam_;
};

} // namespace sapphirelib::mechanism
