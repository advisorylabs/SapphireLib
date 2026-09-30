#pragma once

namespace sapphirelib {

/**
 * @brief Feedforward model of one axis: V = kS * sign(v) + kV * v + kA * a
 *
 * Units follow the axis: inches for forward and strafe, degrees for turning. The voltage is the
 * axis command before wheel mixing, the same number a PID on that axis outputs. Auto-Tune measures
 * these (see tuning::characterizeAxis())
 *
 * @b Example
 * @code {.cpp}
 * // a drive axis that needs 1V to start moving, 0.2V per in/s, and 0.045V per in/s^2
 * sapphirelib::MotorFeedforward forward{.kS = 1.0, .kV = 0.2, .kA = 0.045};
 * double volts = forward.volts(30.0); // volts to hold 30 in/s
 * @endcode
 */
struct MotorFeedforward {
    /** volts needed to break static friction */
    double kS = 0.0;

    /** volts per unit/s of steady speed */
    double kV = 0.0;

    /** volts per unit/s^2 of acceleration */
    double kA = 0.0;

    /**
     * @brief Whether the model has been measured
     *
     * @return true kV and kA are both positive
     */
    bool valid() const;

    /**
     * @brief Get the volts needed for a velocity and acceleration
     *
     * The friction term takes the sign of the velocity, or of the acceleration when starting from
     * rest. Not clamped to 12V
     *
     * @param velocity target velocity, in units/s
     * @param acceleration target acceleration, in units/s^2. 0 by default
     * @return double the voltage
     */
    double volts(double velocity, double acceleration = 0.0) const;

    /**
     * @brief Get the steady speed a voltage can reach: (V - kS) / kV
     *
     * @param availableVolts the voltage available. 12 by default
     * @return double the speed, in units/s. 0 if the model is invalid or the voltage is below kS
     */
    double maxVelocity(double availableVolts = 12.0) const;
};

} // namespace sapphirelib
