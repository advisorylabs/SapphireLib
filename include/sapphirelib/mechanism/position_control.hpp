#pragma once

#include <cstdint>

#include "sapphirelib/control/pid.hpp"

namespace sapphirelib::mechanism {

/**
 * @brief Volts added to hold a mechanism up against gravity
 *
 * Without it, a PID settles short of every target by the error where kP * error equals the holding
 * voltage
 *
 * @b Example
 * @code {.cpp}
 * // an elevator lift that takes 1.5V to hold up at any height
 * sapphirelib::mechanism::GravityFeedforward lift{.constantVolts = 1.5};
 * // an arm that takes 2V to hold level, horizontal at 90 degrees on its sensor
 * sapphirelib::mechanism::GravityFeedforward arm{.cosineVolts = 2.0, .horizontalPosition = 90};
 * @endcode
 */
struct GravityFeedforward {
    /** volts added at every position, for an elevator lift */
    double constantVolts = 0.0;

    /** volts to hold an arm level, scaled by cos(arm angle) */
    double cosineVolts = 0.0;

    /** the position reading where the arm is horizontal */
    double horizontalPosition = 0.0;

    /**
     * arm degrees per unit of position reading. 1 for a sensor on the arm's pivot, 1 / ratio for
     * one on the motor side of a reduction
     */
    double armDegreesPerUnit = 1.0;

    /**
     * @brief Get the feedforward at a position: constantVolts + cosineVolts * cos(arm angle)
     *
     * @param position the position reading
     * @return double volts. Exactly constantVolts when cosineVolts is 0
     */
    double volts(double position) const;
};

/**
 * @brief Seat and rest at a hard stop
 *
 * With plain PID, a mechanism on its hard stop sits a little short, and a target past the stop has
 * the PID pushing into it forever. With this on, a target at or below the floor drives down with
 * at least seatVolts until the mechanism is within restBand of the floor, then cuts to 0V and rests
 */
struct SeatConfig {
    /** whether seating is on. false by default */
    bool enabled = false;

    /** the hard stop's position. Targets at or below it seat instead of using the PID */
    double floor = 0.0;

    /** minimum downward volts while seating, as a positive number. No feedforward is added */
    double seatVolts = 0.0;

    /** rest at 0V once the position is within this of the floor */
    double restBand = 0.0;
};

/**
 * @brief Settings for a position mechanism
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::mechanism::PositionConfig liftConfig{
 *     .pid = {.gains = {.kP = 0.3, .kI = 0.0, .kD = 0.01},
 *             .outputLimit = 12.0,
 *             .nominalDtS = 0.02},
 *     .gravity = {.constantVolts = 1.5},
 *     .seat = {.enabled = true, .floor = 0.0, .seatVolts = 3.0, .restBand = 5.0},
 *     .tolerance = 5.0,
 *     .settleTimeMs = 100,
 * };
 * @endcode
 */
struct PositionConfig {
    /**
     * PID settings: error in position units, output in volts. Set nominalDtS to how often update()
     * runs (0.02 for a 20ms opcontrol loop, 0.01 for startTask(10))
     */
    PID::Config pid{};

    /** gravity feedforward */
    GravityFeedforward gravity{};

    /** seat and rest at the hard stop */
    SeatConfig seat{};

    /** limit on the commanded volts, after feedforward. 12 by default */
    double maxVolts = 12.0;

    /** how close counts as at the target, in position units. 1 by default */
    double tolerance = 1.0;

    /** how long it must stay within tolerance to be settled, in milliseconds. 0 by default */
    std::uint32_t settleTimeMs = 0;

    /**
     * reset the PID when update() calls are further apart than this, in milliseconds, so the
     * derivative doesn't kick. 100 by default. 0 disables it
     */
    std::uint32_t resetAfterGapMs = 100;
};

/**
 * @brief Which branch of the control law ran. Its values never change; new ones go at the end
 */
enum class PositionLaw : std::uint8_t {
    track,      // PID on the target plus gravity feedforward
    seat,       // heading for the floor, driving down at least seatVolts
    rest,       // at the floor: 0V, PID reset
    sensorLost, // no position reading: brake, PID reset
    manual,     // open-loop volts from PositionMechanism::setVolts()
    off,        // brake: PositionMechanism::stop(), or its task while disabled
    external,   // something else has the motors, like a tuning run
};

/**
 * @brief Get a short name for a control law, 9 characters at most
 *
 * @param law the law
 * @return const char* "track", "seat", "rest", "no sensor", "manual", "off", or "external"
 */
const char* positionLawName(PositionLaw law);

/**
 * @brief What the control law decided to command
 */
struct PositionCommand {
    /** which branch ran */
    PositionLaw law = PositionLaw::off;

    /** volts to command, already clamped to +-maxVolts. 0 when braking */
    double volts = 0.0;

    /** whether to brake instead of commanding volts */
    bool brake = false;

    /**
     * @brief Get volts as millivolts for pros::Motor::move_voltage(), truncated toward zero
     */
    std::int32_t millivolts() const { return static_cast<std::int32_t>(volts * 1000.0); }
};

/**
 * @brief Run one step of the position control law
 *
 * In order:
 *   - position isn't finite: reset the PID and brake (sensorLost)
 *   - seat off, or target above the floor: PID plus gravity feedforward (track)
 *   - position above floor + restBand: PID, but at most -seatVolts (seat)
 *   - otherwise: 0V and reset the PID (rest)
 *
 * then clamp to +-maxVolts
 *
 * @param pid the live PID, whose gains may have been tuned since construction
 * @param config the mechanism's settings
 * @param target the target position. Must be finite
 * @param position the position reading
 * @return PositionCommand what to command
 */
PositionCommand computePositionCommand(PID& pid, const PositionConfig& config, double target,
                                       double position);

} // namespace sapphirelib::mechanism
