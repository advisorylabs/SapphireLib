#pragma once

#include <cstdint>

namespace sapphirelib {

/**
 * @brief Gains for a PID controller
 *
 * The gains are continuous-time: kI is in output per (error * second) and kD in output per
 * (error / second). PID::update() scales by its timestep, so the gains stay valid if the loop
 * period changes, and gains from tuning::designPositionGains() can be used directly
 */
struct PIDGains {
    /** proportional gain */
    double kP = 0.0;
    /** integral gain */
    double kI = 0.0;
    /** derivative gain */
    double kD = 0.0;
};

class PID;

/**
 * @brief Everything one PID::update() worked out, not just its output
 *
 * Used by telemetry and readouts to show why the controller did what it did. Some loops pass the
 * error as the target and 0 as the measurement (the drivetrain's turn loops, the pose motions'
 * distance loops), so read error rather than target or measurement
 */
struct PidStep {
    /** flag: the output hit Config::outputLimit and was clamped */
    static constexpr std::uint8_t kSaturated = 1u << 0;
    /** flag: Config::slewRate limited this step's change in output */
    static constexpr std::uint8_t kSlewLimited = 1u << 1;
    /** flag: anti-windup undid this step's integration */
    static constexpr std::uint8_t kIntegralHeld = 1u << 2;
    /** flag: first update() since construction or reset(), so no derivative yet */
    static constexpr std::uint8_t kFirstStep = 1u << 3;
    /** flag: the timestep passed in was bad, so Config::nominalDtS was used */
    static constexpr std::uint8_t kDtFallback = 1u << 4;

    /** the target passed to update() */
    double target = 0.0;
    /** the measurement passed to update() */
    double measurement = 0.0;
    /** target minus measurement */
    double error = 0.0;

    /** kP * error, in output units */
    double pTerm = 0.0;
    /** kI * integral, in output units */
    double iTerm = 0.0;
    /** kD * derivative, in output units */
    double dTerm = 0.0;

    /** output after anti-windup, before slew limiting and clamping */
    double rawOutput = 0.0;
    /** what update() returned */
    double output = 0.0;
    /** the timestep used, in seconds */
    double dtS = 0.0;
    /** the k* flags above */
    std::uint8_t flags = 0;
};

/**
 * @brief Receives every step of the PID it's attached to. See PID::setObserver()
 */
class PidObserver {
public:
    virtual ~PidObserver() = default;

    /**
     * @brief Called at the end of every update(), on the task that called it
     *
     * @note runs inside a control loop, so it must not block or allocate
     *
     * @param pid the PID that updated
     * @param step what the update worked out
     */
    virtual void onPidUpdate(const PID& pid, const PidStep& step) = 0;

    /**
     * @brief Called from reset(), only when the PID had state to clear
     *
     * @param pid the PID that was reset
     */
    virtual void onPidReset(const PID& /*pid*/) {}
};

/**
 * @brief PID controller
 *
 * One instance drives one control loop, e.g. drive distance or heading
 */
class PID {
public:
    /**
     * @brief PID settings
     */
    struct Config {
        /** kP, kI, and kD */
        PIDGains gains;

        /**
         * integral limit, in error * seconds. The integral is clamped to +-integralLimit before
         * it's multiplied by kI. 0 disables the limit, which is fine when outputLimit is set,
         * since that adds its own anti-windup
         */
        double integralLimit = 0.0;

        /**
         * output limit. The output is clamped to +-outputLimit, and while it's saturated the
         * integral stops growing (anti-windup). 0 disables the limit
         */
        double outputLimit = 0.0;

        /**
         * the most the output can change between update() calls. Per call, not per second, so it
         * ramps twice as fast in a 10ms loop as in a 20ms one. 0 disables the limit
         */
        double slewRate = 0.0;

        /**
         * whether to take the derivative of the measurement instead of the error, which avoids a
         * kick when the target jumps. false by default. Leave it false for loops that pass 0 as
         * the measurement, like the drivetrain's turn loops
         */
        bool derivativeOnMeasurement = false;

        /**
         * timestep assumed by the two-argument update(), in seconds. 0.01 by default, matching
         * the drivetrains' 10ms loops. A fixed timestep keeps loop jitter out of the derivative
         */
        double nominalDtS = 0.01;
    };

    /**
     * @brief Construct a new PID
     *
     * @param config gains and settings
     *
     * @b Example
     * @code {.cpp}
     * // a lift PID with kP 0.3, kD 0.01, and its output limited to 12V
     * sapphirelib::PID liftPID({.gains = {.kP = 0.3, .kI = 0.0, .kD = 0.01}, .outputLimit = 12.0});
     * @endcode
     */
    explicit PID(Config config);

    /**
     * @brief Update the PID, assuming Config::nominalDtS has passed since the last call
     *
     * @param target where the system should be
     * @param measurement where the system is, in the same units as target
     * @return double the output, clamped and slew limited per Config
     *
     * @b Example
     * @code {.cpp}
     * while (true) {
     *     double volts = liftPID.update(90.0, liftSensor.get_position() / 100.0);
     *     liftMotors.move_voltage(volts * 1000);
     *     pros::delay(10);
     * }
     * @endcode
     */
    double update(double target, double measurement);

    /**
     * @brief Update the PID over a given timestep, for loops that don't run at a fixed rate
     *
     * @param target where the system should be
     * @param measurement where the system is, in the same units as target
     * @param dtS time since the last call, in seconds. A bad value falls back to nominalDtS
     * @return double the output, clamped and slew limited per Config
     */
    double update(double target, double measurement, double dtS);

    /**
     * @brief Reset the integral, the derivative, and the previous output. Call this before
     * reusing the PID for a new motion
     */
    void reset();

    /**
     * @brief Set the gains
     *
     * @param gains the new kP, kI, and kD
     */
    void setGains(PIDGains gains);

    /**
     * @brief Get the gains
     */
    const PIDGains& gains() const;

    /**
     * @brief Get the config, with the gains as last set
     */
    const Config& config() const;

    /**
     * @brief Get what the last update() worked out. All zeros before the first update
     *
     * reset() leaves it alone, so a readout can still show the last step of a finished motion
     *
     * @note read it from the task that calls update(), or accept a torn read
     */
    const PidStep& lastStep() const;

    /**
     * @brief Attach an observer that receives every step and reset
     *
     * This is how telemetry::Logger::pid() records a controller
     *
     * @note not synchronized: attach it before any task starts updating this PID. The observer
     * must outlive the attachment
     *
     * @param observer the observer. nullptr detaches
     */
    void setObserver(PidObserver* observer);

    /**
     * @brief Get the attached observer, or nullptr
     */
    PidObserver* observer() const;

private:
    Config config_;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    double prevMeasurement_ = 0.0;
    double prevOutput_ = 0.0;
    bool hasPrev_ = false;
    PidStep lastStep_;
    PidObserver* observer_ = nullptr;
};

} // namespace sapphirelib
