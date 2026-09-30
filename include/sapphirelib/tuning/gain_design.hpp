#pragma once

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/control/pid.hpp"

// pole placement: with feedforward cancelling friction, one axis under PD control is a spring and
// damper, kA * x'' + (kV + kD) * x' + kP * x = kP * target. Picking how fast it responds (natural
// frequency w) and how much it may overshoot (damping ratio z) sets both gains:
// kP = kA * w^2, kD = 2 * z * w * kA - kV. The formula doesn't know about delay, so
// designPositionGains() checks the phase margin the delay leaves and slows the design down until
// it's safe

namespace sapphirelib::tuning {

/**
 * @brief How one closed loop motion should behave
 *
 * Different uses of one axis can want different specs (a snappy autonomous turn, soft driver
 * heading hold), all designed from the same model
 */
struct ResponseSpec {
    /** time to settle within 2% of the step, in seconds. 0.6 by default */
    double settleTimeS = 0.6;

    /**
     * damping ratio. 1 (the default) is the fastest response with no overshoot; below 1 trades
     * overshoot for speed. Clamped to [0.1, 5]
     */
    double dampingRatio = 1.0;

    /**
     * lowest acceptable phase margin with the measured delay, in degrees. 45-60 is usual; lower
     * is faster but rings more and handles less change in the robot. 50 by default
     */
    double minPhaseMarginDeg = 50.0;
};

/**
 * @brief Result of designPositionGains()
 */
struct GainDesign {
    /** false only for an invalid model or spec. A design slowed for delay is still ok */
    bool ok = false;

    /** kP and kD. kI is always 0, since feedforward already handles friction */
    PIDGains gains;

    /** the natural frequency used, in rad/s */
    double naturalFrequency = 0.0;

    /** the settle time the design achieves, in seconds. Longer than asked if limitedByDelay */
    double settleTimeS = 0.0;

    /** phase margin with the given delay, in degrees */
    double phaseMarginDeg = 0.0;

    /** whether the design had to be slowed down to meet minPhaseMarginDeg */
    bool limitedByDelay = false;

    /**
     * kS / kP: the largest error static friction can hold the loop at. A band wider than the exit
     * threshold (or mechanism tolerance) is a motion that may never settle. Infinity when kP is 0
     */
    double staticErrorBound = 0.0;
};

/**
 * @brief Get the 2% settle time of a spring and damper with a natural frequency of 1
 *
 * Divide by a settle time to get the natural frequency that achieves it. Critical damping settles
 * in about 5.83 / w
 *
 * @param dampingRatio the damping ratio, clamped to [0.1, 5]
 * @return double settle time, in seconds, at w = 1
 */
double normalizedSettleTime(double dampingRatio);

/**
 * @brief Get the phase margin of PD gains on an axis with delay
 *
 * @param model the axis model
 * @param gains kP and kD. kI is ignored
 * @param delayS delay, in seconds
 * @return double phase margin, in degrees. 180 when kP is 0; negative is unstable
 */
double phaseMarginDeg(const MotorFeedforward& model, PIDGains gains, double delayS);

/**
 * @brief Design PD gains for position control of one axis
 *
 * If kV alone already damps more than asked, kD would be negative; it's clamped to 0 instead. A
 * mechanism is designed from its MechanismModel's motion, with gravity cancelled by feedforward
 *
 * @param model the axis model
 * @param spec how the motion should behave
 * @param delayS the axis's response delay, in seconds. 0 by default
 * @return GainDesign the gains and how the design turned out
 *
 * @b Example
 * @code {.cpp}
 * auto design = sapphirelib::tuning::designPositionGains(
 *     result.fit.model, {.settleTimeS = 0.5, .dampingRatio = 1.0}, result.delayS);
 * if (design.ok) drivetrain().turnPID().setGains(design.gains);
 * @endcode
 */
GainDesign designPositionGains(const MotorFeedforward& model, ResponseSpec spec,
                               double delayS = 0.0);

} // namespace sapphirelib::tuning
