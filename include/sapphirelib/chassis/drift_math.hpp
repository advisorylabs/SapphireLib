#pragma once

namespace sapphirelib::chassis {

/**
 * @brief Get the forward/backward drift over one tick of a strafe
 *
 * Takes the vertical tracking wheel's travel and removes the two parts that aren't drift: its arc
 * from the chassis turning, and the forward travel the command asked for (for a diagonal). Forward
 * travel the corner encoders report is deliberately kept, since uneven corners are the most common
 * cause of strafe drift
 *
 * @param verticalWheelDeltaIn the vertical tracking wheel's travel this tick, in inches
 * @param verticalOffsetIn the wheel's offset from the tracking center, in inches, as
 * odom::OdometryConfig::verticalOffsetIn takes it: positive is left of center
 * @param rotationDeltaDeg the chassis's heading change this tick, in degrees. Clockwise positive
 * @param strafeTravelIn sideways travel from the corner encoders this tick, in inches. Positive is
 * right
 * @param throttleVolts forward part of this tick's corner command, in volts
 * @param strafeVolts sideways part of this tick's corner command, in volts
 * @return double drift this tick, in inches. 0 when strafeVolts is 0
 */
double strafeDriftIn(double verticalWheelDeltaIn, double verticalOffsetIn, double rotationDeltaDeg,
                     double strafeTravelIn, double throttleVolts, double strafeVolts);

} // namespace sapphirelib::chassis
