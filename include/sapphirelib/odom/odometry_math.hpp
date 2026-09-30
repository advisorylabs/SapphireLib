#pragma once

namespace sapphirelib::odom {

/**
 * @brief How far the robot moved on the field in one odometry update
 */
struct PoseDelta {
    /** change in x, in inches */
    double dxIn = 0.0;
    /** change in y, in inches */
    double dyIn = 0.0;
};

/**
 * @brief Work out one odometry update's movement on the field
 *
 * Removes each wheel's arc from the chassis turning (offset * radians turned), then rotates the
 * remaining local movement onto the field using the average heading across the update
 *
 * @param lastHeadingDeg heading before the update, 0-360 degrees, clockwise positive
 * @param headingDeg heading after the update
 * @param verticalDeltaIn vertical wheel travel since the last update, in inches. 0 if none
 * @param horizontalDeltaIn horizontal wheel travel since the last update, in inches. 0 if none
 * @param verticalOffsetIn vertical wheel offset from the tracking center, in inches
 * @param horizontalOffsetIn horizontal wheel offset from the tracking center, in inches
 * @return PoseDelta movement on the field
 */
PoseDelta computeOdometryDelta(double lastHeadingDeg, double headingDeg, double verticalDeltaIn,
                                double horizontalDeltaIn, double verticalOffsetIn,
                                double horizontalOffsetIn);

/**
 * @brief Work out a tracking wheel's offset from a turn in place
 *
 * Spin the chassis in place through a known rotation (say 10 turns, from unwrapped IMU readings)
 * and record how far the wheel traveled over it
 *
 * @param wheelDistanceIn the wheel's total travel, in inches
 * @param rotatedRadians the total rotation, in radians (10 turns is 10 * 2 * pi)
 * @return double the wheel's offset from the tracking center, in inches
 */
double calibrateTrackingWheelOffsetIn(double wheelDistanceIn, double rotatedRadians);

} // namespace sapphirelib::odom
