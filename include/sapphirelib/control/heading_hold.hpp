#pragma once

namespace sapphirelib {

/**
 * @brief Settings for heading hold driver control
 *
 * With heading hold, the turn stick steers a heading and the chassis is held on it, so letting go
 * leaves it pointed somewhere definite and bumps get corrected. See
 * HolonomicDrivetrain::holonomicFieldCentricHeadingHold()
 */
struct HeadingHoldConfig {
    /**
     * how fast a full turn stick moves the held heading, in degrees per second. 180 by default.
     * If turning feels slow, raise maxLeadDeg or the heading hold PID's kP first: past what the
     * chassis can keep up with, more slew does nothing
     */
    double slewDegPerSec = 180.0;

    /**
     * stick deadband, 0 to 1. 0.05 by default. A stick at rest still reads a count or two, and
     * without a deadband the held heading would slowly drift all match
     */
    double deadband = 0.05;

    /**
     * how far the held heading can get ahead of the chassis, in degrees. 20 by default. Stops the
     * chassis from coasting past where the driver let go. 0 or less disables the limit
     */
    double maxLeadDeg = 20.0;
};

/**
 * @brief Advance a held heading by one tick of turn stick input
 *
 * The result stays within maxLeadDeg of the current heading. It isn't wrapped to any range, since
 * it's only ever compared to a live heading with wrapDegrees180()
 *
 * @param heldHeadingDeg the held heading from the last tick, in degrees
 * @param currentHeadingDeg where the chassis points now, in degrees
 * @param turnInput turn stick, -1 to 1. Positive turns the way the heading grows
 * @param dtS time since the last call, in seconds. A bad value advances nothing
 * @param config heading hold settings
 * @return double the new held heading, in degrees
 */
double advanceHeldHeadingDeg(double heldHeadingDeg, double currentHeadingDeg, double turnInput,
                             double dtS, HeadingHoldConfig config);

} // namespace sapphirelib
