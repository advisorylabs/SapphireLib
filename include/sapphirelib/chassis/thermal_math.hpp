#pragma once

namespace sapphirelib::chassis {

/**
 * @brief One value per corner wheel, in HolonomicDrivetrain's port order
 */
struct CornerValues {
    double frontLeft = 0.0;
    double frontRight = 0.0;
    double backLeft = 0.0;
    double backRight = 0.0;
};

/**
 * @brief What the center wheels should add to their command to make up for hot corners
 */
struct CenterCorrection {
    /** added to both center wheels, for missing or unwanted forward/backward thrust, in volts */
    double commonVolts = 0.0;

    /** added to the left center wheel and subtracted from the right, in volts */
    double differentialVolts = 0.0;
};

/**
 * @brief Estimate how much power a V5 motor still delivers at a temperature
 *
 * The V5 cuts power in steps (about 50% at 55C, 25% at 60C, 12.5% at 65C, off at 70C). This ramps
 * across a couple of degrees at each step instead of jumping, so a reading that dithers across a
 * step doesn't make anything driven from it jump back and forth
 *
 * @param tempC motor temperature, in degrees Celsius. A non-finite reading (an unplugged motor)
 * reads as 1
 * @return double fraction of full power, 0 to 1
 *
 * @b Example
 * @code {.cpp}
 * double fraction = sapphirelib::chassis::thermalPowerFraction(motor.get_temperature());
 * @endcode
 */
double thermalPowerFraction(double tempC);

/**
 * @brief Work out what the center wheels should add so the chassis does what the corners were
 * asked to do
 *
 * Each corner falls short by commandedVolts * (1 - survivingFraction). The shortfalls are summed
 * with the mixer's forward and yaw patterns and split across the two center wheels. Corners
 * heating together while driving forward make the center wheels drive harder; one hot corner
 * while strafing makes them hold the chassis straight (they can't add sideways thrust, so the
 * strafe is still slower)
 *
 * @param commandedVolts each corner's commanded voltage
 * @param survivingFraction each corner's thermalPowerFraction()
 * @param centerPowerFraction the center wheels' own power fraction. Fades the correction out as
 * they heat up
 * @param gain how much of the shortfall to make up. 0 disables it
 * @param maxVolts the most each component can be, in volts
 * @return CenterCorrection what to add to the center wheels
 */
CenterCorrection centerThermalCorrection(CornerValues commandedVolts,
                                         CornerValues survivingFraction,
                                         double centerPowerFraction, double gain,
                                         double maxVolts);

} // namespace sapphirelib::chassis
