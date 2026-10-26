#pragma once

#include <cstdint>

#include "sapphirelib/localization/field_map.hpp"
#include "sapphirelib/localization/sensor_model.hpp"

namespace sapphirelib::localization {

/**
 * @brief Which readings a distance reset trusts
 *
 * @b Example
 * @code {.cpp}
 * // Override: the loaders sit in the corners, so stay 18in clear of them, and never move the
 * // robot more than 8in on one reading
 * sapphirelib::localization::WallResetConfig config{.cornerMarginIn = 18, .maxJumpIn = 8};
 * @endcode
 */
struct WallResetConfig {
    /**
     * skip a reading whose beam meets the wall more than this far off square, in degrees. A
     * steep beam reads long off the slightest tilt. 30 by default
     */
    double maxAngleDeg = 30.0;

    /**
     * skip a reading that lands within this far of a corner, along the wall, in inches: near a
     * corner, a little heading error is the difference between one wall and the other, and
     * corners are where field elements sit (Override's loaders). 0 by default
     */
    double cornerMarginIn = 0.0;

    /**
     * skip a reset that would move the robot more than this, in inches: a reading that short is
     * likely off something in front of the wall (another robot, a game element). 0, the default,
     * never skips
     */
    double maxJumpIn = 0.0;
};

/**
 * @brief Which coordinate a distance reset sets
 */
enum class WallAxis : std::uint8_t {
    /** none: the reset was skipped */
    none,
    /** x, from the left or right wall */
    x,
    /** y, from the near or far wall */
    y,
};

/**
 * @brief What one distance reading says about the robot's x or y
 */
struct WallFix {
    /** whether the reading passed every check */
    bool ok = false;

    /** which coordinate it sets */
    WallAxis axis = WallAxis::none;

    /** the robot's x or y from the reading, in inches */
    double valueIn = 0.0;

    /** how far that moves the robot from where it thought it was, in inches */
    double shiftIn = 0.0;

    /** why it was skipped, in a few words. Empty when ok */
    const char* why = "";
};

/**
 * @brief Work out the robot's x or y from one distance sensor reading against the perimeter
 *
 * Traces the sensor's beam from the pose to whichever wall it heads for, and puts the robot where
 * that wall would be the reading away. Only the coordinate across that wall changes: a beam into
 * the right wall sets x, one into the far wall sets y. Skips readings that hit the wall too
 * steeply, land too near a corner, or would move the robot too far (see WallResetConfig). Only the
 * four perimeter walls count: anything else added to the map is ignored here
 *
 * @param map the field, for its walls
 * @param xIn the robot's x now, in inches
 * @param yIn the robot's y now, in inches
 * @param headingDeg the robot's heading, in degrees clockwise from +y
 * @param mount where the sensor is on the robot
 * @param readingIn what the sensor reads, in inches
 * @param config which readings to trust
 * @return WallFix the coordinate the reading gives, or why it was skipped
 */
WallFix wallFix(const FieldMap& map, double xIn, double yIn, double headingDeg,
                const DistanceSensorMount& mount, double readingIn, const WallResetConfig& config);

} // namespace sapphirelib::localization
