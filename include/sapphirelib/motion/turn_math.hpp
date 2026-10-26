#pragma once

#include <cstdint>

namespace sapphirelib::motion {

/**
 * @brief Which way a turn goes
 */
enum class TurnDirection : std::uint8_t {
    /** the short way. What every turn does by default */
    shortest,
    /** to the right (heading increasing), even when left is shorter */
    clockwise,
    /** to the left (heading decreasing), even when right is shorter */
    counterclockwise,
};

/**
 * @brief The heading error for a turn, kept going one way until the short way agrees
 *
 * A turn of about 180 degrees can go either way depending on a fraction of a degree of heading,
 * so a routine that needs it to go right says so. The error is the long way round (past 180
 * degrees) until the robot is within 180 degrees of the target the way it's turning, then the
 * ordinary short-way error, so a small overshoot at the end is corrected back instead of going
 * all the way round again. A robot that starts up to overshootDeg past the target in the asked
 * direction turns back the short way, rather than nearly a full turn
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::motion::DirectedTurn turn(180, imu.heading(), TurnDirection::clockwise);
 * while (...) {
 *     double error = turn.errorDeg(imu.heading()); // positive turns right
 * }
 * @endcode
 */
class DirectedTurn {
public:
    /**
     * @brief Construct a new DirectedTurn
     *
     * @param targetDeg the heading to end on, in degrees
     * @param startHeadingDeg the heading the turn starts from, in degrees
     * @param direction which way to turn
     * @param overshootDeg how far past the target counts as already there. 5 by default
     */
    DirectedTurn(double targetDeg, double startHeadingDeg, TurnDirection direction,
                 double overshootDeg = 5.0);

    /**
     * @brief Get the error from a heading, in degrees: positive turns right (clockwise)
     *
     * @param headingDeg the heading now, in degrees
     * @return double the error, (-360, 360)
     */
    double errorDeg(double headingDeg);

    /**
     * @brief Whether the turn is still going the long way round
     */
    bool forcing() const;

private:
    double targetDeg_;
    TurnDirection direction_;
    bool forcing_;
};

} // namespace sapphirelib::motion
