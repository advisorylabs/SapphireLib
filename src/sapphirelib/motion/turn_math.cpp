#include "sapphirelib/motion/turn_math.hpp"

#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::motion {

namespace {

// [0, 360)
double wrap360(double degrees) {
    const double wrapped = std::fmod(degrees, 360.0);
    return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

// how far the turn still has to go the way it's asked to: 0 to 360
double remainingDeg(double targetDeg, double headingDeg, TurnDirection direction) {
    return direction == TurnDirection::clockwise ? wrap360(targetDeg - headingDeg)
                                                 : wrap360(headingDeg - targetDeg);
}

} // namespace

DirectedTurn::DirectedTurn(double targetDeg, double startHeadingDeg, TurnDirection direction,
                           double overshootDeg)
    : targetDeg_(targetDeg), direction_(direction), forcing_(false) {
    if (direction_ == TurnDirection::shortest) return;
    const double remaining = remainingDeg(targetDeg_, startHeadingDeg, direction_);
    // past 180 the short way goes the other way, so this is where forcing it matters. Nearly a
    // full turn is the robot sitting just past the target, which turns back instead
    forcing_ = remaining > 180.0 && remaining < 360.0 - overshootDeg;
}

double DirectedTurn::errorDeg(double headingDeg) {
    if (forcing_) {
        const double remaining = remainingDeg(targetDeg_, headingDeg, direction_);
        // within 180 the short way agrees, and from here on it takes over. Pushed back across the
        // target (remaining wrapping to near 0) also lands here
        if (remaining > 180.0) {
            return direction_ == TurnDirection::clockwise ? remaining : -remaining;
        }
        forcing_ = false;
    }
    return wrapDegrees180(targetDeg_ - headingDeg);
}

bool DirectedTurn::forcing() const { return forcing_; }

} // namespace sapphirelib::motion
