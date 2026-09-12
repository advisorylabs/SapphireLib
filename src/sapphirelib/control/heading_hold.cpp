#include "sapphirelib/control/heading_hold.hpp"

#include <algorithm>
#include <cmath>

#include "sapphirelib/util/angle.hpp"

namespace sapphirelib {

namespace {

/// Matches PID::update()'s own guard: anything longer is a scheduling
/// hiccup, not elapsed control time, and integrating a stick across it would
/// swing the held heading by an arbitrary amount.
constexpr double kMaxPlausibleDtS = 0.5;

/// A deadband this wide leaves no usable stick travel above it, so treat
/// anything past it as a misconfiguration rather than dividing the rescale
/// below by zero (or by a negative).
constexpr double kMaxDeadband = 0.9;

/// Removes the deadband from `input` and rescales what's left to span the
/// full [-1, 1] range, so the first degree of stick past the deadband
/// commands a rate just above zero instead of jumping straight to
/// `deadband * slewDegPerSec`.
double applyDeadband(double input, double deadband) {
    deadband = std::clamp(deadband, 0.0, kMaxDeadband);

    const double magnitude = std::fabs(input);
    if (magnitude <= deadband) return 0.0;

    return std::copysign((magnitude - deadband) / (1.0 - deadband), input);
}

} // namespace

double advanceHeldHeadingDeg(double heldHeadingDeg, double currentHeadingDeg, double turnInput,
                             double dtS, HeadingHoldConfig config) {
    if (dtS > 0.0 && dtS <= kMaxPlausibleDtS) {
        const double rate = applyDeadband(turnInput, config.deadband) * config.slewDegPerSec;
        heldHeadingDeg += rate * dtS;
    }

    // Re-anchor to where the chassis actually is. Done every tick, not just
    // while the stick is deflected: the chassis can also fall behind by
    // being shoved, and this is what stops the resulting error from being
    // banked indefinitely.
    //
    // Going through wrapDegrees180() is what makes this work across the
    // 0/360 seam — a held heading of 5 against a live heading of 355 is 10
    // degrees of lead, not 350.
    if (config.maxLeadDeg > 0.0) {
        const double leadDeg = wrapDegrees180(heldHeadingDeg - currentHeadingDeg);
        heldHeadingDeg =
            currentHeadingDeg + std::clamp(leadDeg, -config.maxLeadDeg, config.maxLeadDeg);
    }

    return heldHeadingDeg;
}

} // namespace sapphirelib
