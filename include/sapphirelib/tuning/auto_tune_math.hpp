/**
 * \file sapphirelib/tuning/auto_tune_math.hpp
 *
 * Overshoot measurement and the Twiddle (coordinate-ascent) search that
 * drives automatic PID tuning — pure math, no PROS dependency, so it can be
 * unit-tested on a desktop compiler (see
 * tests/tuning/auto_tune_math_test.cpp). tuning::runAndSample() and the
 * gui::PidTunerPage auto-tune flow wrap this with the actual sensor
 * sampling and motion commands.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <vector>

#include "sapphirelib/control/pid.hpp"

namespace sapphirelib::tuning {

/// The result of measuring one test motion's response against its target.
struct OvershootResult {
    /// How far past `target` the signal peaked, in the direction of travel
    /// (0 if it never overshot).
    double overshoot = 0.0;

    /// Absolute error between `target` and the signal's last sample —
    /// non-zero if the motion didn't actually reach the target (e.g. it
    /// timed out short), which overshoot alone wouldn't catch.
    double finalError = 0.0;
};

/// Measures overshoot/final error from a time-ordered series of samples of
/// one signal (e.g. distance traveled, or heading) taken during a test
/// motion toward `target`. `samples` should start at (approximately) the
/// pre-motion value and end at the post-motion value; direction of travel
/// is inferred from `samples.front()` vs `target`, so this works whether
/// the target is above or below the starting value. `samples` must be
/// non-empty.
OvershootResult measureOvershoot(const std::vector<double>& samples, double target);

/// Combines an OvershootResult into a single scalar cost for the Twiddle
/// search to minimize — lower is better. `finalErrorWeight` controls how
/// much a motion that didn't reach its target counts against it, relative
/// to overshoot; without this term the degenerate "never move" response
/// would score as a perfect (zero-overshoot) result.
double autoTuneCost(const OvershootResult& result, double finalErrorWeight);

/// State for one Twiddle (coordinate-ascent hill-climbing) search over a
/// PID's three gains. Not meant to be inspected directly — construct with
/// makeTwiddleState(), and drive it entirely through currentGains()/
/// reportCost()/totalStepSize().
struct TwiddleState {
    double params[3];
    double stepSizes[3];
    int index = 0;
    bool triedDecrease = false;
    double bestCost = -1.0; // -1 = no measurement recorded yet
};

/// Starts a search from `initial`, with each gain's search step starting at
/// `stepSizes` (bigger = explores faster but coarser; the search shrinks a
/// gain's step whenever neither direction helps, and grows it whenever a
/// direction does, so the exact starting value mostly just affects how many
/// iterations convergence takes).
TwiddleState makeTwiddleState(PIDGains initial, PIDGains stepSizes);

/// The gains to test next. Test these (run the motion, measure its cost),
/// then call reportCost() with the result before asking again.
PIDGains currentGains(const TwiddleState& state);

/// Reports the cost measured using the gains currentGains() most recently
/// returned, and advances the search. Never proposes a negative gain
/// (clamped to 0) — a negative PID gain flips the controller's correction
/// direction, which isn't a "worse" result to search past, it's unsafe to
/// ever apply.
void reportCost(TwiddleState& state, double cost);

/// Sum of the search's current step sizes — shrinks as the search
/// converges. Twiddle traditionally stops once this drops below a small
/// tolerance; callers should also cap the number of iterations
/// independently, since a noisy or flat cost landscape can keep some step
/// sizes from ever shrinking below a fixed tolerance.
double totalStepSize(const TwiddleState& state);

} // namespace sapphirelib::tuning
