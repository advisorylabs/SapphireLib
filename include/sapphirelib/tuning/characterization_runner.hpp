/**
 * \file sapphirelib/tuning/characterization_runner.hpp
 *
 * Drives one axis through the voltage ramps and steps
 * tuning::characterizeAxis() (or characterizeMechanism()) needs, recording
 * what it measures — the piece of system identification that has to run on
 * the robot. It reads time and sleeps only through util/clock.hpp, so a host
 * test runs it against a simulated axis with a fake clock
 * (tests/tuning/characterization_runner_test.cpp).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <functional>

#include "sapphirelib/tuning/characterization_math.hpp"

namespace sapphirelib::tuning {

/// Configuration for characterizing one axis — see runCharacterization().
struct CharacterizationConfig {
    /// Applies a raw open-loop voltage to this axis alone, before wheel
    /// mixing (e.g. HolonomicDrivetrain::holonomicVolts(v, 0, 0) for
    /// forward). Must be the same kind of command the axis's PIDs output,
    /// or the model won't describe what those PIDs are driving.
    std::function<void(double)> actuate;

    /// Reads the axis's position in the units its PIDs work in — inches
    /// along the axis for translation, *cumulative* (unwrapped) degrees for
    /// turning. Any fixed origin; only differences matter. A non-finite
    /// reading (a sensor that stopped answering) ends the segment, so the run
    /// never drives on blind.
    std::function<double()> measure;

    /// Voltage each step jumps to. High enough to reach a good fraction of
    /// top speed, since that's where kA and delay show most clearly.
    double stepVolts = 6.0;

    /// How fast each ramp raises the voltage, and the most it ramps to.
    double rampVoltsPerS = 4.0;
    double rampMaxVolts = 8.0;

    /// Each segment stops once it has moved this far from where it started,
    /// in measure()'s units — the only thing keeping a translation run on
    /// the field. Leave room for coasting after cut-off. 0 means unlimited
    /// (fine for turning in place). Ignored by runMechanismCharacterization(),
    /// which has absolute limits instead.
    double maxTravel = 0.0;

    /// Hard cap on any one segment's duration.
    std::uint32_t maxSegmentMs = 2500;

    /// Samples recorded at 0V before each segment's voltage starts, so the
    /// first moving samples have a full differentiation window behind them.
    std::uint32_t preRollMs = 100;

    /// Slowest speed, in measure()'s units per second, that counts toward
    /// the fit — see tuning::fitFeedforward(). Pick something comfortably
    /// above what sensor noise alone reads while sitting still: a few in/s
    /// for translation, a few deg/s for turning.
    double minSpeed = 2.0;

    /// Longest to wait for the axis to stop between segments.
    std::uint32_t settleTimeoutMs = 1500;

    std::uint32_t samplePeriodMs = 10;

    /// Checked before every sample and while waiting for the axis to stop.
    /// True ends the run at once — 0V (or the hold, for a mechanism), no
    /// further segments — and the result comes back with
    /// CharacterizationData::aborted set. For a Stop button, or the robot
    /// being disabled mid-run. Empty never aborts.
    std::function<bool()> shouldAbort{};

    /// Called once before the run commands anything, and once after its last
    /// command, however it ended (finished, aborted, a lost sensor). For
    /// taking an axis's motors away from whatever else commands them for the
    /// duration — e.g. PositionMechanism::beginExternalControl() and
    /// endExternalControl() — or for marking the run in a log. Run on the
    /// characterization task. Either may be empty.
    std::function<void()> start{};
    std::function<void()> finish{};
};

/// Runs, in order: a ramp forward, a ramp back, a step forward, a step back
/// — alternating direction so a translation axis ends up close to where it
/// began. Each segment ends at its travel limit or duration cap; between
/// segments the axis is commanded to 0V and given time to stop. Always
/// commands 0V before returning. Blocks for several seconds.
CharacterizationData runCharacterization(const CharacterizationConfig& config);

/// Configuration for characterizing a lift, an arm, or anything else gravity
/// loads — see runMechanismCharacterization().
struct MechanismCharacterizationConfig {
    /// actuate/measure, the voltages, and the timing, as for a drive axis —
    /// with "forward" meaning up: a positive voltage must raise the
    /// measurement. `maxTravel` is ignored; the limits below bound the travel.
    CharacterizationConfig axis{};

    /// Absolute travel bounds, in measure()'s units. An upward segment ends
    /// once the reading reaches upperLimit, a downward one once it reaches
    /// lowerLimit. The mechanism keeps moving for a moment after that while
    /// its motors brake, so keep each limit short of its hard stop by more
    /// than that. Start the run with the mechanism at or near lowerLimit.
    double lowerLimit = 0.0;
    double upperLimit = 0.0;

    /// The downward step's volts, as a magnitude; 0 uses axis.stepVolts.
    /// Gravity helps a lift down, so the same volts reach a much higher
    /// speed going down than up — often worth asking for less.
    double downStepVolts = 0.0;

    /// Holds the mechanism still: between segments, during each pre-roll,
    /// and at the end — e.g. motors().brake() with the brake mode set to
    /// hold. Empty commands 0V instead, which only suits a mechanism that
    /// stays put unpowered. Samples taken while held record NaN volts, which
    /// the fit skips (see CharacterizationSample).
    std::function<void()> hold{};

    /// How gravity loads it — what characterizeMechanism() fits kG against.
    GravityShape gravity{};
};

/// A mechanism's characterization: ramp up, ramp down, step up, step down,
/// each from rest (held) at the far end of the last, each ending at the limit
/// in its direction or at the duration cap. Up and down both matter — friction
/// opposes the motion either way while gravity always pulls down, which is
/// the only thing that separates kS from kG. Holds the mechanism between
/// segments and before returning. Blocks for several seconds. Feed the
/// result to characterizeMechanism() with the same `gravity`.
CharacterizationData runMechanismCharacterization(const MechanismCharacterizationConfig& config);

} // namespace sapphirelib::tuning
