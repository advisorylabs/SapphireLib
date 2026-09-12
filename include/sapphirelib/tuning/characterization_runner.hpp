/**
 * \file sapphirelib/tuning/characterization_runner.hpp
 *
 * Drives one axis through the voltage ramps and steps
 * tuning::characterizeAxis() needs, recording what it measures — the piece
 * of system identification that has to run on the robot.
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
    /// turning. Any fixed origin; only differences matter.
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
    /// (fine for turning in place).
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
};

/// Runs, in order: a ramp forward, a ramp back, a step forward, a step back
/// — alternating direction so a translation axis ends up close to where it
/// began. Each segment ends at its travel limit or duration cap; between
/// segments the axis is commanded to 0V and given time to stop. Always
/// commands 0V before returning. Blocks for several seconds.
CharacterizationData runCharacterization(const CharacterizationConfig& config);

} // namespace sapphirelib::tuning
