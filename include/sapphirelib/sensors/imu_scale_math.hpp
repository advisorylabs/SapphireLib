/**
 * \file sapphirelib/sensors/imu_scale_math.hpp
 *
 * The V5 IMU under- or over-reports heading change by a small, fairly
 * consistent percentage — negligible over one turn, but compounds badly
 * over a match's worth of turning. This is the pure math behind correcting
 * for it: track cumulative (unwrapped) rotation from the raw sensor, scale
 * that cumulative value by a calibrated factor, then re-wrap to a 0-360
 * heading. No PROS dependency — see
 * tests/sensors/imu_scale_math_test.cpp. sensors::Imu (imu.hpp) wraps this
 * with the actual sensor reads.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::sensors {

/// The wrapped signed change from `lastRawHeadingDeg` to `rawHeadingDeg`
/// (both 0-360, clockwise-positive, matching pros::Imu::get_heading()) —
/// the piece of cumulative-rotation tracking that's pure math. Assumes
/// consecutive calls are frequent enough that the true rotation between
/// them is under 180 degrees, true for any reasonable control loop rate.
double rawHeadingDeltaDeg(double lastRawHeadingDeg, double rawHeadingDeg);

/// Wraps a cumulative (unwrapped, any magnitude) degree value to [0, 360) —
/// turns a scaled cumulative rotation back into an absolute heading
/// reading.
double wrapDegrees360(double degrees);

/// Calibration helper: with an Imu constructed at headingScale = 1.0 (no
/// correction), physically rotate the chassis a known number of full turns
/// — more turns average out the IMU's per-turn error better — then pass how
/// many turns actually happened (`actualTurns`, e.g. 10.0) and how many the
/// Imu measured over that same rotation (`measuredTurns`, e.g.
/// imu.getCumulativeHeadingDeg() / 360.0). Returns the headingScale to
/// construct future Imu instances for this robot with.
double calibrateHeadingScale(double actualTurns, double measuredTurns);

} // namespace sapphirelib::sensors
