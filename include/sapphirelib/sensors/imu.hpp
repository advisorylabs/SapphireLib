/**
 * \file sapphirelib/sensors/imu.hpp
 *
 * Calibrated drop-in replacement for pros::Imu, used everywhere SapphireLib
 * needs a heading source (TankDrivetrain, HolonomicDrivetrain,
 * odom::Odometry) instead of a raw pros::Imu — see the class comment for
 * why.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

#include "pros/imu.hpp"

namespace sapphirelib::sensors {

/// The V5 IMU under- or over-reports heading change by a small, fairly
/// consistent percentage — negligible over one turn, but compounds badly
/// over a match's worth of turning (a few degrees of error per rotation
/// becomes tens of degrees after enough turns). Imu corrects for this: it
/// tracks cumulative (unwrapped) rotation internally and multiplies it by a
/// calibrated `headingScale` before re-wrapping to a 0-360 heading, instead
/// of scaling the raw 0-360 reading directly (which would be meaningless —
/// there's no sane way to "scale" a value that wraps at 360). Calibrate
/// once per robot with calibrateHeadingScale() (imu_scale_math.hpp), then
/// pass the resulting scale to every Imu you construct for that robot.
/// `headingScale = 1.0` (the default) disables correction entirely, so this
/// is a safe drop-in for a raw pros::Imu even before you've calibrated one.
class Imu {
public:
    /// `port` follows pros::Imu's convention. Blocks until IMU calibration
    /// finishes, so getHeadingDeg() is valid as soon as the constructor
    /// returns instead of reading 0 until calibration happens to finish on
    /// its own.
    explicit Imu(std::uint8_t port, double headingScale = 1.0);

    /// Absolute heading, 0-360, clockwise-positive — same contract as
    /// pros::Imu::get_heading(), but with headingScale applied to
    /// cumulative rotation since construction before re-wrapping. Not const
    /// — every call advances the internal cumulative-rotation tracker, so
    /// call this (or getCumulativeHeadingDeg()) at your control loop's rate,
    /// not just occasionally, or an in-between multi-turn spin could wrap
    /// past 180 degrees between reads and get misdetected as a much smaller
    /// turn the other way.
    double getHeadingDeg();

    /// Cumulative signed rotation since construction, in degrees, with
    /// headingScale applied and *not* wrapped to 0-360 — e.g. 3.5 full
    /// clockwise turns reads back as 1260, not 180. Useful for calibration
    /// (see calibrateHeadingScale()) and for detecting how many times the
    /// chassis has spun. Same call-frequency caveat as getHeadingDeg().
    double getCumulativeHeadingDeg();

    void setHeadingScale(double headingScale);
    double headingScale() const;

private:
    pros::Imu imu_;
    double headingScale_;
    double lastRawHeadingDeg_;
    double rawCumulativeDeg_ = 0.0;

    void updateCumulative();
};

} // namespace sapphirelib::sensors
