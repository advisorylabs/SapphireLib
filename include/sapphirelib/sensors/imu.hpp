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

#include <atomic>
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
///
/// Safe to read from several tasks at once — odometry, the drive loop, the
/// GUI's HomePage and a tuning run all do. The cumulative tracking is
/// lock-free (no mutex for a deleted competition task to leave locked), and
/// a read the sensor can't answer — PROS_ERR_F while it's unplugged or
/// recalibrating after a brownout — is skipped rather than folded in, so it
/// can't turn the heading into NaN for the rest of the program. The first
/// good read after one becomes a fresh baseline, so a sensor that restarted
/// near 0 doesn't count that as a turn; only rotation during the gap is lost.
/// Non-copyable, like the drivetrains that own one.
class Imu {
public:
    /// `port` follows pros::Imu's convention. Blocks until IMU calibration
    /// finishes, so getHeadingDeg() is valid as soon as the constructor
    /// returns instead of reading 0 until calibration happens to finish on
    /// its own. Logs an error if calibration fails — see calibrated().
    explicit Imu(std::uint8_t port, double headingScale = 1.0);

    /// False if calibration failed at construction, typically because
    /// there's no IMU on the port (or the wrong device is). Headings from an
    /// uncalibrated Imu stay at 0 until it starts answering, so check this
    /// once at startup rather than trusting a motion to notice.
    bool calibrated() const;

    /// Absolute *field* heading, 0-360, clockwise-positive — same contract as
    /// pros::Imu::get_heading(), but with headingScale applied to cumulative
    /// rotation since construction before re-wrapping, and shifted by
    /// whatever setHeadingDeg() last set (nothing, until something calls it —
    /// then this reads 0 wherever the chassis faced at calibration, as
    /// before). Not const — every call advances the internal
    /// cumulative-rotation tracker, so call this (or
    /// getCumulativeHeadingDeg()) at your control loop's rate, not just
    /// occasionally, or an in-between multi-turn spin could wrap past 180
    /// degrees between reads and get misdetected as a much smaller turn the
    /// other way.
    double getHeadingDeg();

    /// Cumulative signed rotation since construction, in degrees, with
    /// headingScale applied and *not* wrapped to 0-360 — e.g. 3.5 full
    /// clockwise turns reads back as 1260, not 180. Useful for calibration
    /// (see calibrateHeadingScale()) and for detecting how many times the
    /// chassis has spun. Same call-frequency caveat as getHeadingDeg().
    ///
    /// Deliberately *not* shifted by setHeadingDeg(): it's rotation, not a
    /// heading, so re-framing the field heading never makes it jump.
    /// Everything that only takes differences of it — Auto-Tune's turn
    /// experiment, OdometryPage's offset calibration, the Asterisk drift
    /// correction — is unaffected by a setPose() landing mid-measurement.
    double getCumulativeHeadingDeg();

    /// Redefines the field heading so getHeadingDeg() reads `headingDeg`
    /// (any value; wrapped to 0-360) with the chassis where it is right now.
    /// It turns nothing and changes no rotation reading — it only picks
    /// which physical direction "heading 0" means from here on. Safe from any
    /// task.
    ///
    /// odom::Odometry::setPose() calls this on the Imu it was given, which is
    /// why odometry should share the drivetrain's imu() rather than open a
    /// second Imu on the same port: then turnToHeading(), moveToPose(), the
    /// odometry pose, and the HomePage readout are all in one frame.
    void setHeadingDeg(double headingDeg);

    /// What getHeadingDeg() adds to the scaled cumulative rotation, in
    /// degrees, (-180, 180]; 0 until setHeadingDeg() is called. For code that
    /// needs to convert between the field heading and the rotation frame
    /// getCumulativeHeadingDeg() reports in (odometry does, so a re-frame
    /// between two of its updates isn't mistaken for a turn).
    double headingOffsetDeg() const;

    void setHeadingScale(double headingScale);
    double headingScale() const;

private:
    static_assert(std::atomic<double>::is_always_lock_free,
                  "Imu's cross-task heading tracking relies on lock-free atomic doubles");

    pros::Imu imu_;
    std::atomic<double> headingScale_;

    /// The raw reading the next update's delta is measured from. NaN until
    /// the sensor has answered once, so a first good reading after a failed
    /// construction-time read becomes the baseline instead of a jump.
    std::atomic<double> lastRawHeadingDeg_;
    std::atomic<double> rawCumulativeDeg_{0.0};

    /// See setHeadingDeg(). A separate atomic from the rotation tracking, so
    /// re-framing never touches the deltas several tasks are adding up.
    std::atomic<double> headingOffsetDeg_{0.0};
    bool calibrated_ = false;

    void updateCumulative();
};

} // namespace sapphirelib::sensors
