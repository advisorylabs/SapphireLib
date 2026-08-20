/**
 * \file sapphirelib/gui/odometry_page.hpp
 *
 * SapphireLib's default pose-visualization page.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::gui {

/// Numeric x/y/heading readout plus a small live position-and-heading
/// indicator on a scaled field rectangle. Optionally also an auto-offset-
/// calibration button — see enableOffsetCalibration().
class OdometryPage : public Page {
public:
    /// `odometry` must outlive this page. `fieldWidthIn`/`fieldHeightIn`
    /// default to a 12x12ft VRC field (144in square) — override for a
    /// different game/field size.
    explicit OdometryPage(odom::Odometry& odometry, double fieldWidthIn = 144.0,
                          double fieldHeightIn = 144.0);

    /// Adds a "Calibrate Offsets" button that automates
    /// odom::calibrateTrackingWheelOffsetIn(): spins the chassis in place
    /// for `turns` full rotations (negative spins the other way) — tracked
    /// via `imu`'s cumulative heading, so the exact spin speed doesn't
    /// matter — while recording how far each tracking wheel travels over
    /// that same rotation, then applies
    /// the computed offset(s) directly to the live Odometry passed to the
    /// constructor. Pass nullptr for whichever wheel isn't wired up
    /// (matching Odometry::Sensors) to skip calibrating that axis; passing
    /// both nullptr makes the button a no-op. `verticalWheel`/
    /// `horizontalWheel` should be the same TrackingWheel objects given to
    /// that Odometry's Sensors — reading a sensor from two places is safe,
    /// only *commanding* a device from more than one place would conflict.
    ///
    /// `setSpin` drives the chassis purely in place given a normalized
    /// [-1, 1] turn command — e.g.
    /// `[&](double turn){ drivetrain.holonomic(0.0, 0.0, turn); }` — so this
    /// page doesn't depend on any concrete drivetrain type. It's called
    /// with 0.0 to stop once the target rotation is reached.
    ///
    /// Runs on a background task, like Run Test/Auto-Tune elsewhere in the
    /// GUI, so the screen doesn't freeze for the several seconds a
    /// calibration spin takes. Must be called before this page is passed to
    /// Gui::addPage() (which is when build() actually creates the button).
    void enableOffsetCalibration(sensors::Imu& imu, const odom::TrackingWheel* verticalWheel,
                                 const odom::TrackingWheel* horizontalWheel,
                                 std::function<void(double)> setSpin, double spinPower = 0.35,
                                 double turns = 8.0);

    /// True from the moment "Calibrate Offsets" is tapped until the spin
    /// finishes and the offset(s) are applied. Driver-control code (e.g.
    /// opcontrol()'s joystick loop) should skip calling
    /// holonomic()/holonomicFieldCentric() while this is true — those calls
    /// unconditionally command the same motors the calibration spin is
    /// using, so a concurrent driver-control loop (which keeps running
    /// whenever there's no competition switch, even with centered sticks)
    /// will fight it: the spin twitches once, then gets overwritten back
    /// toward zero on the driver-control loop's next tick, and the
    /// calibration hangs forever waiting for rotation that's no longer
    /// happening.
    bool isCalibrating() const;

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    void runCalibration();
    static void calibrateClicked(lv_event_t* e);

    odom::Odometry& odometry_;
    double fieldWidthIn_;
    double fieldHeightIn_;

    lv_obj_t* poseLabel_ = nullptr;
    lv_obj_t* fieldView_ = nullptr;
    lv_obj_t* robotDot_ = nullptr;
    lv_obj_t* headingLine_ = nullptr;
    std::int32_t fieldViewWidthPx_ = 0;
    std::int32_t fieldViewHeightPx_ = 0;

    // --- Offset calibration — see enableOffsetCalibration() ---
    sensors::Imu* calibImu_ = nullptr;
    const odom::TrackingWheel* calibVertical_ = nullptr;
    const odom::TrackingWheel* calibHorizontal_ = nullptr;
    std::function<void(double)> calibSetSpin_;
    double calibSpinPower_ = 0.35;
    double calibTurns_ = 8.0;

    lv_obj_t* calibrateButton_ = nullptr;
    lv_obj_t* calibStatusLabel_ = nullptr;

    std::atomic<bool> calibrating_{false};
    std::atomic<bool> calibResultsReady_{false};
    std::atomic<double> calibVerticalOffsetIn_{0.0};
    std::atomic<double> calibHorizontalOffsetIn_{0.0};
};

} // namespace sapphirelib::gui
