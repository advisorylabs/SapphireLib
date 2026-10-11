#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/localization/monte_carlo_localizer.hpp"
#include "sapphirelib/localization/mount_calibration.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"
#include "sapphirelib/sensors/imu.hpp"

namespace sapphirelib::gui {

/**
 * @brief Pose readout and a live field view, with optional tracking wheel offset and distance
 * sensor mount calibrations
 *
 * @b Example
 * @code {.cpp}
 * gui.addPage(std::make_unique<sapphirelib::gui::OdometryPage>(odometry()));
 * @endcode
 */
class OdometryPage : public Page {
public:
    /**
     * @brief Construct a new OdometryPage
     *
     * @param odometry the odometry to show. Must outlive the page
     * @param fieldWidthIn field width, in inches. 144 by default
     * @param fieldHeightIn field height, in inches. 144 by default
     */
    explicit OdometryPage(odom::Odometry& odometry, double fieldWidthIn = 144.0,
                          double fieldHeightIn = 144.0);

    /**
     * @brief Add a "Calibrate Offsets" button that measures the tracking wheel offsets
     *
     * Spins the chassis in place for `turns` turns, records how far each tracking wheel travels,
     * and applies the offsets to the odometry (see odom::calibrateTrackingWheelOffsetIn()). Runs on
     * a background task, so the screen doesn't freeze. Must be called before the page is added to
     * the Gui
     *
     * @param imu measures the rotation
     * @param verticalWheel the odometry's vertical wheel, or nullptr to skip it
     * @param horizontalWheel the odometry's horizontal wheel, or nullptr to skip it
     * @param setSpin spins the chassis in place, given a turn command from -1 to 1. Called with 0
     * to stop
     * @param spinPower turn command for the spin. 0.35 by default
     * @param turns how many turns to spin. Negative spins the other way. 8 by default
     *
     * @b Example
     * @code {.cpp}
     * auto page = std::make_unique<sapphirelib::gui::OdometryPage>(odometry());
     * page->enableOffsetCalibration(drivetrain().imu(), &verticalWheel, &horizontalWheel,
     *                               [](double turn) { drivetrain().holonomic(0, 0, turn); });
     * gui.addPage(std::move(page));
     * @endcode
     */
    void enableOffsetCalibration(sensors::Imu& imu, const odom::TrackingWheel* verticalWheel,
                                 const odom::TrackingWheel* horizontalWheel,
                                 std::function<void(double)> setSpin, double spinPower = 0.35,
                                 double turns = 8.0);

    /**
     * @brief Add a "Calibrate Sensors" button that finds where the localizer's distance sensors
     * really sit
     *
     * Spins the chassis in place, one turn each way, and fits each sensor's mount against the
     * field walls, then applies the ones it could pin down to the running localizer (see
     * localization::MonteCarloLocalizer::calibrateSensorMounts(), which says where to set the
     * robot). The status line shows each sensor's forward and right offsets, and the terminal logs
     * them in full. Runs on a background task, so the screen doesn't freeze. Must be called before
     * the page is added to the Gui
     *
     * @param localizer the localizer whose sensors to calibrate. Must outlive the page
     * @param setSpin spins the chassis in place, given a turn command from -1 to 1. Called with 0
     * to stop
     * @param config the spin, and what counts as calibrated
     *
     * @b Example
     * @code {.cpp}
     * auto page = std::make_unique<sapphirelib::gui::OdometryPage>(odometry());
     * page->enableSensorCalibration(localizer(), [](double turn) {
     *     drivetrain().holonomicVolts(0, 0, turn * 12.0);
     * });
     * gui.addPage(std::move(page));
     * @endcode
     */
    void enableSensorCalibration(localization::MonteCarloLocalizer& localizer,
                                 std::function<void(double)> setSpin,
                                 localization::MountCalibrationConfig config = {});

    /**
     * @brief Whether a calibration spin is running, of the wheel offsets or the sensors
     *
     * Driver control must not command the drivetrain meanwhile, or it fights the spin and the
     * calibration never finishes. See Gui::anyPageBusy()
     */
    bool isCalibrating() const;

    /**
     * @brief Busy while calibrating. See Page::isBusy()
     */
    bool isBusy() const override { return isCalibrating(); }

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    void runCalibration();
    void runSensorCalibration();
    static void calibrateClicked(lv_event_t* e);
    static void calibrateSensorsClicked(lv_event_t* e);

    odom::Odometry& odometry_;
    double fieldWidthIn_;
    double fieldHeightIn_;

    lv_obj_t* poseLabel_ = nullptr;
    lv_obj_t* fieldView_ = nullptr;
    lv_obj_t* robotDot_ = nullptr;
    lv_obj_t* headingLine_ = nullptr;
    std::int32_t fieldViewWidthPx_ = 0;
    std::int32_t fieldViewHeightPx_ = 0;

    // the heading line's endpoints. A member, not a local: lv_line_set_points() keeps only the
    // array's address and reads it on every redraw
    lv_point_precise_t headingPoints_[2] = {};

    // the last pixels drawn, so a pose that hasn't moved a pixel doesn't redraw. Start somewhere
    // no position maps to, so the first update always draws
    std::int32_t lastDotX_ = INT32_MIN;
    std::int32_t lastDotY_ = INT32_MIN;
    std::int32_t lastTipX_ = INT32_MIN;
    std::int32_t lastTipY_ = INT32_MIN;

    // offset calibration, see enableOffsetCalibration()
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

    // sensor mount calibration, see enableSensorCalibration()
    localization::MonteCarloLocalizer* sensorLocalizer_ = nullptr;
    std::function<void(double)> sensorSetSpin_;
    localization::MountCalibrationConfig sensorConfig_;

    lv_obj_t* calibrateSensorsButton_ = nullptr;

    std::atomic<bool> sensorCalibrating_{false};
    // written by the calibration's task before it sets sensorResultReady_, and read by update()
    // once it sees that. The next calibration can only start from the screen's task, after
    std::atomic<bool> sensorResultReady_{false};
    localization::MountCalibrationResult sensorResult_;
};

} // namespace sapphirelib::gui
