/**
 * \file screen.cpp
 *
 * Builds the brain screen: every page, in tab order, wired to this robot's
 * devices. The Diagnostics list comes from config.hpp, so it can't drift from
 * the ports the devices are actually built on.
 *
 * Team 96671H: Hitmen
 */

#include "robot/screen.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robot/autons.hpp"
#include "robot/config.hpp"
#include "robot/devices.hpp"
#include "robot/driver.hpp"
#include "robot/macros.hpp"
#include "robot/telemetry.hpp"
#include "robot/tune.hpp"
#include "robot/tuning.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/auton/auton.hpp"
#include "sapphirelib/diag/sensor_check.hpp"
#include "sapphirelib/gui/auton_selector_page.hpp"
#include "sapphirelib/gui/diagnostics_page.hpp"
#include "sapphirelib/gui/face_screen.hpp"
#include "sapphirelib/gui/home_page.hpp"
#include "sapphirelib/gui/odometry_page.hpp"
#include "sapphirelib/gui/pid_tuner_page.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"
#include "sapphirelib/util/timing.hpp"

namespace robot {

using sapphirelib::diag::DeviceKind;
using sapphirelib::diag::SensorCheck;
using sapphirelib::gui::AutonSelectorPage;
using sapphirelib::gui::DiagnosticsPage;
using sapphirelib::gui::FaceScreen;
using sapphirelib::gui::Gui;
using sapphirelib::gui::HomePage;
using sapphirelib::gui::OdometryPage;
using sapphirelib::gui::PidTunerPage;
using sapphirelib::input::Button;
using sapphirelib::input::Controller;

namespace {

// Owned by the Gui (addPage() takes the unique_ptr); this is just where
// runSelectedAuton() finds it. Set in buildScreen(), before any competition
// task can start.
AutonSelectorPage* autonSelector = nullptr;

// The Auton tab's line: how sure the localizer is, how many sensors it can
// read (a sensor knocked loose shows here before a match), and what an
// update costs. It only corrects once a routine's setPose() puts it on the
// field.
std::string mclStatusText() {
    const sapphirelib::localization::LocalizationStatus status = localizer().status();
    char text[80];
    if (status.blockedBy & sapphirelib::localization::LocalizationStatus::kNoSetPose) {
        std::snprintf(text, sizeof(text), "MCL: waiting for setPose(), %u sensors read",
                      static_cast<unsigned>(status.sensorsUsed));
    } else {
        std::snprintf(text, sizeof(text), "MCL: +-%.1fin, %u sensors read, %.1fms an update",
                      status.spreadIn, static_cast<unsigned>(status.sensorsUsed),
                      status.updateUs / 1000.0);
    }
    return text;
}

// The face squints while the lift climbs faster than this, in motor degrees
// per second, its speed smoothed over about kLiftRiseSmoothingMs. Well under
// a real climb (hundreds of degrees a second), well over the few degrees the
// lift hunts by while it holds a level.
constexpr double kLiftRisingDegPerS = 100.0;
constexpr double kLiftRiseSmoothingMs = 100.0;

/// Whether the lift is on its way up, for the face's squint, from its
/// position sampled every face frame. Only the face's frame timer calls it,
/// on the GUI's task, so its state needs no lock. Works in autonomous too,
/// where macros::update() isn't running: it reads the motors, not the macros.
class LiftRiseWatch {
public:
    bool rising() {
        const std::uint32_t now = sapphirelib::millis();
        const double deg = macros::liftMechanism().position();
        const std::uint32_t dtMs = sapphirelib::elapsedMs(lastMs_, now);
        if (dtMs == 0) return degPerS_ > kLiftRisingDegPerS;
        if (!std::isfinite(deg) || !std::isfinite(lastDeg_) || dtMs > 200) {
            // No lift motors, or the first sample after a pause (the face was
            // down): no speed to go on yet.
            degPerS_ = 0.0;
        } else {
            const double sample = (deg - lastDeg_) * 1000.0 / dtMs;
            degPerS_ += (1.0 - std::exp(-static_cast<double>(dtMs) / kLiftRiseSmoothingMs)) *
                        (sample - degPerS_);
        }
        lastDeg_ = deg;
        lastMs_ = now;
        return degPerS_ > kLiftRisingDegPerS;
    }

private:
    double lastDeg_ = std::numeric_limits<double>::quiet_NaN();
    std::uint32_t lastMs_ = 0;
    double degPerS_ = 0.0;
};

/// The face (see FaceScreen): up on its own through autonomous and driver
/// control under competition control, or from the Home page's Face button,
/// and down at a tap. It squints while the lift climbs, winks as each score
/// (R2) opens the claw, and looks surprised when the IMU feels a bump.
FaceScreen& face() {
    static LiftRiseWatch lift;
    static FaceScreen instance({
        .squinting = [] { return lift.rising(); },
        .winks = [] { return macros::scoreCount(); },
        // TODO: tune BumpConfig::thresholdG (0.8g) if hits go unnoticed or
        // the robot's own driving looks surprised: the log's face,bump
        // events have each counted bump's size.
        .imu = &drivetrain().imu(),
    });
    return instance;
}

} // namespace

Gui& screen() {
    // SapphireLib's default brain-screen GUI, which fully replaces LLEMU (which
    // wasn't loading here anyway). This is entirely opt-in: if you'd rather
    // build your own UI, just don't construct a Gui; nothing else in the
    // library depends on it. See sapphirelib::gui::Gui's class comment.
    //
    // Constructed first, ahead of any hardware: a Gui's constructor only
    // touches LVGL, so the branded header paints the moment the program
    // starts. If a device constructor then blocks or faults (an IMU that
    // never finishes calibrating, a sensor in the wrong port), the screen
    // still shows that header instead of staying black, which is the
    // difference between "the GUI itself is broken" and "initialize() never
    // got far enough to build it". The stage logs in initialize() narrow it
    // the rest of the way over `pros terminal`.
    static Gui gui("SapphireLib - 96671H");
    return gui;
}

void buildScreen() {
    Gui& gui = screen();

    auto homePage = std::make_unique<HomePage>(&drivetrain().imu());
    // An "SD:" row with a Start/Stop log button, so a missing or pulled card
    // is noticed in the pits and a test run is one tap to record. Matches
    // record on their own (see logger()).
    homePage->setTelemetry(&logger());
    // Which TUNE.CFG this run is using, or that it's on the code's values.
    homePage->addRow(&tuneStatusText);
    // The macros' debug mode: L1/L2 power the lift directly, no levels, no
    // scoring and no autoClaw (see macros.hpp). Top right of the page, red
    // while on.
    homePage->addToggle(
        "Debug", [] { return macros::debug(); }, [](bool on) { macros::setDebug(on); });
    // Practice mode (driver.hpp): pushing a drive stick past
    // kPracticeMaxThrottlePct (config.hpp) cuts the drive's power until it's
    // back under. Red while on.
    homePage->addToggle(
        "Practice", [] { return practiceMode(); }, [](bool on) { setPracticeMode(on); });
    // The face outside a match, bottom right of the page. A tap anywhere on
    // it brings this page back.
    homePage->addButton("Face", [] { face().show(); });
    gui.addPage(std::move(homePage));

    auto autonSelectorPage = std::make_unique<AutonSelectorPage>();
    autonSelector = autonSelectorPage.get();
    registerAutons(*autonSelectorPage);
    // The pick survives a restart (AUTON.TXT on the SD card), and the line
    // above the routines says how sure the localizer is.
    autonSelectorPage->rememberSelection();
    autonSelectorPage->setStatus(&mclStatusText);
    gui.addPage(std::move(autonSelectorPage));

    // Re-checks these every tick (not just at startup), a sensor that
    // works at power-on but gets knocked loose mid-match should still show
    // up. Built from config.hpp, the same ports the devices use.
    const auto check = [](std::string label, std::int8_t port, DeviceKind expected) {
        return SensorCheck{.label = std::move(label), .port = port, .expected = expected};
    };
    std::vector<SensorCheck> checks{
        check("Front-left drive motor", ports::kFrontLeft, DeviceKind::motor),
        check("Front-right drive motor", ports::kFrontRight, DeviceKind::motor),
        check("Back-left drive motor", ports::kBackLeft, DeviceKind::motor),
        check("Back-right drive motor", ports::kBackRight, DeviceKind::motor),
        check("Middle-left drive motor", ports::kMiddleLeft, DeviceKind::motor),
        check("Middle-right drive motor", ports::kMiddleRight, DeviceKind::motor),
        check("IMU", ports::kImu, DeviceKind::imu),
        check("Vertical tracking wheel", ports::kVerticalTracking, DeviceKind::rotation),
        check("Horizontal tracking wheel", ports::kHorizontalTracking, DeviceKind::rotation),
    };
    // However many the lift has, lettered as the controller's LEFT readout
    // letters them. The bottom switch and the pistons are on 3-wire ports,
    // which this can't check: debug mode can (docs/NEW_CHASSIS.md, step 1).
    for (std::size_t i = 0; i < ports::kLiftMotors.size(); ++i) {
        checks.push_back(check(std::string("Lift motor ") + static_cast<char>('A' + i),
                               ports::kLiftMotors[i], DeviceKind::motor));
    }
    checks.push_back(check("Pin distance sensor", ports::kClawDistance, DeviceKind::distance));
    checks.push_back(check("North distance sensor", ports::kDistanceFront, DeviceKind::distance));
    checks.push_back(check("East distance sensor", ports::kDistanceRight, DeviceKind::distance));
    checks.push_back(check("South distance sensor", ports::kDistanceBack, DeviceKind::distance));
    checks.push_back(check("West distance sensor", ports::kDistanceLeft, DeviceKind::distance));
    gui.addPage(std::make_unique<DiagnosticsPage>(std::move(checks), &gui));

    auto pidTunerPage = std::make_unique<PidTunerPage>();
    registerTuning(*pidTunerPage);
    gui.addPage(std::move(pidTunerPage));

    // fieldWidthIn/fieldHeightIn default to a 144x144in (12x12ft) VRC field;
    // pass your own for a different game/field size.
    auto odometryPage = std::make_unique<OdometryPage>(odometry());
    // "Calibrate Offsets" button: spins the chassis in place (via the
    // drivetrain's raw-volts turn axis, so it stays robot-centric regardless
    // of field heading and of the driver stick mode) and derives verticalOffsetIn/
    // horizontalOffsetIn from how far the tracking wheels move over a known
    // rotation, replacing the offsets from config.hpp with calibrated values,
    // applied straight to the running Odometry (copy them into config.hpp to
    // keep them).
    odometryPage->enableOffsetCalibration(
        drivetrain().imu(), &verticalWheel(), &horizontalWheel(),
        [](double turn) { drivetrain().holonomicVolts(0.0, 0.0, turn * 12.0); });
    // "Calibrate Sensors" button: spins one turn each way at ~45 deg/s with
    // the robot within 3 ft of a wall or a corner, or inside the box
    // config.hpp's kSensorCalibrationBox*In describe, fits where each
    // localization distance sensor really sits against the walls, and applies
    // the ones it pinned down straight to the running localizer, as Calibrate
    // Offsets does for odometry. The terminal logs each mount ready to copy
    // into config.hpp's k*Sensor*In to keep it. Run Calibrate Offsets first:
    // the fit trusts odometry's record of the spin.
    odometryPage->enableSensorCalibration(
        localizer(), [](double turn) { drivetrain().holonomicVolts(0.0, 0.0, turn * 12.0); },
        {.boxWidthIn = kSensorCalibrationBoxWidthIn, .boxDepthIn = kSensorCalibrationBoxDepthIn});
    gui.addPage(std::move(odometryPage));
    SAPPHIRELIB_LOG_INFO("init", "all pages built");

    // Add your own pages here too: gui.addPage(std::make_unique<MyPage>(...))

    gui.start();
    // Over the whole screen, Gui and all, whenever it's up.
    face().start();
}

void runSelectedAuton() {
    if (autonSelector == nullptr) return;
    // A copy, so the end event still names this routine if someone taps a
    // different one on the screen while it runs. (run() reads the selection
    // again itself, so a tap in the microseconds between the two reads could
    // still mislabel it, not a concern with the field in charge.)
    const std::string name = autonSelector->selectedName();
    SAPPHIRELIB_LOG_INFO("auton", "running \"%s\"", name.c_str());
    // Logged from the task running the routine, so a routine the competition
    // switch cuts short shows a start with no end.
    sapphirelib::telemetry::event("auton", "start,%s", name.c_str());
    autonSelector->run();
    // A routine is done once its background motions are (runAsync()), and
    // the drive stops then, even if the last motion chained into nothing.
    sapphirelib::auton::waitForMotion();
    drivetrain().stop();
    sapphirelib::telemetry::event("auton", "end,%s", name.c_str());
    SAPPHIRELIB_LOG_INFO("auton", "finished \"%s\"", name.c_str());
}

void pickAutonWithController() {
    if (autonSelector == nullptr) return;
    // A Controller of its own, not driver control's: sampling that one here
    // would hide the gap it sees when driver control starts (resumed()), which
    // the macros and the drive modes count on to drop stale state.
    static Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        master.update();
        if (master.pressed(Button::left)) autonSelector->previous();
        if (master.pressed(Button::right)) autonSelector->next();
        master.screen().setLine(0, "AUTON <- ->");
        master.screen().setLine(1, "%s", autonSelector->selectedName().c_str());
        master.screen().setLine(2, "%s", "");
        master.flushScreen();
        pros::delay(kDriverLoopMs);
    }
}

bool screenBusy() { return screen().anyPageBusy(); }

} // namespace robot
