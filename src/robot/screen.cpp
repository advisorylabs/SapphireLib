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

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robot/autons.hpp"
#include "robot/config.hpp"
#include "robot/devices.hpp"
#include "robot/telemetry.hpp"
#include "robot/tune.hpp"
#include "robot/tuning.hpp"
#include "sapphirelib/diag/sensor_check.hpp"
#include "sapphirelib/gui/auton_selector_page.hpp"
#include "sapphirelib/gui/diagnostics_page.hpp"
#include "sapphirelib/gui/home_page.hpp"
#include "sapphirelib/gui/odometry_page.hpp"
#include "sapphirelib/gui/pid_tuner_page.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/util/log.hpp"

namespace robot {

using sapphirelib::diag::DeviceKind;
using sapphirelib::diag::SensorCheck;
using sapphirelib::gui::AutonSelectorPage;
using sapphirelib::gui::DiagnosticsPage;
using sapphirelib::gui::Gui;
using sapphirelib::gui::HomePage;
using sapphirelib::gui::OdometryPage;
using sapphirelib::gui::PidTunerPage;

namespace {

// Owned by the Gui (addPage() takes the unique_ptr); this is just where
// runSelectedAuton() finds it. Set in buildScreen(), before any competition
// task can start.
AutonSelectorPage* autonSelector = nullptr;

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
    gui.addPage(std::move(homePage));

    auto autonSelectorPage = std::make_unique<AutonSelectorPage>();
    autonSelector = autonSelectorPage.get();
    registerAutons(*autonSelectorPage);
    gui.addPage(std::move(autonSelectorPage));

    // Re-checks these every tick (not just at startup), a sensor that
    // works at power-on but gets knocked loose mid-match should still show
    // up. Built from config.hpp, the same ports the devices use.
    const auto check = [](const char* label, std::int8_t port, DeviceKind expected) {
        return SensorCheck{.label = label, .port = port, .expected = expected};
    };
    gui.addPage(std::make_unique<DiagnosticsPage>(
        std::vector<SensorCheck>{
            check("Front-left drive motor", ports::kFrontLeft, DeviceKind::motor),
            check("Front-right drive motor", ports::kFrontRight, DeviceKind::motor),
            check("Back-left drive motor", ports::kBackLeft, DeviceKind::motor),
            check("Back-right drive motor", ports::kBackRight, DeviceKind::motor),
            check("Middle-left drive motor", ports::kMiddleLeft, DeviceKind::motor),
            check("Middle-right drive motor", ports::kMiddleRight, DeviceKind::motor),
            check("IMU", ports::kImu, DeviceKind::imu),
            check("Vertical tracking wheel", ports::kVerticalTracking, DeviceKind::rotation),
            check("Horizontal tracking wheel", ports::kHorizontalTracking, DeviceKind::rotation),
            check("Lift motor A", ports::kLiftA, DeviceKind::motor),
            check("Lift motor B", ports::kLiftB, DeviceKind::motor),
            // TODO: uncomment each of these once its port in config.hpp is
            // the real one. They're placeholders today, and a check on a
            // wrong port would put a red warning banner across every tab for
            // the whole match.
            // check("Intake motor", ports::kIntake, DeviceKind::motor),
            // check("Claw motor", ports::kClawMotor, DeviceKind::motor),
            // check("Lift rotation sensor", ports::kLiftSensor, DeviceKind::rotation),
            // check("Claw distance sensor", ports::kClawDistance, DeviceKind::distance),
            // check("Front distance sensor", ports::kDistanceFront, DeviceKind::distance),
            // check("Right distance sensor", ports::kDistanceRight, DeviceKind::distance),
            // check("Back distance sensor", ports::kDistanceBack, DeviceKind::distance),
            // check("Left distance sensor", ports::kDistanceLeft, DeviceKind::distance),
        },
        &gui));

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
    gui.addPage(std::move(odometryPage));
    SAPPHIRELIB_LOG_INFO("init", "all pages built");

    // Add your own pages here too: gui.addPage(std::make_unique<MyPage>(...))

    gui.start();
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
    sapphirelib::telemetry::event("auton", "end,%s", name.c_str());
    SAPPHIRELIB_LOG_INFO("auton", "finished \"%s\"", name.c_str());
}

bool screenBusy() { return screen().anyPageBusy(); }

} // namespace robot
