/**
 * \file sapphirelib/gui/pid_tuner_page.hpp
 *
 * SapphireLib's default PID tuning page: adjust gains by hand and re-run a
 * test motion to see the effect, or tap Auto-Tune to measure the robot and
 * design every controller's gains from that measurement.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"
#include "sapphirelib/tuning/gain_design.hpp"

namespace sapphirelib::gui {

/// Manual live tuning (+/- buttons, re-run a bound test motion) plus
/// model-based automatic tuning.
///
/// Auto-Tune works on *axes*, not controllers. Each axis registered with
/// addAxis() (forward, strafe, turn, ...) is driven through a short set of
/// voltage ramps and steps (tuning::runCharacterization()), and the result
/// is fitted to a model of that axis — friction, speed constant, inertia,
/// and response delay (tuning::characterizeAxis()). Every controller
/// registered with addController() names the axis it drives and a
/// tuning::ResponseSpec describing how it should behave, and gets gains
/// designed from that axis's model by pole placement
/// (tuning::designPositionGains()).
///
/// So one tap measures the robot once and tunes everything from it, and
/// two controllers on the same axis — an autonomous turn and driver heading
/// hold, say — get different gains from the same measurement, each matching
/// its own spec.
class PidTunerPage : public Page {
public:
    /// Registers an axis for Auto-Tune to measure. `buildExperiment` is
    /// called fresh on each run (not once here), so it can capture "here" —
    /// current pose, current heading — as the run's reference frame.
    /// `onMeasured`, if given, receives each successful measurement (e.g. to
    /// install it with HolonomicDrivetrain::setAxisModels()); it runs on the
    /// tuning task, not the GUI's. Axes are measured in registration order.
    void addAxis(std::string name,
                 std::function<tuning::CharacterizationConfig()> buildExperiment,
                 std::function<void(const tuning::AxisCharacterization&)> onMeasured = nullptr);

    /// Registers a controller to tune.
    ///
    /// `runTest`, if given, runs on a background PROS task when "Run Test"
    /// is tapped — e.g. bind it to a driveDistance() call, so you can watch
    /// the response to the current gains live without freezing the screen.
    ///
    /// `axis` names an addAxis() axis; after Auto-Tune measures it, this
    /// controller's gains are designed from its model to meet `response`.
    /// Leave empty for a controller Auto-Tune shouldn't touch.
    ///
    /// Only one test or auto-tune run happens at a time across the whole
    /// page; gain adjustments, entry selection, and new runs are all
    /// ignored while one is in progress, since a run and the tuning UI
    /// would otherwise read/write the same PID object concurrently. Safe
    /// to call before or after build().
    void addController(std::string name, PID& pid, std::function<void()> runTest = nullptr,
                       std::string axis = "", tuning::ResponseSpec response = {});

    /// Adds a single on/off style button under Run Test/Auto-Tune — e.g. the
    /// driver stick mode. `label` is polled for the button's text; `onTap`
    /// runs on the GUI task when tapped (ignored while a run is active).
    void setToggle(std::function<std::string()> label, std::function<void()> onTap);

    /// True from the moment "Run Test" or "Auto-Tune" is tapped until that
    /// run finishes. Driver-control code (e.g. opcontrol()'s joystick loop)
    /// should skip commanding the drivetrain while this is true — a
    /// concurrent driver loop (which keeps running whenever there's no
    /// competition switch, even with centered sticks) would fight the run
    /// for the same motors; see gui::OdometryPage::isCalibrating()'s comment
    /// for the failure mode in more detail.
    bool isRunning() const;

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Axis {
        std::string name;
        std::function<tuning::CharacterizationConfig()> buildExperiment;
        std::function<void(const tuning::AxisCharacterization&)> onMeasured;

        // Written only by the auto-tune task while testRunning_ is true, read
        // only by update() once it's false again — see runAutoTune().
        bool measured = false;
        tuning::AxisCharacterization result;
    };

    struct Entry {
        std::string name;
        PID* pid;
        std::function<void()> runTest;
        std::string axis;
        tuning::ResponseSpec response;
        lv_obj_t* selectorButton = nullptr;

        // Same threading rule as Axis::result.
        bool designed = false;
        tuning::GainDesign design;
    };

    void addSelectorButton(Entry& entry, std::size_t row);
    void select(std::size_t index);
    void adjustGain(int gainIndex, double delta);
    void refreshGainLabels();
    void refreshReadout();
    void runSelectedTest();
    void runAutoTune();
    void setDisplayedGains(PIDGains gains);
    const Axis* findAxis(const std::string& name) const;

    static void selectorClicked(lv_event_t* e);
    static void kpMinusClicked(lv_event_t* e);
    static void kpPlusClicked(lv_event_t* e);
    static void kiMinusClicked(lv_event_t* e);
    static void kiPlusClicked(lv_event_t* e);
    static void kdMinusClicked(lv_event_t* e);
    static void kdPlusClicked(lv_event_t* e);
    static void runTestClicked(lv_event_t* e);
    static void autoTuneClicked(lv_event_t* e);
    static void toggleClicked(lv_event_t* e);

    std::vector<std::unique_ptr<Axis>> axes_;
    std::vector<std::unique_ptr<Entry>> entries_;
    std::size_t selectedIndex_ = 0;

    std::function<std::string()> toggleLabel_;
    std::function<void()> toggleTap_;

    // The only source of truth refreshGainLabels() reads from — the
    // auto-tune task writes gains while update() reads them, so neither
    // touches PID::gains() for display.
    std::atomic<double> displayedP_{0.0};
    std::atomic<double> displayedI_{0.0};
    std::atomic<double> displayedD_{0.0};

    std::atomic<bool> testRunning_{false};
    std::atomic<bool> autoTuneActive_{false};
    std::atomic<int> measuringAxis_{-1};
    std::atomic<int> failedAxis_{-1};
    std::atomic<bool> readoutDirty_{true};

    lv_obj_t* container_ = nullptr;
    lv_obj_t* kpLabel_ = nullptr;
    lv_obj_t* kiLabel_ = nullptr;
    lv_obj_t* kdLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* resultLabel_ = nullptr;
    lv_obj_t* toggleButton_ = nullptr;
    lv_obj_t* toggleButtonLabel_ = nullptr;
};

} // namespace sapphirelib::gui
