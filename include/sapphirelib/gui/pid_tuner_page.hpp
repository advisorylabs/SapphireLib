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
///
/// Mechanisms — a lift, an arm — are axes too (addMechanismAxis()), measured
/// with gravity (tuning::runMechanismCharacterization()) and fitted with a kG
/// term, so their controllers are designed with gravity cancelled by
/// feedforward. Auto-Tune measures one *group* of axes per tap: the one the
/// selected controller's axis belongs to. Every addAxis() axis is in one
/// group (the drivetrain's, measured together as before); each mechanism
/// axis is a group of its own. So selecting Lift and tapping Auto-Tune runs
/// the lift alone, and selecting Drive runs forward, strafe and turn.
///
/// While Auto-Tune runs, its button reads "Stop": tapping it ends the run at
/// once (so does the robot being disabled), and nothing is applied.
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

    /// Registers a lift, an arm, or another gravity-loaded mechanism for
    /// Auto-Tune to measure, on its own (see the class comment on groups).
    /// `buildExperiment` is called fresh on each run, like addAxis()'s; use its
    /// axis.start/axis.finish hooks to take the mechanism's motors from its
    /// own loop for the run (PositionMechanism::beginExternalControl()).
    /// Auto-Tune holds every experiment's finish hook (addAxis()'s too) until
    /// the new gains are set, so a loop given its motors back never runs a
    /// step while its gains are being written; it runs however the tap ended.
    /// `onMeasured`, if given, receives each successful measurement on the
    /// tuning task — e.g. to install its gravity feedforward with
    /// PositionMechanism::setGravity(result.fit.model.gravityFeedforward()).
    void addMechanismAxis(
        std::string name, std::function<tuning::MechanismCharacterizationConfig()> buildExperiment,
        std::function<void(const tuning::MechanismCharacterization&)> onMeasured = nullptr);

    /// Registers a controller to tune.
    ///
    /// `runTest`, if given, runs on a background PROS task when "Run Test"
    /// is tapped — e.g. bind it to a driveDistance() call, so you can watch
    /// the response to the current gains live without freezing the screen.
    ///
    /// `axis` names an addAxis() or addMechanismAxis() axis; after Auto-Tune
    /// measures it, this controller's gains are designed from its model to
    /// meet `response`. Leave empty for a controller Auto-Tune shouldn't touch.
    /// The design allows for the loop's own period (the PID's nominalDtS): a
    /// loop slower than the characterization's sampling holds each command
    /// longer, which is extra latency — half the difference, on average.
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

    /// Busy for as long as isRunning() — see Page::isBusy().
    bool isBusy() const override { return isRunning(); }

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Axis {
        std::string name;
        /// Which Auto-Tune tap measures it: "" for every addAxis() axis, the
        /// axis's own name for a mechanism.
        std::string group;
        bool mechanism = false;
        std::function<tuning::CharacterizationConfig()> buildExperiment;
        std::function<void(const tuning::AxisCharacterization&)> onMeasured;
        std::function<tuning::MechanismCharacterizationConfig()> buildMechanismExperiment;
        std::function<void(const tuning::MechanismCharacterization&)> onMechanismMeasured;

        // Written only by the auto-tune task while testRunning_ is true, read
        // only by update() once it's false again — see runAutoTune(). A drive
        // axis's result is kept as a mechanism result with no gravity, so
        // design and readout have one shape to read.
        bool measured = false;
        tuning::MechanismCharacterization result;
        /// The characterization's sample period, to judge how much latency a
        /// slower control loop adds on top of the measured delay.
        double samplePeriodS = 0.01;
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
    /// Measures one axis on the tuning task; false if it didn't produce a
    /// usable model (or the run was stopped). The experiment's finish hook
    /// isn't run: it's appended to `finishes`, for runAutoTune() to call once
    /// the new gains are in place.
    bool measureAxis(Axis& axis, std::vector<std::function<void()>>& finishes);
    /// The group the selected controller's axis is in ("" — the drive axes —
    /// for a controller with no axis).
    std::string selectedGroup() const;
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
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> stopped_{false}; // the last Auto-Tune was stopped
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
    lv_obj_t* autoTuneLabel_ = nullptr;
};

} // namespace sapphirelib::gui
