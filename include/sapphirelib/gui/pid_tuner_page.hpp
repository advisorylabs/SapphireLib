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

/**
 * @brief PID tuning page: adjust gains by hand and run a test motion, or Auto-Tune
 *
 * Auto-Tune works on axes, not controllers. Each axis (forward, strafe, turn, a lift) is driven
 * through voltage ramps and steps and fitted to a model (friction, speed, inertia, delay). Every
 * controller names the axis it drives and a tuning::ResponseSpec, and gets gains designed from that
 * model. So one tap tunes everything on an axis, and two controllers on one axis (an autonomous
 * turn and driver heading hold) can get different gains from the same measurement
 *
 * One tap measures the selected controller's group: all the addAxis() axes together, or one
 * mechanism on its own. While it runs, the Auto-Tune button reads "Stop"; tapping it, or disabling
 * the robot, ends the run and applies nothing
 *
 * @b Example
 * @code {.cpp}
 * auto tuner = std::make_unique<sapphirelib::gui::PidTunerPage>();
 * tuner->addAxis("Turn", [] {
 *     return sapphirelib::tuning::CharacterizationConfig{
 *         .actuate = [](double v) { drivetrain().holonomicVolts(0, 0, v); },
 *         .measure = [] { return drivetrain().imu().getCumulativeHeadingDeg(); },
 *         .minSpeed = 5.0,
 *     };
 * });
 * tuner->addController("Turn", drivetrain().turnPID(), [] { drivetrain().turnToHeading(90); },
 *                      "Turn", {.settleTimeS = 0.5});
 * gui.addPage(std::move(tuner));
 * @endcode
 */
class PidTunerPage : public Page {
public:
    /**
     * @brief Add an axis for Auto-Tune to measure. Axes are measured in the order added
     *
     * @param name the axis name, which controllers refer to
     * @param buildExperiment makes the run's config. Called fresh on every run, so it can capture
     * the current pose or heading
     * @param onMeasured receives each successful measurement on the tuning task, e.g. to install it
     * with HolonomicDrivetrain::setAxisModels(). nullptr by default
     */
    void addAxis(std::string name,
                 std::function<tuning::CharacterizationConfig()> buildExperiment,
                 std::function<void(const tuning::AxisCharacterization&)> onMeasured = nullptr);

    /**
     * @brief Add a lift, arm, or other gravity-loaded mechanism for Auto-Tune to measure on its own
     *
     * Use the config's axis.start and axis.finish to take the motors from the mechanism's own loop
     * (PositionMechanism::beginExternalControl()). Every finish hook is held until the new gains
     * are set, so the loop never runs while its gains are being written
     *
     * @param name the axis name, which controllers refer to
     * @param buildExperiment makes the run's config. Called fresh on every run
     * @param onMeasured receives each successful measurement on the tuning task, e.g. to install
     * PositionMechanism::setGravity(result.fit.model.gravityFeedforward()). nullptr by default
     */
    void addMechanismAxis(
        std::string name, std::function<tuning::MechanismCharacterizationConfig()> buildExperiment,
        std::function<void(const tuning::MechanismCharacterization&)> onMeasured = nullptr);

    /**
     * @brief Add a controller to tune
     *
     * Only one test or Auto-Tune runs at a time, and gain changes and selection are ignored during
     * one, since both would touch the same PID. The design allows for the loop's own period (the
     * PID's nominalDtS) as extra delay. Safe before or after build()
     *
     * @param name the name on its button
     * @param pid the PID
     * @param runTest runs on a background task when "Run Test" is tapped, e.g. a driveDistance().
     * nullptr by default
     * @param axis the axis it drives, to be designed from after Auto-Tune. "" (the default) for a
     * controller Auto-Tune shouldn't touch
     * @param response how the controller should behave
     */
    void addController(std::string name, PID& pid, std::function<void()> runTest = nullptr,
                       std::string axis = "", tuning::ResponseSpec response = {});

    /**
     * @brief Add an on/off button under Run Test and Auto-Tune, e.g. the driver stick mode
     *
     * @param label polled for the button's text
     * @param onTap runs on the GUI task when tapped. Ignored during a run
     */
    void setToggle(std::function<std::string()> label, std::function<void()> onTap);

    /**
     * @brief Whether a test or Auto-Tune is running
     *
     * Driver control must not command the drivetrain meanwhile, or it fights the run. See
     * Gui::anyPageBusy()
     */
    bool isRunning() const;

    /**
     * @brief Busy while running. See Page::isBusy()
     */
    bool isBusy() const override { return isRunning(); }

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Axis {
        std::string name;
        // which Auto-Tune tap measures it: "" for every addAxis() axis, its own name for a
        // mechanism
        std::string group;
        bool mechanism = false;
        std::function<tuning::CharacterizationConfig()> buildExperiment;
        std::function<void(const tuning::AxisCharacterization&)> onMeasured;
        std::function<tuning::MechanismCharacterizationConfig()> buildMechanismExperiment;
        std::function<void(const tuning::MechanismCharacterization&)> onMechanismMeasured;

        // written only by the Auto-Tune task while testRunning_ is true, read only by update()
        // once it's false. A drive axis's result is kept as a mechanism result with no gravity,
        // so the design and readout have one shape to read
        bool measured = false;
        tuning::MechanismCharacterization result;
        // the run's sample period, to judge how much delay a slower loop adds
        double samplePeriodS = 0.01;
    };

    struct Entry {
        std::string name;
        PID* pid;
        std::function<void()> runTest;
        std::string axis;
        tuning::ResponseSpec response;
        lv_obj_t* selectorButton = nullptr;

        // same threading rule as Axis::result
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
    // measure one axis on the tuning task. false if it gave no usable model or was stopped. The
    // finish hook isn't run here; it's added to finishes for runAutoTune() to run once the gains
    // are set
    bool measureAxis(Axis& axis, std::vector<std::function<void()>>& finishes);
    // the group the selected controller's axis is in ("" for the drive axes, or no axis)
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

    // what refreshGainLabels() shows. The Auto-Tune task writes gains while update() reads them,
    // so neither reads PID::gains() for display
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
