/**
 * \file sapphirelib/gui/pid_tuner_page.hpp
 *
 * SapphireLib's default PID tuning page: adjust gains by hand and re-run a
 * test motion to see the effect, or let a Ziegler-Nichols relay-feedback
 * auto-tune do it automatically.
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
#include "sapphirelib/tuning/auto_tune_runner.hpp"

namespace sapphirelib::gui {

/// Manual live tuning (+/- buttons, re-run a bound test motion) plus an
/// optional automatic search: a Ziegler-Nichols relay-feedback auto-tune —
/// see addController()'s buildAutoTuneConfig parameter. Tapping "Auto-Tune"
/// drives the bound controller's plant open-loop with a small, bounded
/// relay (tuning::runRelayExperiment()) to force a sustained oscillation,
/// measures that oscillation's period/amplitude
/// (tuning::analyzeRelayOscillation()), and derives the controller's ultimate
/// gain/period from it (tuning::ultimateParamsFromRelay()) to compute gains
/// via a closed-loop tuning rule (tuning::gainsFromUltimate(); pick which
/// one with setTuningRule()).
/// Unlike a naive "keep raising kP until it oscillates" gain sweep, the
/// relay's output magnitude is fixed up front
/// (tuning::RelayTuneConfig::relayAmplitude) rather than climbing toward
/// instability, so the experiment can't run away even if the plant turns
/// out to be far more aggressive than expected — pick that amplitude
/// conservatively and it stays a bounded, ordinary test motion.
class PidTunerPage : public Page {
public:
    /// Registers a controller to tune.
    ///
    /// `runTest`, if given, runs on a background PROS task when the driver
    /// taps "Run Test" — e.g. bind it to a driveDistance() call, so you can
    /// watch the response to the current gains live without freezing the
    /// screen for the run's duration.
    ///
    /// `buildAutoTuneConfig`, if given, enables "Auto-Tune": called fresh
    /// each time the driver taps it (not once at registration), so it can
    /// capture "here" — current position, current heading — as the relay
    /// experiment's reference frame. Should return a
    /// tuning::RelayTuneConfig whose `actuate` drives the same plant `pid`
    /// closes the loop on (bypassing `pid` itself — the relay experiment
    /// replaces it for the experiment's duration) and whose `measure`
    /// reads that plant's process variable relative to the reading at the
    /// moment `measure` is first called. See RelayTuneConfig's own field
    /// comments for the rest (relay amplitude, setpoint, timeout).
    ///
    /// Only one test or auto-tune run happens at a time across the whole
    /// page; gain adjustments, entry selection, and new runs are all
    /// ignored while one is in progress, since a run and the tuning UI
    /// would otherwise read/write the same PID object concurrently. Safe
    /// to call before or after build().
    void addController(std::string name, PID& pid, std::function<void()> runTest = nullptr,
                       std::function<tuning::RelayTuneConfig()> buildAutoTuneConfig = nullptr);

    /// True from the moment "Run Test" or "Auto-Tune" is tapped until that
    /// run finishes. Driver-control code (e.g. opcontrol()'s joystick loop)
    /// should skip calling holonomic()/holonomicFieldCentric() (or a
    /// TankDrivetrain equivalent) while this is true — those calls
    /// unconditionally command the same motors the bound runTest/
    /// autoTuneCost callback is driving via driveDistance()/turnToHeading(),
    /// so a concurrent driver-control loop (which keeps running whenever
    /// there's no competition switch, even with centered sticks) will fight
    /// it — see gui::OdometryPage::isCalibrating()'s comment for the same
    /// failure mode in more detail.
    bool isRunning() const;

    /// Which closed-loop tuning rule "Auto-Tune" applies to the Ku/Tu it
    /// measures. Defaults to tuning::TuningRule::noOvershoot — see that
    /// enum's comment for why the classic Ziegler-Nichols rule isn't the
    /// default despite being the one this flow is named after. Applies to
    /// every registered controller; call it before an auto-tune run starts.
    void setTuningRule(tuning::TuningRule rule);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Entry {
        std::string name;
        PID* pid;
        std::function<void()> runTest;
        std::function<tuning::RelayTuneConfig()> buildAutoTuneConfig;
        lv_obj_t* selectorButton = nullptr;
    };

    void select(std::size_t index);
    void adjustGain(int gainIndex, double delta);
    void refreshGainLabels();
    void runSelectedTest();
    void runSelectedAutoTune();
    void setDisplayedGains(PIDGains gains);

    static void selectorClicked(lv_event_t* e);
    static void kpMinusClicked(lv_event_t* e);
    static void kpPlusClicked(lv_event_t* e);
    static void kiMinusClicked(lv_event_t* e);
    static void kiPlusClicked(lv_event_t* e);
    static void kdMinusClicked(lv_event_t* e);
    static void kdPlusClicked(lv_event_t* e);
    static void runTestClicked(lv_event_t* e);
    static void autoTuneClicked(lv_event_t* e);

    std::vector<std::unique_ptr<Entry>> entries_;
    std::size_t selectedIndex_ = 0;
    std::atomic<tuning::TuningRule> tuningRule_{tuning::TuningRule::noOvershoot};

    // The only source of truth refreshGainLabels() reads from — see the
    // comment in runSelectedAutoTune() for why direct PID::gains() reads
    // from update() would race the auto-tune background task.
    std::atomic<double> displayedP_{0.0};
    std::atomic<double> displayedI_{0.0};
    std::atomic<double> displayedD_{0.0};

    std::atomic<bool> testRunning_{false};
    std::atomic<bool> autoTuneActive_{false};
    std::atomic<bool> resultsReady_{false};
    std::atomic<bool> autoTuneFailed_{false};
    std::atomic<double> tunedP_{0.0};
    std::atomic<double> tunedI_{0.0};
    std::atomic<double> tunedD_{0.0};
    std::atomic<double> tunedUltimateGain_{0.0};
    std::atomic<double> tunedUltimatePeriodMs_{0.0};

    lv_obj_t* container_ = nullptr;
    lv_obj_t* kpLabel_ = nullptr;
    lv_obj_t* kiLabel_ = nullptr;
    lv_obj_t* kdLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* resultLabel_ = nullptr;
};

} // namespace sapphirelib::gui
