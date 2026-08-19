/**
 * \file sapphirelib/gui/pid_tuner_page.hpp
 *
 * SapphireLib's default PID tuning page: adjust gains by hand and re-run a
 * test motion to see the effect, or let an overshoot-driven search do it
 * automatically.
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

namespace sapphirelib::gui {

/// Manual live tuning (+/- buttons, re-run a bound test motion) plus an
/// optional automatic search: repeatedly runs a bound test cycle at
/// different gains and keeps whichever minimized measured overshoot —
/// see addController()'s autoTuneCost parameter. This is a Twiddle
/// (coordinate-ascent hill-climbing) search over the three gains, not
/// automatic relay/Ziegler-Nichols tuning — that family of methods works by
/// deliberately driving the system into sustained oscillation, which isn't
/// something to do unsupervised on a multi-pound robot. Twiddle instead
/// just tries nearby gains and keeps whatever measurably did better, so
/// every trial is a bounded, ordinary test motion.
class PidTunerPage : public Page {
public:
    /// Registers a controller to tune.
    ///
    /// `runTest`, if given, runs on a background PROS task when the driver
    /// taps "Run Test" — e.g. bind it to a driveDistance() call, so you can
    /// watch the response to the current gains live without freezing the
    /// screen for the run's duration.
    ///
    /// `autoTuneCost`, if given, enables "Auto-Tune": when called, it
    /// should run one complete bounded test cycle at the PID's *current*
    /// gains and return a cost to minimize (lower = better) — typically
    /// built from sapphirelib::tuning::runAndSample() +
    /// measureOvershoot() + autoTuneCost() over an odometry-sampled
    /// distance or heading signal. For a drive controller, that cycle
    /// should be a round trip (e.g. forward then back the same distance)
    /// so repeated trials don't walk the robot away; for a turn
    /// controller, whatever round trip is convenient (e.g. turn to a
    /// heading and back), since turning in place doesn't accumulate
    /// position drift the way driving does.
    ///
    /// Only one test or auto-tune run happens at a time across the whole
    /// page; gain adjustments, entry selection, and new runs are all
    /// ignored while one is in progress, since a run and the tuning UI
    /// would otherwise read/write the same PID object concurrently. Safe
    /// to call before or after build().
    void addController(std::string name, PID& pid, std::function<void()> runTest = nullptr,
                       std::function<double()> autoTuneCost = nullptr);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    struct Entry {
        std::string name;
        PID* pid;
        std::function<void()> runTest;
        std::function<double()> autoTuneCost;
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

    // The only source of truth refreshGainLabels() reads from — see the
    // comment in runSelectedAutoTune() for why direct PID::gains() reads
    // from update() would race the auto-tune background task.
    std::atomic<double> displayedP_{0.0};
    std::atomic<double> displayedI_{0.0};
    std::atomic<double> displayedD_{0.0};

    std::atomic<bool> testRunning_{false};
    std::atomic<bool> autoTuneActive_{false};
    std::atomic<bool> resultsReady_{false};
    std::atomic<double> tunedP_{0.0};
    std::atomic<double> tunedI_{0.0};
    std::atomic<double> tunedD_{0.0};

    lv_obj_t* container_ = nullptr;
    lv_obj_t* kpLabel_ = nullptr;
    lv_obj_t* kiLabel_ = nullptr;
    lv_obj_t* kdLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    lv_obj_t* resultLabel_ = nullptr;
};

} // namespace sapphirelib::gui
