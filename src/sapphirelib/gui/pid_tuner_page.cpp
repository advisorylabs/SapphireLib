#include "sapphirelib/gui/pid_tuner_page.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <utility>

#include "pros/rtos.hpp"
#include "sapphirelib/tuning/auto_tune_math.hpp"

namespace sapphirelib::gui {

namespace {

constexpr double kStepP = 0.05;
constexpr double kStepI = 0.01;
constexpr double kStepD = 0.01;

constexpr std::uint32_t kSelectedBgColor = 0x2563eb;
constexpr std::uint32_t kSelectedTextColor = 0xffffff;

constexpr std::int32_t kRowY0 = 0;
constexpr std::int32_t kRowHeight = 30;
constexpr std::int32_t kBtnW = 34;
constexpr std::int32_t kBtnH = 26;
constexpr std::int32_t kBtnMinusX = 150;
constexpr std::int32_t kBtnPlusX = 188;

constexpr std::int32_t kSelectorColumnX = 240;
constexpr std::int32_t kSelectorButtonW = 88;
constexpr std::int32_t kSelectorButtonH = 32;
constexpr std::int32_t kSelectorRowHeight = 36;

// Auto-tune search bounds — a hard cap on trials matters more than
// Twiddle's own step-size convergence check, since a flat or noisy cost
// landscape (plausible on real hardware, where every trial is a physical
// motion with sensor noise) could otherwise keep the search running
// indefinitely.
constexpr int kMaxAutoTuneIterations = 40;
constexpr double kAutoTuneStepTolerance = 1e-3;
constexpr double kAutoTuneStepFraction = 0.3;
constexpr double kAutoTuneMinStepP = 0.05;
constexpr double kAutoTuneMinStepI = 0.01;
constexpr double kAutoTuneMinStepD = 0.01;

lv_obj_t* makeIconButton(lv_obj_t* parent, std::int32_t x, std::int32_t y, const char* text,
                         lv_event_cb_t cb, void* userData) {
    lv_obj_t* button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, kBtnW, kBtnH);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, userData);
    return button;
}

void styleSelectorButton(lv_obj_t* button) {
    lv_obj_set_style_bg_color(button, lv_color_hex(kSelectedBgColor), LV_STATE_CHECKED);
    lv_obj_set_style_text_color(button, lv_color_hex(kSelectedTextColor), LV_STATE_CHECKED);
}

} // namespace

const char* PidTunerPage::title() const { return "PID"; }

void PidTunerPage::build(lv_obj_t* container) {
    container_ = container;
    // The tab content area doesn't need to scroll for anything this page
    // draws — everything fits in fixed positions — and a stray scroll
    // gesture is easy to trigger by accident on a touchscreen this small,
    // so disable it outright rather than let it fight with taps.
    lv_obj_remove_flag(container_, LV_OBJ_FLAG_SCROLLABLE);

    for (std::size_t i = 0; i < entries_.size(); ++i) {
        Entry& entry = *entries_[i];
        entry.selectorButton = lv_button_create(container_);
        lv_obj_set_pos(entry.selectorButton, kSelectorColumnX,
                       static_cast<std::int32_t>(i) * kSelectorRowHeight);
        lv_obj_set_size(entry.selectorButton, kSelectorButtonW, kSelectorButtonH);
        lv_obj_t* label = lv_label_create(entry.selectorButton);
        lv_label_set_text(label, entry.name.c_str());
        lv_obj_center(label);
        styleSelectorButton(entry.selectorButton);
        lv_obj_add_event_cb(entry.selectorButton, &PidTunerPage::selectorClicked, LV_EVENT_CLICKED, this);
    }

    kpLabel_ = lv_label_create(container_);
    lv_obj_set_pos(kpLabel_, 4, kRowY0 + 5);
    makeIconButton(container_, kBtnMinusX, kRowY0, "-", &PidTunerPage::kpMinusClicked, this);
    makeIconButton(container_, kBtnPlusX, kRowY0, "+", &PidTunerPage::kpPlusClicked, this);

    kiLabel_ = lv_label_create(container_);
    lv_obj_set_pos(kiLabel_, 4, kRowY0 + kRowHeight + 5);
    makeIconButton(container_, kBtnMinusX, kRowY0 + kRowHeight, "-", &PidTunerPage::kiMinusClicked, this);
    makeIconButton(container_, kBtnPlusX, kRowY0 + kRowHeight, "+", &PidTunerPage::kiPlusClicked, this);

    kdLabel_ = lv_label_create(container_);
    lv_obj_set_pos(kdLabel_, 4, kRowY0 + 2 * kRowHeight + 5);
    makeIconButton(container_, kBtnMinusX, kRowY0 + 2 * kRowHeight, "-", &PidTunerPage::kdMinusClicked, this);
    makeIconButton(container_, kBtnPlusX, kRowY0 + 2 * kRowHeight, "+", &PidTunerPage::kdPlusClicked, this);

    lv_obj_t* runTestButton = lv_button_create(container_);
    lv_obj_set_pos(runTestButton, 4, kRowY0 + 3 * kRowHeight + 6);
    lv_obj_set_size(runTestButton, 100, 30);
    lv_obj_t* runTestLabel = lv_label_create(runTestButton);
    lv_label_set_text(runTestLabel, "Run Test");
    lv_obj_center(runTestLabel);
    lv_obj_add_event_cb(runTestButton, &PidTunerPage::runTestClicked, LV_EVENT_CLICKED, this);

    lv_obj_t* autoTuneButton = lv_button_create(container_);
    lv_obj_set_pos(autoTuneButton, 110, kRowY0 + 3 * kRowHeight + 6);
    lv_obj_set_size(autoTuneButton, 110, 30);
    lv_obj_t* autoTuneLabel = lv_label_create(autoTuneButton);
    lv_label_set_text(autoTuneLabel, "Auto-Tune");
    lv_obj_center(autoTuneLabel);
    lv_obj_add_event_cb(autoTuneButton, &PidTunerPage::autoTuneClicked, LV_EVENT_CLICKED, this);

    statusLabel_ = lv_label_create(container_);
    lv_obj_set_pos(statusLabel_, 4, kRowY0 + 4 * kRowHeight + 12);

    resultLabel_ = lv_label_create(container_);
    lv_obj_set_pos(resultLabel_, 4, kRowY0 + 5 * kRowHeight + 10);
    lv_label_set_text(resultLabel_, "");

    if (!entries_.empty()) select(0);
    refreshGainLabels();
}

void PidTunerPage::addController(std::string name, PID& pid, std::function<void()> runTest,
                                 std::function<double()> autoTuneCost) {
    auto entry = std::make_unique<Entry>();
    entry->name = std::move(name);
    entry->pid = &pid;
    entry->runTest = std::move(runTest);
    entry->autoTuneCost = std::move(autoTuneCost);

    if (container_) {
        entry->selectorButton = lv_button_create(container_);
        const std::int32_t row = static_cast<std::int32_t>(entries_.size());
        lv_obj_set_pos(entry->selectorButton, kSelectorColumnX, row * kSelectorRowHeight);
        lv_obj_set_size(entry->selectorButton, kSelectorButtonW, kSelectorButtonH);
        lv_obj_t* label = lv_label_create(entry->selectorButton);
        lv_label_set_text(label, entry->name.c_str());
        lv_obj_center(label);
        styleSelectorButton(entry->selectorButton);
        lv_obj_add_event_cb(entry->selectorButton, &PidTunerPage::selectorClicked, LV_EVENT_CLICKED, this);
    }

    const bool wasEmpty = entries_.empty();
    entries_.push_back(std::move(entry));
    if (wasEmpty && container_) select(0);
}

void PidTunerPage::select(std::size_t index) {
    if (index >= entries_.size() || testRunning_.load()) return;

    for (auto& entry : entries_) {
        if (entry->selectorButton) lv_obj_remove_state(entry->selectorButton, LV_STATE_CHECKED);
    }
    selectedIndex_ = index;
    if (entries_[index]->selectorButton) lv_obj_add_state(entries_[index]->selectorButton, LV_STATE_CHECKED);

    setDisplayedGains(entries_[index]->pid->gains());
}

void PidTunerPage::setDisplayedGains(PIDGains gains) {
    // Atomics only — never touch an LVGL widget here. This is called from
    // the auto-tune background task as well as from the LVGL-owned click
    // handlers, and LVGL isn't thread-safe; update() (running on LVGL's own
    // timer) is the only place that's allowed to turn these into label
    // text.
    displayedP_.store(gains.kP);
    displayedI_.store(gains.kI);
    displayedD_.store(gains.kD);
}

void PidTunerPage::adjustGain(int gainIndex, double delta) {
    if (entries_.empty() || testRunning_.load()) return;

    PID* pid = entries_[selectedIndex_]->pid;
    PIDGains gains = pid->gains();
    switch (gainIndex) {
        case 0: gains.kP = std::max(0.0, gains.kP + delta); break;
        case 1: gains.kI = std::max(0.0, gains.kI + delta); break;
        case 2: gains.kD = std::max(0.0, gains.kD + delta); break;
        default: break;
    }
    pid->setGains(gains);
    setDisplayedGains(gains);
}

void PidTunerPage::refreshGainLabels() {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "kP: %.3f", displayedP_.load());
    lv_label_set_text(kpLabel_, buf);
    std::snprintf(buf, sizeof(buf), "kI: %.3f", displayedI_.load());
    lv_label_set_text(kiLabel_, buf);
    std::snprintf(buf, sizeof(buf), "kD: %.3f", displayedD_.load());
    lv_label_set_text(kdLabel_, buf);
}

void PidTunerPage::runSelectedTest() {
    if (entries_.empty() || testRunning_.load()) return;

    Entry* entry = entries_[selectedIndex_].get();
    if (!entry->runTest) return;

    testRunning_.store(true);
    autoTuneActive_.store(false);
    // Runs on a background task, not this LVGL-owned callback, so the
    // (possibly multi-second) test motion doesn't freeze the whole screen.
    // The task only ever touches testRunning_ (an atomic flag) — never an
    // LVGL widget directly, since LVGL isn't thread-safe; update() reads
    // the flag and reflects it in statusLabel_ from the correct context.
    pros::Task([this, entry] {
        entry->runTest();
        testRunning_.store(false);
    });
}

void PidTunerPage::runSelectedAutoTune() {
    if (entries_.empty() || testRunning_.load()) return;

    Entry* entry = entries_[selectedIndex_].get();
    if (!entry->autoTuneCost) return;

    testRunning_.store(true);
    autoTuneActive_.store(true);
    resultsReady_.store(false);

    pros::Task([this, entry] {
        const PIDGains startGains = entry->pid->gains();
        const PIDGains stepSizes{
            .kP = std::max(kAutoTuneMinStepP, startGains.kP * kAutoTuneStepFraction),
            .kI = std::max(kAutoTuneMinStepI, startGains.kI * kAutoTuneStepFraction),
            .kD = std::max(kAutoTuneMinStepD, startGains.kD * kAutoTuneStepFraction),
        };
        tuning::TwiddleState state = tuning::makeTwiddleState(startGains, stepSizes);

        // Twiddle's internal `params` can hold an as-yet-unconfirmed trial
        // right after reportCost() returns (it always perturbs ahead for
        // the *next* index before coming back to us) — so the gains it's
        // holding when the loop ends aren't necessarily the best ones it
        // found. Track the best (gains, cost) pair independently instead
        // of trusting the search state's final snapshot.
        PIDGains bestGains = startGains;
        double bestCost = std::numeric_limits<double>::infinity();

        const auto testCurrent = [&] {
            const PIDGains testedGains = tuning::currentGains(state);
            entry->pid->setGains(testedGains);
            // Only writer of displayed*_ while this task is running (see
            // the class comment above testRunning_) — safe alongside
            // update()'s concurrent reads of the same atomics.
            this->setDisplayedGains(testedGains);
            const double cost = entry->autoTuneCost();
            if (cost < bestCost) {
                bestCost = cost;
                bestGains = testedGains;
            }
            tuning::reportCost(state, cost);
        };

        testCurrent(); // baseline
        int iteration = 0;
        while (tuning::totalStepSize(state) > kAutoTuneStepTolerance &&
              iteration < kMaxAutoTuneIterations) {
            testCurrent();
            ++iteration;
        }

        entry->pid->setGains(bestGains);
        this->setDisplayedGains(bestGains);

        this->tunedP_.store(bestGains.kP);
        this->tunedI_.store(bestGains.kI);
        this->tunedD_.store(bestGains.kD);
        this->resultsReady_.store(true);

        this->autoTuneActive_.store(false);
        this->testRunning_.store(false);
    });
}

void PidTunerPage::update() {
    if (resultsReady_.load()) {
        char buf[72];
        std::snprintf(buf, sizeof(buf), "Tuned - kP: %.3f  kI: %.3f  kD: %.3f  (hardcode these)",
                      tunedP_.load(), tunedI_.load(), tunedD_.load());
        lv_label_set_text(resultLabel_, buf);
        resultsReady_.store(false);
    }

    if (testRunning_.load()) {
        lv_label_set_text(statusLabel_, autoTuneActive_.load() ? "Auto-tuning..." : "Running...");
    } else {
        lv_label_set_text(statusLabel_, "Ready");
    }

    // The only place gain labels actually get redrawn — see
    // setDisplayedGains()'s comment for why select()/adjustGain()/the
    // auto-tune task only ever update the underlying atomics, never a
    // label directly.
    refreshGainLabels();
}

void PidTunerPage::selectorClicked(lv_event_t* e) {
    auto* page = static_cast<PidTunerPage*>(lv_event_get_user_data(e));
    lv_obj_t* clicked = lv_event_get_target_obj(e);
    for (std::size_t i = 0; i < page->entries_.size(); ++i) {
        if (page->entries_[i]->selectorButton == clicked) {
            page->select(i);
            return;
        }
    }
}

void PidTunerPage::kpMinusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustGain(0, -kStepP);
}
void PidTunerPage::kpPlusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustGain(0, kStepP);
}
void PidTunerPage::kiMinusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustGain(1, -kStepI);
}
void PidTunerPage::kiPlusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustGain(1, kStepI);
}
void PidTunerPage::kdMinusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustGain(2, -kStepD);
}
void PidTunerPage::kdPlusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustGain(2, kStepD);
}
void PidTunerPage::runTestClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->runSelectedTest();
}
void PidTunerPage::autoTuneClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->runSelectedAutoTune();
}

} // namespace sapphirelib::gui
