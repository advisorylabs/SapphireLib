#include "sapphirelib/gui/pid_tuner_page.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/tuning/characterization_math.hpp"

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

// the Run Test/Auto-Tune row, and the optional toggle under it, which ends at y=162
constexpr std::int32_t kActionRowY = kRowY0 + 3 * kRowHeight + 6;
constexpr std::int32_t kToggleY = kActionRowY + 36;
constexpr std::int32_t kToggleW = 216;

// status and results column, right of the selector buttons (which end at x=328). The page
// only gets 480x179 (a 480x240 surface minus the header and tab bar), so results go in a
// narrow column, one value per line, instead of under the controls or off the right edge
constexpr std::int32_t kReadoutX = 336;
constexpr std::int32_t kReadoutW = 140;
constexpr std::int32_t kStatusY = 4;
constexpr std::int32_t kResultY = 26;

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

lv_obj_t* makeTextButton(lv_obj_t* parent, std::int32_t x, std::int32_t y, std::int32_t w,
                         const char* text, lv_event_cb_t cb, void* userData,
                         lv_obj_t** labelOut = nullptr) {
    lv_obj_t* button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, w, 30);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, userData);
    if (labelOut) *labelOut = label;
    return button;
}

void styleSelectorButton(lv_obj_t* button) {
    lv_obj_set_style_bg_color(button, lv_color_hex(kSelectedBgColor), LV_STATE_CHECKED);
    lv_obj_set_style_text_color(button, lv_color_hex(kSelectedTextColor), LV_STATE_CHECKED);
}

} // namespace

bool PidTunerPage::isRunning() const { return testRunning_.load(); }

const char* PidTunerPage::title() const { return "PID"; }

void PidTunerPage::build(lv_obj_t* container) {
    container_ = container;
    // nothing here needs scrolling, and a stray scroll gesture on a small touchscreen fights with
    // taps
    lv_obj_remove_flag(container_, LV_OBJ_FLAG_SCROLLABLE);
    // zero padding so positions are exactly the 480x179 the page gets. The readout runs to x=476
    lv_obj_set_style_pad_all(container_, 0, 0);

    for (std::size_t i = 0; i < entries_.size(); ++i) addSelectorButton(*entries_[i], i);

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

    makeTextButton(container_, 4, kActionRowY, 100, "Run Test", &PidTunerPage::runTestClicked, this);
    makeTextButton(container_, 110, kActionRowY, 110, "Auto-Tune", &PidTunerPage::autoTuneClicked,
                   this, &autoTuneLabel_);

    toggleButton_ = makeTextButton(container_, 4, kToggleY, kToggleW, "",
                                   &PidTunerPage::toggleClicked, this, &toggleButtonLabel_);
    if (!toggleLabel_) lv_obj_add_flag(toggleButton_, LV_OBJ_FLAG_HIDDEN);

    statusLabel_ = lv_label_create(container_);
    lv_obj_set_pos(statusLabel_, kReadoutX, kStatusY);

    resultLabel_ = lv_label_create(container_);
    lv_obj_set_pos(resultLabel_, kReadoutX, kResultY);
    // fixed width with wrapping, since the failure message is a full sentence
    lv_obj_set_width(resultLabel_, kReadoutW);
    lv_label_set_long_mode(resultLabel_, LV_LABEL_LONG_WRAP);
    lv_label_set_text(resultLabel_, "");

    if (!entries_.empty()) select(0);
    refreshGainLabels();
}

void PidTunerPage::addAxis(std::string name,
                           std::function<tuning::CharacterizationConfig()> buildExperiment,
                           std::function<void(const tuning::AxisCharacterization&)> onMeasured) {
    if (testRunning_.load()) return;
    auto axis = std::make_unique<Axis>();
    axis->name = std::move(name);
    axis->buildExperiment = std::move(buildExperiment);
    axis->onMeasured = std::move(onMeasured);
    axes_.push_back(std::move(axis));
}

void PidTunerPage::addMechanismAxis(
    std::string name, std::function<tuning::MechanismCharacterizationConfig()> buildExperiment,
    std::function<void(const tuning::MechanismCharacterization&)> onMeasured) {
    if (testRunning_.load()) return;
    auto axis = std::make_unique<Axis>();
    axis->group = name;
    axis->name = std::move(name);
    axis->mechanism = true;
    axis->buildMechanismExperiment = std::move(buildExperiment);
    axis->onMechanismMeasured = std::move(onMeasured);
    axes_.push_back(std::move(axis));
}

void PidTunerPage::addController(std::string name, PID& pid, std::function<void()> runTest,
                                 std::string axis, tuning::ResponseSpec response) {
    if (testRunning_.load()) return;
    auto entry = std::make_unique<Entry>();
    entry->name = std::move(name);
    entry->pid = &pid;
    entry->runTest = std::move(runTest);
    entry->axis = std::move(axis);
    entry->response = response;

    if (container_) addSelectorButton(*entry, entries_.size());

    const bool wasEmpty = entries_.empty();
    entries_.push_back(std::move(entry));
    if (wasEmpty && container_) select(0);
}

void PidTunerPage::setToggle(std::function<std::string()> label, std::function<void()> onTap) {
    toggleLabel_ = std::move(label);
    toggleTap_ = std::move(onTap);
    if (toggleButton_) lv_obj_remove_flag(toggleButton_, LV_OBJ_FLAG_HIDDEN);
}

void PidTunerPage::addSelectorButton(Entry& entry, std::size_t row) {
    entry.selectorButton = lv_button_create(container_);
    lv_obj_set_pos(entry.selectorButton, kSelectorColumnX,
                   static_cast<std::int32_t>(row) * kSelectorRowHeight);
    lv_obj_set_size(entry.selectorButton, kSelectorButtonW, kSelectorButtonH);
    lv_obj_t* label = lv_label_create(entry.selectorButton);
    lv_label_set_text(label, entry.name.c_str());
    lv_obj_center(label);
    styleSelectorButton(entry.selectorButton);
    lv_obj_add_event_cb(entry.selectorButton, &PidTunerPage::selectorClicked, LV_EVENT_CLICKED, this);
}

void PidTunerPage::select(std::size_t index) {
    if (index >= entries_.size() || testRunning_.load()) return;

    for (auto& entry : entries_) {
        if (entry->selectorButton) lv_obj_remove_state(entry->selectorButton, LV_STATE_CHECKED);
    }
    selectedIndex_ = index;
    if (entries_[index]->selectorButton) lv_obj_add_state(entries_[index]->selectorButton, LV_STATE_CHECKED);

    setDisplayedGains(entries_[index]->pid->gains());
    readoutDirty_.store(true);
}

void PidTunerPage::setDisplayedGains(PIDGains gains) {
    // atomics only, never a widget: this is called from the Auto-Tune task too, and LVGL isn't
    // thread-safe. update() turns these into label text
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
    setLabelText(kpLabel_, buf);
    std::snprintf(buf, sizeof(buf), "kI: %.3f", displayedI_.load());
    setLabelText(kiLabel_, buf);
    std::snprintf(buf, sizeof(buf), "kD: %.3f", displayedD_.load());
    setLabelText(kdLabel_, buf);
}

const PidTunerPage::Axis* PidTunerPage::findAxis(const std::string& name) const {
    for (const auto& axis : axes_) {
        if (axis->name == name) return axis.get();
    }
    return nullptr;
}

void PidTunerPage::refreshReadout() {
    char buf[192];

    if (stopped_.load()) {
        setLabelText(resultLabel_, "Stopped\nNothing applied");
        return;
    }

    const int failed = failedAxis_.load();
    if (failed >= 0 && failed < static_cast<int>(axes_.size())) {
        // every way the fit fails (sensor backwards, not enough travel, voltage below friction)
        // shows up as a poor R^2 and few moving samples, so show both and name the usual suspects
        const Axis& axis = *axes_[failed];
        std::snprintf(buf, sizeof(buf), "%s: no fit\nR2 %.2f, %d pts\nCheck sensor sign, %s, volts",
                      axis.name.c_str(), axis.result.fit.rSquared, axis.result.fit.samplesUsed,
                      axis.mechanism ? "limits" : "travel");
        setLabelText(resultLabel_, buf);
        return;
    }

    if (entries_.empty()) {
        setLabelText(resultLabel_, "");
        return;
    }
    const Entry& entry = *entries_[selectedIndex_];
    const Axis* axis = findAxis(entry.axis);

    if (entry.axis.empty() || axis == nullptr) {
        setLabelText(resultLabel_, "Manual tuning only");
        return;
    }
    if (!entry.designed || !axis->measured) {
        std::snprintf(buf, sizeof(buf), "Axis: %s\nTap Auto-Tune", axis->name.c_str());
        setLabelText(resultLabel_, buf);
        return;
    }

    // one value per line to fit the column. The model is worth showing: it's what to copy into
    // source, and an odd kS or lag is the quickest sign of a bad run
    const MotorFeedforward& model = axis->result.fit.model.motion;
    char kA[24];
    if (axis->mechanism) {
        std::snprintf(kA, sizeof(kA), "kA %.4f kG %.2f", model.kA, axis->result.fit.model.kG);
    } else {
        std::snprintf(kA, sizeof(kA), "kA %.4f", model.kA);
    }
    // "fric" is GainDesign::staticErrorBound, how far short friction can leave the loop
    std::snprintf(buf, sizeof(buf),
                  "%s model:\nkS %.2f kV %.4f\n%s\nR2 %.2f lag %.0fms\nsettle %.2fs\nPM %.0f%s\n"
                  "fric +-%.2g\n(not saved)",
                  axis->name.c_str(), model.kS, model.kV, kA, axis->result.fit.rSquared,
                  axis->result.delayS * 1000.0, entry.design.settleTimeS,
                  entry.design.phaseMarginDeg, entry.design.limitedByDelay ? " lag-capped" : "",
                  entry.design.staticErrorBound);
    setLabelText(resultLabel_, buf);
}

void PidTunerPage::runSelectedTest() {
    if (entries_.empty() || testRunning_.load()) return;

    Entry* entry = entries_[selectedIndex_].get();
    if (!entry->runTest) return;

    testRunning_.store(true);
    autoTuneActive_.store(false);
    // runs on a background task so the test doesn't freeze the screen. It only touches
    // testRunning_, never a widget
    pros::Task([this, entry] {
        entry->runTest();
        testRunning_.store(false);
    });
}

std::string PidTunerPage::selectedGroup() const {
    if (entries_.empty()) return "";
    const Axis* axis = findAxis(entries_[selectedIndex_]->axis);
    return axis != nullptr ? axis->group : "";
}

bool PidTunerPage::measureAxis(Axis& axis, std::vector<std::function<void()>>& finishes) {
    // Stop, or the robot disabled (VEXos ignores the motors then, so the run measures nothing)
    const auto stopRequested = [this] {
        return stopRequested_.load() || pros::competition::is_disabled();
    };
    const auto withStop = [&](std::function<bool()> own) {
        return [own = std::move(own), stopRequested] { return stopRequested() || (own && own()); };
    };

    // built fresh on this task, so the factory can capture "here"
    tuning::CharacterizationData data;
    // the finish hook gives a mechanism's motors back to its loop, which mustn't run while
    // setGains() writes from this task. So it waits for runAutoTune(), after the design
    const auto deferFinish = [&finishes](tuning::CharacterizationConfig& config) {
        if (config.finish) finishes.push_back(std::move(config.finish));
        config.finish = nullptr;
    };
    if (axis.mechanism) {
        tuning::MechanismCharacterizationConfig config = axis.buildMechanismExperiment();
        config.axis.shouldAbort = withStop(std::move(config.axis.shouldAbort));
        deferFinish(config.axis);
        axis.samplePeriodS = config.axis.samplePeriodMs / 1000.0;
        data = tuning::runMechanismCharacterization(config);
        axis.result = tuning::characterizeMechanism(data, config.gravity, config.axis.minSpeed);
    } else {
        tuning::CharacterizationConfig config = axis.buildExperiment();
        config.shouldAbort = withStop(std::move(config.shouldAbort));
        deferFinish(config);
        axis.samplePeriodS = config.samplePeriodMs / 1000.0;
        data = tuning::runCharacterization(config);
        const tuning::AxisCharacterization result = tuning::characterizeAxis(data, config.minSpeed);
        axis.result =
            tuning::MechanismCharacterization{.ok = result.ok,
                                              .fit = {.ok = result.fit.ok,
                                                      .model = {.motion = result.fit.model},
                                                      .rSquared = result.fit.rSquared,
                                                      .samplesUsed = result.fit.samplesUsed},
                                              .delayS = result.delayS};
    }
    if (data.aborted) {
        stopped_.store(true);
        axis.measured = false;
        return false;
    }
    axis.measured = axis.result.ok;
    if (!axis.measured) return false;

    if (axis.mechanism) {
        if (axis.onMechanismMeasured) axis.onMechanismMeasured(axis.result);
    } else if (axis.onMeasured) {
        axis.onMeasured(
            tuning::AxisCharacterization{.ok = axis.result.ok,
                                         .fit = {.ok = axis.result.fit.ok,
                                                 .model = axis.result.fit.model.motion,
                                                 .rSquared = axis.result.fit.rSquared,
                                                 .samplesUsed = axis.result.fit.samplesUsed},
                                         .delayS = axis.result.delayS});
    }
    return true;
}

void PidTunerPage::runAutoTune() {
    if (axes_.empty() || testRunning_.load()) return;
    const std::string group = selectedGroup();

    testRunning_.store(true);
    autoTuneActive_.store(true);
    stopRequested_.store(false);
    stopped_.store(false);
    failedAxis_.store(-1);

    // results are plain fields: this task is their only writer, only while testRunning_ is true,
    // and update() only reads them after. Nothing can be added or selected mid-run
    pros::Task([this, group] {
        bool allMeasured = true;
        std::vector<std::function<void()>> finishes;
        for (std::size_t i = 0; i < axes_.size(); ++i) {
            Axis& axis = *axes_[i];
            if (axis.group != group) continue;
            measuringAxis_.store(static_cast<int>(i));
            if (!measureAxis(axis, finishes)) {
                // stop here: the robot is probably set up wrong, which likely affects the next axis
                // too (or someone tapped Stop). Nothing is applied, so every controller keeps its
                // old gains
                if (!stopped_.load()) failedAxis_.store(static_cast<int>(i));
                allMeasured = false;
                break;
            }
        }
        measuringAxis_.store(-1);

        if (allMeasured) {
            // only this group's controllers; the others keep what an earlier run designed
            for (auto& entry : entries_) {
                const Axis* axis = findAxis(entry->axis);
                if (axis == nullptr || axis->group != group) continue;
                entry->designed = false;
                if (!axis->measured) continue;

                // a loop slower than the characterization sampled holds each command longer: on
                // average half the extra period of delay
                const double slowerLoopS =
                    std::fmax(0.0, entry->pid->config().nominalDtS - axis->samplePeriodS);
                entry->design =
                    tuning::designPositionGains(axis->result.fit.model.motion, entry->response,
                                                axis->result.delayS + 0.5 * slowerLoopS);
                if (!entry->design.ok) continue;
                entry->pid->setGains(entry->design.gains);
                entry->designed = true;
            }
            if (!entries_.empty()) setDisplayedGains(entries_[selectedIndex_]->pid->gains());
        }
        // stopped, failed, or finished: now the gains are set, every run that started finishes
        for (const auto& finish : finishes) finish();

        readoutDirty_.store(true);
        autoTuneActive_.store(false);
        stopRequested_.store(false);
        testRunning_.store(false);
    });
}

void PidTunerPage::update() {
    // setLabelText() so these only redraw when they change, not every tick
    const bool autoTuning = testRunning_.load() && autoTuneActive_.load();
    setLabelText(autoTuneLabel_, autoTuning ? "Stop" : "Auto-Tune");

    if (testRunning_.load()) {
        const int measuring = measuringAxis_.load();
        if (!autoTuneActive_.load()) {
            setLabelText(statusLabel_, "Running...");
        } else if (stopRequested_.load()) {
            setLabelText(statusLabel_, "Stopping...");
        } else if (measuring >= 0 && measuring < static_cast<int>(axes_.size())) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "Measuring %s", axes_[measuring]->name.c_str());
            setLabelText(statusLabel_, buf);
        } else {
            setLabelText(statusLabel_, "Designing...");
        }
    } else {
        setLabelText(statusLabel_, "Ready");
        if (readoutDirty_.exchange(false)) refreshReadout();
    }

    if (toggleLabel_) setLabelText(toggleButtonLabel_, toggleLabel_().c_str());

    // the only place gain labels get redrawn, see setDisplayedGains()
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
    auto* page = static_cast<PidTunerPage*>(lv_event_get_user_data(e));
    // the same button is Stop during Auto-Tune. The run notices within a sample period
    if (page->testRunning_.load()) {
        if (page->autoTuneActive_.load()) page->stopRequested_.store(true);
        return;
    }
    page->runAutoTune();
}
void PidTunerPage::toggleClicked(lv_event_t* e) {
    auto* page = static_cast<PidTunerPage*>(lv_event_get_user_data(e));
    if (page->testRunning_.load() || !page->toggleTap_) return;
    page->toggleTap_();
}

} // namespace sapphirelib::gui
