#include "sapphirelib/gui/pid_tuner_page.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/odom/odometry_math.hpp"
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
// a row per controller: 36px each while they fit, then the page's height shared between them, so
// a drivetrain's three tracking controllers fit beside Drive, Turn, Hold and Lift
constexpr std::int32_t kSelectorRowHeight = 36;
constexpr std::int32_t kSelectorButtonGap = 4;
constexpr std::int32_t kPageHeight = 179;

// the Run Test/Auto-Tune row, and the optional toggle under it, which ends at y=162
constexpr std::int32_t kActionRowY = kRowY0 + 3 * kRowHeight + 6;
constexpr std::int32_t kToggleY = kActionRowY + 36;
constexpr std::int32_t kToggleW = 216;
constexpr std::int32_t kRunTestW = 100;
constexpr std::int32_t kAutoTuneX = 110;
constexpr std::int32_t kAutoTuneW = 110;
// with a wheel calibration, Run Test and Auto-Tune narrow to fit the Wheel button beside them,
// which ends 4px short of the selector buttons
constexpr std::int32_t kNarrowRunTestW = 80;
constexpr std::int32_t kNarrowAutoTuneX = 88;
constexpr std::int32_t kNarrowAutoTuneW = 84;
constexpr std::int32_t kWheelButtonX = 176;
constexpr std::int32_t kWheelButtonW = 60;

// the left column, which the gains and the Wheel panel take turns in
constexpr std::int32_t kLeftColumnW = kSelectorColumnX - 4;

// the Wheel panel: Drive and Back, Actual and its -/+ (where the kP row's are), Apply, and a hint
// that wraps over the rest of the column. With a strafe too, Fwd and Strafe split Drive's width
constexpr std::int32_t kWheelDriveW = 140;
constexpr std::int32_t kWheelSplitDriveW = 68;
constexpr std::int32_t kWheelStrafeX = 76;
constexpr std::int32_t kWheelBackX = 150;
constexpr std::int32_t kWheelBackW = 82;
constexpr std::int32_t kActualRowY = 40;
constexpr std::int32_t kApplyRowY = 76;
constexpr std::int32_t kWheelHintY = 114;
// an eighth of an inch, a tape measure's usual marks. Hold -/+ to repeat
constexpr double kActualStepIn = 0.125;
// once the drive returns, how long the robot gets to roll to a stop before the wheels are read
constexpr std::uint32_t kWheelSettleMs = 300;
// wheelRuns_' order
constexpr std::size_t kForwardRun = 0;
constexpr std::size_t kStrafeRun = 1;

// status and results column, right of the selector buttons (which end at x=328). The page
// only gets 480x179 (a 480x240 surface minus the header and tab bar), so results go in a
// narrow column, one value per line, instead of under the controls or off the right edge
constexpr std::int32_t kReadoutX = 336;
constexpr std::int32_t kReadoutW = 140;
constexpr std::int32_t kStatusY = 4;
constexpr std::int32_t kResultY = 26;

lv_obj_t* makeIconButton(lv_obj_t* parent, std::int32_t x, std::int32_t y, const char* text,
                         lv_event_cb_t cb, void* userData,
                         lv_event_code_t event = LV_EVENT_CLICKED) {
    lv_obj_t* button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, kBtnW, kBtnH);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, cb, event, userData);
    return button;
}

// a -/+ that steps once on a tap and keeps stepping while held. A short click rather than a click,
// so letting go after a hold doesn't step once more
lv_obj_t* makeRepeatButton(lv_obj_t* parent, std::int32_t x, std::int32_t y, const char* text,
                           lv_event_cb_t cb, void* userData) {
    lv_obj_t* button = makeIconButton(parent, x, y, text, cb, userData, LV_EVENT_SHORT_CLICKED);
    lv_obj_add_event_cb(button, cb, LV_EVENT_LONG_PRESSED_REPEAT, userData);
    return button;
}

// a bare container at the top left of its parent: no background, border, padding or scrolling, so
// what's in it sits exactly where it would on the parent
lv_obj_t* makePanel(lv_obj_t* parent, std::int32_t w, std::int32_t h) {
    lv_obj_t* panel = lv_obj_create(parent);
    lv_obj_remove_style_all(panel);
    lv_obj_set_pos(panel, 0, 0);
    lv_obj_set_size(panel, w, h);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    return panel;
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

    for (auto& entry : entries_) addSelectorButton(*entry);
    layoutSelectorButtons();

    // the left column's gain controls, in a panel of their own so the Wheel panel can take their
    // place
    gainPanel_ = makePanel(container_, kLeftColumnW, kPageHeight);

    kpLabel_ = lv_label_create(gainPanel_);
    lv_obj_set_pos(kpLabel_, 4, kRowY0 + 5);
    makeIconButton(gainPanel_, kBtnMinusX, kRowY0, "-", &PidTunerPage::kpMinusClicked, this);
    makeIconButton(gainPanel_, kBtnPlusX, kRowY0, "+", &PidTunerPage::kpPlusClicked, this);

    kiLabel_ = lv_label_create(gainPanel_);
    lv_obj_set_pos(kiLabel_, 4, kRowY0 + kRowHeight + 5);
    makeIconButton(gainPanel_, kBtnMinusX, kRowY0 + kRowHeight, "-", &PidTunerPage::kiMinusClicked,
                   this);
    makeIconButton(gainPanel_, kBtnPlusX, kRowY0 + kRowHeight, "+", &PidTunerPage::kiPlusClicked,
                   this);

    kdLabel_ = lv_label_create(gainPanel_);
    lv_obj_set_pos(kdLabel_, 4, kRowY0 + 2 * kRowHeight + 5);
    makeIconButton(gainPanel_, kBtnMinusX, kRowY0 + 2 * kRowHeight, "-",
                   &PidTunerPage::kdMinusClicked, this);
    makeIconButton(gainPanel_, kBtnPlusX, kRowY0 + 2 * kRowHeight, "+",
                   &PidTunerPage::kdPlusClicked, this);

    const bool wheelCalibration = hasWheelRun(kForwardRun) || hasWheelRun(kStrafeRun);
    makeTextButton(gainPanel_, 4, kActionRowY, wheelCalibration ? kNarrowRunTestW : kRunTestW,
                   "Run Test", &PidTunerPage::runTestClicked, this);
    makeTextButton(gainPanel_, wheelCalibration ? kNarrowAutoTuneX : kAutoTuneX, kActionRowY,
                   wheelCalibration ? kNarrowAutoTuneW : kAutoTuneW, "Auto-Tune",
                   &PidTunerPage::autoTuneClicked, this, &autoTuneLabel_);
    if (wheelCalibration) {
        makeTextButton(gainPanel_, kWheelButtonX, kActionRowY, kWheelButtonW, "Wheel",
                       &PidTunerPage::wheelClicked, this);
    }

    toggleButton_ = makeTextButton(gainPanel_, 4, kToggleY, kToggleW, "",
                                   &PidTunerPage::toggleClicked, this, &toggleButtonLabel_);
    if (!toggleLabel_) lv_obj_add_flag(toggleButton_, LV_OBJ_FLAG_HIDDEN);

    if (wheelCalibration) buildWheelPanel();

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

    if (container_) addSelectorButton(*entry);

    const bool wasEmpty = entries_.empty();
    entries_.push_back(std::move(entry));
    if (container_) layoutSelectorButtons();
    if (wasEmpty && container_) select(0);
}

void PidTunerPage::setToggle(std::function<std::string()> label, std::function<void()> onTap) {
    toggleLabel_ = std::move(label);
    toggleTap_ = std::move(onTap);
    if (toggleButton_) lv_obj_remove_flag(toggleButton_, LV_OBJ_FLAG_HIDDEN);
}

void PidTunerPage::enableWheelCalibration(std::vector<CalibratedWheel> wheels,
                                          std::function<void(double)> driveForward,
                                          double distanceIn) {
    // the Wheel button is laid out with the rest in build()
    if (container_ != nullptr || testRunning_.load()) return;
    wheelRuns_[kForwardRun] = WheelRun{std::move(wheels), std::move(driveForward), distanceIn};
}

void PidTunerPage::enableStrafeCalibration(std::vector<CalibratedWheel> wheels,
                                           std::function<void(double)> driveRight,
                                           double distanceIn) {
    if (container_ != nullptr || testRunning_.load()) return;
    wheelRuns_[kStrafeRun] = WheelRun{std::move(wheels), std::move(driveRight), distanceIn};
}

bool PidTunerPage::hasWheelRun(std::size_t run) const { return !wheelRuns_[run].wheels.empty(); }

void PidTunerPage::buildWheelPanel() {
    wheelPanel_ = makePanel(container_, kLeftColumnW, kPageHeight);

    const auto driveButton = [this](std::int32_t x, std::int32_t w, const char* format,
                                    std::size_t run, lv_event_cb_t cb) {
        char text[24];
        std::snprintf(text, sizeof(text), format, wheelRuns_[run].distanceIn);
        makeTextButton(wheelPanel_, x, 0, w, text, cb, this);
    };
    // split, there's only room for the short names, without the units
    if (hasWheelRun(kForwardRun) && hasWheelRun(kStrafeRun)) {
        driveButton(4, kWheelSplitDriveW, "Fwd %.0f", kForwardRun,
                    &PidTunerPage::wheelForwardClicked);
        driveButton(kWheelStrafeX, kWheelSplitDriveW, "Strafe %.0f", kStrafeRun,
                    &PidTunerPage::wheelStrafeClicked);
    } else if (hasWheelRun(kForwardRun)) {
        driveButton(4, kWheelDriveW, "Drive %.0fin", kForwardRun,
                    &PidTunerPage::wheelForwardClicked);
    } else {
        driveButton(4, kWheelDriveW, "Strafe %.0fin", kStrafeRun,
                    &PidTunerPage::wheelStrafeClicked);
    }
    makeTextButton(wheelPanel_, kWheelBackX, 0, kWheelBackW, "Back",
                   &PidTunerPage::wheelBackClicked, this);

    actualLabel_ = lv_label_create(wheelPanel_);
    lv_obj_set_pos(actualLabel_, 4, kActualRowY + 5);
    makeRepeatButton(wheelPanel_, kBtnMinusX, kActualRowY, "-", &PidTunerPage::actualMinusClicked,
                     this);
    makeRepeatButton(wheelPanel_, kBtnPlusX, kActualRowY, "+", &PidTunerPage::actualPlusClicked,
                     this);

    makeTextButton(wheelPanel_, 4, kApplyRowY, kWheelDriveW, "Apply",
                   &PidTunerPage::wheelApplyClicked, this);

    wheelHintLabel_ = lv_label_create(wheelPanel_);
    lv_obj_set_pos(wheelHintLabel_, 4, kWheelHintY);
    lv_obj_set_width(wheelHintLabel_, kLeftColumnW - 8);
    lv_label_set_long_mode(wheelHintLabel_, LV_LABEL_LONG_WRAP);

    lv_obj_add_flag(wheelPanel_, LV_OBJ_FLAG_HIDDEN);
    refreshWheelLabels();
}

void PidTunerPage::setWheelMode(bool on) {
    if (wheelPanel_ == nullptr || on == wheelMode_ || testRunning_.load()) return;
    wheelMode_ = on;
    if (on) {
        lv_obj_add_flag(gainPanel_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(wheelPanel_, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(wheelPanel_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(gainPanel_, LV_OBJ_FLAG_HIDDEN);
    }
    refreshWheelLabels();
    readoutDirty_.store(true);
}

void PidTunerPage::runWheelDrive(std::size_t run) {
    if (!hasWheelRun(run) || !wheelRuns_[run].drive || testRunning_.load()) return;

    testRunning_.store(true);
    autoTuneActive_.store(false);
    wheelDriving_.store(true);
    refreshWheelLabels();
    // on a background task, like Run Test, so the screen keeps going. It only touches the run's
    // numbers and atomics, never a widget; update() takes the numbers once wheelRunReady_ is set
    pros::Task([this, run] {
        const WheelRun& wheelRun = wheelRuns_[run];
        std::vector<double> diameters;
        std::vector<double> starts;
        for (const auto& wheel : wheelRun.wheels) {
            diameters.push_back(wheel.diameterIn());
            starts.push_back(wheel.distanceIn());
        }
        wheelRun.drive(wheelRun.distanceIn);
        pros::delay(kWheelSettleMs);
        std::vector<double> reads;
        for (std::size_t i = 0; i < wheelRun.wheels.size(); ++i) {
            reads.push_back(wheelRun.wheels[i].distanceIn() - starts[i]);
        }
        wheelReadRun_ = run;
        wheelDiameterAtRunIn_ = std::move(diameters);
        wheelReadIn_ = std::move(reads);
        wheelRunReady_.store(true);
        wheelDriving_.store(false);
        testRunning_.store(false);
    });
}

void PidTunerPage::adjustActual(double deltaIn) {
    if (!wheelMeasured_ || testRunning_.load()) return;
    actualIn_ = std::max(kActualStepIn, actualIn_ + deltaIn);
    refreshWheelLabels();
}

void PidTunerPage::applyWheelSizes() {
    if (!wheelMeasured_ || testRunning_.load()) return;
    // from each wheel's diameter at the drive, not its diameter now, so applying again after
    // changing Actual corrects the run's reading rather than compounding the last Apply
    const std::vector<CalibratedWheel>& wheels = wheelRuns_[wheelReadRun_].wheels;
    wheelAppliedIn_.assign(wheels.size(), std::nan(""));
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const double diameterIn =
            odom::calibrateWheelDiameterIn(wheelDiameterAtRunIn_[i], wheelReadIn_[i], actualIn_);
        if (std::isnan(diameterIn)) continue;
        wheels[i].setDiameterIn(diameterIn);
        wheelAppliedIn_[i] = diameterIn;
    }
    wheelApplied_ = true;
    appliedActualIn_ = actualIn_;
    refreshWheelLabels();
    readoutDirty_.store(true);
}

void PidTunerPage::refreshWheelLabels() {
    if (wheelPanel_ == nullptr) return;
    char buf[32];
    if (wheelMeasured_) {
        std::snprintf(buf, sizeof(buf), "Actual: %.3fin", actualIn_);
    } else {
        std::snprintf(buf, sizeof(buf), "Actual: --");
    }
    setLabelText(actualLabel_, buf);

    // the edge to mark is the one facing the way it'll drive: Strafe goes right
    const char* hint =
        !hasWheelRun(kStrafeRun)
            ? "Mark the robot's edge, room ahead. Drive, then tape the mark to that edge"
        : !hasWheelRun(kForwardRun)
            ? "Mark the robot's right edge, room to the right. Drive, then tape the mark to that "
              "edge"
            : "Mark its front (Fwd) or right (Strafe) edge, room that way. Drive, then tape the "
              "mark to that edge";
    if (wheelDriving_.load()) {
        hint = "Driving... keep clear";
    } else if (wheelMeasured_ && !wheelApplied_) {
        hint = "Set Actual to what the tape says, then Apply";
    } else if (wheelApplied_ && actualIn_ != appliedActualIn_) {
        hint = "Actual changed since Apply: Apply again to use it";
    } else if (wheelApplied_) {
        hint = "Applied, not saved: copy the new sizes into the code. Drive again to check";
    }
    setLabelText(wheelHintLabel_, hint);
}

std::string PidTunerPage::wheelReadoutText() const {
    char line[48];
    std::string text;
    if (!wheelMeasured_) {
        // every run's, since none has driven yet
        text = "Diameters now:";
        for (const auto& run : wheelRuns_) {
            for (const auto& wheel : run.wheels) {
                std::snprintf(line, sizeof(line), "\n%s %.3fin", wheel.name.c_str(),
                              wheel.diameterIn());
                text += line;
            }
        }
        return text;
    }
    const std::vector<CalibratedWheel>& wheels = wheelRuns_[wheelReadRun_].wheels;
    text = "Read over the run:";
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        std::snprintf(line, sizeof(line), "\n%s %.2fin", wheels[i].name.c_str(), wheelReadIn_[i]);
        text += line;
    }
    if (!wheelApplied_) return text;
    text += "\nNow (not saved):";
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const double appliedIn = wheelAppliedIn_[i];
        if (!std::isnan(appliedIn)) {
            std::snprintf(line, sizeof(line), "\n%s %.3fin", wheels[i].name.c_str(), appliedIn);
        } else {
            // why calibrateWheelDiameterIn() said no
            const char* why = std::fabs(wheelReadIn_[i]) < 1.0 ? "didn't turn"
                              : actualIn_ < 1.0                ? "too short"
                                                               : "reads backward";
            std::snprintf(line, sizeof(line), "\n%s --, %s", wheels[i].name.c_str(), why);
        }
        text += line;
    }
    return text;
}

void PidTunerPage::layoutSelectorButtons() {
    if (entries_.empty()) return;
    const auto count = static_cast<std::int32_t>(entries_.size());
    const std::int32_t rowHeight = std::min(kSelectorRowHeight, kPageHeight / count);
    for (std::int32_t row = 0; row < count; ++row) {
        lv_obj_t* button = entries_[static_cast<std::size_t>(row)]->selectorButton;
        if (button == nullptr) continue;
        lv_obj_set_pos(button, kSelectorColumnX, row * rowHeight);
        lv_obj_set_size(button, kSelectorButtonW, rowHeight - kSelectorButtonGap);
    }
}

void PidTunerPage::addSelectorButton(Entry& entry) {
    entry.selectorButton = lv_button_create(container_);
    lv_obj_t* label = lv_label_create(entry.selectorButton);
    lv_label_set_text(label, entry.name.c_str());
    lv_obj_center(label);
    styleSelectorButton(entry.selectorButton);
    lv_obj_add_event_cb(entry.selectorButton, &PidTunerPage::selectorClicked, LV_EVENT_CLICKED,
                        this);
}

void PidTunerPage::select(std::size_t index) {
    if (index >= entries_.size() || testRunning_.load()) return;
    // a controller's button goes back to its gains from the Wheel panel too
    setWheelMode(false);

    for (auto& entry : entries_) {
        if (entry->selectorButton) lv_obj_remove_state(entry->selectorButton, LV_STATE_CHECKED);
    }
    selectedIndex_ = index;
    if (entries_[index]->selectorButton)
        lv_obj_add_state(entries_[index]->selectorButton, LV_STATE_CHECKED);

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

    if (wheelMode_) {
        setLabelText(resultLabel_, wheelReadoutText().c_str());
        return;
    }

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
        if (wheelDriving_.load()) {
            setLabelText(statusLabel_, "Driving...");
        } else if (!autoTuneActive_.load()) {
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
        if (wheelRunReady_.exchange(false)) {
            // a drive just finished: Actual starts at what the run's first wheel read, to the
            // nearest eighth, or at the distance asked for if it read nothing usable
            wheelMeasured_ = true;
            wheelApplied_ = false;
            const double readIn = wheelReadIn_.empty() ? 0.0 : wheelReadIn_.front();
            actualIn_ = readIn >= 1.0 ? std::round(readIn / kActualStepIn) * kActualStepIn
                                      : wheelRuns_[wheelReadRun_].distanceIn;
            readoutDirty_.store(true);
        }
        if (readoutDirty_.exchange(false)) refreshReadout();
    }

    if (wheelMode_) refreshWheelLabels();
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
void PidTunerPage::wheelClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->setWheelMode(true);
}
void PidTunerPage::wheelBackClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->setWheelMode(false);
}
void PidTunerPage::wheelForwardClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->runWheelDrive(kForwardRun);
}
void PidTunerPage::wheelStrafeClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->runWheelDrive(kStrafeRun);
}
void PidTunerPage::actualMinusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustActual(-kActualStepIn);
}
void PidTunerPage::actualPlusClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->adjustActual(kActualStepIn);
}
void PidTunerPage::wheelApplyClicked(lv_event_t* e) {
    static_cast<PidTunerPage*>(lv_event_get_user_data(e))->applyWheelSizes();
}

} // namespace sapphirelib::gui
