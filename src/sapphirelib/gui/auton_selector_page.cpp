#include "sapphirelib/gui/auton_selector_page.hpp"

#include <cstdint>
#include <utility>

#include "sapphirelib/telemetry/sd_card.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"

namespace sapphirelib::gui {

namespace {

constexpr std::uint32_t kSelectedBgColor = 0x991b1b;
constexpr std::uint32_t kSelectedTextColor = 0xffffff;

// how often the status line is refreshed
constexpr std::uint32_t kStatusIntervalMs = 250;

// rememberSelection()'s file: never more than a name, and how long to wait for the card. The read
// is at startup, before the logger writes much; a write waits out at most one of its passes
constexpr std::size_t kMaxFileBytes = 128;
constexpr std::uint32_t kCardWaitMs = 200;

// what selectedName() returns with nothing registered. At namespace scope so it's built
// during static init
const std::string kNoRoutineName;

void styleButton(lv_obj_t* button) {
    lv_obj_set_style_bg_color(button, lv_color_hex(kSelectedBgColor), LV_STATE_CHECKED);
    lv_obj_set_style_text_color(button, lv_color_hex(kSelectedTextColor), LV_STATE_CHECKED);
}

// the name on the file's first line, without the line ending or trailing spaces
std::string firstLine(const std::string& text) {
    std::string line = text.substr(0, text.find_first_of("\r\n"));
    while (!line.empty() && line.back() == ' ') line.pop_back();
    return line;
}

} // namespace

const char* AutonSelectorPage::title() const { return "Auton"; }

void AutonSelectorPage::build(lv_obj_t* container) {
    container_ = container;
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(container, 4, 0);

    if (status_) statusLabel_ = lv_label_create(container);
    list_ = lv_list_create(container);
    lv_obj_set_width(list_, LV_PCT(100));
    lv_obj_set_flex_grow(list_, 1);

    for (auto& routine : routines_) addButton(*routine);
    // whatever is picked already: rememberSelection() may have run first
    highlight();
    update();
}

void AutonSelectorPage::addButton(Routine& routine) {
    routine.button = lv_list_add_button(list_, nullptr, routine.name.c_str());
    styleButton(routine.button);
    lv_obj_add_event_cb(routine.button, &AutonSelectorPage::buttonClicked, LV_EVENT_CLICKED, this);
}

void AutonSelectorPage::addRoutine(std::string name, std::function<void()> routine) {
    auto entry = std::make_unique<Routine>();
    entry->name = std::move(name);
    entry->callback = std::move(routine);
    if (container_) addButton(*entry);

    const bool wasEmpty = routines_.empty();
    routines_.push_back(std::move(entry));
    if (wasEmpty && container_) highlight();
}

void AutonSelectorPage::pick(std::size_t index) {
    if (index >= routines_.size()) return;
    selectedIndex_ = index;
    highlightStale_ = true;
    if (!path_.empty()) saveDue_ = true;
}

void AutonSelectorPage::select(std::size_t index) {
    pick(index);
    highlight();
}

void AutonSelectorPage::highlight() {
    highlightStale_ = false;
    if (routines_.empty()) return;
    const std::size_t selected = selectedIndex_.load();
    for (std::size_t i = 0; i < routines_.size(); ++i) {
        lv_obj_t* button = routines_[i]->button;
        if (button == nullptr) continue;
        if (i == selected) {
            lv_obj_add_state(button, LV_STATE_CHECKED);
            lv_obj_scroll_to_view(button, LV_ANIM_OFF);
        } else {
            lv_obj_remove_state(button, LV_STATE_CHECKED);
        }
    }
}

void AutonSelectorPage::next() {
    if (routines_.empty()) return;
    pick((selectedIndex_.load() + 1) % routines_.size());
}

void AutonSelectorPage::previous() {
    if (routines_.empty()) return;
    pick((selectedIndex_.load() + routines_.size() - 1) % routines_.size());
}

void AutonSelectorPage::rememberSelection(std::string path) {
    path_ = std::move(path);
    std::string text;
    if (!telemetry::readCardFile(path_.c_str(), text, kMaxFileBytes, kCardWaitMs)) return;
    const std::string name = firstLine(text);
    for (std::size_t i = 0; i < routines_.size(); ++i) {
        if (routines_[i]->name == name) {
            selectedIndex_ = i;
            highlightStale_ = true;
            SAPPHIRELIB_LOG_INFO("auton", "picked \"%s\" from %s", name.c_str(), path_.c_str());
            return;
        }
    }
    SAPPHIRELIB_LOG_WARN("auton", "%s names \"%s\", which isn't a routine any more", path_.c_str(),
                         name.c_str());
}

void AutonSelectorPage::setStatus(std::function<std::string()> text) { status_ = std::move(text); }

void AutonSelectorPage::update() {
    if (highlightStale_.load()) highlight();
    if (saveDue_.exchange(false)) {
        if (!telemetry::writeCardFile(path_.c_str(), selectedName() + "\n", kCardWaitMs)) {
            SAPPHIRELIB_LOG_WARN("auton", "couldn't save the pick to %s", path_.c_str());
        }
    }
    if (statusLabel_ == nullptr || !status_) return;
    const std::uint32_t now = sapphirelib::millis();
    if (lastStatusMs_ != 0 && now - lastStatusMs_ < kStatusIntervalMs) return;
    lastStatusMs_ = now;
    setLabelText(statusLabel_, status_().c_str());
}

void AutonSelectorPage::run() const {
    if (routines_.empty()) return;
    // read the index once, so a tap mid-call can't check one routine and run another
    const Routine& routine = *routines_[selectedIndex_.load()];
    if (routine.callback) routine.callback();
}

const std::string& AutonSelectorPage::selectedName() const {
    if (routines_.empty()) return kNoRoutineName;
    return routines_[selectedIndex_.load()]->name;
}

void AutonSelectorPage::buttonClicked(lv_event_t* e) {
    auto* page = static_cast<AutonSelectorPage*>(lv_event_get_user_data(e));
    lv_obj_t* clicked = lv_event_get_target_obj(e);
    for (std::size_t i = 0; i < page->routines_.size(); ++i) {
        if (page->routines_[i]->button == clicked) {
            page->select(i);
            return;
        }
    }
}

} // namespace sapphirelib::gui
