#include "sapphirelib/gui/auton_selector_page.hpp"

#include <cstdint>
#include <utility>

namespace sapphirelib::gui {

namespace {

constexpr std::uint32_t kSelectedBgColor = 0x2563eb;
constexpr std::uint32_t kSelectedTextColor = 0xffffff;

void styleButton(lv_obj_t* button) {
    lv_obj_set_style_bg_color(button, lv_color_hex(kSelectedBgColor), LV_STATE_CHECKED);
    lv_obj_set_style_text_color(button, lv_color_hex(kSelectedTextColor), LV_STATE_CHECKED);
}

} // namespace

const char* AutonSelectorPage::title() const { return "Auton"; }

void AutonSelectorPage::build(lv_obj_t* container) {
    container_ = container;
    list_ = lv_list_create(container);
    lv_obj_set_size(list_, LV_PCT(100), LV_PCT(100));

    for (auto& routine : routines_) {
        routine->button = lv_list_add_button(list_, nullptr, routine->name.c_str());
        styleButton(routine->button);
        lv_obj_add_event_cb(routine->button, &AutonSelectorPage::buttonClicked, LV_EVENT_CLICKED, this);
    }
    if (!routines_.empty()) select(0);
}

void AutonSelectorPage::addRoutine(std::string name, std::function<void()> routine) {
    auto entry = std::make_unique<Routine>();
    entry->name = std::move(name);
    entry->callback = std::move(routine);

    if (container_) {
        entry->button = lv_list_add_button(list_, nullptr, entry->name.c_str());
        styleButton(entry->button);
        lv_obj_add_event_cb(entry->button, &AutonSelectorPage::buttonClicked, LV_EVENT_CLICKED, this);
    }

    const bool wasEmpty = routines_.empty();
    routines_.push_back(std::move(entry));
    if (wasEmpty && container_) select(0);
}

void AutonSelectorPage::select(std::size_t index) {
    if (index >= routines_.size()) return;
    for (auto& routine : routines_) {
        if (routine->button) lv_obj_remove_state(routine->button, LV_STATE_CHECKED);
    }
    selectedIndex_ = index;
    if (routines_[index]->button) lv_obj_add_state(routines_[index]->button, LV_STATE_CHECKED);
}

void AutonSelectorPage::run() const {
    if (routines_.empty()) return;
    if (routines_[selectedIndex_]->callback) routines_[selectedIndex_]->callback();
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
