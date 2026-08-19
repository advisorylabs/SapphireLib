#include "sapphirelib/gui/gui.hpp"

namespace sapphirelib::gui {

namespace {

constexpr std::int32_t kHeaderHeightPx = 18;
constexpr std::int32_t kTabBarSizePx = 43; // 28 + 10%, then +12px more

constexpr std::uint32_t kBrandBgColor = 0x1a1a2e;
constexpr std::uint32_t kBrandTextColor = 0x7dd3fc;
constexpr std::uint32_t kWarningBgColor = 0x7f1d1d;
constexpr std::uint32_t kWarningTextColor = 0xfef2f2;

} // namespace

Gui::Gui(const char* brandText) : brandText_(brandText) {
    lv_obj_t* screen = lv_screen_active();

    header_ = lv_obj_create(screen);
    lv_obj_set_size(header_, LV_HOR_RES, kHeaderHeightPx);
    lv_obj_set_pos(header_, 0, 0);
    lv_obj_set_style_radius(header_, 0, 0);
    lv_obj_set_style_border_width(header_, 0, 0);
    lv_obj_set_style_pad_all(header_, 2, 0);
    lv_obj_set_style_bg_color(header_, lv_color_hex(kBrandBgColor), 0);

    headerLabel_ = lv_label_create(header_);
    lv_label_set_text(headerLabel_, brandText_.c_str());
    lv_obj_set_style_text_color(headerLabel_, lv_color_hex(kBrandTextColor), 0);
    lv_obj_center(headerLabel_);

    tabview_ = lv_tabview_create(screen);
    lv_obj_set_size(tabview_, LV_HOR_RES, LV_VER_RES - kHeaderHeightPx);
    lv_obj_set_pos(tabview_, 0, kHeaderHeightPx);
    lv_tabview_set_tab_bar_position(tabview_, LV_DIR_BOTTOM);
    lv_tabview_set_tab_bar_size(tabview_, kTabBarSizePx);
}

void Gui::addPage(std::unique_ptr<Page> page) {
    lv_obj_t* content = lv_tabview_add_tab(tabview_, page->title());
    page->build(content);
    pages_.push_back(std::move(page));
}

void Gui::start(std::uint32_t periodMs) { timer_ = lv_timer_create(&Gui::timerTrampoline, periodMs, this); }

void Gui::showWarning(const std::string& text) {
    if (warningActive_ && text == std::string(lv_label_get_text(headerLabel_))) return;
    warningActive_ = true;
    lv_obj_set_style_bg_color(header_, lv_color_hex(kWarningBgColor), 0);
    lv_obj_set_style_text_color(headerLabel_, lv_color_hex(kWarningTextColor), 0);
    lv_label_set_text(headerLabel_, text.c_str());
}

void Gui::clearWarning() {
    if (!warningActive_) return;
    warningActive_ = false;
    lv_obj_set_style_bg_color(header_, lv_color_hex(kBrandBgColor), 0);
    lv_obj_set_style_text_color(headerLabel_, lv_color_hex(kBrandTextColor), 0);
    lv_label_set_text(headerLabel_, brandText_.c_str());
}

void Gui::timerTrampoline(lv_timer_t* timer) {
    static_cast<Gui*>(lv_timer_get_user_data(timer))->tick();
}

void Gui::tick() {
    for (auto& page : pages_) page->update();
}

} // namespace sapphirelib::gui
