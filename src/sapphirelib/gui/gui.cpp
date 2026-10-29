#include "sapphirelib/gui/gui.hpp"

#include "liblvgl/themes/default/lv_theme_default.h"

namespace sapphirelib::gui {

namespace {

constexpr std::int32_t kHeaderHeightPx = 18;
constexpr std::int32_t kTabBarSizePx = 43; // 28 + 10%, then +12px more

constexpr std::uint32_t kBrandBgColor = 0x2e1a1a;
constexpr std::uint32_t kBrandTextColor = 0xf87171;
// purple, which a red header couldn't be told apart from
constexpr std::uint32_t kWarningBgColor = 0x581c87;
constexpr std::uint32_t kWarningTextColor = 0xfaf5ff;

// LVGL's default theme in red: buttons, the selected tab, sliders and switches. A checked button
// (a toggle left on) is purple, so it stands out against them
constexpr std::uint32_t kThemePrimaryColor = 0xdc2626;
constexpr std::uint32_t kThemeSecondaryColor = 0x9333ea;

} // namespace

Gui::Gui(const char* brandText) : brandText_(brandText) {
    lv_obj_t* screen = lv_screen_active();

    // the kernel starts LVGL on its default theme in blue. The same theme again, dark as before
    // and in the same font, recolors every widget on every page, the ones already made included
    lv_display_t* display = lv_display_get_default();
    lv_display_set_theme(display, lv_theme_default_init(display, lv_color_hex(kThemePrimaryColor),
                                                        lv_color_hex(kThemeSecondaryColor),
                                                        LV_THEME_DEFAULT_DARK,
                                                        lv_theme_get_font_normal(screen)));

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

void Gui::start(std::uint32_t periodMs) {
    // a second timer would refresh every page twice as often
    if (timer_) return;
    timer_ = lv_timer_create(&Gui::timerTrampoline, periodMs, this);
}

bool Gui::anyPageBusy() const {
    for (const auto& page : pages_) {
        if (page->isBusy()) return true;
    }
    return false;
}

void Gui::showWarning(const std::string& text) {
    if (warningActive_ && text == lv_label_get_text(headerLabel_)) return;
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
    // tab indices match pages_: addPage() adds both together, and neither is ever removed
    const std::size_t active = lv_tabview_get_tab_active(tabview_);

    for (std::size_t i = 0; i < pages_.size(); ++i) {
        // a page that just became visible refreshes on the same tick, so switching tabs never shows
        // a stale frame
        if (i == active || pages_[i]->updatesWhenHidden()) pages_[i]->update();
    }
}

} // namespace sapphirelib::gui
