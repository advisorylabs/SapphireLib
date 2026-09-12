#include "sapphirelib/gui/page.hpp"

#include <cstring>

namespace sapphirelib::gui {

void setLabelText(lv_obj_t* label, const char* text) {
    if (label == nullptr) return;
    const char* current = lv_label_get_text(label);
    if (current != nullptr && std::strcmp(current, text) == 0) return;
    lv_label_set_text(label, text);
}

} // namespace sapphirelib::gui
