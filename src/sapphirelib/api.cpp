#include "sapphirelib/api.hpp"

namespace sapphirelib {

void initialize() {
    SAPPHIRELIB_LOG_INFO("core", "SapphireLib v%s initialized", version());
}

} // namespace sapphirelib
