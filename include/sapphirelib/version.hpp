#pragma once

#define SAPPHIRELIB_VERSION_MAJOR 0
#define SAPPHIRELIB_VERSION_MINOR 1
#define SAPPHIRELIB_VERSION_PATCH 0
#define SAPPHIRELIB_VERSION "0.1.0"

namespace sapphirelib {

/**
 * @brief Get the library version
 *
 * @return const char* the version string, e.g. "0.1.0"
 */
const char* version();

} // namespace sapphirelib
