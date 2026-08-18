/**
 * \file sapphirelib/version.hpp
 *
 * Library version information.
 */

#pragma once

#define SAPPHIRELIB_VERSION_MAJOR 0
#define SAPPHIRELIB_VERSION_MINOR 1
#define SAPPHIRELIB_VERSION_PATCH 0
#define SAPPHIRELIB_VERSION "0.1.0"

namespace sapphirelib {

/// Returns the library version as a string, e.g. "0.0.1-dev".
const char* version();

} // namespace sapphirelib
