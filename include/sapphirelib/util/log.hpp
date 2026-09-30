#pragma once

#include <cstdio>

#include "pros/rtos.hpp"

/**
 * @brief Minimum level that gets printed to the terminal
 *
 * Define it before including this header, or as a build flag
 * (EXTRA_CXXFLAGS+=-DSAPPHIRELIB_LOG_LEVEL=0). Info by default
 */
#ifndef SAPPHIRELIB_LOG_LEVEL
#define SAPPHIRELIB_LOG_LEVEL 1
#endif

namespace sapphirelib {

/**
 * @brief Log levels, lowest to highest
 */
enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3, None = 4 };

namespace detail {

inline const char* logLevelTag(LogLevel level) {
    switch (level) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info: return "INFO";
        case LogLevel::Warn: return "WARN";
        case LogLevel::Error: return "ERROR";
        default: return "";
    }
}

} // namespace detail

} // namespace sapphirelib

/**
 * @brief Print a message to the terminal, with the time, level, and a subsystem tag
 *
 * @b Example
 * @code {.cpp}
 * SAPPHIRELIB_LOG_INFO("lift", "reached level %d", level);
 * @endcode
 */
#define SAPPHIRELIB_LOG(level, tag, fmt, ...)                                                    \
    do {                                                                                         \
        if (static_cast<int>(level) >= SAPPHIRELIB_LOG_LEVEL) {                                  \
            std::printf("[%8lu][%-5s][%s] " fmt "\n", static_cast<unsigned long>(pros::millis()), \
                         sapphirelib::detail::logLevelTag(level), tag, ##__VA_ARGS__);            \
        }                                                                                         \
    } while (0)

#define SAPPHIRELIB_LOG_DEBUG(tag, fmt, ...)                                                     \
    SAPPHIRELIB_LOG(sapphirelib::LogLevel::Debug, tag, fmt, ##__VA_ARGS__)
#define SAPPHIRELIB_LOG_INFO(tag, fmt, ...)                                                       \
    SAPPHIRELIB_LOG(sapphirelib::LogLevel::Info, tag, fmt, ##__VA_ARGS__)
#define SAPPHIRELIB_LOG_WARN(tag, fmt, ...)                                                       \
    SAPPHIRELIB_LOG(sapphirelib::LogLevel::Warn, tag, fmt, ##__VA_ARGS__)
#define SAPPHIRELIB_LOG_ERROR(tag, fmt, ...)                                                      \
    SAPPHIRELIB_LOG(sapphirelib::LogLevel::Error, tag, fmt, ##__VA_ARGS__)
