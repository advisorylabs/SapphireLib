/**
 * \file sapphirelib/util/log.hpp
 *
 * Minimal logging/telemetry macros used across every SapphireLib subsystem.
 * Prints to stdout (PROS routes this to the USB terminal), tagged with an
 * uptime timestamp, level, and caller-supplied subsystem tag.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdio>

#include "pros/rtos.hpp"

/**
 * Minimum level that gets printed. Define before including this header (or
 * as a build flag, e.g. EXTRA_CXXFLAGS+=-DSAPPHIRELIB_LOG_LEVEL=0) to change
 * verbosity; defaults to Info so Debug-level spam is compiled in but silent.
 */
#ifndef SAPPHIRELIB_LOG_LEVEL
#define SAPPHIRELIB_LOG_LEVEL 1
#endif

namespace sapphirelib {

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
