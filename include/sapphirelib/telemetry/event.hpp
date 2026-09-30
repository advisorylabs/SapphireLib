#pragma once

namespace sapphirelib::telemetry {

/**
 * @brief Log an event to the running telemetry::Logger, if there is one, printf style
 *
 * Library code uses this instead of holding a Logger, so nothing depends on one existing. With no
 * Logger started it does nothing. Formats on the calling task, so it's for markers, not data:
 * fine at a motion's start and end, wrong inside a 100Hz loop (use a Channel there). Never blocks
 *
 * @param tag event tag, cut to 15 characters
 * @param format printf format string. The message is cut to 191 characters
 * @return true the event was logged
 * @return false no Logger is running, or the row was dropped
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::telemetry::event("mark", "driver");
 * sapphirelib::telemetry::event("auton", "start,%s", routineName);
 * @endcode
 */
bool event(const char* tag, const char* format, ...) __attribute__((format(printf, 2, 3)));

} // namespace sapphirelib::telemetry
