/**
 * \file sapphirelib/telemetry/event.hpp
 *
 * The one telemetry call the rest of SapphireLib makes: log a low-rate event
 * (a motion starting or ending, an autonomous routine being picked) to
 * whichever telemetry::Logger is running, if any. Library code calls this
 * instead of holding a Logger, so nothing outside telemetry/ depends on one
 * existing — with no Logger started it does nothing and costs one atomic load.
 *
 * Team 96671H — Hitmen
 */

#pragma once

namespace sapphirelib::telemetry {

/// Logs an `E` row (see docs/TELEMETRY_FORMAT.md) to the Logger that most
/// recently called Logger::start(). printf-style; the tag is cut to 15
/// characters and the message to 191 (a long message travels as up to four
/// records, committed together). Returns false if no Logger is running or the
/// row had to be dropped.
///
/// Formats on the calling task, so it's for markers, not data: fine at a
/// motion's start and end, wrong inside a 100Hz loop (use a Channel there).
/// Never blocks.
bool event(const char* tag, const char* format, ...) __attribute__((format(printf, 2, 3)));

} // namespace sapphirelib::telemetry
