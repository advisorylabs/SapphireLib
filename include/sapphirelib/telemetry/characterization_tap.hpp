/**
 * \file sapphirelib/telemetry/characterization_tap.hpp
 *
 * Mirrors an Auto-Tune characterization run into a telemetry channel without
 * touching tuning/: wraps the CharacterizationConfig a PidTunerPage axis
 * factory returns. Pure, unit-tested in
 * tests/telemetry/characterization_tap_test.cpp.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/telemetry/channel.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"

namespace sapphirelib::telemetry {

/// Returns `config` with measure()/actuate() wrapped so each tick of the run
/// also logs (volts, position) to `channel` — the same pair
/// tuning::CharacterizationSample records. The app can then refit an axis
/// offline from real runs and compare against what Auto-Tune fitted on the
/// robot. `channel` needs two columns, volts first (e.g. {"volts", "pos"}).
///
/// runCharacterization() measures, then actuates, once per tick, so a row is
/// logged on each actuate() that follows a fresh measure(): exactly one per
/// CharacterizationSample, with the same volts and position. The runner's
/// other calls — the 0V it commands before waiting for the axis to stop, and
/// its polling while it waits — log nothing, so segments show up in the file
/// as bursts of rows ~samplePeriodMs apart separated by gaps of 100ms or more.
/// The one extra row: a segment cut short by its travel limit ends with a 0V
/// row at the out-of-range position (docs/TELEMETRY_FORMAT.md, "Refitting an
/// axis offline").
///
/// The original callbacks are still called, with the same arguments, in the
/// same order. Allocates here (one small shared state), never per tick. A
/// config whose measure or actuate is empty is returned unchanged.
tuning::CharacterizationConfig tapCharacterization(tuning::CharacterizationConfig config,
                                                   Channel& channel);

/// tapCharacterization() for runMechanismCharacterization(): the same rows,
/// plus one for each hold() that follows a fresh measure() — a held sample,
/// logged with NaN volts exactly as the runner records it. So the pre-roll
/// before each segment is in the log, and a segment cut short by its limit
/// ends with one NaN row at the out-of-range position (where a drive axis's
/// ends with a 0V one). With no hold set, the runner holds with actuate(0),
/// which logs as it always does.
tuning::MechanismCharacterizationConfig
tapMechanismCharacterization(tuning::MechanismCharacterizationConfig config, Channel& channel);

} // namespace sapphirelib::telemetry
