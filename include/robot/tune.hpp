/**
 * \file robot/tune.hpp
 *
 * TUNE.CFG: tuned values on the SD card, loaded once at startup, so what the
 * simulator's MCL tuner and the telemetry analyzer's PID refinement propose
 * runs without rebuilding. The file's format is in
 * include/sapphirelib/tuning/tune_profile.hpp, and the whole workflow in
 * docs/TUNING.md.
 *
 * The values in the code (devices.cpp, macros.cpp, LocalizerConfig's
 * defaults) stay the fallback: with no card, no file, or a file with any bad
 * line, the robot runs exactly what's compiled in, and the Home page says
 * which.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <cstdint>
#include <string>

#include "sapphirelib/localization/localizer_config.hpp"
#include "sapphirelib/telemetry/logger.hpp"
#include "sapphirelib/tuning/tune_profile.hpp"

namespace robot {

/// Where the robot looks for its tuning, in order: next to the logs, then
/// the card's root (where logs go when the sl folder is missing).
constexpr const char* kTuneFilePaths[] = {"/usd/sl/TUNE.CFG", "/usd/TUNE.CFG"};

/// What loading TUNE.CFG came to.
struct TuneStatus {
    enum class State {
        noCard,   // no SD card in the brain
        none,     // no TUNE.CFG: the code's values
        loaded,   // every line checked out and applies
        rejected, // a bad line: none of it applies
    };
    State state = State::none;
    /// The file that was read, if any.
    std::string path;
    /// The parse result: the profile when loaded, the first problem when
    /// rejected.
    sapphirelib::tuning::TuneParseResult result;
};

/// Reads and checks TUNE.CFG the first time it's called, from initialize()
/// (localizer() is the first caller). Never fails: a missing or bad file
/// just means the code's values.
const TuneStatus& tuneStatus();

/// The localizer's settings: LocalizerConfig's defaults with the file's
/// mcl.* lines on top.
sapphirelib::localization::LocalizerConfig localizerSettings();

/// The localizer's update period: the file's mcl.periodMs, or 50ms.
std::uint32_t localizerPeriodMs();

/// Sets the drivetrain's and the lift's gains, the drivetrain's axis models
/// and the lift's gravity volts from the file, wherever it has them. Call
/// once from initDevices(), after every device exists and before anything
/// runs a PID.
void applyTuneProfile();

/// The Home page's "Tune:" line, e.g. "Tune: TUNE.CFG r7, 11 settings".
std::string tuneStatusText();

/// Adds #meta lines to every log: which TUNE.CFG revision this run used (or
/// why none), and the localizer's full settings and sensor mounts. Call from
/// startTelemetry(), before the logger starts.
void describeTuning(sapphirelib::telemetry::Logger& logger);

} // namespace robot
