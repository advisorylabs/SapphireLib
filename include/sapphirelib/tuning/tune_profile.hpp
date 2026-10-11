#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "sapphirelib/control/feedforward.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/localization/localizer_config.hpp"

// TUNE.CFG: tuned values the robot loads from its SD card at startup, so a change from the
// simulator's or the telemetry analyzer's tuners runs without rebuilding. Plain text, one
// "key=value" per line, '#' to the end of a line is a comment:
//
//   format=1
//   rev=7
//   note=turn refit from SL000041-SL000048
//   pid.turn=0.42,0,0.031          # kP,kI,kD
//   model.fwd=1.02,0.198,0.047     # kS,kV,kA
//   lift.gravityVolts=1.35
//   mcl.filter.motionNoise.perInch=0.07
//
// The robot program names which pid.*, model.* and other keys it understands (TuneSchema); mcl.*
// keys are LocalizerConfig fields by path (localizer_config.hpp). A file is used whole or not at
// all: one bad line rejects it, so the robot never runs half a profile. See docs/TUNING.md

namespace sapphirelib::tuning {

/** the version on a TUNE.CFG's format= line that this library reads */
constexpr int kTuneFormatVersion = 1;

/** the longest TUNE.CFG accepted, in bytes. The analyzer keeps its changelog comments under it */
constexpr std::size_t kMaxTuneFileBytes = 32 * 1024;

/**
 * @brief What a robot program accepts in its TUNE.CFG, besides the mcl.* settings
 */
struct TuneSchema {
    /** a plain number setting, e.g. "lift.gravityVolts" */
    struct Value {
        /** the full key */
        std::string key;
        /** the smallest value accepted */
        double min = 0.0;
        /** the largest value accepted */
        double max = 0.0;
        /** whether it must be a whole number */
        bool whole = false;
    };

    /** controllers, as pid.<name>=kP,kI,kD */
    std::vector<std::string> pids;

    /** axis models, as model.<name>=kS,kV,kA */
    std::vector<std::string> models;

    /** anything else */
    std::vector<Value> values;
};

/**
 * @brief A parsed TUNE.CFG
 */
struct TuneProfile {
    /** the file's revision; the analyzer adds one every time it writes the file */
    int revision = 0;

    /** a free text note on what this revision changed */
    std::string note;

    /** pid.<name> lines, in file order */
    std::vector<std::pair<std::string, PIDGains>> pids;

    /** model.<name> lines, in file order */
    std::vector<std::pair<std::string, MotorFeedforward>> models;

    /** schema values, in file order */
    std::vector<std::pair<std::string, double>> values;

    /** mcl.* settings, by LocalizerConfig path without the "mcl." */
    std::vector<std::pair<std::string, double>> mcl;

    /**
     * @brief Find a pid.<name> line
     *
     * @return const PIDGains* the gains, or nullptr if the file doesn't set them
     */
    const PIDGains* pid(std::string_view name) const;

    /**
     * @brief Find a model.<name> line
     *
     * @return const MotorFeedforward* the model, or nullptr if the file doesn't set it
     */
    const MotorFeedforward* model(std::string_view name) const;

    /**
     * @brief Find a schema value
     *
     * @param key the full key, e.g. "lift.gravityVolts"
     * @param fallback returned when the file doesn't set it
     */
    double value(std::string_view key, double fallback) const;

    /**
     * @brief Get a LocalizerConfig with this file's mcl.* settings on top
     *
     * @param config the robot program's own settings
     * @return LocalizerConfig those settings, changed where the file says
     */
    localization::LocalizerConfig applyTo(localization::LocalizerConfig config) const;

    /**
     * @brief Get how many settings the file has, of every kind
     */
    std::size_t settingCount() const;
};

/**
 * @brief Result of parseTuneProfile()
 */
struct TuneParseResult {
    /** whether the whole file was valid. When false, use none of it */
    bool ok = false;

    /** the file's settings. Empty unless ok */
    TuneProfile profile;

    /** the 1-based line the first problem is on, 0 for a problem with the whole file */
    std::size_t errorLine = 0;

    /** what was wrong, e.g. "pid.turn: needs kP,kI,kD". Empty when ok */
    std::string error;
};

/**
 * @brief Parse and check a TUNE.CFG
 *
 * Every line must be a key this schema (or LocalizerConfig) knows, with a value in range, and no
 * key may appear twice. The first setting must be format=1. Windows line endings are fine
 *
 * @param text the file's contents
 * @param schema what the robot program accepts
 * @return TuneParseResult the profile, or the first problem found
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::tuning::TuneSchema schema{.pids = {"drive", "turn"}, .models = {"fwd", "turn"}};
 * auto result = sapphirelib::tuning::parseTuneProfile(text, schema);
 * if (result.ok) {
 *     if (auto* gains = result.profile.pid("turn")) drivetrain().turnPID().setGains(*gains);
 * }
 * @endcode
 */
TuneParseResult parseTuneProfile(std::string_view text, const TuneSchema& schema);

} // namespace sapphirelib::tuning
