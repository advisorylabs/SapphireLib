#include "sapphirelib/tuning/tune_profile.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace sapphirelib::tuning {

namespace {

constexpr std::size_t kMaxNoteChars = 160;
constexpr std::string_view kPidPrefix = "pid.";
constexpr std::string_view kModelPrefix = "model.";
constexpr std::string_view kMclPrefix = "mcl.";

std::string_view trim(std::string_view text) {
    const auto isSpace = [](char c) { return c == ' ' || c == '\t' || c == '\r'; };
    while (!text.empty() && isSpace(text.front())) text.remove_prefix(1);
    while (!text.empty() && isSpace(text.back())) text.remove_suffix(1);
    return text;
}

bool startsWith(std::string_view text, std::string_view prefix) {
    return text.size() >= prefix.size() && text.substr(0, prefix.size()) == prefix;
}

// one number, all of `text`, finite. strtod() wants a terminated string, and accepts "nan" and
// "inf", which a tuning value never is
bool parseNumber(std::string_view text, double& out) {
    text = trim(text);
    char buffer[48];
    if (text.empty() || text.size() >= sizeof(buffer)) return false;
    std::memcpy(buffer, text.data(), text.size());
    buffer[text.size()] = '\0';
    char* end = nullptr;
    const double value = std::strtod(buffer, &end);
    if (end != buffer + text.size() || !std::isfinite(value)) return false;
    out = value;
    return true;
}

// exactly `count` comma-separated numbers
bool parseNumbers(std::string_view text, double* out, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t comma = text.find(',');
        const bool last = i + 1 == count;
        if (last != (comma == std::string_view::npos)) return false;
        if (!parseNumber(last ? text : text.substr(0, comma), out[i])) return false;
        if (!last) text.remove_prefix(comma + 1);
    }
    return true;
}

bool contains(const std::vector<std::string>& names, std::string_view name) {
    for (const std::string& n : names) {
        if (n == name) return true;
    }
    return false;
}

const TuneSchema::Value* findValue(const TuneSchema& schema, std::string_view key) {
    for (const TuneSchema::Value& value : schema.values) {
        if (value.key == key) return &value;
    }
    return nullptr;
}

template <typename T>
const T* findIn(const std::vector<std::pair<std::string, T>>& entries, std::string_view name) {
    for (const auto& [key, value] : entries) {
        if (key == name) return &value;
    }
    return nullptr;
}

std::string quoted(std::string_view key, const char* problem) {
    std::string text(key);
    text += ": ";
    text += problem;
    return text;
}

} // namespace

const PIDGains* TuneProfile::pid(std::string_view name) const { return findIn(pids, name); }

const MotorFeedforward* TuneProfile::model(std::string_view name) const {
    return findIn(models, name);
}

double TuneProfile::value(std::string_view key, double fallback) const {
    const double* found = findIn(values, key);
    return found != nullptr ? *found : fallback;
}

localization::LocalizerConfig TuneProfile::applyTo(localization::LocalizerConfig config) const {
    // every entry was checked when the file was parsed, so none of these can fail
    for (const auto& [key, value] : mcl) localization::setLocalizerSetting(config, key, value);
    return config;
}

std::size_t TuneProfile::settingCount() const {
    return pids.size() + models.size() + values.size() + mcl.size();
}

TuneParseResult parseTuneProfile(std::string_view text, const TuneSchema& schema) {
    TuneParseResult result;
    if (text.size() > kMaxTuneFileBytes) {
        result.error = "file is too big";
        return result;
    }

    TuneProfile profile;
    std::vector<std::string> seen;
    bool sawFormat = false;
    std::size_t lineNumber = 0;

    const auto fail = [&](std::string error) {
        result.errorLine = lineNumber;
        result.error = std::move(error);
        return result;
    };

    while (!text.empty()) {
        ++lineNumber;
        const std::size_t newline = text.find('\n');
        std::string_view line = text.substr(0, newline);
        text.remove_prefix(newline == std::string_view::npos ? text.size() : newline + 1);

        const std::size_t comment = line.find('#');
        if (comment != std::string_view::npos) line = line.substr(0, comment);
        line = trim(line);
        if (line.empty()) continue;

        const std::size_t equals = line.find('=');
        if (equals == std::string_view::npos) return fail("expected key=value");
        const std::string_view key = trim(line.substr(0, equals));
        const std::string_view value = trim(line.substr(equals + 1));
        if (key.empty()) return fail("missing key");
        if (contains(seen, key)) return fail(quoted(key, "set twice"));
        seen.emplace_back(key);

        if (!sawFormat) {
            if (key != "format") return fail("the first setting must be format=1");
            double version = 0.0;
            if (!parseNumber(value, version) || version != std::floor(version) || version < 1) {
                return fail("format: needs a version number");
            }
            if (version != kTuneFormatVersion) {
                char message[64];
                std::snprintf(message, sizeof(message),
                              "format %d is newer than this program reads",
                              static_cast<int>(version));
                return fail(message);
            }
            sawFormat = true;
            continue;
        }

        if (key == "format") return fail(quoted(key, "set twice"));
        if (key == "rev") {
            double revision = 0.0;
            if (!parseNumber(value, revision) || revision != std::floor(revision) || revision < 0 ||
                revision > 1e9) {
                return fail("rev: needs a whole number");
            }
            profile.revision = static_cast<int>(revision);
        } else if (key == "note") {
            if (value.size() > kMaxNoteChars) return fail("note: too long");
            profile.note = std::string(value);
        } else if (const TuneSchema::Value* known = findValue(schema, key)) {
            // ahead of the prefixes, so a program can name its own key under one (mcl.periodMs)
            double number = 0.0;
            if (!parseNumber(value, number)) return fail(quoted(key, "needs a number"));
            if (number < known->min || number > known->max) {
                return fail(quoted(key, "out of range"));
            }
            if (known->whole && number != std::floor(number)) {
                return fail(quoted(key, "must be a whole number"));
            }
            profile.values.emplace_back(std::string(key), number);
        } else if (startsWith(key, kPidPrefix)) {
            const std::string_view name = key.substr(kPidPrefix.size());
            if (!contains(schema.pids, name)) return fail(quoted(key, "no such controller"));
            double gains[3];
            if (!parseNumbers(value, gains, 3)) return fail(quoted(key, "needs kP,kI,kD"));
            if (gains[0] < 0 || gains[1] < 0 || gains[2] < 0) {
                return fail(quoted(key, "gains can't be negative"));
            }
            profile.pids.emplace_back(std::string(name),
                                      PIDGains{.kP = gains[0], .kI = gains[1], .kD = gains[2]});
        } else if (startsWith(key, kModelPrefix)) {
            const std::string_view name = key.substr(kModelPrefix.size());
            if (!contains(schema.models, name)) return fail(quoted(key, "no such axis"));
            double terms[3];
            if (!parseNumbers(value, terms, 3)) return fail(quoted(key, "needs kS,kV,kA"));
            const MotorFeedforward model{.kS = terms[0], .kV = terms[1], .kA = terms[2]};
            if (model.kS < 0 || !model.valid()) {
                return fail(quoted(key, "kS can't be negative, kV and kA must be positive"));
            }
            profile.models.emplace_back(std::string(name), model);
        } else if (startsWith(key, kMclPrefix)) {
            const std::string_view path = key.substr(kMclPrefix.size());
            double number = 0.0;
            if (!parseNumber(value, number)) return fail(quoted(key, "needs a number"));
            localization::LocalizerConfig scratch;
            const localization::SettingError error =
                localization::setLocalizerSetting(scratch, path, number);
            if (error != localization::SettingError::none) {
                return fail(quoted(key, localization::settingErrorText(error)));
            }
            profile.mcl.emplace_back(std::string(path), number);
        } else {
            return fail(quoted(key, "unknown setting"));
        }
    }

    if (!sawFormat) {
        lineNumber = 0;
        return fail("no format=1 line");
    }
    result.ok = true;
    result.profile = std::move(profile);
    return result;
}

} // namespace sapphirelib::tuning
