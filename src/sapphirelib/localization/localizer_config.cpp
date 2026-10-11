#include "sapphirelib/localization/localizer_config.hpp"

#include <cmath>

namespace sapphirelib::localization {

namespace {

// one row per settable field: its LocalizerSetting, and how to read and write it. Captureless
// lambdas, so the table is plain data. tools/analyzer/js/tunefile.js mirrors the keys and ranges,
// and its test reads them out of this table, so keep one entry per line
struct Field {
    LocalizerSetting setting;
    double (*get)(const LocalizerConfig&);
    void (*set)(LocalizerConfig&, double);
};

// clang-format off
constexpr Field kFields[] = {
    {{"filter.particleCount", 10, 5000, true},
     [](const LocalizerConfig& c) { return static_cast<double>(c.filter.particleCount); },
     [](LocalizerConfig& c, double v) { c.filter.particleCount = static_cast<std::size_t>(v); }},
    {{"filter.motionNoise.baseIn", 0, 2, false},
     [](const LocalizerConfig& c) { return c.filter.motionNoise.baseIn; },
     [](LocalizerConfig& c, double v) { c.filter.motionNoise.baseIn = v; }},
    {{"filter.motionNoise.perInch", 0, 1, false},
     [](const LocalizerConfig& c) { return c.filter.motionNoise.perInch; },
     [](LocalizerConfig& c, double v) { c.filter.motionNoise.perInch = v; }},
    {{"filter.motionNoise.perDegreeIn", 0, 0.5, false},
     [](const LocalizerConfig& c) { return c.filter.motionNoise.perDegreeIn; },
     [](LocalizerConfig& c, double v) { c.filter.motionNoise.perDegreeIn = v; }},
    {{"filter.beam.maxRangeIn", 1, 200, false},
     [](const LocalizerConfig& c) { return c.filter.beam.maxRangeIn; },
     [](LocalizerConfig& c, double v) { c.filter.beam.maxRangeIn = v; }},
    {{"filter.beam.minSigmaIn", 0.01, 20, false},
     [](const LocalizerConfig& c) { return c.filter.beam.minSigmaIn; },
     [](LocalizerConfig& c, double v) { c.filter.beam.minSigmaIn = v; }},
    {{"filter.beam.sigmaFraction", 0, 1, false},
     [](const LocalizerConfig& c) { return c.filter.beam.sigmaFraction; },
     [](LocalizerConfig& c, double v) { c.filter.beam.sigmaFraction = v; }},
    {{"filter.beam.outlierProbability", 0.001, 0.99, false},
     [](const LocalizerConfig& c) { return c.filter.beam.outlierProbability; },
     [](LocalizerConfig& c, double v) { c.filter.beam.outlierProbability = v; }},
    {{"filter.resampleThreshold", 0, 1, false},
     [](const LocalizerConfig& c) { return c.filter.resampleThreshold; },
     [](LocalizerConfig& c, double v) { c.filter.resampleThreshold = v; }},
    {{"filter.recovery.enabled", 0, 1, true},
     [](const LocalizerConfig& c) { return c.filter.recovery.enabled ? 1.0 : 0.0; },
     [](LocalizerConfig& c, double v) { c.filter.recovery.enabled = v != 0.0; }},
    {{"filter.recovery.slowRate", 0, 1, false},
     [](const LocalizerConfig& c) { return c.filter.recovery.slowRate; },
     [](LocalizerConfig& c, double v) { c.filter.recovery.slowRate = v; }},
    {{"filter.recovery.fastRate", 0, 1, false},
     [](const LocalizerConfig& c) { return c.filter.recovery.fastRate; },
     [](LocalizerConfig& c, double v) { c.filter.recovery.fastRate = v; }},
    {{"filter.recovery.triggerRatio", 0, 1, false},
     [](const LocalizerConfig& c) { return c.filter.recovery.triggerRatio; },
     [](LocalizerConfig& c, double v) { c.filter.recovery.triggerRatio = v; }},
    {{"filter.recovery.radiusIn", 0, 200, false},
     [](const LocalizerConfig& c) { return c.filter.recovery.radiusIn; },
     [](LocalizerConfig& c, double v) { c.filter.recovery.radiusIn = v; }},
    {{"filter.recovery.maxFraction", 0, 1, false},
     [](const LocalizerConfig& c) { return c.filter.recovery.maxFraction; },
     [](LocalizerConfig& c, double v) { c.filter.recovery.maxFraction = v; }},
    {{"filter.seed", 0, 4294967295.0, true},
     [](const LocalizerConfig& c) { return static_cast<double>(c.filter.seed); },
     [](LocalizerConfig& c, double v) { c.filter.seed = static_cast<std::uint32_t>(v); }},
    {{"startSpreadIn", 0, 72, false},
     [](const LocalizerConfig& c) { return c.startSpreadIn; },
     [](LocalizerConfig& c, double v) { c.startSpreadIn = v; }},
    {{"sensorLatencyMs", 0, 500, false},
     [](const LocalizerConfig& c) { return c.sensorLatencyMs; },
     [](LocalizerConfig& c, double v) { c.sensorLatencyMs = v; }},
    {{"minConfidence", 0, 63, true},
     [](const LocalizerConfig& c) { return static_cast<double>(c.minConfidence); },
     [](LocalizerConfig& c, double v) { c.minConfidence = static_cast<std::int32_t>(v); }},
    {{"maxTurnRateDegPerS", 0, 5000, false},
     [](const LocalizerConfig& c) { return c.maxTurnRateDegPerS; },
     [](LocalizerConfig& c, double v) { c.maxTurnRateDegPerS = v; }},
    {{"correctOdometry", 0, 1, true},
     [](const LocalizerConfig& c) { return c.correctOdometry ? 1.0 : 0.0; },
     [](LocalizerConfig& c, double v) { c.correctOdometry = v != 0.0; }},
    {{"waitForSetPose", 0, 1, true},
     [](const LocalizerConfig& c) { return c.waitForSetPose ? 1.0 : 0.0; },
     [](LocalizerConfig& c, double v) { c.waitForSetPose = v != 0.0; }},
    {{"maxCorrectionSpreadIn", 0, 100, false},
     [](const LocalizerConfig& c) { return c.maxCorrectionSpreadIn; },
     [](LocalizerConfig& c, double v) { c.maxCorrectionSpreadIn = v; }},
    {{"minAgreeingSensors", 0, 16, true},
     [](const LocalizerConfig& c) { return static_cast<double>(c.minAgreeingSensors); },
     [](LocalizerConfig& c, double v) { c.minAgreeingSensors = static_cast<std::size_t>(v); }},
    {{"agreementSigmas", 0, 100, false},
     [](const LocalizerConfig& c) { return c.agreementSigmas; },
     [](LocalizerConfig& c, double v) { c.agreementSigmas = v; }},
    {{"maxCorrectionRateInPerS", 0, 1000, false},
     [](const LocalizerConfig& c) { return c.maxCorrectionRateInPerS; },
     [](LocalizerConfig& c, double v) { c.maxCorrectionRateInPerS = v; }},
};
// clang-format on

constexpr std::size_t kFieldCount = sizeof(kFields) / sizeof(kFields[0]);

// the settings on their own, for localizerSettings()'s span
constexpr auto kSettings = [] {
    struct Table {
        LocalizerSetting rows[kFieldCount];
    } table{};
    for (std::size_t i = 0; i < kFieldCount; ++i) table.rows[i] = kFields[i].setting;
    return table;
}();

const Field* findField(std::string_view key) {
    for (const Field& field : kFields) {
        if (key == field.setting.key) return &field;
    }
    return nullptr;
}

} // namespace

std::span<const LocalizerSetting> localizerSettings() { return {kSettings.rows, kFieldCount}; }

const LocalizerSetting* findLocalizerSetting(std::string_view key) {
    const Field* field = findField(key);
    return field != nullptr ? &field->setting : nullptr;
}

SettingError setLocalizerSetting(LocalizerConfig& config, std::string_view key, double value) {
    const Field* field = findField(key);
    if (field == nullptr) return SettingError::unknownKey;
    if (!std::isfinite(value)) return SettingError::notFinite;
    if (value < field->setting.min || value > field->setting.max) return SettingError::outOfRange;
    if (field->setting.whole && value != std::floor(value)) return SettingError::notWhole;
    field->set(config, value);
    return SettingError::none;
}

bool getLocalizerSetting(const LocalizerConfig& config, std::string_view key, double& value) {
    const Field* field = findField(key);
    if (field == nullptr) return false;
    value = field->get(config);
    return true;
}

const char* settingErrorText(SettingError error) {
    switch (error) {
        case SettingError::none: return "ok";
        case SettingError::unknownKey: return "unknown setting";
        case SettingError::notFinite: return "not a number";
        case SettingError::outOfRange: return "out of range";
        case SettingError::notWhole: return "must be a whole number";
    }
    return "invalid";
}

} // namespace sapphirelib::localization
