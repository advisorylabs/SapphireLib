#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "sapphirelib/localization/particle_filter.hpp"

namespace sapphirelib::localization {

/**
 * @brief Settings for a MonteCarloLocalizer
 *
 * The defaults suit a V5 robot with good tracking wheels and four distance sensors. The simulator
 * (tools/sim) runs the same settings, and its Tune tab searches for better ones, so try changes
 * there first. Every field can also be set by name (setLocalizerSetting()), which is how a
 * TUNE.CFG on the SD card overrides them without rebuilding
 */
struct LocalizerConfig {
    /** the particle filter: particle count, noise, the sensor model, recovery */
    ParticleFilterConfig filter;

    /**
     * how far odometry's pose may be off, in inches (1 standard deviation), when particles are
     * placed around it: at the first update and after every Odometry::setPose()
     */
    double startSpreadIn = 2.0;

    /**
     * how old a distance reading is by the time it's read, in milliseconds. The sensor measures
     * at about 30Hz, so a reading is on average a few tens of milliseconds old; at 60in/s that's
     * nearly 2in. Compensated for with odometry's velocity. 0 turns compensation off
     */
    double sensorLatencyMs = 30.0;

    /** readings past 200mm with a lower confidence (0 to 63) are skipped */
    std::int32_t minConfidence = 20;

    /**
     * skip the sensors while turning faster than this, in degrees per second. Mid-spin, a beam
     * sweeps across whatever it's pointed at while the sensor measures. Prediction keeps running
     */
    double maxTurnRateDegPerS = 200.0;

    /** whether to correct odometry's pose. See MonteCarloLocalizer::setCorrectionEnabled() */
    bool correctOdometry = true;

    /**
     * don't correct odometry until Odometry::setPose() has run once. Until then the pose is
     * relative to wherever the robot sat at startup, not the field the map describes, and the
     * particles can only hunt for the robot. True by default; set it false if the Odometry is
     * constructed with the robot's real field pose
     */
    bool waitForSetPose = true;

    /** only correct while the particles' spread is below this, in inches */
    double maxCorrectionSpreadIn = 3.0;

    /**
     * only correct when at least this many sensors agree with the estimate: a sensor blocked by
     * another robot disagrees, the walls the others see still agree
     */
    std::size_t minAgreeingSensors = 2;

    /** a sensor agrees when it's within this many sigmas of what the map says from the estimate */
    double agreementSigmas = 3.0;

    /**
     * how fast a correction may move odometry's pose, in inches per second. A motion's derivative
     * term sees this as extra speed, so keep it well under the drive's top speed. 0 or less jumps
     */
    double maxCorrectionRateInPerS = 4.0;
};

/**
 * @brief One LocalizerConfig field, by name
 *
 * The name is the field's path in LocalizerConfig, the same one the simulator's MCL tab and
 * TUNE.CFG use: "filter.motionNoise.perInch", "sensorLatencyMs". Switches are 0 or 1
 */
struct LocalizerSetting {
    /** the field's path, e.g. "filter.beam.outlierProbability" */
    const char* key;

    /** the smallest value accepted */
    double min;

    /** the largest value accepted */
    double max;

    /** whether the value must be a whole number: counts, and 0/1 switches */
    bool whole;
};

/**
 * @brief Why setLocalizerSetting() refused a value
 */
enum class SettingError : std::uint8_t {
    none,       // set
    unknownKey, // no field by that name
    notFinite,  // NaN or infinity
    outOfRange, // outside the setting's [min, max]
    notWhole,   // a fraction for a count or a switch
};

/**
 * @brief Get every setting setLocalizerSetting() knows, in a fixed order
 */
std::span<const LocalizerSetting> localizerSettings();

/**
 * @brief Find a setting by name
 *
 * @param key the field's path
 * @return const LocalizerSetting* the setting, or nullptr for an unknown name
 */
const LocalizerSetting* findLocalizerSetting(std::string_view key);

/**
 * @brief Set one field of a LocalizerConfig by name, checking it against the setting's range
 *
 * @param config the config to change. Left as it was when the value is refused
 * @param key the field's path, e.g. "filter.particleCount"
 * @param value the new value. Switches take 0 or 1
 * @return SettingError none if it was set
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::localization::LocalizerConfig config;
 * setLocalizerSetting(config, "filter.motionNoise.perInch", 0.08);
 * @endcode
 */
SettingError setLocalizerSetting(LocalizerConfig& config, std::string_view key, double value);

/**
 * @brief Read one field of a LocalizerConfig by name
 *
 * @param config the config
 * @param key the field's path
 * @param value set to the field's value. Switches read as 0 or 1
 * @return true the name is known
 */
bool getLocalizerSetting(const LocalizerConfig& config, std::string_view key, double& value);

/**
 * @brief Describe a SettingError in a few words, for a status line
 */
const char* settingErrorText(SettingError error);

} // namespace sapphirelib::localization
