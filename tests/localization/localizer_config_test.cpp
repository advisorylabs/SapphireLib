// Host-side unit test for sapphirelib::localization's LocalizerConfig settings by name, no
// PROS/embedded dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/localization/localizer_config_test.cpp src/sapphirelib/localization/localizer_config.cpp -o localizer_config_test && ./localizer_config_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "sapphirelib/localization/localizer_config.hpp"

using namespace sapphirelib::localization;

namespace {

void testEveryKeyRoundTrips() {
    // set each setting to a value inside its range, read it back, and check nothing else moved
    for (const LocalizerSetting& setting : localizerSettings()) {
        LocalizerConfig config;
        const double value = setting.whole ? std::floor((setting.min + setting.max) / 2.0)
                                           : setting.min + 0.37 * (setting.max - setting.min);
        assert(setLocalizerSetting(config, setting.key, value) == SettingError::none);
        double readBack = -1.0;
        assert(getLocalizerSetting(config, setting.key, readBack));
        const double expected = setting.whole && setting.max == 1.0 ? (value != 0.0) : value;
        if (std::fabs(readBack - expected) > 1e-9) {
            std::printf("FAIL %s: set %g, read %g\n", setting.key, value, readBack);
            assert(false);
        }
        for (const LocalizerSetting& other : localizerSettings()) {
            if (std::string(other.key) == setting.key) continue;
            double before = 0.0;
            double after = 0.0;
            getLocalizerSetting(LocalizerConfig{}, other.key, before);
            getLocalizerSetting(config, other.key, after);
            if (before != after) {
                std::printf("FAIL setting %s also changed %s\n", setting.key, other.key);
                assert(false);
            }
        }
    }
}

void testDefaultsAreInRange() {
    const LocalizerConfig defaults;
    for (const LocalizerSetting& setting : localizerSettings()) {
        double value = 0.0;
        assert(getLocalizerSetting(defaults, setting.key, value));
        if (value < setting.min || value > setting.max) {
            std::printf("FAIL default %s = %g outside [%g, %g]\n", setting.key, value, setting.min,
                        setting.max);
            assert(false);
        }
    }
}

void testRefusals() {
    LocalizerConfig config;
    assert(setLocalizerSetting(config, "filter.warp", 1) == SettingError::unknownKey);
    assert(setLocalizerSetting(config, "filter.particleCount", 300.5) == SettingError::notWhole);
    assert(setLocalizerSetting(config, "filter.particleCount", 5) == SettingError::outOfRange);
    assert(setLocalizerSetting(config, "sensorLatencyMs", NAN) == SettingError::notFinite);
    assert(setLocalizerSetting(config, "waitForSetPose", 2) == SettingError::outOfRange);
    // refused values leave the config alone
    assert(config.filter.particleCount == LocalizerConfig{}.filter.particleCount);
    double unused = 0.0;
    assert(!getLocalizerSetting(config, "nothing", unused));
    assert(findLocalizerSetting("agreementSigmas") != nullptr);
    assert(findLocalizerSetting("agreement") == nullptr);
}

void testSwitches() {
    LocalizerConfig config;
    assert(setLocalizerSetting(config, "correctOdometry", 0) == SettingError::none);
    assert(!config.correctOdometry);
    assert(setLocalizerSetting(config, "filter.recovery.enabled", 0) == SettingError::none);
    assert(!config.filter.recovery.enabled);
    assert(setLocalizerSetting(config, "minAgreeingSensors", 3) == SettingError::none);
    assert(config.minAgreeingSensors == 3);
}

} // namespace

int main() {
    testEveryKeyRoundTrips();
    testDefaultsAreInRange();
    testRefusals();
    testSwitches();
    std::printf("localizer_config_test: all tests passed (%zu settings)\n",
                localizerSettings().size());
    return 0;
}
