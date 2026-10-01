/**
 * \file tune.cpp
 *
 * Loads TUNE.CFG from the SD card and applies it: see robot/tune.hpp.
 *
 * Team 96671H: Hitmen
 */

#include "robot/tune.hpp"

#include <cstdio>
#include <string>

#include "pros/misc.hpp"
#include "robot/devices.hpp"
#include "robot/macros.hpp"
#include "sapphirelib/util/log.hpp"

namespace robot {

using sapphirelib::MotorFeedforward;
using sapphirelib::PIDGains;
using sapphirelib::chassis::HolonomicAxisModels;
using sapphirelib::localization::LocalizerConfig;
using sapphirelib::localization::LocalizerSetting;
using sapphirelib::tuning::kMaxTuneFileBytes;
using sapphirelib::tuning::parseTuneProfile;
using sapphirelib::tuning::TuneProfile;
using sapphirelib::tuning::TuneSchema;

namespace {

constexpr std::uint32_t kDefaultLocalizerPeriodMs = 50;

// Everything this robot's TUNE.CFG may set besides mcl.*: the PID page's four
// controllers (by their telemetry channel names), Auto-Tune's three drive
// axes, the lift's gravity volts, and the localizer's update period. The
// analyzer writes exactly these keys (tools/analyzer/js/tunefile.js).
TuneSchema schema() {
    return TuneSchema{
        .pids = {"drive", "turn", "hold", "lift"},
        .models = {"fwd", "strafe", "turn"},
        .values = {{.key = "lift.gravityVolts", .min = -12.0, .max = 12.0},
                   {.key = "mcl.periodMs", .min = 10.0, .max = 1000.0, .whole = true}},
    };
}

// The whole file, or false if there's none (or it's too big to be ours).
bool readFile(const char* path, std::string& text) {
    std::FILE* file = std::fopen(path, "r");
    if (file == nullptr) return false;
    text.clear();
    char buffer[512];
    std::size_t read = 0;
    while ((read = std::fread(buffer, 1, sizeof(buffer), file)) > 0) {
        text.append(buffer, read);
        // one byte past the limit is enough for the parser to say "too big"
        if (text.size() > kMaxTuneFileBytes) break;
    }
    std::fclose(file);
    return true;
}

TuneStatus load() {
    TuneStatus status;
    if (pros::usd::is_installed() == 0) {
        status.state = TuneStatus::State::noCard;
        SAPPHIRELIB_LOG_INFO("tune", "no SD card: running the code's tuning");
        return status;
    }
    std::string text;
    for (const char* path : kTuneFilePaths) {
        if (!readFile(path, text)) continue;
        status.path = path;
        status.result = parseTuneProfile(text, schema());
        if (status.result.ok) {
            status.state = TuneStatus::State::loaded;
            SAPPHIRELIB_LOG_INFO("tune", "%s r%d: %u settings", path,
                                 status.result.profile.revision,
                                 static_cast<unsigned>(status.result.profile.settingCount()));
        } else {
            status.state = TuneStatus::State::rejected;
            SAPPHIRELIB_LOG_ERROR("tune", "%s rejected, line %u: %s; running the code's tuning",
                                  path, static_cast<unsigned>(status.result.errorLine),
                                  status.result.error.c_str());
        }
        return status;
    }
    status.state = TuneStatus::State::none;
    SAPPHIRELIB_LOG_INFO("tune", "no TUNE.CFG: running the code's tuning");
    return status;
}

// The profile to apply: the file's when it loaded, nothing otherwise.
const TuneProfile& profile() {
    static const TuneProfile empty;
    const TuneStatus& status = tuneStatus();
    return status.state == TuneStatus::State::loaded ? status.result.profile : empty;
}

std::string number(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.9g", value);
    return text;
}

} // namespace

const TuneStatus& tuneStatus() {
    static const TuneStatus status = load();
    return status;
}

LocalizerConfig localizerSettings() { return profile().applyTo(LocalizerConfig{}); }

std::uint32_t localizerPeriodMs() {
    return static_cast<std::uint32_t>(profile().value("mcl.periodMs", kDefaultLocalizerPeriodMs));
}

void applyTuneProfile() {
    const TuneProfile& tuning = profile();
    if (tuning.settingCount() == 0) return;

    // By the names the telemetry channels (and the analyzer) use for them.
    const auto setGains = [&tuning](const char* name, sapphirelib::PID& pid) {
        if (const PIDGains* gains = tuning.pid(name)) {
            pid.setGains(*gains);
            SAPPHIRELIB_LOG_INFO("tune", "%s gains kP=%g kI=%g kD=%g", name, gains->kP, gains->kI,
                                 gains->kD);
        }
    };
    setGains("drive", drivetrain().drivePID());
    setGains("turn", drivetrain().turnPID());
    setGains("hold", drivetrain().headingHoldPID());
    setGains("lift", macros::liftMechanism().pid());

    // Axis models for DriverInputMode::velocity, as Auto-Tune would install
    // them (tuning.cpp's installModel()).
    HolonomicAxisModels models = drivetrain().axisModels();
    bool modelsChanged = false;
    const auto setModel = [&](const char* name, MotorFeedforward HolonomicAxisModels::* axis) {
        if (const MotorFeedforward* model = tuning.model(name)) {
            models.*axis = *model;
            modelsChanged = true;
        }
    };
    setModel("fwd", &HolonomicAxisModels::forward);
    setModel("strafe", &HolonomicAxisModels::strafe);
    setModel("turn", &HolonomicAxisModels::turn);
    if (modelsChanged) drivetrain().setAxisModels(models);

    // Only the constant term: the lift is an elevator (see macros.cpp).
    sapphirelib::mechanism::GravityFeedforward gravity = macros::liftMechanism().gravity();
    gravity.constantVolts = tuning.value("lift.gravityVolts", gravity.constantVolts);
    macros::liftMechanism().setGravity(gravity);
}

std::string tuneStatusText() {
    const TuneStatus& status = tuneStatus();
    char text[64];
    switch (status.state) {
        case TuneStatus::State::loaded:
            std::snprintf(text, sizeof(text), "Tune: TUNE.CFG r%d, %u settings",
                          status.result.profile.revision,
                          static_cast<unsigned>(status.result.profile.settingCount()));
            return text;
        case TuneStatus::State::rejected:
            // the line to fix; the reason is on the terminal and in every log
            std::snprintf(text, sizeof(text), "Tune: TUNE.CFG REJECTED, line %u",
                          static_cast<unsigned>(status.result.errorLine));
            return text;
        case TuneStatus::State::noCard: return "Tune: code values (no card)";
        default: return "Tune: code values (no TUNE.CFG)";
    }
}

void describeTuning(sapphirelib::telemetry::Logger& logger) {
    const TuneStatus& status = tuneStatus();
    switch (status.state) {
        case TuneStatus::State::loaded: {
            logger.addMeta("tune", "loaded");
            logger.addMeta("tune.rev", std::to_string(status.result.profile.revision).c_str());
            logger.addMeta("tune.path", status.path.c_str());
            if (!status.result.profile.note.empty()) {
                logger.addMeta("tune.note", status.result.profile.note.c_str());
            }
            break;
        }
        case TuneStatus::State::rejected: {
            logger.addMeta("tune", "rejected");
            logger.addMeta("tune.path", status.path.c_str());
            const std::string error =
                "line " + std::to_string(status.result.errorLine) + ": " + status.result.error;
            logger.addMeta("tune.error", error.c_str());
            break;
        }
        case TuneStatus::State::noCard: logger.addMeta("tune", "no_card"); break;
        default: logger.addMeta("tune", "none"); break;
    }

    // The localizer's settings in full, so a log can be tuned from without
    // knowing what the code's defaults were when it ran.
    const LocalizerConfig& config = localizer().config();
    for (const LocalizerSetting& setting : sapphirelib::localization::localizerSettings()) {
        double value = 0.0;
        sapphirelib::localization::getLocalizerSetting(config, setting.key, value);
        logger.addMeta((std::string("mcl.") + setting.key).c_str(), number(value).c_str());
    }
    logger.addMeta("mcl.periodMs", std::to_string(localizerPeriodMs()).c_str());
    const auto& mounts = localizer().sensorMounts();
    for (std::size_t i = 0; i < mounts.size(); ++i) {
        // forward, right, facing: the beams' geometry, for offline checks of the readings
        const std::string mount = number(mounts[i].forwardIn) + "," + number(mounts[i].rightIn) +
                                  "," + number(mounts[i].facingDeg);
        logger.addMeta(("mcl.sensor" + std::to_string(i)).c_str(), mount.c_str());
    }
}

} // namespace robot
