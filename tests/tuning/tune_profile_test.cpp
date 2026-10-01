// Host-side unit test for sapphirelib::tuning's TUNE.CFG parser, no PROS/embedded dependencies,
// so it builds and runs with a normal desktop compiler. Run it from the repository root: it reads
// tests/tuning/tune_profile_golden.cfg, which tools/analyzer/test/tunefile.test.js parses too.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/tuning/tune_profile_test.cpp src/sapphirelib/tuning/tune_profile.cpp src/sapphirelib/localization/localizer_config.cpp src/sapphirelib/control/feedforward.cpp -o tune_profile_test && ./tune_profile_test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "sapphirelib/tuning/tune_profile.hpp"

using namespace sapphirelib::tuning;
using sapphirelib::localization::LocalizerConfig;

namespace {

void expectNear(double actual, double expected, const char* label, double tolerance = 1e-12) {
    if (!(std::fabs(actual - expected) < tolerance)) {
        std::printf("FAIL %s: got %.12f, expected %.12f\n", label, actual, expected);
        assert(false);
    }
}

void expectError(const char* text, std::size_t line, const char* error) {
    const TuneParseResult result = parseTuneProfile(text, TuneSchema{.pids = {"turn"}});
    if (result.ok || result.errorLine != line || result.error != error) {
        std::printf("FAIL for:\n%s\n  got ok=%d line %zu \"%s\"\n  expected line %zu \"%s\"\n",
                    text, result.ok ? 1 : 0, result.errorLine, result.error.c_str(), line, error);
        assert(false);
    }
}

// the robot program's own schema, as src/robot/tune.cpp declares it
TuneSchema robotSchema() {
    return TuneSchema{
        .pids = {"drive", "turn", "hold", "lift"},
        .models = {"fwd", "strafe", "turn"},
        .values = {{.key = "lift.gravityVolts", .min = -12, .max = 12},
                   {.key = "mcl.periodMs", .min = 10, .max = 1000, .whole = true}},
    };
}

std::string readGolden() {
    std::FILE* file = std::fopen("tests/tuning/tune_profile_golden.cfg", "rb");
    if (file == nullptr) {
        std::printf("FAIL: run from the repository root, tests/tuning/tune_profile_golden.cfg\n");
        assert(false);
    }
    std::string text;
    char buffer[1024];
    std::size_t read = 0;
    while ((read = std::fread(buffer, 1, sizeof(buffer), file)) > 0) text.append(buffer, read);
    std::fclose(file);
    return text;
}

void testGoldenFile() {
    const TuneParseResult result = parseTuneProfile(readGolden(), robotSchema());
    if (!result.ok) std::printf("golden: line %zu: %s\n", result.errorLine, result.error.c_str());
    assert(result.ok);
    const TuneProfile& p = result.profile;
    assert(p.revision == 7);
    assert(p.note == "turn refit from SL000041-SL000048; MCL from the sim tuner");
    assert(p.settingCount() == 11);

    assert(p.pid("drive") != nullptr);
    expectNear(p.pid("drive")->kP, 18.2, "drive kP");
    expectNear(p.pid("drive")->kD, 0.71, "drive kD");
    expectNear(p.pid("turn")->kP, 0.42, "turn kP");
    expectNear(p.pid("turn")->kI, 0.0, "turn kI");
    expectNear(p.pid("turn")->kD, 0.031, "turn kD");
    assert(p.pid("hold") == nullptr);

    expectNear(p.model("fwd")->kS, 1.02, "fwd kS");
    expectNear(p.model("fwd")->kV, 0.198, "fwd kV");
    expectNear(p.model("fwd")->kA, 0.047, "fwd kA");
    expectNear(p.model("turn")->kA, 0.0066, "turn kA");
    assert(p.model("strafe") == nullptr);

    expectNear(p.value("lift.gravityVolts", 0.0), 1.35, "lift gravity");
    expectNear(p.value("mcl.periodMs", 0.0), 50.0, "period");
    expectNear(p.value("nothing", -3.0), -3.0, "fallback");

    // only the mcl.* lines that are LocalizerConfig fields; the rest stay at the code's values
    const LocalizerConfig config = p.applyTo(LocalizerConfig{});
    assert(config.filter.particleCount == 400);
    expectNear(config.filter.motionNoise.perInch, 0.07, "perInch");
    expectNear(config.filter.beam.outlierProbability, 0.15, "outliers");
    assert(config.filter.recovery.enabled);
    expectNear(config.sensorLatencyMs, 42.5, "latency");
    expectNear(config.maxCorrectionRateInPerS, LocalizerConfig{}.maxCorrectionRateInPerS, "rate");
    assert(p.mcl.size() == 5);
}

void testWindowsLineEndingsAndComments() {
    const TuneParseResult result = parseTuneProfile(
        "\r\n# hand edited\r\nformat=1\r\n  pid.turn = 1 , 0 , 0.5   # comment\r\n", robotSchema());
    assert(result.ok);
    expectNear(result.profile.pid("turn")->kD, 0.5, "crlf kD");
    assert(result.profile.revision == 0);
}

void testFileMustStartWithFormat() {
    expectError("rev=1\nformat=1\n", 1, "the first setting must be format=1");
    expectError("format=2\n", 1, "format 2 is newer than this program reads");
    expectError("format=one\n", 1, "format: needs a version number");
    expectError("# nothing but comments\n\n", 0, "no format=1 line");
    expectError("", 0, "no format=1 line");
}

void testBadLinesRejectTheWholeFile() {
    expectError("format=1\npid.turn=1,0,0\nbogus\n", 3, "expected key=value");
    expectError("format=1\n=3\n", 2, "missing key");
    expectError("format=1\npid.turn=1,0,0\npid.turn=2,0,0\n", 3, "pid.turn: set twice");
    expectError("format=1\nformat=1\n", 2, "format: set twice");
    expectError("format=1\npid.drive=1,0,0\n", 2, "pid.drive: no such controller");
    expectError("format=1\npid.turn=1,0\n", 2, "pid.turn: needs kP,kI,kD");
    expectError("format=1\npid.turn=1,0,0,4\n", 2, "pid.turn: needs kP,kI,kD");
    expectError("format=1\npid.turn=1,,0\n", 2, "pid.turn: needs kP,kI,kD");
    expectError("format=1\npid.turn=1,nan,0\n", 2, "pid.turn: needs kP,kI,kD");
    expectError("format=1\npid.turn=1,0,-0.1\n", 2, "pid.turn: gains can't be negative");
    expectError("format=1\nmodel.fwd=1,0.2,0.04\n", 2, "model.fwd: no such axis");
    expectError("format=1\nrev=1.5\n", 2, "rev: needs a whole number");
    expectError("format=1\nspeed=11\n", 2, "speed: unknown setting");
}

void testModelsMustBeMeasured() {
    const TuneSchema schema{.models = {"fwd"}};
    assert(parseTuneProfile("format=1\nmodel.fwd=1,0.2,0.04\n", schema).ok);
    const TuneParseResult zero = parseTuneProfile("format=1\nmodel.fwd=1,0,0.04\n", schema);
    assert(!zero.ok && zero.errorLine == 2);
    assert(zero.error == "model.fwd: kS can't be negative, kV and kA must be positive");
    assert(!parseTuneProfile("format=1\nmodel.fwd=-1,0.2,0.04\n", schema).ok);
}

void testMclSettingsAreRangeChecked() {
    expectError("format=1\nmcl.filter.particleCount=2\n", 2,
                "mcl.filter.particleCount: out of range");
    expectError("format=1\nmcl.filter.particleCount=300.5\n", 2,
                "mcl.filter.particleCount: must be a whole number");
    expectError("format=1\nmcl.filter.warp=1\n", 2, "mcl.filter.warp: unknown setting");
    expectError("format=1\nmcl.sensorLatencyMs=fast\n", 2, "mcl.sensorLatencyMs: needs a number");
    expectError("format=1\nmcl.correctOdometry=2\n", 2, "mcl.correctOdometry: out of range");
}

void testSchemaValues() {
    const TuneSchema schema = robotSchema();
    assert(!parseTuneProfile("format=1\nlift.gravityVolts=13\n", schema).ok);
    const TuneParseResult period = parseTuneProfile("format=1\nmcl.periodMs=50.5\n", schema);
    assert(!period.ok && period.error == "mcl.periodMs: must be a whole number");
}

void testOversizedFile() {
    std::string text = "format=1\n";
    text.append(kMaxTuneFileBytes, '#');
    const TuneParseResult result = parseTuneProfile(text, robotSchema());
    assert(!result.ok && result.errorLine == 0 && result.error == "file is too big");
}

} // namespace

int main() {
    testGoldenFile();
    testWindowsLineEndingsAndComments();
    testFileMustStartWithFormat();
    testBadLinesRejectTheWholeFile();
    testModelsMustBeMeasured();
    testMclSettingsAreRangeChecked();
    testSchemaValues();
    testOversizedFile();
    std::printf("tune_profile_test: all tests passed\n");
    return 0;
}
