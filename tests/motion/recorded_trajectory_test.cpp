// Host-side unit test for sapphirelib::motion::trajectoryFromRecording() and
// the recording file format, no PROS/embedded dependencies, so it builds and
// runs with a normal desktop compiler.
//
// A "driver" is a planned trajectory sampled the way teach mode records
// (every 20ms, odometry's resolution, a wait before and after), driven harder
// than the replay's budget. The replay is then followed in closed loop on a
// simulated chassis (each axis kS/kV/kA with a command delay), as in
// holonomic_tracking_test.cpp.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/recorded_trajectory_test.cpp src/sapphirelib/motion/recorded_trajectory.cpp src/sapphirelib/motion/holonomic_tracking.cpp src/sapphirelib/motion/holonomic_feedforward.cpp src/sapphirelib/motion/trajectory_generator.cpp src/sapphirelib/motion/trajectory.cpp src/sapphirelib/motion/trajectory_path.cpp src/sapphirelib/motion/pure_pursuit_math.cpp src/sapphirelib/motion/path.cpp src/sapphirelib/motion/exit_tracker.cpp src/sapphirelib/input/button_playback.cpp src/sapphirelib/control/pid.cpp src/sapphirelib/control/feedforward.cpp src/sapphirelib/tuning/gain_design.cpp src/sapphirelib/util/angle.cpp -o recorded_trajectory_test && ./recorded_trajectory_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "sapphirelib/input/button_playback.hpp"
#include "sapphirelib/motion/holonomic_feedforward.hpp"
#include "sapphirelib/motion/holonomic_tracking.hpp"
#include "sapphirelib/motion/recorded_trajectory.hpp"
#include "sapphirelib/motion/trajectory_generator.hpp"
#include "sapphirelib/tuning/gain_design.hpp"

using sapphirelib::MotorFeedforward;
using sapphirelib::PID;
using sapphirelib::chassis::AxisVolts;
using sapphirelib::chassis::HolonomicAxisModels;
using sapphirelib::input::ButtonPlayback;
using sapphirelib::motion::ExitReason;
using sapphirelib::motion::formatRecordedSample;
using sapphirelib::motion::generateTrajectory;
using sapphirelib::motion::kRecordingHeader;
using sapphirelib::motion::parseRecording;
using sapphirelib::motion::RecordedSample;
using sapphirelib::motion::ReplayPlan;
using sapphirelib::motion::TrackingTick;
using sapphirelib::motion::Trajectory;
using sapphirelib::motion::trajectoryFeedforward;
using sapphirelib::motion::TrajectoryFollower;
using sapphirelib::motion::trajectoryFromRecording;
using sapphirelib::motion::TrajectorySample;
using sapphirelib::odom::Pose;
using sapphirelib::odom::Velocity;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// the simulator's default chassis (tools/sim/js/world.js)
const HolonomicAxisModels kModels{
    .forward = {.kS = 1.0, .kV = 0.2, .kA = 0.045},
    .strafe = {.kS = 1.3, .kV = 0.24, .kA = 0.055},
    .turn = {.kS = 0.6, .kV = 0.045, .kA = 0.0065},
};

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.6f, expected %.6f (+-%g)\n", label, actual, expected,
                    tolerance);
        assert(false);
    }
}

double wheelVolts(const TrajectorySample& s) {
    const AxisVolts v = trajectoryFeedforward(kModels, s);
    return std::fabs(v.forward) + std::fabs(v.strafe) + std::fabs(v.turn);
}

// what teach mode records of a drive: the pose every 20ms at odometry's resolution, after
// waiting leadS before moving, then sitting still for tailS
std::vector<RecordedSample> record(const Trajectory& drive, double leadS, double tailS,
                                   double headingOffsetDeg = 0.0) {
    std::vector<RecordedSample> out;
    const double endS = leadS + drive.durationS() + tailS;
    for (std::uint32_t tMs = 0; tMs <= endS * 1000.0; tMs += 20) {
        const TrajectorySample s = drive.sample(tMs / 1000.0 - leadS);
        const double heading = std::fmod(s.headingDeg + headingOffsetDeg + 3600.0, 360.0);
        out.push_back({.tMs = tMs,
                       .xIn = std::round(s.xIn / 0.002) * 0.002,
                       .yIn = std::round(s.yIn / 0.002) * 0.002,
                       .headingDeg = std::round(heading / 0.01) * 0.01});
    }
    return out;
}

// a driver going harder than the replay's 10V: planned on 11.5V
Trajectory hardDrive() {
    const auto result = generateTrajectory(
        {{.waypoints = {{.xIn = 0, .yIn = 0, .headingDeg = 0},
                        {.xIn = 10, .yIn = 30},
                        {.xIn = 40, .yIn = 40, .headingDeg = 90}},
          .constraints = {.voltageBudget = 11.5}}},
        kModels);
    assert(result.ok);
    return result.trajectory;
}

double distanceToPath(const Trajectory& path, double x, double y) {
    double best = 1e9;
    const auto& s = path.samples();
    for (std::size_t k = 0; k + 1 < s.size(); ++k) {
        const double dx = s[k + 1].xIn - s[k].xIn;
        const double dy = s[k + 1].yIn - s[k].yIn;
        const double lengthSq = dx * dx + dy * dy;
        double f = lengthSq > 0.0 ? ((x - s[k].xIn) * dx + (y - s[k].yIn) * dy) / lengthSq : 0.0;
        f = std::clamp(f, 0.0, 1.0);
        best = std::fmin(best, std::hypot(s[k].xIn + f * dx - x, s[k].yIn + f * dy - y));
    }
    return best;
}

// --- Closed loop: the chassis from holonomic_tracking_test.cpp --------------------------------

double signOf(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }

struct Chassis {
    Pose pose;
    double v[3] = {0.0, 0.0, 0.0};
    std::deque<AxisVolts> pending;
    int delayTicks = 2;

    Velocity fieldVelocity() const {
        const double h = pose.headingDeg * kDegToRad;
        return Velocity{.xInPerS = v[0] * std::sin(h) + v[1] * std::cos(h),
                        .yInPerS = v[0] * std::cos(h) - v[1] * std::sin(h),
                        .headingDegPerS = v[2]};
    }

    void step(AxisVolts command, double dtS) {
        const double corners[4] = {command.forward + command.strafe + command.turn,
                                   command.forward - command.strafe - command.turn,
                                   command.forward - command.strafe + command.turn,
                                   command.forward + command.strafe - command.turn};
        double largest = 12.0;
        for (double c : corners) largest = std::fmax(largest, std::fabs(c));
        const double scale = 12.0 / largest;
        pending.push_back({command.forward * scale, command.strafe * scale, command.turn * scale});
        AxisVolts applied{};
        if (static_cast<int>(pending.size()) > delayTicks) {
            applied = pending.front();
            pending.pop_front();
        }
        const double volts[3] = {applied.forward, applied.strafe, applied.turn};
        const MotorFeedforward* models[3] = {&kModels.forward, &kModels.strafe, &kModels.turn};
        const int substeps = 20;
        const double h = dtS / substeps;
        for (int k = 0; k < substeps; ++k) {
            for (int i = 0; i < 3; ++i) {
                const MotorFeedforward& m = *models[i];
                const double friction =
                    std::fabs(v[i]) > 1e-3
                        ? m.kS * signOf(v[i])
                        : signOf(volts[i]) * std::fmin(std::fabs(volts[i]), m.kS);
                v[i] += (volts[i] - friction - m.kV * v[i]) / m.kA * h;
            }
            const Velocity field = fieldVelocity();
            pose.xIn += field.xInPerS * h;
            pose.yIn += field.yInPerS * h;
            pose.headingDeg += field.headingDegPerS * h;
        }
    }
};

struct Run {
    ExitReason reason = ExitReason::running;
    double worstErrorIn = 0.0;
    double worstHeadingDeg = 0.0;
    double finalErrorIn = 0.0;
    Pose end;
};

Run follow(const ReplayPlan& plan) {
    const sapphirelib::tuning::ResponseSpec spec{.settleTimeS = 0.4};
    PID forward({.gains = sapphirelib::tuning::designPositionGains(kModels.forward, spec, 0.02).gains,
                 .outputLimit = 12.0});
    PID strafe({.gains = sapphirelib::tuning::designPositionGains(kModels.strafe, spec, 0.02).gains,
                .outputLimit = 12.0});
    PID turn({.gains = sapphirelib::tuning::designPositionGains(kModels.turn, spec, 0.02).gains,
              .outputLimit = 12.0});
    TrajectoryFollower follower(forward, strafe, turn);
    Chassis chassis;
    chassis.pose = plan.start;
    Run run;
    std::uint32_t now = 1000;
    follower.begin(plan.trajectory, kModels, {}, now);
    while (true) {
        const TrackingTick tick = follower.update(chassis.pose, chassis.fieldVelocity(), now);
        for (const double value : {tick.volts.forward, tick.volts.strafe, tick.volts.turn}) {
            assert(std::isfinite(value));
        }
        if (!tick.holding) {
            run.worstErrorIn = std::fmax(run.worstErrorIn,
                                         std::hypot(tick.errors.forwardIn, tick.errors.strafeIn));
            run.worstHeadingDeg = std::fmax(run.worstHeadingDeg, std::fabs(tick.errors.headingDeg));
        }
        if (follower.reason() != ExitReason::running) break;
        chassis.step(tick.volts, 0.01);
        now += 10;
    }
    run.reason = follower.reason();
    run.finalErrorIn = follower.result().finalError;
    run.end = chassis.pose;
    return run;
}

// --- The cases --------------------------------------------------------------------------------

void testReplaysADriveWithinTheBudget() {
    const Trajectory drive = hardDrive();
    std::vector<RecordedSample> recording = record(drive, 1.0, 0.6);
    // R1 pressed at 1.5s and let go at 2.0s, into the drive
    for (RecordedSample& s : recording) {
        if (s.tMs >= 1500 && s.tMs < 2000) s.buttons = 0x4;
    }
    const ReplayPlan plan = trajectoryFromRecording(recording, kModels);
    if (!plan.ok) std::printf("FAIL replay: %s\n", plan.error.c_str());
    assert(plan.ok && plan.error.empty());

    // slowed to fit 10V, evenly, but not by much
    std::printf("replay: %.3fs at %.0f%% of the driver's %.3fs drive\n",
                plan.trajectory.durationS(), plan.speedScale * 100.0, drive.durationS());
    assert(plan.speedScale < 1.0 && plan.speedScale > 0.75);
    double worst = 0.0;
    for (const TrajectorySample& s : plan.trajectory.samples()) worst = std::fmax(worst, wheelVolts(s));
    assert(worst <= 10.0 + 1e-6);
    // the second of waiting before the drive is trimmed to the smoothing's reach (0.2s), and the
    // tail to 0.3s
    expectNear(plan.trajectory.durationS(), (drive.durationS() + 0.2 + 0.3) / plan.speedScale,
               0.05, "duration: the drive, slowed, plus a moment either side");
    // where it starts, still, and where it ends, still
    expectNear(plan.start.xIn, 0.0, 0.01, "start x");
    expectNear(plan.start.yIn, 0.0, 0.01, "start y");
    expectNear(plan.start.headingDeg, 0.0, 0.05, "start heading");
    const TrajectorySample& first = plan.trajectory.samples().front();
    const TrajectorySample& last = plan.trajectory.samples().back();
    assert(first.stop && last.stop);
    expectNear(first.speedInPerS, 0.0, 0.05, "starts at rest");
    expectNear(last.speedInPerS, 0.0, 0.05, "ends at rest");
    expectNear(last.xIn, 40.0, 0.05, "ends at the drive's end x");
    expectNear(last.yIn, 40.0, 0.05, "ends at the drive's end y");
    expectNear(last.headingDeg, 90.0, 0.1, "ends at the drive's end heading");
    // the path is the driver's, give or take what the smoothing rounds off
    double furthest = 0.0;
    for (const TrajectorySample& s : plan.trajectory.samples()) {
        furthest = std::fmax(furthest, distanceToPath(drive, s.xIn, s.yIn));
        for (const double value : {s.forwardPerIn, s.strafePerIn, s.turnDegPerIn, s.forwardInPerS,
                                   s.forwardAccelInPerS2, s.turnAccelDegPerS2}) {
            assert(std::isfinite(value));
        }
        assert(std::hypot(s.forwardPerIn, s.strafePerIn) <= 1.0 + 1e-9);
    }
    assert(furthest < 0.3);

    // the press lands on the replay's clock at the same point of the drive
    assert(plan.buttons.size() == 3);
    assert(plan.buttons[0].buttons == 0 && plan.buttons[0].tS == 0.0);
    // where the replay starts in the recording: the smoothing's reach before the last still sample
    std::size_t firstMoved = 0;
    while (std::hypot(recording[firstMoved].xIn, recording[firstMoved].yIn) <= 0.05) ++firstMoved;
    const double startS = recording[firstMoved - 1].tMs / 1000.0 - 0.2;
    assert(startS > 0.75 && startS < 0.85);
    expectNear(plan.buttons[1].tS, (1.5 - startS) / plan.speedScale, 1e-9, "press time");
    assert(plan.buttons[1].buttons == 0x4);
    expectNear(plan.buttons[2].tS, (2.0 - startS) / plan.speedScale, 1e-9, "release time");
    ButtonPlayback playback(plan.buttons);
    assert(playback.advance(plan.buttons[1].tS - 0.01) == 0);
    assert(playback.advance(plan.buttons[1].tS) == 0x4);

    // driven: it tracks the replay as it tracks any plan, and lands on the drive's end
    const Run run = follow(plan);
    std::printf("replayed: %s, worst %.3fin %.2fdeg, final %.3fin\n",
                sapphirelib::motion::exitReasonName(run.reason), run.worstErrorIn,
                run.worstHeadingDeg, run.finalErrorIn);
    assert(run.reason == ExitReason::settled);
    assert(run.worstErrorIn < 1.0 && run.worstHeadingDeg < 3.0 && run.finalErrorIn <= 1.0);

    // a cap on the speed: never faster than asked, even with budget to spare
    const ReplayPlan capped =
        trajectoryFromRecording(recording, kModels, {.voltageBudget = 12.0, .maxSpeedScale = 0.5});
    assert(capped.ok);
    expectNear(capped.speedScale, 0.5, 0.0, "capped at half speed");
}

void testTurnsInPlaceAndThroughZero() {
    // a turn in place from 300 to 60 degrees, the short way, through 0: something a path can't
    // plan, since it only turns as it drives
    std::vector<RecordedSample> recording;
    for (std::uint32_t tMs = 0; tMs <= 3000; tMs += 20) {
        const double u = std::clamp((tMs - 500.0) / 1500.0, 0.0, 1.0);
        const double eased = u * u * (3.0 - 2.0 * u);
        recording.push_back({.tMs = tMs,
                             .xIn = 12.0,
                             .yIn = -6.0,
                             .headingDeg = std::fmod(300.0 + 120.0 * eased, 360.0)});
    }
    const ReplayPlan plan = trajectoryFromRecording(recording, kModels);
    if (!plan.ok) std::printf("FAIL turn: %s\n", plan.error.c_str());
    assert(plan.ok);
    expectNear(plan.start.headingDeg, 300.0, 0.05, "starts at 300");
    double previous = plan.trajectory.samples().front().headingDeg;
    double fastestTurn = 0.0;
    for (const TrajectorySample& s : plan.trajectory.samples()) {
        assert(std::fabs(s.headingDeg - previous) < 2.0); // no jump at 0
        previous = s.headingDeg;
        expectNear(s.xIn, 12.0, 1e-9, "doesn't move x");
        expectNear(s.speedInPerS, 0.0, 1e-9, "doesn't move");
        if (std::fabs(s.omegaDegPerS) > 30.0) {
            // friction all goes to turning
            expectNear(std::hypot(s.forwardPerIn, s.strafePerIn), 0.0, 1e-9, "turning, not driving");
            assert(s.turnDegPerIn > 0.0);
        }
        fastestTurn = std::fmax(fastestTurn, s.omegaDegPerS);
    }
    expectNear(plan.trajectory.samples().back().headingDeg, 420.0, 0.1, "ends at 60, as 420");
    assert(fastestTurn > 80.0);

    const Run run = follow(plan);
    assert(run.reason == ExitReason::settled);
    expectNear(std::fmod(run.end.headingDeg + 360.0, 360.0), 60.0, 2.0, "turned to 60");
    expectNear(std::hypot(run.end.xIn - 12.0, run.end.yIn + 6.0), 0.0, 1.0, "stayed put");
}

void testRejections() {
    const std::vector<RecordedSample> still{{.tMs = 0}, {.tMs = 20}, {.tMs = 40}};
    ReplayPlan plan = trajectoryFromRecording(still, kModels);
    assert(!plan.ok && plan.error == "the robot never moved and no button was pressed");
    // pressing a button is something to replay, even without moving
    plan = trajectoryFromRecording({{.tMs = 0}, {.tMs = 20, .buttons = 1}, {.tMs = 40}}, kModels);
    assert(plan.ok && plan.buttons.size() == 3);

    plan = trajectoryFromRecording({{.tMs = 0}}, kModels);
    assert(!plan.ok && plan.error == "the recording is too short");
    plan = trajectoryFromRecording({{.tMs = 0}, {.tMs = 20, .xIn = 5}, {.tMs = 20, .xIn = 6}},
                                   kModels);
    assert(!plan.ok && plan.error == "the recording's times don't increase at sample 2");
    HolonomicAxisModels unmeasured = kModels;
    unmeasured.turn = {};
    plan = trajectoryFromRecording(record(hardDrive(), 0.2, 0.2), unmeasured);
    assert(!plan.ok &&
           plan.error == "the Turn model isn't measured (kV and kA must be above 0): run Auto-Tune");
}

void testTheFileFormatRoundTrips() {
    const std::vector<RecordedSample> recording{
        {.tMs = 0, .xIn = -48.0, .yIn = -60.0, .headingDeg = 359.99, .buttons = 0},
        {.tMs = 20, .xIn = -47.9876, .yIn = -59.5, .headingDeg = 0.25, .buttons = 0x804},
    };
    std::string text(kRecordingHeader);
    for (const RecordedSample& s : recording) {
        char row[96];
        const int length = formatRecordedSample(row, sizeof(row), s);
        assert(length > 0 && length < static_cast<int>(sizeof(row)));
        text += row;
    }
    assert(text.rfind("# SapphireLib recorded drive v1\nt_ms,x,y,heading,buttons\n0,-48.000,", 0) ==
           0);
    std::vector<RecordedSample> read;
    std::string error;
    assert(parseRecording(text, read, error));
    assert(read.size() == 2 && read[1].tMs == 20 && read[1].buttons == 0x804);
    expectNear(read[1].xIn, -47.988, 1e-9, "x to a thousandth");
    expectNear(read[0].headingDeg, 359.99, 1e-9, "heading");

    // Windows line ends, a comment and a blank line are fine
    std::string crlf;
    for (char c : text) {
        if (c == '\n') crlf += '\r';
        crlf += c;
    }
    crlf += "\r\n# a note\r\n\r\n40,1,2,3,4\r\n";
    assert(parseRecording(crlf, read, error) && read.size() == 3 && read[2].buttons == 4);

    assert(!parseRecording("hello\n", read, error));
    assert(error.rfind("not a recorded drive", 0) == 0);
    assert(!parseRecording(std::string(kRecordingHeader) + "0,1,2,3,4\n20,1,2,x,4\n", read, error));
    assert(error == "line 4: expected t_ms,x,y,heading,buttons, got \"20,1,2,x,4\"");
    assert(!parseRecording(std::string(kRecordingHeader) + "0,1,2,3\n", read, error));
    assert(error.rfind("line 3:", 0) == 0);
    assert(!parseRecording(std::string(kRecordingHeader), read, error));
    assert(error == "the recording has no samples");
    assert(!parseRecording("", read, error) && error == "the file is empty");
}

} // namespace

int main() {
    testReplaysADriveWithinTheBudget();
    testTurnsInPlaceAndThroughZero();
    testRejections();
    testTheFileFormatRoundTrips();
    std::puts("recorded_trajectory_test: all assertions passed");
    return 0;
}
