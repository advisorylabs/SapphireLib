#include "sapphirelib/motion/recorded_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

#include "sapphirelib/motion/holonomic_feedforward.hpp"
#include "sapphirelib/util/angle.hpp"

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// the plan's sample spacing, in seconds of the recording
constexpr double kGridS = 0.01;

// still time kept before the drive's first movement or button, and after its last, in seconds:
// at least this, and at least the smoothing's reach (3 standard deviations), so the smoothing
// doesn't move the start or the end
constexpr double kLeadS = 0.1;
constexpr double kTailS = 0.3;

// how far the robot must get from where it sat to count as moving: well past what odometry
// reads while the robot sits still (a tracking wheel's count, the IMU's drift over seconds)
constexpr double kMovedIn = 0.05;
constexpr double kMovedDeg = 0.25;

// below this speed the direction of motion is scaled down rather than blown up, in inches per
// second: a turn in place's direction is then mostly turn
constexpr double kDirectionFloorInPerS = 1.0;

// a still robot speeding up harder than this gets friction in the direction it's speeding up,
// so it breaks away on time: inches and degrees per second squared
constexpr double kStartingInPerS2 = 4.0;
constexpr double kStartingDegPerS2 = 20.0;

// halvings of the speed scale when fitting the voltage budget
constexpr int kScaleSteps = 30;

ReplayPlan failure(const std::string& error) {
    ReplayPlan plan;
    plan.error = error;
    return plan;
}

// Gaussian smoothing, the ends held: as if the robot sat where it starts and ends
std::vector<double> smooth(const std::vector<double>& values, double sigmaSamples) {
    const std::size_t n = values.size();
    if (!(sigmaSamples > 0.0) || n < 2) return values;
    const int radius = static_cast<int>(std::ceil(3.0 * sigmaSamples));
    std::vector<double> kernel(2 * radius + 1);
    double sum = 0.0;
    for (int j = -radius; j <= radius; ++j) {
        const double w = std::exp(-0.5 * (j * j) / (sigmaSamples * sigmaSamples));
        kernel[j + radius] = w;
        sum += w;
    }
    for (double& w : kernel) w /= sum;
    std::vector<double> out(n, 0.0);
    const auto last = static_cast<long>(n) - 1;
    for (long k = 0; k <= last; ++k) {
        double acc = 0.0;
        for (int j = -radius; j <= radius; ++j) {
            acc += kernel[j + radius] * values[std::clamp(k + j, 0L, last)];
        }
        out[k] = acc;
    }
    return out;
}

// central differences, one-sided at the ends
std::vector<double> derivative(const std::vector<double>& values, double dtS) {
    const std::size_t n = values.size();
    std::vector<double> out(n, 0.0);
    if (n < 2) return out;
    out.front() = (values[1] - values[0]) / dtS;
    out.back() = (values[n - 1] - values[n - 2]) / dtS;
    for (std::size_t k = 1; k + 1 < n; ++k) out[k] = (values[k + 1] - values[k - 1]) / (2.0 * dtS);
    return out;
}

// the recording at a time, in seconds, linearly between samples; held before and after it
struct Interpolated {
    double xIn;
    double yIn;
    double headingDeg;
};

Interpolated at(const std::vector<RecordedSample>& recording, const std::vector<double>& heading,
                double tS) {
    const double tMs = tS * 1000.0;
    if (!(tMs > recording.front().tMs)) {
        return {recording.front().xIn, recording.front().yIn, heading.front()};
    }
    if (tMs >= recording.back().tMs) {
        return {recording.back().xIn, recording.back().yIn, heading.back()};
    }
    const auto after =
        std::upper_bound(recording.begin(), recording.end(), tMs,
                         [](double t, const RecordedSample& sample) { return t < sample.tMs; });
    const std::size_t b = static_cast<std::size_t>(after - recording.begin());
    const std::size_t a = b - 1;
    const double f = (tMs - recording[a].tMs) / (recording[b].tMs - recording[a].tMs);
    return {recording[a].xIn + f * (recording[b].xIn - recording[a].xIn),
            recording[a].yIn + f * (recording[b].yIn - recording[a].yIn),
            heading[a] + f * (heading[b] - heading[a])};
}

bool movedFrom(const RecordedSample& sample, double headingDeg, const RecordedSample& from,
               double fromHeadingDeg) {
    return std::hypot(sample.xIn - from.xIn, sample.yIn - from.yIn) > kMovedIn ||
           std::fabs(headingDeg - fromHeadingDeg) > kMovedDeg || sample.buttons != from.buttons;
}

} // namespace

ReplayPlan trajectoryFromRecording(const std::vector<RecordedSample>& recording,
                                   const chassis::HolonomicAxisModels& models,
                                   const ReplayConfig& config) {
    const char* const names[3] = {"Fwd", "Strafe", "Turn"};
    const MotorFeedforward* const axisModels[3] = {&models.forward, &models.strafe, &models.turn};
    for (int i = 0; i < 3; ++i) {
        if (!axisModels[i]->valid()) {
            return failure(std::string("the ") + names[i] +
                           " model isn't measured (kV and kA must be above 0): run Auto-Tune");
        }
    }
    if (!(config.maxSpeedScale > 0.0)) return failure("maxSpeedScale must be above 0");
    const std::size_t n = recording.size();
    if (n < 2) return failure("the recording is too short");
    for (std::size_t k = 1; k < n; ++k) {
        if (recording[k].tMs <= recording[k - 1].tMs) {
            char text[96];
            std::snprintf(text, sizeof(text), "the recording's times don't increase at sample %u",
                          static_cast<unsigned>(k));
            return failure(text);
        }
    }

    // heading, unwrapped: a turn through 0 is a turn, not a 360 degree jump back
    std::vector<double> heading(n);
    heading[0] = recording[0].headingDeg;
    for (std::size_t k = 1; k < n; ++k) {
        heading[k] = heading[k - 1] + sapphirelib::wrapDegrees180(recording[k].headingDeg -
                                                                  recording[k - 1].headingDeg);
    }

    // the drive: from the first sample that moved or changed a button to the last, plus a
    // moment of stillness either side
    std::size_t first = n;
    for (std::size_t k = 1; k < n && first == n; ++k) {
        if (movedFrom(recording[k], heading[k], recording[0], heading[0])) first = k;
    }
    if (first == n) return failure("the robot never moved and no button was pressed");
    std::size_t last = first;
    for (std::size_t k = n - 1; k >= 1; --k) {
        if (movedFrom(recording[k - 1], heading[k - 1], recording[n - 1], heading[n - 1])) {
            last = k;
            break;
        }
    }
    // the smoothing's 3 standard deviations, and the two differences on top for the acceleration
    const double reachS = 3.0 * std::max(0.0, config.smoothingS) + 2.0 * kGridS;
    const double startS = recording[first - 1].tMs / 1000.0 - std::max(kLeadS, reachS);
    const double endS = recording[last].tMs / 1000.0 + std::max(kTailS, reachS);
    const auto count = static_cast<std::size_t>(std::lround((endS - startS) / kGridS)) + 1;

    // on an even grid, smoothed, and differentiated. The derivatives of a Gaussian-smoothed pose
    // are smooth themselves: odometry's resolution and the linear steps between 20ms samples
    // leave hundredths of an inch per second, and of that per second squared
    std::vector<double> x(count);
    std::vector<double> y(count);
    std::vector<double> h(count);
    for (std::size_t i = 0; i < count; ++i) {
        const Interpolated p = at(recording, heading, startS + i * kGridS);
        x[i] = p.xIn;
        y[i] = p.yIn;
        h[i] = p.headingDeg;
    }
    const double sigma = config.smoothingS / kGridS;
    x = smooth(x, sigma);
    y = smooth(y, sigma);
    h = smooth(h, sigma);
    const std::vector<double> vx = derivative(x, kGridS);
    const std::vector<double> vy = derivative(y, kGridS);
    const std::vector<double> w = derivative(h, kGridS);
    // the robot-frame velocity, and its rate of change: what the axis models are in
    std::vector<double> vf(count);
    std::vector<double> vs(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double heading = h[i] * kDegToRad;
        vf[i] = vx[i] * std::sin(heading) + vy[i] * std::cos(heading);
        vs[i] = vx[i] * std::cos(heading) - vy[i] * std::sin(heading);
    }
    const std::vector<double> af = derivative(vf, kGridS);
    const std::vector<double> as = derivative(vs, kGridS);
    const std::vector<double> aw = derivative(w, kGridS);
    const std::vector<double> ax = derivative(vx, kGridS);
    const std::vector<double> ay = derivative(vy, kGridS);

    // the direction friction pushes, per sample: of the velocity, or of the acceleration as a
    // still robot starts, or none while still. Its translation part is never longer than 1, as
    // trackingErrors() wants
    std::vector<AxisMotion> direction(count);
    std::vector<bool> still(count, false);
    for (std::size_t i = 0; i < count; ++i) {
        const double speed = std::hypot(vf[i], vs[i]);
        const double accel = std::hypot(af[i], as[i]);
        if (speed > config.stillInPerS || std::fabs(w[i]) > config.stillDegPerS) {
            const double d = std::max(speed, kDirectionFloorInPerS);
            direction[i] = {vf[i] / d, vs[i] / d, w[i] / d};
        } else if (accel > kStartingInPerS2 || std::fabs(aw[i]) > kStartingDegPerS2) {
            const double d = std::max(accel, kDirectionFloorInPerS);
            direction[i] = {af[i] / d, as[i] / d, aw[i] / d};
        } else {
            still[i] = true;
        }
    }

    // the speed scale: as fast as the driver, at most maxSpeedScale, slowed evenly until every
    // sample's worst wheel fits the budget. Time scaled by 1/s scales velocities by s and
    // accelerations by s^2, so an axis needs friction + s * kV v + s^2 * kA a
    struct Terms {
        double p[3];
        double q[3];
        double r[3];
    };
    std::vector<Terms> terms(count);
    for (std::size_t i = 0; i < count; ++i) {
        const chassis::AxisVolts friction =
            still[i] ? chassis::AxisVolts{} : frictionVolts(models, direction[i]);
        terms[i] =
            Terms{{friction.forward, friction.strafe, friction.turn},
                  {models.forward.kV * vf[i], models.strafe.kV * vs[i], models.turn.kV * w[i]},
                  {models.forward.kA * af[i], models.strafe.kA * as[i], models.turn.kA * aw[i]}};
    }
    const double budget = std::min(config.voltageBudget, 12.0);
    const auto fits = [&](double s) {
        for (const Terms& t : terms) {
            double sum = 0.0;
            for (int a = 0; a < 3; ++a) sum += std::fabs(t.p[a] + s * t.q[a] + s * s * t.r[a]);
            if (sum > budget) return false;
        }
        return true;
    };
    if (!fits(0.0)) return failure("the voltage budget is below what friction needs");
    double scale = config.maxSpeedScale;
    if (!fits(scale)) {
        double good = 0.0;
        double bad = scale;
        for (int step = 0; step < kScaleSteps; ++step) {
            const double middle = 0.5 * (good + bad);
            if (fits(middle)) {
                good = middle;
            } else {
                bad = middle;
            }
        }
        scale = good;
    }
    if (!(scale > 0.0)) return failure("the voltage budget is too low to replay this drive");

    std::vector<TrajectorySample> samples(count);
    double sIn = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        TrajectorySample& sample = samples[i];
        sample.tS = i * kGridS / scale;
        sample.xIn = x[i];
        sample.yIn = y[i];
        sample.headingDeg = h[i];
        sample.vxInPerS = scale * vx[i];
        sample.vyInPerS = scale * vy[i];
        sample.omegaDegPerS = scale * w[i];
        sample.speedInPerS = std::hypot(sample.vxInPerS, sample.vyInPerS);
        if (i > 0) {
            sIn += 0.5 * (samples[i - 1].speedInPerS + sample.speedInPerS) *
                   (sample.tS - samples[i - 1].tS);
        }
        sample.sIn = sIn;
        // along the path: the part of the acceleration in the direction of travel
        const double unscaledSpeed = std::hypot(vx[i], vy[i]);
        sample.accelInPerS2 =
            scale * scale *
            (unscaledSpeed > 1e-9 ? (vx[i] * ax[i] + vy[i] * ay[i]) / unscaledSpeed
                                  : std::hypot(ax[i], ay[i]));
        sample.forwardPerIn = direction[i].forward;
        sample.strafePerIn = direction[i].strafe;
        sample.turnDegPerIn = direction[i].turn;
        sample.forwardInPerS = scale * vf[i];
        sample.strafeInPerS = scale * vs[i];
        sample.forwardAccelInPerS2 = scale * scale * af[i];
        sample.strafeAccelInPerS2 = scale * scale * as[i];
        sample.turnAccelDegPerS2 = scale * scale * aw[i];
        sample.stop = still[i];
    }
    // the end is where the robot settles, never where a chain would carry on from
    samples.back().stop = true;

    // the driver's buttons, from the start of the replay, on its clock
    ReplayPlan plan;
    const double startMs = startS * 1000.0;
    const double endMs = endS * 1000.0;
    std::size_t k = 0;
    while (k + 1 < n && recording[k + 1].tMs <= startMs) ++k;
    plan.buttons.push_back({0.0, recording[k].buttons});
    for (++k; k < n && recording[k].tMs <= endMs; ++k) {
        if (recording[k].buttons == plan.buttons.back().buttons) continue;
        plan.buttons.push_back(
            {(recording[k].tMs - startMs) / 1000.0 / scale, recording[k].buttons});
    }

    plan.ok = true;
    plan.start = odom::Pose{
        .xIn = x[0], .yIn = y[0], .headingDeg = std::fmod(std::fmod(h[0], 360.0) + 360.0, 360.0)};
    plan.speedScale = scale;
    plan.trajectory = Trajectory(std::move(samples), {SectionStart{}});
    return plan;
}

int formatRecordedSample(char* out, std::size_t size, const RecordedSample& sample) {
    return std::snprintf(out, size, "%u,%.3f,%.3f,%.3f,%u\n", static_cast<unsigned>(sample.tMs),
                         sample.xIn, sample.yIn, sample.headingDeg,
                         static_cast<unsigned>(sample.buttons));
}

bool parseRecording(std::string_view text, std::vector<RecordedSample>& out, std::string& error) {
    out.clear();
    const std::string_view kTitle = kRecordingHeader.substr(0, kRecordingHeader.find('\n'));
    bool titled = false;
    std::size_t lineNumber = 0;
    char message[128];
    while (!text.empty()) {
        const std::size_t end = text.find('\n');
        std::string line(text.substr(0, end));
        text = end == std::string_view::npos ? std::string_view{} : text.substr(end + 1);
        ++lineNumber;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!titled) {
            if (line.empty()) continue;
            if (line != kTitle) {
                error =
                    "not a recorded drive (its first line isn't \"" + std::string(kTitle) + "\")";
                return false;
            }
            titled = true;
            continue;
        }
        if (line.empty() || line[0] == '#' || line.rfind("t_ms", 0) == 0) continue;

        RecordedSample sample;
        const char* p = line.c_str();
        char* next = nullptr;
        bool ok = true;
        const unsigned long tMs = std::strtoul(p, &next, 10);
        ok = ok && next != p && *next == ',';
        double* const fields[3] = {&sample.xIn, &sample.yIn, &sample.headingDeg};
        for (double* field : fields) {
            if (!ok) break;
            p = next + 1;
            *field = std::strtod(p, &next);
            ok = next != p && *next == ',' && std::isfinite(*field);
        }
        unsigned long buttons = 0;
        if (ok) {
            p = next + 1;
            buttons = std::strtoul(p, &next, 10);
            ok = next != p && *next == '\0' && buttons <= 0xFFFF;
        }
        if (!ok) {
            std::snprintf(message, sizeof(message),
                          "line %u: expected t_ms,x,y,heading,buttons, got \"%.40s\"",
                          static_cast<unsigned>(lineNumber), line.c_str());
            error = message;
            return false;
        }
        sample.tMs = static_cast<std::uint32_t>(tMs);
        sample.buttons = static_cast<std::uint16_t>(buttons);
        out.push_back(sample);
    }
    if (!titled) {
        error = "the file is empty";
        return false;
    }
    if (out.empty()) {
        error = "the recording has no samples";
        return false;
    }
    return true;
}

} // namespace sapphirelib::motion
