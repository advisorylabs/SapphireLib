#include "sapphirelib/motion/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "sapphirelib/motion/holonomic_feedforward.hpp"
#include "sapphirelib/motion/trajectory_path.hpp"

namespace sapphirelib::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// coefficients this small are zero: an axis the path doesn't use here
constexpr double kTiny = 1e-12;

// halvings of the speed range when finding the top speed at a point: 2^-40 of it
constexpr int kBisectionSteps = 40;

// the most the wheels can be asked for, whatever a constraint says
constexpr double kMaxVolts = 12.0;

double sumOfSizes(const double p[3], const double q[3], double a) {
    return std::fabs(p[0] + q[0] * a) + std::fabs(p[1] + q[1] * a) + std::fabs(p[2] + q[2] * a);
}

// one sample's kinematics, per unit of path speed: an axis moves at v * g, and accelerates at
// a * g + v^2 * h, for path speed v and path acceleration a
struct Point {
    double g[3];
    double h[3];
    double friction[3];
    const TrajectoryConstraints* constraints;
    // the next section's too, where a section ends: the point both share must suit both
    const TrajectoryConstraints* nextConstraints;
    double budgetVolts;
};

struct Axes {
    const MotorFeedforward* model[3];
    const AxisLimits* limits[3];
};

Axes axesOf(const chassis::HolonomicAxisModels& models, const TrajectoryConstraints& constraints) {
    return Axes{{&models.forward, &models.strafe, &models.turn},
                {&constraints.forward, &constraints.strafe, &constraints.turn}};
}

// the path accelerations allowed at a point and path speed: the voltage budget, then each axis's
// acceleration cap
bool accelRange(const Point& point, const chassis::HolonomicAxisModels& models, double v,
                double& lo, double& hi) {
    const Axes axes = axesOf(models, *point.constraints);
    double p[3];
    double q[3];
    for (int i = 0; i < 3; ++i) {
        const MotorFeedforward& model = *axes.model[i];
        p[i] = point.friction[i] + model.kV * v * point.g[i] + model.kA * v * v * point.h[i];
        q[i] = model.kA * point.g[i];
    }
    if (!voltageAccelRange(p, q, point.budgetVolts, lo, hi)) return false;
    for (const TrajectoryConstraints* constraints : {point.constraints, point.nextConstraints}) {
        if (constraints == nullptr) continue;
        const Axes capped = axesOf(models, *constraints);
        for (int i = 0; i < 3; ++i) {
            const double cap = capped.limits[i]->maxAcceleration;
            if (!(cap > 0.0)) continue;
            const double turning = v * v * point.h[i];
            if (std::fabs(point.g[i]) > kTiny) {
                const double a1 = (-cap - turning) / point.g[i];
                const double a2 = (cap - turning) / point.g[i];
                lo = std::max(lo, std::min(a1, a2));
                hi = std::min(hi, std::max(a1, a2));
            } else if (std::fabs(turning) > cap) {
                return false;
            }
        }
    }
    return lo <= hi;
}

TrajectoryResult failure(const std::string& error) {
    TrajectoryResult result;
    result.error = error;
    return result;
}

std::string atDistance(const char* what, double sIn) {
    char text[160];
    std::snprintf(text, sizeof(text), "%s at %.1fin along the path", what, sIn);
    return text;
}

// a trajectory's direction of travel at a sample, on the field, in degrees clockwise from +y:
// from the planned robot-frame direction, which is defined even at rest
double directionOfTravelDeg(const TrajectorySample& sample) {
    const double heading = sample.headingDeg * kDegToRad;
    const double ux =
        sample.forwardPerIn * std::sin(heading) + sample.strafePerIn * std::cos(heading);
    const double uy =
        sample.forwardPerIn * std::cos(heading) - sample.strafePerIn * std::sin(heading);
    if (std::hypot(ux, uy) > kTiny) return std::atan2(ux, uy) / kDegToRad;
    return std::atan2(sample.vxInPerS, sample.vyInPerS) / kDegToRad;
}

// a speed carried in from another trajectory fits where a bisection put a limit if it's this close
bool aboveLimit(double speed, double limit) { return speed > limit + 1e-6 * std::max(1.0, limit); }

} // namespace

bool voltageAccelRange(const double p[3], const double q[3], double budgetVolts, double& lo,
                       double& hi) {
    // the sum of sizes is convex and piecewise linear in a, bending only where an axis's volts
    // cross 0, so it's lowest at one of those breakpoints and rises linearly away from it
    double breaks[3];
    int count = 0;
    double slope = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(q[i]) <= kTiny) continue;
        breaks[count++] = -p[i] / q[i];
        slope += std::fabs(q[i]);
    }
    if (count == 0) {
        lo = -kInfinity;
        hi = kInfinity;
        return sumOfSizes(p, q, 0.0) <= budgetVolts;
    }
    std::sort(breaks, breaks + count);
    int best = 0;
    for (int k = 1; k < count; ++k) {
        if (sumOfSizes(p, q, breaks[k]) < sumOfSizes(p, q, breaks[best])) best = k;
    }
    const double lowest = sumOfSizes(p, q, breaks[best]);
    if (lowest > budgetVolts) return false;

    // up from the minimum: through each breakpoint until one is over budget, then back along that
    // straight piece to where it crosses. Past the last breakpoint every axis rises together
    double a = breaks[best];
    double f = lowest;
    hi = kInfinity;
    for (int k = best + 1; k < count && hi == kInfinity; ++k) {
        const double fk = sumOfSizes(p, q, breaks[k]);
        if (fk > budgetVolts) {
            hi = a + (budgetVolts - f) * (breaks[k] - a) / (fk - f);
        } else {
            a = breaks[k];
            f = fk;
        }
    }
    if (hi == kInfinity) hi = a + (budgetVolts - f) / slope;

    // and down
    a = breaks[best];
    f = lowest;
    lo = -kInfinity;
    for (int k = best - 1; k >= 0 && lo == -kInfinity; --k) {
        const double fk = sumOfSizes(p, q, breaks[k]);
        if (fk > budgetVolts) {
            lo = a - (budgetVolts - f) * (a - breaks[k]) / (fk - f);
        } else {
            a = breaks[k];
            f = fk;
        }
    }
    if (lo == -kInfinity) lo = a - (budgetVolts - f) / slope;
    return true;
}

TrajectoryResult generateTrajectory(const std::vector<TrajectorySection>& givenSections,
                                    const chassis::HolonomicAxisModels& models,
                                    const TrajectoryOptions& options) {
    const char* const names[3] = {"Fwd", "Strafe", "Turn"};
    const MotorFeedforward* const axisModels[3] = {&models.forward, &models.strafe, &models.turn};
    for (int i = 0; i < 3; ++i) {
        if (!axisModels[i]->valid()) {
            return failure(std::string("the ") + names[i] +
                           " model isn't measured (kV and kA must be above 0): run Auto-Tune");
        }
    }

    // carrying on from another trajectory: its last point, with its heading, starts the chain,
    // and the robot leaves it at that trajectory's speed and in its direction of travel
    std::vector<TrajectorySection> joined;
    PathEnds ends;
    double startSpeed = 0.0;
    if (options.from != nullptr) {
        if (options.from->empty()) return failure("the trajectory to continue from is empty");
        const TrajectorySample& end = options.from->samples().back();
        joined = givenSections;
        if (joined.empty()) joined.emplace_back();
        joined.front().waypoints.insert(
            joined.front().waypoints.begin(),
            TrajectoryWaypoint{.xIn = end.xIn, .yIn = end.yIn, .headingDeg = end.headingDeg});
        startSpeed = std::max(0.0, end.speedInPerS);
        if (startSpeed > 0.0) ends.startTangentDeg = directionOfTravelDeg(end);
    }
    const std::vector<TrajectorySection>& sections =
        options.from != nullptr ? joined : givenSections;
    ends.movingAtEnd = options.endSpeedInPerS > 0.0;

    const SampledPath path = samplePath(sections, ends);
    if (!path.ok) return failure(path.error);
    const std::vector<PathSample>& samples = path.samples;
    const std::size_t n = samples.size();

    // each sample's kinematics
    std::vector<Point> points(n);
    for (std::size_t k = 0; k < n; ++k) {
        const PathSample& sample = samples[k];
        // direction of travel relative to the robot, and how fast that turns along the path
        const double phi = (sample.tangentDeg - sample.headingDeg) * kDegToRad;
        const double dphi = sample.curvatureRadPerIn - sample.headingRateDegPerIn * kDegToRad;
        Point& point = points[k];
        point.g[0] = std::cos(phi);
        point.g[1] = std::sin(phi);
        point.g[2] = sample.headingRateDegPerIn;
        point.h[0] = -std::sin(phi) * dphi;
        point.h[1] = std::cos(phi) * dphi;
        point.h[2] = sample.headingAccelDegPerIn2;
        const chassis::AxisVolts friction =
            frictionVolts(models, AxisMotion{point.g[0], point.g[1], point.g[2]});
        point.friction[0] = friction.forward;
        point.friction[1] = friction.strafe;
        point.friction[2] = friction.turn;
        point.constraints = &sections[sample.section].constraints;
        point.nextConstraints = nullptr;
        point.budgetVolts = std::min(point.constraints->voltageBudget, kMaxVolts);
        if (k + 1 < n && samples[k + 1].section != sample.section) {
            const std::size_t next = samples[k + 1].section;
            if (std::fabs(sample.sIn - path.sectionStartIn[next]) < 1e-9) {
                point.nextConstraints = &sections[next].constraints;
                point.budgetVolts = std::min(
                    point.budgetVolts, std::min(point.nextConstraints->voltageBudget, kMaxVolts));
            }
        }
    }

    // the top speed at each sample on its own: the fastest the axes' caps and the budget allow
    std::vector<double> cap(n, 0.0);
    for (std::size_t k = 0; k < n; ++k) {
        const Point& point = points[k];
        double lo = 0.0;
        double hi = 0.0;
        if (!accelRange(point, models, 0.0, lo, hi)) {
            return failure(
                atDistance("the voltage budget is below what friction needs", samples[k].sIn));
        }
        if (samples[k].stop) continue;
        double upper = kInfinity;
        for (const TrajectoryConstraints* constraints :
             {point.constraints, point.nextConstraints}) {
            if (constraints == nullptr) continue;
            const Axes axes = axesOf(models, *constraints);
            for (int i = 0; i < 3; ++i) {
                if (std::fabs(point.g[i]) <= kTiny) continue;
                const double limit = axes.limits[i]->maxVelocity;
                const double top =
                    limit > 0.0 ? limit : axes.model[i]->maxVelocity(point.budgetVolts);
                upper = std::min(upper, top / std::fabs(point.g[i]));
            }
        }
        if (accelRange(point, models, upper, lo, hi)) {
            cap[k] = upper;
            continue;
        }
        double feasible = 0.0;
        double infeasible = upper;
        for (int step = 0; step < kBisectionSteps; ++step) {
            const double middle = 0.5 * (feasible + infeasible);
            if (accelRange(point, models, middle, lo, hi)) {
                feasible = middle;
            } else {
                infeasible = middle;
            }
        }
        cap[k] = feasible;
    }
    // ending moving, for another trajectory to continue from: no faster than asked
    if (!samples.back().stop) cap[n - 1] = std::min(cap[n - 1], options.endSpeedInPerS);
    // starting moving: the speed carried in has to fit where this path starts
    if (aboveLimit(startSpeed, cap[0])) {
        char text[200];
        std::snprintf(text, sizeof(text),
                      "can't carry on at %.1fin/s: at most %.1fin/s fits where this path starts. "
                      "Plan the trajectory it continues from with a lower endSpeedInPerS",
                      startSpeed, cap[0]);
        return failure(text);
    }

    // forward pass: accelerate as hard as each step allows, never past the next point's top speed.
    // What a point allows falls as speed rises, so a step taken at its start's limit would ask for
    // more than the budget by its end, and one taken at its end's limit falls short. The average of
    // the two is right to second order (the trapezoid rule) and never more than the start allows.
    // The largest end speed that fits is found by bisection (the required acceleration rises with
    // the end speed, the allowed one falls)
    std::vector<double> speed(n, 0.0);
    speed[0] = startSpeed;
    for (std::size_t k = 0; k + 1 < n; ++k) {
        const double ds = samples[k + 1].sIn - samples[k].sIn;
        if (ds <= 0.0) continue; // the two samples of a stop: both at rest
        double lo = 0.0;
        double hiStart = 0.0;
        if (!accelRange(points[k], models, speed[k], lo, hiStart)) hiStart = 0.0;
        if (speed[k] <= 0.0 && hiStart <= 0.0) {
            return failure(
                atDistance("the voltage budget can't start the robot moving", samples[k].sIn));
        }
        const auto fits = [&](double end) {
            double endLo = 0.0;
            double endHi = 0.0;
            if (!accelRange(points[k + 1], models, end, endLo, endHi)) return false;
            const double required = (end * end - speed[k] * speed[k]) / (2.0 * ds);
            return required <= std::min(hiStart, 0.5 * (hiStart + endHi));
        };
        const double reach = std::sqrt(std::max(0.0, speed[k] * speed[k] + 2.0 * hiStart * ds));
        const double upper = std::min(cap[k + 1], reach);
        if (fits(upper)) {
            speed[k + 1] = upper;
            continue;
        }
        double good = 0.0;
        double bad = upper;
        for (int step = 0; step < kBisectionSteps; ++step) {
            const double middle = 0.5 * (good + bad);
            if (fits(middle)) {
                good = middle;
            } else {
                bad = middle;
            }
        }
        speed[k + 1] = good;
    }
    // backward pass: brake in time for every slower point ahead
    for (std::size_t k = n - 1; k >= 1; --k) {
        const double ds = samples[k].sIn - samples[k - 1].sIn;
        if (ds <= 0.0) continue;
        double lo = 0.0;
        double hi = 0.0;
        if (!accelRange(points[k], models, speed[k], lo, hi)) lo = 0.0;
        const double squared = speed[k] * speed[k] - 2.0 * lo * ds;
        speed[k - 1] = std::min(speed[k - 1], std::sqrt(std::max(0.0, squared)));
    }
    // braking for what's ahead from the start would change the speed carried in
    if (aboveLimit(startSpeed, speed[0])) {
        char text[200];
        std::snprintf(text, sizeof(text),
                      "can't brake from %.1fin/s in time for this path (at most %.1fin/s): plan "
                      "the trajectory it continues from with a lower endSpeedInPerS",
                      startSpeed, speed[0]);
        return failure(text);
    }
    speed[0] = startSpeed;

    // each step's acceleration and time; a sample takes the acceleration of the step after it, or
    // the one before it where there's no step after (the end, a stop's arriving sample)
    std::vector<double> stepAccel(n, 0.0);
    std::vector<double> time(n, 0.0);
    for (std::size_t k = 0; k + 1 < n; ++k) {
        const double ds = samples[k + 1].sIn - samples[k].sIn;
        double dt = 0.0;
        if (ds > 0.0) {
            const double sum = speed[k] + speed[k + 1];
            if (!(sum > 0.0)) {
                return failure(atDistance("the plan stalls", samples[k].sIn));
            }
            dt = 2.0 * ds / sum;
            stepAccel[k] = (speed[k + 1] * speed[k + 1] - speed[k] * speed[k]) / (2.0 * ds);
        }
        time[k + 1] = time[k] + dt;
    }

    std::vector<SectionStart> starts;
    for (double startIn : path.sectionStartIn) {
        // the time the plan reaches startIn, between the samples either side of it
        std::size_t k = 0;
        while (k + 1 < n && samples[k + 1].sIn < startIn) ++k;
        double tS = time[k];
        if (k + 1 < n && samples[k + 1].sIn > samples[k].sIn) {
            const double f = (startIn - samples[k].sIn) / (samples[k + 1].sIn - samples[k].sIn);
            tS = time[k] + std::clamp(f, 0.0, 1.0) * (time[k + 1] - time[k]);
        }
        starts.push_back(SectionStart{.tS = tS, .sIn = startIn});
    }

    std::vector<TrajectorySample> out(n);
    for (std::size_t k = 0; k < n; ++k) {
        const PathSample& sample = samples[k];
        const Point& point = points[k];
        const bool stepAfter = k + 1 < n && samples[k + 1].sIn > sample.sIn;
        const double v = speed[k];
        const double a = stepAfter || k == 0 ? stepAccel[k] : stepAccel[k - 1];
        const double tangent = sample.tangentDeg * kDegToRad;

        TrajectorySample& s = out[k];
        s.tS = time[k];
        s.sIn = sample.sIn;
        s.section = 0;
        while (s.section + 1 < starts.size() && starts[s.section + 1].sIn <= sample.sIn) {
            ++s.section;
        }
        s.stop = sample.stop;
        s.xIn = sample.xIn;
        s.yIn = sample.yIn;
        s.headingDeg = sample.headingDeg;
        s.speedInPerS = v;
        s.accelInPerS2 = a;
        s.vxInPerS = v * std::sin(tangent);
        s.vyInPerS = v * std::cos(tangent);
        s.omegaDegPerS = v * point.g[2];
        s.forwardPerIn = point.g[0];
        s.strafePerIn = point.g[1];
        s.turnDegPerIn = point.g[2];
        s.forwardInPerS = v * point.g[0];
        s.strafeInPerS = v * point.g[1];
        s.forwardAccelInPerS2 = a * point.g[0] + v * v * point.h[0];
        s.strafeAccelInPerS2 = a * point.g[1] + v * v * point.h[1];
        s.turnAccelDegPerS2 = a * point.g[2] + v * v * point.h[2];
    }

    TrajectoryResult result;
    result.ok = true;
    result.trajectory = Trajectory(std::move(out), std::move(starts));
    return result;
}

TrajectoryResult generateTrajectory(const std::vector<TrajectoryWaypoint>& waypoints,
                                    const TrajectoryConstraints& constraints,
                                    const chassis::HolonomicAxisModels& models) {
    return generateTrajectory(
        {TrajectorySection{.waypoints = waypoints, .constraints = constraints}}, models);
}

} // namespace sapphirelib::motion
