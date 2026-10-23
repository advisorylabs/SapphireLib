#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "sapphirelib/motion/trajectory_config.hpp"

namespace sapphirelib::motion {

/** the most arc length between two samples of a path, in inches. Each piece is split evenly */
constexpr double kTrajectorySampleSpacingIn = 0.5;

/**
 * @brief One point of a sampled path: where it is, which way it goes, and the planned heading
 */
struct PathSample {
    /** arc length from the start, in inches */
    double sIn = 0.0;
    /** x position, in inches */
    double xIn = 0.0;
    /** y position, in inches */
    double yIn = 0.0;
    /** direction of travel, in degrees clockwise from +y */
    double tangentDeg = 0.0;
    /** how fast the direction of travel turns along the path, in radians per inch, clockwise + */
    double curvatureRadPerIn = 0.0;
    /** planned robot heading, in degrees. Continuous, so it may run past 0-360 */
    double headingDeg = 0.0;
    /** heading change per inch of path, in degrees per inch */
    double headingRateDegPerIn = 0.0;
    /** change of headingRateDegPerIn per inch, in degrees per inch squared */
    double headingAccelDegPerIn2 = 0.0;
    /** the section this sample is in */
    std::size_t section = 0;
    /**
     * whether the robot is at rest here: the start, a section's stopAtEnd point, or the end
     * (the start and end only unless PathEnds says the robot is moving there)
     */
    bool stop = false;
};

/**
 * @brief How a path starts and ends, for one that carries on from another trajectory or into one.
 * The default starts and ends at rest
 */
struct PathEnds {
    /**
     * the direction of travel at the start, in degrees clockwise from +y, for a robot already
     * moving that way: the spline leaves the first waypoint along it, and the first sample isn't
     * at rest. Empty starts at rest, free to leave in any direction
     */
    std::optional<double> startTangentDeg;

    /** whether the robot is still moving at the end, rather than at rest */
    bool movingAtEnd = false;
};

/**
 * @brief Result of samplePath()
 */
struct SampledPath {
    /** false if the sections can't make a path; error says why */
    bool ok = false;
    /** what was wrong, e.g. "the first waypoint needs a heading". Empty when ok */
    std::string error;
    /**
     * the samples, in order. At a stop point between two pieces there are two samples at the same
     * place, one per direction of travel
     */
    std::vector<PathSample> samples;
    /** the whole path's length, in inches */
    double lengthIn = 0.0;
    /**
     * where each section starts along the path, in inches: 0 for the first, then the previous
     * section's last waypoint. Exact even for a section shorter than the sample spacing
     */
    std::vector<double> sectionStartIn;
};

/**
 * @brief Fit and sample the path through a chain of sections
 *
 * The position is a natural cubic spline through the waypoints (chord-length parameterized), so
 * curvature changes smoothly through every waypoint. It's split into separate splines at every
 * stopAtEnd point, where the robot is at rest and may turn sharply. The heading eases between
 * waypoints that have one (smootherstep over arc length, the shortest way round), with no turn
 * rate or turn acceleration at those waypoints, and holds after the last
 *
 * @param sections the chain. Needs 2 waypoints in all, the first with a heading, and no two in a
 * row at the same spot
 * @param ends whether the robot is moving at the start (and which way) or the end
 * @return SampledPath the samples, about kTrajectorySampleSpacingIn apart
 */
SampledPath samplePath(const std::vector<TrajectorySection>& sections, const PathEnds& ends = {});

} // namespace sapphirelib::motion
