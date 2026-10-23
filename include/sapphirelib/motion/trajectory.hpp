#pragma once

#include <cstddef>
#include <vector>

namespace sapphirelib::motion {

/**
 * @brief Where a trajectory plans the robot to be at one moment, and how it's moving
 *
 * Field values are in the same frame as odom::Pose. Robot-frame values are along the planned
 * heading: forward, strafe (positive to the right) and turn (clockwise positive)
 */
struct TrajectorySample {
    /** time from the start, in seconds */
    double tS = 0.0;
    /** distance along the path, in inches */
    double sIn = 0.0;
    /** the section this point is in */
    std::size_t section = 0;
    /** whether the robot is planned to be at rest here */
    bool stop = false;

    /** x position, in inches */
    double xIn = 0.0;
    /** y position, in inches */
    double yIn = 0.0;
    /** heading, in degrees. Continuous, so it may run past 0-360: compare with wrapDegrees180 */
    double headingDeg = 0.0;

    /** speed along the path, in inches per second */
    double speedInPerS = 0.0;
    /** acceleration along the path, in inches per second squared */
    double accelInPerS2 = 0.0;
    /** field x velocity, in inches per second */
    double vxInPerS = 0.0;
    /** field y velocity, in inches per second */
    double vyInPerS = 0.0;
    /** turn rate, in degrees per second */
    double omegaDegPerS = 0.0;

    /**
     * robot-frame direction of travel: forward inches per inch of path. With strafePerIn, a unit
     * vector; it's defined even at rest, so static friction always points along the path
     */
    double forwardPerIn = 0.0;
    /** robot-frame direction of travel: strafe inches per inch of path */
    double strafePerIn = 0.0;
    /** heading change per inch of path, in degrees per inch */
    double turnDegPerIn = 0.0;

    /** robot-frame forward velocity, in inches per second */
    double forwardInPerS = 0.0;
    /** robot-frame strafe velocity, in inches per second */
    double strafeInPerS = 0.0;
    /** robot-frame forward acceleration, in inches per second squared */
    double forwardAccelInPerS2 = 0.0;
    /** robot-frame strafe acceleration, in inches per second squared */
    double strafeAccelInPerS2 = 0.0;
    /** turn acceleration, in degrees per second squared */
    double turnAccelDegPerS2 = 0.0;
};

/**
 * @brief When and where a section of a trajectory starts
 */
struct SectionStart {
    /** time from the start of the trajectory, in seconds */
    double tS = 0.0;
    /** distance along the path, in inches */
    double sIn = 0.0;
};

/**
 * @brief A planned motion: the path, and the speed and heading at every moment along it
 *
 * Made by generateTrajectory() and driven by HolonomicDrivetrain::followTrajectory(). Building
 * one allocates; reading it doesn't, so a control loop can sample it every tick
 */
class Trajectory {
public:
    /**
     * @brief Construct an empty trajectory, one followTrajectory() refuses to drive
     */
    Trajectory() = default;

    /**
     * @brief Construct a trajectory from its samples. generateTrajectory() does this
     *
     * @param samples the samples, in time order
     * @param sectionStarts when and where each section starts, the first at 0
     */
    Trajectory(std::vector<TrajectorySample> samples, std::vector<SectionStart> sectionStarts);

    /**
     * @brief Whether there's nothing to drive
     */
    bool empty() const;

    /**
     * @brief Get the planned duration, in seconds. 0 when empty
     */
    double durationS() const;

    /**
     * @brief Get the path's length, in inches. 0 when empty
     */
    double lengthIn() const;

    /**
     * @brief Get the number of sections
     */
    std::size_t sectionCount() const;

    /**
     * @brief Get when and where a section starts
     *
     * @param section the section. Past the last, the end of the trajectory
     * @return SectionStart its start time and distance along the path
     */
    SectionStart sectionStart(std::size_t section) const;

    /**
     * @brief Get the section a distance along the path is in
     *
     * @param sIn distance along the path, in inches
     * @return std::size_t the section
     */
    std::size_t sectionAt(double sIn) const;

    /**
     * @brief Get the planned state at a time, interpolated between samples
     *
     * @param tS time from the start, in seconds. Clamped to the trajectory
     * @return TrajectorySample the state. All zeros when empty
     */
    TrajectorySample sample(double tS) const;

    /**
     * @brief Get every sample, in time order
     */
    const std::vector<TrajectorySample>& samples() const;

private:
    std::vector<TrajectorySample> samples_;
    std::vector<SectionStart> sectionStarts_;
};

} // namespace sapphirelib::motion
