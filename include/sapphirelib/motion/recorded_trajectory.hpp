#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/input/button_playback.hpp"
#include "sapphirelib/motion/trajectory.hpp"
#include "sapphirelib/odom/pose.hpp"

namespace sapphirelib::motion {

/**
 * @brief One tick of a driver's drive, recorded to replay as an autonomous routine
 */
struct RecordedSample {
    /** time since the recording started, in milliseconds */
    std::uint32_t tMs = 0;
    /** x position, in inches, in the same field frame as odom::Pose */
    double xIn = 0.0;
    /** y position, in inches */
    double yIn = 0.0;
    /** heading, in degrees, 0-360 like odometry's or continuous: replaying unwraps it */
    double headingDeg = 0.0;
    /** the buttons held, a bit per input::Button (input::ButtonMask) */
    std::uint16_t buttons = 0;
};

/**
 * @brief How trajectoryFromRecording() turns a recorded drive into a plan
 */
struct ReplayConfig {
    /**
     * the most any wheel may be planned to need, in volts, as TrajectoryConstraints's budget: a
     * drive that needs more (a driver at full stick needs 12) is replayed slower, all of it
     * evenly, until it fits, leaving the rest for the tracking feedback. 10 by default
     */
    double voltageBudget = 10.0;

    /**
     * the fastest the replay may be, as a fraction of the driver's own speed. 1 (the default)
     * replays as fast as they drove, if the budget allows
     */
    double maxSpeedScale = 1.0;

    /**
     * how much the recorded pose is smoothed before working out its speeds, in seconds (a
     * Gaussian's standard deviation). Too little and odometry's small steps become kicks in the
     * feedforward; too much and corners get cut. 0.06 by default
     */
    double smoothingS = 0.06;

    /**
     * slower than this, in inches per second, and turning slower than stillDegPerS, the robot
     * counts as still: no friction feedforward pushes it there, so it holds on feedback
     */
    double stillInPerS = 0.5;

    /** see stillInPerS, in degrees per second */
    double stillDegPerS = 3.0;
};

/**
 * @brief Result of trajectoryFromRecording(): a recorded drive, ready to follow
 */
struct ReplayPlan {
    /** false if the recording can't be replayed; error says why */
    bool ok = false;
    /** what was wrong, e.g. "the recording is too short". Empty when ok */
    std::string error;
    /**
     * the drive as a trajectory, timed by the recording (scaled by speedScale): one sample every
     * 10ms, starting and ending at rest. Follow it with HolonomicDrivetrain::followTrajectory()
     */
    Trajectory trajectory;
    /**
     * where the robot was as the replay starts. Put the robot back on that spot and
     * Odometry::setPose() this before following
     */
    odom::Pose start;
    /** how fast the replay is, as a fraction of the driver's speed: at most maxSpeedScale */
    double speedScale = 1.0;
    /**
     * the driver's buttons from each moment they changed, on the plan's clock
     * (TrajectoryProgress::timeS): play them into the driver's own macros with
     * input::ButtonPlayback and Controller::replay()
     */
    std::vector<input::TimedButtons> buttons;
};

/**
 * @brief Turn a recorded drive into a trajectory to replay as an autonomous routine
 *
 * The robot replays where the driver drove, not the sticks they pushed, so the tracking feedback,
 * odometry and the localizer correct it the way they correct any trajectory, battery, wheel slip
 * and a start a little off included. It trims the wait before the drive starts and after it ends
 * (keeping any button presses), smooths the pose, works out the velocity and acceleration the
 * feedforward needs from it, and slows the whole drive evenly if the driver went faster than the
 * voltage budget allows. Turning in place is fine: the plan is in time, not along a path
 *
 * @note allocates, and takes some milliseconds for a long drive: build it before the motion
 *
 * @param recording the drive, at least two samples in time order (a sample every 10-20ms)
 * @param models the chassis's measured axis models. All three must be valid
 * @param config the voltage budget, a speed cap, and the smoothing
 * @return ReplayPlan the trajectory, where it starts, and the buttons, or why there isn't one
 *
 * @b Example
 * @code {.cpp}
 * static sapphirelib::motion::ReplayPlan plan;
 * plan = sapphirelib::motion::trajectoryFromRecording(recording, drivetrain().axisModels());
 * if (!plan.ok) return;
 * odometry().setPose(plan.start);
 * drivetrain().followTrajectory(plan.trajectory);
 * @endcode
 */
ReplayPlan trajectoryFromRecording(const std::vector<RecordedSample>& recording,
                                   const chassis::HolonomicAxisModels& models,
                                   const ReplayConfig& config = {});

/** the first lines of a recording file: what it is, and its columns */
inline constexpr std::string_view kRecordingHeader =
    "# SapphireLib recorded drive v1\nt_ms,x,y,heading,buttons\n";

/**
 * @brief Format one sample as a recording file's row, newline included
 *
 * @param out where to write it
 * @param size out's size, in bytes. 96 is always enough
 * @param sample the sample
 * @return int the row's length, as snprintf() returns it
 */
int formatRecordedSample(char* out, std::size_t size, const RecordedSample& sample);

/**
 * @brief Read a recording file: kRecordingHeader, then formatRecordedSample() rows
 *
 * Blank lines and lines starting with # are skipped; \r\n line ends are fine
 *
 * @param text the file's contents
 * @param out set to the samples
 * @param error set to what's wrong, naming the line, when it returns false
 * @return true if it read
 */
bool parseRecording(std::string_view text, std::vector<RecordedSample>& out, std::string& error);

} // namespace sapphirelib::motion
