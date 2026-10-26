#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "sapphirelib/chassis/holonomic_drivetrain.hpp"
#include "sapphirelib/chassis/tank_drivetrain.hpp"
#include "sapphirelib/localization/monte_carlo_localizer.hpp"
#include "sapphirelib/localization/wall_reset.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"
#include "sapphirelib/motion/path.hpp"
#include "sapphirelib/motion/trajectory_config.hpp"
#include "sapphirelib/motion/turn_math.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/odom/pose.hpp"

/**
 * @brief Writing autonomous routines: short calls for every motion, on one drivetrain
 *
 * Tell it the robot's drivetrain, odometry and (optionally) localizer once with use(), then write
 * routines as plain lists of calls. Every motion blocks until it's done and returns how it ended
 * (motion::MotionResult), so a routine can stop instead of driving on from a spot it never
 * reached. runAsync() runs motions in the background, to time mechanisms against them.
 *
 * Positions are field inches and headings degrees: the field's middle is (0, 0), the walls are at
 * +-70.25, heading 0 faces +y and 90 faces +x (clockwise). Start every routine with setPose() at
 * the robot's real spot, which puts odometry, turns and the localizer in that frame
 *
 * @b Example
 * @code {.cpp}
 * using namespace sapphirelib::auton;
 *
 * void rightSide() {
 *     setPose(-48, -60, 0);
 *     runAsync([] { moveToPoint(-48, -24); });
 *     waitUntilTraveled(20);       // 20in into the move...
 *     macros::setExtended(true);   // ...push the extender out
 *     waitForMotion();
 *     turnTo(90);
 *     if (!moveToPose(-24, -24, 90).settled()) return; // blocked: stop here
 *     resetBack();                 // x from the wall behind
 * }
 * @endcode
 */
namespace sapphirelib::auton {

// --- Setup ---

/**
 * @brief Settings for use()
 */
struct Settings {
    /** which readings resetFront() and the rest trust. See localization::WallResetConfig */
    localization::WallResetConfig wallReset;
};

/**
 * @brief Run every auton function on this holonomic drivetrain and odometry
 *
 * Call once from initialize(), after odometry's task has started. The drivetrain, odometry and
 * localizer must outlive the program (make them static)
 *
 * @param drivetrain the drivetrain
 * @param odometry where the robot is
 * @param localizer the distance-sensor localizer, for setMcl() and the wall resets. nullptr (the
 * default) for a robot without one
 * @param settings the rest
 */
void use(chassis::HolonomicDrivetrain& drivetrain, odom::Odometry& odometry,
         localization::MonteCarloLocalizer* localizer = nullptr, Settings settings = {});

/**
 * @brief Run every auton function on this tank drivetrain and odometry. strafe() and
 * followTrajectory() need a holonomic drivetrain, and do nothing here
 */
void use(chassis::TankDrivetrain& drivetrain, odom::Odometry& odometry,
         localization::MonteCarloLocalizer* localizer = nullptr, Settings settings = {});

// --- Where the robot is ---

/**
 * @brief Say where the robot is and which way it faces, in field coordinates
 *
 * Start every routine with it, at the spot the robot really starts. Puts odometry, every turn,
 * and the localizer in the field frame
 *
 * @param xIn x, in inches
 * @param yIn y, in inches
 * @param headingDeg heading, in degrees clockwise from +y
 */
void setPose(double xIn, double yIn, double headingDeg);

/**
 * @brief Get where the robot is: odometry's pose, with the localizer's correction
 */
odom::Pose pose();

// --- Motions ---

/**
 * @brief How a motion runs. Every field has a default, so pass only what you change
 *
 * @b Example
 * @code {.cpp}
 * drive(24, {.maxVolts = 6});                    // gently
 * moveToPoint(0, 48, {.timeoutMs = 1500});       // give up sooner
 * moveToPoint(0, 24, {.chain = true});           // keep moving into the next motion...
 * moveToPoint(24, 48);                           // ...which ends stopped
 * @endcode
 */
struct Options {
    /** give up after this long, in milliseconds. 0, the default, uses the motion's own (3s, 4s
     * for moveToPose()) */
    std::uint32_t timeoutMs = 0;

    /** top motor voltage: 12 is full speed, lower is gentler. 12 by default */
    double maxVolts = 12.0;

    /**
     * end early while still moving, so the next motion starts at speed: faster, a little less
     * exact. Never chain a routine's last motion. False by default
     */
    bool chain = false;

    /**
     * how close counts as there: inches for drives and points, degrees for turns. 0, the default,
     * uses 1in and 2 degrees, or 3in and 5 degrees when chaining
     */
    double exitError = 0.0;
};

/** turn to the right (heading increasing), even when left is shorter: turnTo(180, clockwise) */
inline constexpr motion::TurnDirection clockwise = motion::TurnDirection::clockwise;

/** turn to the left (heading decreasing), even when right is shorter */
inline constexpr motion::TurnDirection counterclockwise = motion::TurnDirection::counterclockwise;

/**
 * @brief Drive straight, holding the heading it starts on. Negative drives backward
 *
 * @param inches how far
 * @param options how it runs
 * @return motion::MotionResult how it ended
 */
motion::MotionResult drive(double inches, Options options = {});

/**
 * @brief Turn in place to a field heading, the short way
 *
 * @param headingDeg the heading, in degrees clockwise from +y
 * @param options how it runs
 * @return motion::MotionResult how it ended
 */
motion::MotionResult turnTo(double headingDeg, Options options = {});

/**
 * @brief Turn in place to a field heading, one way: clockwise or counterclockwise. For turns of
 * about 180 degrees, which could otherwise go either way
 */
motion::MotionResult turnTo(double headingDeg, motion::TurnDirection direction,
                            Options options = {});

/**
 * @brief Turn in place to face a point on the field
 *
 * @param xIn the point's x, in inches
 * @param yIn the point's y, in inches
 * @param options how it runs
 * @return motion::MotionResult how it ended
 */
motion::MotionResult turnToPoint(double xIn, double yIn, Options options = {});

/**
 * @brief Drive to a point on the field. A holonomic drive holds its heading on the way; a tank
 * drive steers to face the point
 *
 * @param xIn the point's x, in inches
 * @param yIn the point's y, in inches
 * @param options how it runs
 * @return motion::MotionResult how it ended
 */
motion::MotionResult moveToPoint(double xIn, double yIn, Options options = {});

/**
 * @brief Drive to a point on the field and end facing a heading
 *
 * @param xIn the point's x, in inches
 * @param yIn the point's y, in inches
 * @param headingDeg the heading to end on, in degrees
 * @param options how it runs. exitError is the distance; the heading must be within 2 degrees
 * (5 chaining)
 * @return motion::MotionResult how it ended
 */
motion::MotionResult moveToPose(double xIn, double yIn, double headingDeg, Options options = {});

/**
 * @brief Strafe sideways, holding the heading: positive right, negative left. Holonomic only
 *
 * @param inches how far
 * @param options how it runs
 * @return motion::MotionResult how it ended
 */
motion::MotionResult strafe(double inches, Options options = {});

/**
 * @brief How followPath() runs
 */
struct PathOptions {
    /** give up after this long, in milliseconds, before the final approach. 10s by default */
    std::uint32_t timeoutMs = 10000;

    /** the voltage to follow the path at, until the final approach. 8 by default */
    double maxVolts = 8.0;

    /** how far ahead on the path to aim, in inches: bigger is smoother, but cuts corners. 10 by
     * default */
    double lookaheadIn = 10.0;

    /** carry on from the end into the next motion, without stopping. False by default */
    bool chain = false;
};

/**
 * @brief Follow a path through points with pure pursuit, then settle on the last one
 *
 * Start it at the path's first point. A holonomic drive holds its heading the whole way
 *
 * @param points the points, in field inches
 * @param options how it runs
 * @return motion::MotionResult how it ended
 *
 * @b Example
 * @code {.cpp}
 * setPose(-36, -60, 0);
 * followPath({{-36, -60}, {-36, -32}, {-12, -22}, {12, -30}});
 * @endcode
 */
motion::MotionResult followPath(std::vector<motion::Waypoint> points, PathOptions options = {});

/**
 * @brief Drive a planned trajectory: sections from the simulator's Paths tab. Holonomic only
 *
 * Plans it from the drivetrain's axis models (Auto-Tune or TUNE.CFG) the first time, and keeps
 * the plan, so pass the same list each time (a const at file scope, as Copy C++ writes it).
 * planTrajectory() plans it ahead of time. waitUntilWaypoint() times things to its points
 *
 * @param sections the trajectory
 * @param config how it's followed: overtime, the settle, and branching (next)
 * @return motion::MotionResult how it ended. Aborted if it can't be planned (the terminal says
 * why)
 */
motion::MotionResult followTrajectory(const std::vector<motion::TrajectorySection>& sections,
                                      motion::TrajectoryFollowConfig config = {});

/**
 * @brief Plan a trajectory now, so followTrajectory() doesn't have to when it's driven. Call it
 * at the start of a routine, before the robot moves
 *
 * @return true if it planned
 */
bool planTrajectory(const std::vector<motion::TrajectorySection>& sections);

// --- Motions in the background ---

/**
 * @brief Start motions in the background, and carry on with the routine
 *
 * The function runs on a task of its own: put one motion or several in it. A motion called
 * outside it waits for it to finish first, so two never drive at once; so does another
 * runAsync(). Time mechanisms against it with the waitUntil...() calls, and stop it early with
 * cancelMotion()
 *
 * @param motions the motions to run
 *
 * @b Example
 * @code {.cpp}
 * runAsync([] { moveToPoint(0, 48); });
 * waitUntilTraveled(12);               // 12in into the move...
 * intake.move_voltage(12000);          // ...start the intake
 * sapphirelib::waitUntil([] { return !isMotionRunning() || pieceSeen(); }, 3000);
 * cancelMotion();                      // stop early once the sensor sees it
 * waitForMotion();
 * @endcode
 */
void runAsync(std::function<void()> motions);

/**
 * @brief Whether background motions (runAsync()) are running or waiting to
 */
bool isMotionRunning();

/**
 * @brief Wait until the background motions are done
 */
void waitForMotion();

/**
 * @brief Stop the background motions now: the running one ends where it is (interrupted), and the
 * rest of the runAsync() function's motions are skipped. Safe from any task
 */
void cancelMotion();

/**
 * @brief Wait until the running background motion has driven this far
 *
 * @param inches how far since it started, around corners and all
 * @return true when it has; false if the background motions ended first
 */
bool waitUntilTraveled(double inches);

/**
 * @brief Wait until the running background motion has turned this far, either way
 *
 * @param degrees how far since it started
 * @return true when it has; false if the background motions ended first
 */
bool waitUntilTurned(double degrees);

/**
 * @brief Wait until the robot is within a distance of a point, while background motions run
 *
 * @param xIn the point's x, in inches
 * @param yIn the point's y, in inches
 * @param radiusIn how close, in inches
 * @return true when it is; false if the background motions ended first
 */
bool waitUntilNear(double xIn, double yIn, double radiusIn);

/**
 * @brief Wait until a background followTrajectory()'s plan reaches one of its points, without the
 * robot slowing down for it
 *
 * @param waypoint the point's number in the Paths tab: its index among all the sections'
 * waypoints, in order. 0 is the start
 * @return true when it does; false if the background motions ended first, or there's no such point
 */
bool waitUntilWaypoint(std::size_t waypoint);

// --- Localization ---

/**
 * @brief Reset x or y from the sensor facing forward, off the wall it sees. See
 * localization::MonteCarloLocalizer::resetFromWall()
 *
 * @return true if it reset; false if the reading was skipped (the SD log's mcl event says why), or
 * there's no localizer or no sensor facing that way
 */
bool resetFront();

/** @brief resetFront(), with the sensor facing backward */
bool resetBack();

/** @brief resetFront(), with the sensor facing left */
bool resetLeft();

/** @brief resetFront(), with the sensor facing right */
bool resetRight();

/**
 * @brief Turn the localizer's corrections on or off, e.g. off while pushing against a goal the
 * sensors would mistake for a wall. On by default
 */
void setMcl(bool on);

} // namespace sapphirelib::auton
