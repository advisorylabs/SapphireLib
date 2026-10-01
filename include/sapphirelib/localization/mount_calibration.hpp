#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "sapphirelib/localization/field_map.hpp"
#include "sapphirelib/localization/sensor_model.hpp"
#include "sapphirelib/odom/pose.hpp"

namespace sapphirelib::localization {

/**
 * @brief One distance reading taken while calibrating the sensor mounts, with where odometry put
 * the robot when it was taken
 */
struct MountSample {
    /** which sensor, in the order the mounts are given */
    std::size_t sensor = 0;

    /** odometry's raw x, in inches. Any frame: only how far the robot moved matters */
    double xIn = 0.0;

    /** odometry's raw y, in inches */
    double yIn = 0.0;

    /** odometry's heading, in degrees, wrapped or not. Only how far it turned matters */
    double headingDeg = 0.0;

    /** the reading, in inches */
    double distanceIn = 0.0;
};

/**
 * @brief Settings for calibrating the distance sensors' mounts
 *
 * The first three are the spin MonteCarloLocalizer::calibrateSensorMounts() drives, the rest are
 * fitSensorMounts()'s
 */
struct MountCalibrationConfig {
    /** how many turns to spin each way. Spinning both ways cancels most of the sensors' delay */
    double turns = 1.0;

    /**
     * how fast to spin, in degrees per second. Slow enough that a beam barely sweeps while the
     * sensor measures (about 30 ms), fast enough to be done in under 20 s
     */
    double turnRateDegPerS = 45.0;

    /** the largest turn command the spin may use, 0 to 1 */
    double maxSpinPower = 0.5;

    /**
     * only fit readings that hit a wall within this many degrees of square on. The sensor's beam
     * is a cone, and at a slant it can read short of where its center hits
     */
    double maxIncidenceDeg = 20.0;

    /**
     * how far the fit may move a sensor sideways from its configured mount, in inches (1 standard
     * deviation). Sideways shows only in slanted readings, weakly, so it stays near the configured
     * value unless the readings say otherwise
     */
    double lateralPriorIn = 1.0;

    /** the fewest readings matching a wall that a sensor needs to be calibrated */
    std::size_t minReadings = 30;

    /** the fewest readings of one wall for that wall to count as seen */
    std::size_t minWallReadings = 8;

    /** the largest standard error of the along-beam offset that still counts as calibrated, in
     * inches */
    double maxStderrIn = 0.25;

    /**
     * the largest change from the configured mount that's believable, in inches. More means
     * something in the way fooled the fit, so the sensor is left as it was
     */
    double maxChangeIn = 6.0;
};

/**
 * @brief Whether one sensor's mount was calibrated, and if not, why
 */
enum class MountFitStatus : std::uint8_t {
    calibrated,      // the fitted mount is good to use
    noReadings,      // the sensor never had a usable reading
    tooFewReadings,  // fewer than minReadings readings matched a wall
    noOppositeWalls, // no sensor saw both walls of a pair, so the robot's distance to the walls
                     // this one saw couldn't be told apart from its own offset
    uncertain,       // the along-beam offset's standard error is over maxStderrIn
    implausible,     // it moved more than maxChangeIn: something in the way, probably
};

/**
 * @brief Describe a MountFitStatus in a few words, for a status line
 */
const char* mountFitStatusText(MountFitStatus status);

/**
 * @brief One sensor's calibrated mount
 */
struct SensorMountFit {
    /** whether it was calibrated. Only a calibrated mount should be used */
    MountFitStatus status = MountFitStatus::noReadings;

    /**
     * the fitted mount when calibrated (along the beam always, sideways when lateralFitted), the
     * one given otherwise. Facing is never changed
     */
    DistanceSensorMount mount;

    /**
     * the fitted face's distance from the tracking center along the beam, in inches: the offset
     * that decides what the sensor reads from a wall it faces
     */
    double alongIn = 0.0;

    /**
     * the face's offset sideways from the beam through the tracking center, in inches, right of
     * the beam positive: the fitted one when lateralFitted, the configured one otherwise
     */
    double lateralIn = 0.0;

    /** how far alongIn moved from the configured mount, in inches. Positive is farther out */
    double alongChangeIn = 0.0;

    /** how far lateralIn moved from the configured mount, in inches. 0 when it was kept */
    double lateralChangeIn = 0.0;

    /** alongIn's standard error, in inches */
    double alongStderrIn = 0.0;

    /** the fitted sideways offset's standard error, in inches */
    double lateralStderrIn = 0.0;

    /**
     * whether the sideways offset was fitted (its standard error under maxStderrIn) rather than
     * kept. Every sensor shifted sideways the same way looks much like the robot facing a little
     * differently, so it's only pinned down when the walls the sensors see are at quite different
     * distances, and the configured one is the better guess otherwise
     */
    bool lateralFitted = false;

    /** how many of its readings matched a wall and were used */
    std::size_t readings = 0;

    /** those readings' root mean square error against the walls, in inches */
    double rmsErrorIn = 0.0;

    /**
     * which walls it saw at least minWallReadings times: bit 0 the left wall (FieldMap::minXIn()),
     * bit 1 the right (maxXIn()), bit 2 the near (minYIn()), bit 3 the far (maxYIn())
     */
    std::uint8_t walls = 0;
};

/**
 * @brief What calibrating the sensor mounts found
 */
struct MountCalibrationResult {
    /** whether at least one sensor was calibrated */
    bool ok = false;

    /** why none was, or why the calibration didn't run, when not ok. Empty when ok */
    const char* error = "";

    /** one per sensor, in the order the mounts were given */
    std::vector<SensorMountFit> sensors;

    /**
     * where the fit put the tracking center when the first reading was taken, in field
     * coordinates. On a square field it's only known up to a quarter turn about the middle, which
     * changes nothing about the mounts
     */
    odom::Pose start;

    /** how many readings there were */
    std::size_t readings = 0;

    /** how many matched a wall and were used */
    std::size_t readingsUsed = 0;
};

/**
 * @brief Find where each distance sensor really sits, from readings taken while the robot spun in
 * place
 *
 * As the robot turns, each sensor's face circles the tracking center, and what it reads from a wall
 * changes in a way that depends on where the robot is, which way it faces, and where the sensor
 * sits. This fits all of them at once against the map's four perimeter walls: the robot's position
 * and heading when the spin started, and each sensor's offset along its beam and sideways from it.
 * Odometry's own record of the spin supplies the turning and any drift of the tracking center. Only
 * readings that hit a wall nearly square on are used, and readings that match no wall (something
 * in the way) are discarded as the fit goes (iteratively reweighted least squares, Huber then
 * Tukey's biweight)
 *
 * The robot can start anywhere, facing any way. What it can't do is pin a sensor's offset from
 * walls on one side only: a robot an inch nearer the wall with the sensor an inch further back
 * reads the same. Two opposite walls pin it, since the field's width is known, so at least one
 * sensor has to see both walls of a pair during the spin. From the middle of the field, or anywhere
 * on the line halfway between two opposite walls, every sensor does: the V5 Distance Sensor reaches
 * 78 in, and walls are 70 in from the middle. A sensor that only ever saw walls on unpinned sides
 * is left as it was (MountFitStatus::noOppositeWalls)
 *
 * Facing isn't fitted: a sensor's facing and the robot's heading can't be told apart, and a degree
 * or two of facing changes a square-on reading by a fraction of a percent
 *
 * Pure (no PROS). MonteCarloLocalizer::calibrateSensorMounts() takes the readings and applies what
 * this finds
 *
 * @param samples the readings, any order
 * @param mounts each sensor's configured mount, in the order samples' sensor indices count
 * @param map the field. Only its four perimeter walls are used
 * @param beam the sensor model, for each reading's noise
 * @param config which readings to use, and what counts as calibrated
 * @return MountCalibrationResult each sensor's fitted mount
 *
 * @b Example
 * @code {.cpp}
 * using namespace sapphirelib::localization;
 * std::vector<MountSample> samples; // filled while spinning
 * const std::vector<DistanceSensorMount> mounts = {{.forwardIn = 7.0, .facingDeg = 0}};
 * MountCalibrationResult result =
 *     fitSensorMounts(samples, mounts, FieldMap::centered(), BeamModel{});
 * if (result.sensors[0].status == MountFitStatus::calibrated) {
 *     DistanceSensorMount front = result.sensors[0].mount;
 * }
 * @endcode
 */
MountCalibrationResult fitSensorMounts(std::span<const MountSample> samples,
                                       std::span<const DistanceSensorMount> mounts,
                                       const FieldMap& map, const BeamModel& beam,
                                       const MountCalibrationConfig& config = {});

} // namespace sapphirelib::localization
