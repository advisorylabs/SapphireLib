#pragma once

#include <cstdint>

namespace sapphirelib::sensors {

/**
 * @brief What counts as a bump, for BumpDetector
 */
struct BumpConfig {
    /**
     * how far one reading must jump from the recent average to count, in g. 0.8 by default:
     * above what a VEX drivetrain's own starts and stops reach, below a hit from another robot
     */
    double thresholdG = 0.8;

    /**
     * how fast the average follows the readings, in milliseconds (its time constant). 150 by
     * default. Steady acceleration, gravity included, is soaked into the average within a few of
     * these, so only a sudden change stands out
     */
    double averageMs = 150.0;

    /**
     * after a bump, how long until another can count, in milliseconds. 500 by default, so one hit
     * that rings for a few readings is one bump
     */
    std::uint32_t holdoffMs = 500;
};

/**
 * @brief Notices jolts in an accelerometer's readings: a hit from another robot, or driving into
 * something
 *
 * Keeps a running average of the readings and reports a bump when one reading lands thresholdG or
 * more away from it, in any direction. Every axis counts, so it works however the IMU is mounted,
 * and gravity, sitting in the average, never does. A reading that isn't finite (an unplugged or
 * recalibrating IMU) starts the average over, as do calls more than 100ms apart
 *
 * @note not thread-safe. Call update() at a steady rate, 10-20ms
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::sensors::BumpDetector bumps;
 * // every 10ms
 * const pros::imu_accel_s_t accel = drivetrain().imu().getAccelG();
 * if (bumps.update(accel.x, accel.y, accel.z, now)) printf("bump: %.1fg\n", bumps.lastJoltG());
 * @endcode
 */
class BumpDetector {
public:
    /**
     * @brief Construct a new BumpDetector
     *
     * @param config bump settings
     */
    explicit BumpDetector(BumpConfig config = {});

    /**
     * @brief Take one reading
     *
     * @param xG x acceleration, in g
     * @param yG y acceleration, in g
     * @param zG z acceleration, in g
     * @param nowMs the current time, in milliseconds
     * @return true this reading is a bump
     */
    bool update(double xG, double yG, double zG, std::uint32_t nowMs);

    /**
     * @brief Get how far the last reading was from the average, in g. 0 with no average yet
     */
    double lastJoltG() const;

    /**
     * @brief Forget the average and any holdoff, as if just constructed
     */
    void reset();

    /**
     * @brief Get the bump settings
     */
    const BumpConfig& config() const;

private:
    BumpConfig config_;
    bool hasAverage_ = false;
    double averageG_[3] = {};
    std::uint32_t lastMs_ = 0;
    double lastJoltG_ = 0.0;
    bool holding_ = false;
    std::uint32_t bumpMs_ = 0;
};

} // namespace sapphirelib::sensors
