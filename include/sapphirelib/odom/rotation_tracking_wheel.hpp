#pragma once

#include <atomic>
#include <cstdint>

#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "sapphirelib/odom/tracking_wheel.hpp"

namespace sapphirelib::odom {

/**
 * @brief A tracking wheel on a V5 rotation sensor
 *
 * @b Example
 * @code {.cpp}
 * // a 2" tracking wheel on port 11, reversed
 * sapphirelib::odom::RotationTrackingWheel vertical(-11, 2.0);
 * @endcode
 */
class RotationTrackingWheel : public TrackingWheel {
public:
    /**
     * @brief Construct a new RotationTrackingWheel
     *
     * @param port rotation sensor port. Negative reverses it
     * @param wheelDiameterIn tracking wheel diameter, in inches
     * @param externalGearRatio sensor rotations per wheel rotation. 1 by default
     */
    RotationTrackingWheel(std::int8_t port, double wheelDiameterIn, double externalGearRatio = 1.0);

    /**
     * @brief Get the distance traveled
     *
     * While the sensor isn't answering, this holds the last good reading. Safe to call from
     * several tasks
     *
     * @return double distance, in inches
     */
    double getDistanceIn() const override;
    void reset() override;

    /**
     * @brief Get the wheel diameter it measures with. Thread-safe
     *
     * @return double diameter, in inches
     */
    double wheelDiameterIn() const;

    /**
     * @brief Change the wheel diameter, e.g. after calibrating it. Thread-safe
     *
     * The distance carries on from what it reads now, measuring with the new diameter from here, so
     * odometry reading it sees no jump. See calibrateWheelDiameterIn()
     *
     * @param wheelDiameterIn the new diameter, in inches
     */
    void setWheelDiameterIn(double wheelDiameterIn);

private:
    // the diameter, and the reading it measures from: setWheelDiameterIn() moves the base up to the
    // reading it's set at, so getDistanceIn() is baseIn plus the travel since baseCentidegrees
    struct Scale {
        double wheelDiameterIn;
        double baseIn = 0.0;
        std::int32_t baseCentidegrees = 0;
    };

    // the sensor's position, or the last good one while it isn't answering
    std::int32_t readCentidegrees() const;
    double inchesFor(std::int32_t centidegrees, double wheelDiameterIn) const;

    pros::Rotation rotation_;
    double externalGearRatio_;
    mutable pros::MutexVar<Scale> scale_;

    // the last good reading, in centidegrees. Atomic since any reading task updates it
    mutable std::atomic<std::int32_t> lastGoodCentidegrees_{0};
};

} // namespace sapphirelib::odom
