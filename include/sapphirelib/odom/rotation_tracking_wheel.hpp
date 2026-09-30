#pragma once

#include <atomic>
#include <cstdint>

#include "pros/rotation.hpp"
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

private:
    pros::Rotation rotation_;
    double wheelDiameterIn_;
    double externalGearRatio_;

    // the last good reading, in centidegrees. Atomic since any reading task updates it
    mutable std::atomic<std::int32_t> lastGoodCentidegrees_{0};
};

} // namespace sapphirelib::odom
