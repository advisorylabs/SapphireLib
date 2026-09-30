#pragma once

namespace sapphirelib::odom {

/**
 * @brief Something that measures distance along one axis, for odometry
 *
 * See RotationTrackingWheel and MotorGroupTrackingWheel
 */
class TrackingWheel {
public:
    virtual ~TrackingWheel() = default;

    /**
     * @brief Get the distance traveled since construction or the last reset()
     *
     * @return double distance, in inches
     */
    virtual double getDistanceIn() const = 0;

    /**
     * @brief Zero the distance
     */
    virtual void reset() = 0;
};

} // namespace sapphirelib::odom
