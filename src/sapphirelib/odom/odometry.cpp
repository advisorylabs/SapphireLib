#include "sapphirelib/odom/odometry.hpp"

#include "pros/rtos.hpp"
#include "sapphirelib/odom/odometry_math.hpp"

namespace sapphirelib::odom {

Odometry::Odometry(Sensors sensors, OdometryConfig config, Pose startPose)
    : sensors_(sensors), config_(config), pose_(startPose), lastHeadingDeg_(sensors_.imu->get_heading()) {
    if (sensors_.vertical) lastVerticalIn_ = sensors_.vertical->getDistanceIn();
    if (sensors_.horizontal) lastHorizontalIn_ = sensors_.horizontal->getDistanceIn();
}

void Odometry::update() {
    const double headingDeg = sensors_.imu->get_heading();
    const double verticalIn = sensors_.vertical ? sensors_.vertical->getDistanceIn() : lastVerticalIn_;
    const double horizontalIn =
        sensors_.horizontal ? sensors_.horizontal->getDistanceIn() : lastHorizontalIn_;

    const PoseDelta delta = computeOdometryDelta(
        lastHeadingDeg_, headingDeg, verticalIn - lastVerticalIn_, horizontalIn - lastHorizontalIn_,
        config_.verticalOffsetIn, config_.horizontalOffsetIn);

    lastHeadingDeg_ = headingDeg;
    lastVerticalIn_ = verticalIn;
    lastHorizontalIn_ = horizontalIn;

    pros::MutexVarLock<Pose> pose = pose_.lock();
    pose->xIn += delta.dxIn;
    pose->yIn += delta.dyIn;
    pose->headingDeg = headingDeg;
}

Pose Odometry::getPose() const { return *pose_.lock(); }

void Odometry::setPose(Pose pose) { *pose_.lock() = pose; }

void Odometry::startTask(std::uint32_t periodMs) {
    task_ = std::make_unique<pros::Task>(
        [this, periodMs] {
            while (true) {
                update();
                pros::delay(periodMs);
            }
        },
        "Odometry");
}

} // namespace sapphirelib::odom
