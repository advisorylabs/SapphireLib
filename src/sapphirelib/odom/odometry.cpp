#include "sapphirelib/odom/odometry.hpp"

#include "pros/rtos.hpp"
#include "sapphirelib/odom/odometry_math.hpp"
#include "sapphirelib/sensors/imu_scale_math.hpp"

namespace sapphirelib::odom {

Odometry::Odometry(Sensors sensors, OdometryConfig config, Pose startPose)
    : sensors_(sensors), config_(config), pose_(startPose) {
    // the start heading has to reach the IMU, or the first update() would overwrite it
    sensors_.imu->setHeadingDeg(startPose.headingDeg);
    pose_.lock()->headingDeg = sensors::wrapDegrees360(startPose.headingDeg);
    lastRotationDeg_ = sensors_.imu->getCumulativeHeadingDeg();
    if (sensors_.vertical) lastVerticalIn_ = sensors_.vertical->getDistanceIn();
    if (sensors_.horizontal) lastHorizontalIn_ = sensors_.horizontal->getDistanceIn();
}

void Odometry::update() {
    const std::uint32_t generation = poseGeneration_.load();
    if (generation != seenGeneration_) {
        // a setPose() since the last update: measure from the readings it took, so travel from
        // before it stays out of the new pose and travel since it still counts
        lastRotationDeg_ = resetRotationDeg_.load();
        if (sensors_.vertical) lastVerticalIn_ = resetVerticalIn_.load();
        if (sensors_.horizontal) lastHorizontalIn_ = resetHorizontalIn_.load();
        seenGeneration_ = generation;
    }
    const OdometryConfig config = *config_.lock();

    // read the rotation and offset separately, so the previous reading can be put in the current
    // frame. A setPose() between updates then shifts both ends equally instead of looking like a
    // turn
    const double rotationDeg = sensors_.imu->getCumulativeHeadingDeg();
    const double offsetDeg = sensors_.imu->headingOffsetDeg();
    const double headingDeg = sensors::fieldHeadingDeg(rotationDeg, offsetDeg);
    const double lastHeadingDeg = lastRotationDeg_ + offsetDeg;

    const double verticalIn =
        sensors_.vertical ? sensors_.vertical->getDistanceIn() : lastVerticalIn_;
    const double horizontalIn =
        sensors_.horizontal ? sensors_.horizontal->getDistanceIn() : lastHorizontalIn_;

    const PoseDelta delta = computeOdometryDelta(
        lastHeadingDeg, headingDeg, verticalIn - lastVerticalIn_, horizontalIn - lastHorizontalIn_,
        config.verticalOffsetIn, config.horizontalOffsetIn);

    lastRotationDeg_ = rotationDeg;
    lastVerticalIn_ = verticalIn;
    lastHorizontalIn_ = horizontalIn;

    pros::MutexVarLock<Pose> pose = pose_.lock();
    // a setPose() landed after this update read its sensors, so this delta belongs to the old
    // frame. The next update starts from the readings setPose() took
    if (poseGeneration_.load() != generation) return;
    pose->xIn += delta.dxIn;
    pose->yIn += delta.dyIn;
    pose->headingDeg = headingDeg;
}

Pose Odometry::getPose() const { return *pose_.lock(); }

void Odometry::setPose(Pose pose) {
    // the readings the next update() measures from, taken now
    resetRotationDeg_.store(sensors_.imu->getCumulativeHeadingDeg());
    if (sensors_.vertical) resetVerticalIn_.store(sensors_.vertical->getDistanceIn());
    if (sensors_.horizontal) resetHorizontalIn_.store(sensors_.horizontal->getDistanceIn());

    // baselines and IMU first, then the generation, then the pose: an update that read the old
    // offset sees the generation change and throws itself away
    sensors_.imu->setHeadingDeg(pose.headingDeg);
    poseGeneration_.fetch_add(1);
    pose.headingDeg = sensors::wrapDegrees360(pose.headingDeg);
    *pose_.lock() = pose;
}

OdometryConfig Odometry::getConfig() const { return *config_.lock(); }

void Odometry::setConfig(OdometryConfig config) { *config_.lock() = config; }

void Odometry::startTask(std::uint32_t periodMs) {
    // a second task would count every movement twice
    if (task_) return;
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
