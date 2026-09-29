#include "sapphirelib/odom/odometry.hpp"

#include "pros/rtos.hpp"
#include "sapphirelib/odom/odometry_math.hpp"
#include "sapphirelib/sensors/imu_scale_math.hpp"

namespace sapphirelib::odom {

Odometry::Odometry(Sensors sensors, OdometryConfig config, Pose startPose)
    : sensors_(sensors), config_(config), pose_(startPose) {
    // The start pose's heading has to reach the Imu, or the first update()
    // would overwrite it with wherever the IMU happens to read — see
    // setPose().
    sensors_.imu->setHeadingDeg(startPose.headingDeg);
    pose_.lock()->headingDeg = sensors::wrapDegrees360(startPose.headingDeg);
    lastRotationDeg_ = sensors_.imu->getCumulativeHeadingDeg();
    if (sensors_.vertical) lastVerticalIn_ = sensors_.vertical->getDistanceIn();
    if (sensors_.horizontal) lastHorizontalIn_ = sensors_.horizontal->getDistanceIn();
}

void Odometry::update() {
    const std::uint32_t generation = poseGeneration_.load();
    if (generation != seenGeneration_) {
        // A setPose() since the last update: measure from the readings it
        // took at the moment of the reset, so travel from before it stays out
        // of the new pose and travel since it still counts.
        lastRotationDeg_ = resetRotationDeg_.load();
        if (sensors_.vertical) lastVerticalIn_ = resetVerticalIn_.load();
        if (sensors_.horizontal) lastHorizontalIn_ = resetHorizontalIn_.load();
        seenGeneration_ = generation;
    }
    const OdometryConfig config = *config_.lock();

    // Rotation and offset read separately rather than through
    // getHeadingDeg(), so the previous reading can be put in the *current*
    // frame: after a setPose() between updates, both endpoints shift by the
    // same offset and the delta is only re-framed, never read as a turn
    // (which would fire the tracking wheels' arc correction for rotation that
    // didn't happen).
    const double rotationDeg = sensors_.imu->getCumulativeHeadingDeg();
    const double offsetDeg = sensors_.imu->headingOffsetDeg();
    const double headingDeg = sensors::fieldHeadingDeg(rotationDeg, offsetDeg);
    const double lastHeadingDeg = lastRotationDeg_ + offsetDeg;

    const double verticalIn = sensors_.vertical ? sensors_.vertical->getDistanceIn() : lastVerticalIn_;
    const double horizontalIn =
        sensors_.horizontal ? sensors_.horizontal->getDistanceIn() : lastHorizontalIn_;

    const PoseDelta delta = computeOdometryDelta(
        lastHeadingDeg, headingDeg, verticalIn - lastVerticalIn_, horizontalIn - lastHorizontalIn_,
        config.verticalOffsetIn, config.horizontalOffsetIn);

    lastRotationDeg_ = rotationDeg;
    lastVerticalIn_ = verticalIn;
    lastHorizontalIn_ = horizontalIn;

    pros::MutexVarLock<Pose> pose = pose_.lock();
    // A setPose() landed after this update read its sensors: its pose is the
    // truth, and this delta (and possibly this heading, if the offset was read
    // before the reset) belong to the old frame. The next update starts from
    // the readings that setPose() took.
    if (poseGeneration_.load() != generation) return;
    pose->xIn += delta.dxIn;
    pose->yIn += delta.dyIn;
    pose->headingDeg = headingDeg;
}

Pose Odometry::getPose() const { return *pose_.lock(); }

void Odometry::setPose(Pose pose) {
    // The readings the next update() measures from, taken at the moment of
    // the reset (reading these devices from this task is safe; only the
    // odometry task's own lastXxx_ fields aren't).
    resetRotationDeg_.store(sensors_.imu->getCumulativeHeadingDeg());
    if (sensors_.vertical) resetVerticalIn_.store(sensors_.vertical->getDistanceIn());
    if (sensors_.horizontal) resetHorizontalIn_.store(sensors_.horizontal->getDistanceIn());

    // Baselines and Imu first, then the generation, then the pose: an
    // update() that read the old offset sees the generation change and
    // discards itself, and one that starts after this adopts the baselines
    // above and reads the new offset.
    sensors_.imu->setHeadingDeg(pose.headingDeg);
    poseGeneration_.fetch_add(1);
    pose.headingDeg = sensors::wrapDegrees360(pose.headingDeg);
    *pose_.lock() = pose;
}

OdometryConfig Odometry::getConfig() const { return *config_.lock(); }

void Odometry::setConfig(OdometryConfig config) { *config_.lock() = config; }

void Odometry::startTask(std::uint32_t periodMs) {
    // A second task would integrate every wheel delta twice.
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
