#pragma once

#include "sapphirelib/chassis/drivetrain_config.hpp"
#include "sapphirelib/motion/trajectory.hpp"

namespace sapphirelib::motion {

/**
 * @brief One value per axis of a holonomic chassis, in the robot's frame
 *
 * Forward and strafe in inches (strafe positive to the robot's right), turn in degrees (clockwise
 * positive), per second or per second squared as the use says
 */
struct AxisMotion {
    /** forward/backward */
    double forward = 0.0;
    /** sideways, positive to the right */
    double strafe = 0.0;
    /** rotation, clockwise positive */
    double turn = 0.0;
};

/**
 * @brief Split static friction between the axes for a direction of motion
 *
 * Each axis model's kS is measured moving that axis alone. Moving several at once, the chassis
 * breaks friction once, not once per axis, so each axis gets its share: kS times its part of the
 * direction, measured in fractions of each axis's top speed. Moving along one axis gives that
 * axis its whole kS, and an axis barely moving gets almost none, instead of jumping to kS the
 * moment its speed leaves 0
 *
 * @param models the measured axis models. An axis without a valid model gets no friction
 * @param direction the direction of motion, per axis (a velocity, or an acceleration from rest)
 * @return chassis::AxisVolts friction volts per axis, with the direction's signs. All zero for no
 * direction
 */
chassis::AxisVolts frictionVolts(const chassis::HolonomicAxisModels& models, AxisMotion direction);

/**
 * @brief Volts per axis, before wheel mixing, to move a holonomic chassis at a velocity and
 * acceleration
 *
 * kV * velocity + kA * acceleration per axis, plus frictionVolts() in the direction of the
 * velocity, or of the acceleration when starting from rest
 *
 * @param models the measured axis models
 * @param velocity robot-frame velocity: in/s, in/s, deg/s
 * @param acceleration robot-frame acceleration: in/s^2, in/s^2, deg/s^2
 * @return chassis::AxisVolts forward, strafe and turn volts. Not clamped to 12V
 */
chassis::AxisVolts holonomicFeedforward(const chassis::HolonomicAxisModels& models,
                                        AxisMotion velocity, AxisMotion acceleration);

/**
 * @brief Rotate a robot-frame vector for a robot turned from the planned heading
 *
 * The same field vector, seen from a robot whose heading is headingOffsetDeg clockwise of the
 * one the vector was measured from. The turn component is unchanged
 *
 * @param motion the vector in the planned robot frame
 * @param headingOffsetDeg actual heading minus planned heading, in degrees
 * @return AxisMotion the vector in the actual robot frame
 */
AxisMotion rotateAxisMotion(AxisMotion motion, double headingOffsetDeg);

/**
 * @brief Volts per axis to drive a trajectory's planned motion
 *
 * holonomicFeedforward() for the sample's velocity and acceleration, except that static friction
 * always follows the path's direction of travel, which is defined even where the plan is at rest.
 * This is the feedforward generateTrajectory() budgets for and followTrajectory() drives with
 *
 * @param models the measured axis models
 * @param sample the planned state
 * @param headingOffsetDeg how far the robot's heading is clockwise of the plan's, in degrees, so
 * the volts push the planned way on the field. 0 by default
 * @return chassis::AxisVolts forward, strafe and turn volts. Not clamped to 12V
 */
chassis::AxisVolts trajectoryFeedforward(const chassis::HolonomicAxisModels& models,
                                         const TrajectorySample& sample,
                                         double headingOffsetDeg = 0.0);

} // namespace sapphirelib::motion
