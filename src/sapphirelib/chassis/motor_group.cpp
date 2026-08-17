#include "sapphirelib/chassis/motor_group.hpp"

#include <algorithm>
#include <numeric>

namespace sapphirelib::chassis {

namespace {

pros::MotorGears toProsGearset(Gearset gearset) {
    switch (gearset) {
        case Gearset::red: return pros::MotorGears::red;
        case Gearset::green: return pros::MotorGears::green;
        case Gearset::blue: return pros::MotorGears::blue;
    }
    return pros::MotorGears::green;
}

pros::motor_brake_mode_e_t toProsBrakeMode(BrakeMode mode) {
    switch (mode) {
        case BrakeMode::coast: return pros::E_MOTOR_BRAKE_COAST;
        case BrakeMode::brake: return pros::E_MOTOR_BRAKE_BRAKE;
        case BrakeMode::hold: return pros::E_MOTOR_BRAKE_HOLD;
    }
    return pros::E_MOTOR_BRAKE_COAST;
}

} // namespace

MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports, Gearset gearset)
    : motors_(ports, toProsGearset(gearset)), gearset_(gearset) {}

void MotorGroup::moveVoltage(double volts) {
    volts = std::clamp(volts, -12.0, 12.0);
    motors_.move_voltage(static_cast<std::int32_t>(volts * 1000.0));
}

void MotorGroup::moveVelocity(double rpm) {
    rpm = std::clamp(rpm, -maxRPM(), maxRPM());
    motors_.move_velocity(static_cast<std::int32_t>(rpm));
}

void MotorGroup::brake() {
    motors_.brake();
}

void MotorGroup::setBrakeMode(BrakeMode mode) {
    motors_.set_brake_mode_all(toProsBrakeMode(mode));
}

void MotorGroup::tarePosition() {
    motors_.tare_position_all();
}

double MotorGroup::getPositionDegrees() const {
    const auto positions = motors_.get_position_all();
    if (positions.empty()) return 0.0;
    return std::accumulate(positions.begin(), positions.end(), 0.0) /
           static_cast<double>(positions.size());
}

double MotorGroup::getVelocityRPM() const {
    const auto velocities = motors_.get_actual_velocity_all();
    if (velocities.empty()) return 0.0;
    return std::accumulate(velocities.begin(), velocities.end(), 0.0) /
           static_cast<double>(velocities.size());
}

Gearset MotorGroup::gearset() const {
    return gearset_;
}

double MotorGroup::maxRPM() const {
    switch (gearset_) {
        case Gearset::red: return 100.0;
        case Gearset::green: return 200.0;
        case Gearset::blue: return 600.0;
    }
    return 200.0;
}

} // namespace sapphirelib::chassis
