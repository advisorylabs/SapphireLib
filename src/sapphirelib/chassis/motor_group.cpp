#include "sapphirelib/chassis/motor_group.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

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

// mean of the finite values, or fallback if there are none. An unplugged motor reads infinity,
// which would poison the average
double finiteMean(const std::vector<double>& values, double fallback, bool* anyFinite = nullptr) {
    double sum = 0.0;
    int count = 0;
    for (const double value : values) {
        if (!std::isfinite(value)) continue;
        sum += value;
        ++count;
    }
    if (anyFinite != nullptr) *anyFinite = count > 0;
    return count > 0 ? sum / count : fallback;
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
    lastPositionDeg_.store(0.0);
}

double MotorGroup::getPositionDegrees() const {
    bool anyAnswered = false;
    const double degrees =
        finiteMean(motors_.get_position_all(), lastPositionDeg_.load(), &anyAnswered);
    if (anyAnswered) lastPositionDeg_.store(degrees);
    return degrees;
}

double MotorGroup::getVelocityRPM() const {
    // nothing answering reads as not moving. Unlike position, 0 is the honest answer here
    return finiteMean(motors_.get_actual_velocity_all(), 0.0);
}

double MotorGroup::getTemperatureC() const {
    double hottest = 0.0;
    for (const double tempC : motors_.get_temperature_all()) {
        if (std::isfinite(tempC)) hottest = std::max(hottest, tempC);
    }
    return hottest;
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
