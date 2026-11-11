/**
 * \file robot/alliance.hpp
 *
 * Which alliance the robot is on, for the side toggles (macros.hpp): picked
 * on the controller with UP and DOWN while the robot waits under competition
 * control (pickAutonWithController(), screen.hpp), next to the auton. Red
 * until picked, and red whenever no competition switch or field is
 * connected, so practice off the field is always red.
 *
 * Team 96671H: Hitmen
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "pros/misc.hpp"

namespace robot {

enum class Alliance : std::uint8_t { red, blue };

namespace detail {
// What the controller last picked; alliance() decides whether it counts.
inline std::atomic<Alliance> pickedAlliance{Alliance::red};
} // namespace detail

/// Picks the alliance, for while a competition switch or field is connected.
/// Safe from any task.
inline void setAlliance(Alliance alliance) { detail::pickedAlliance.store(alliance); }

/// The alliance picked on the controller while a competition switch or field
/// is connected, and red while none is. Kept until the program restarts, so
/// a pick carries on from disabled into autonomous and driver control. Safe
/// from any task.
inline Alliance alliance() {
    return pros::competition::is_connected() ? detail::pickedAlliance.load() : Alliance::red;
}

/// "RED" or "BLUE", for the controller and the logs.
inline const char* allianceName(Alliance alliance) {
    return alliance == Alliance::red ? "RED" : "BLUE";
}

} // namespace robot
