/**
 * \file robot/tuning.hpp
 *
 * What the PID tuning page tunes on this robot: the drivetrain's three
 * controllers and the lift's, and the Auto-Tune axes their gains are designed
 * from.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/gui/pid_tuner_page.hpp"

namespace robot {

/// Registers the forward, strafe, turn and lift Auto-Tune axes, the Drive,
/// Turn, Hold and Lift controllers, and the driver stick-mode toggle.
/// buildScreen() calls this, after startTelemetry(): it creates the char.*
/// telemetry channels each axis's characterization runs are mirrored into
/// (see docs/TELEMETRY_FORMAT.md, "Refitting an axis offline").
void registerTuning(sapphirelib::gui::PidTunerPage& page);

} // namespace robot
