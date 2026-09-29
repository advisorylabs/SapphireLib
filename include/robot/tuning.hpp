/**
 * \file robot/tuning.hpp
 *
 * What the PID tuning page tunes on this robot: the drivetrain's three
 * controllers, and the Auto-Tune axes their gains are designed from.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/gui/pid_tuner_page.hpp"

namespace robot {

/// Registers the forward, strafe, and turn Auto-Tune axes, the Drive, Turn,
/// and Hold controllers, and the driver stick-mode toggle. buildScreen()
/// calls this, after startTelemetry(): it creates the char.* telemetry
/// channels each axis's characterization runs are mirrored into (see
/// docs/TELEMETRY_FORMAT.md, "Refitting an axis offline").
void registerTuning(sapphirelib::gui::PidTunerPage& page);

} // namespace robot
