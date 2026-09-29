/**
 * \file robot/macros.hpp
 *
 * Driver macros for the intake, claw, and lift, built on SapphireLib's
 * input/, mechanism/, and util/ primitives. Robot-specific, so it sits
 * outside include/sapphirelib and is kept out of the library archive (see
 * EXCLUDE_SRC_FROM_LIB in the Makefile).
 *
 * Controls:
 *   R1 (hold)  Intake: spins the intake and claw in, and resets the lift to
 *              stowed (claw piston retracted, lift at level 0).
 *   Y (hold)   Claw-only intake: spins just the claw in (intake idle), with
 *              the claw piston deployed and the lift halfway between levels
 *              0 and 1. On release, lowers to level 0, re-seats the piece by
 *              retracting the piston briefly and redeploying it, then leaves
 *              the claw to hold the piece on its own.
 *   R2         Score. At level 0: deploys the claw piston if needed, then
 *              outtakes the claw and intake for as long as R2 is held,
 *              starting once the piston has had time to deploy. Above
 *              level 0: dips halfway to the level below, outtakes the claw
 *              for a moment, then raises the lift one level.
 *   L1         Up one notch. The first press only deploys the claw piston
 *              (level 0); after that, each press raises the lift a level.
 *   L2         Down one notch, stopping at level 0.
 *   RIGHT      Cycle scoring mode (ALLIANCE, MEDIUM, CENTER), each with its
 *              own level heights and level count.
 *   LEFT       Toggle the controller's bottom two lines between the claw and
 *              lift position readout and the lift motors' volts, current,
 *              and temperature.
 *
 * The claw also spins in on its own whenever its distance sensor sees a
 * piece, to keep hold of it. The lift holds each level on a PID loop around
 * its rotation sensor.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/input/controller.hpp"

namespace sapphirelib::telemetry {
class Logger;
} // namespace sapphirelib::telemetry

namespace robot::macros {

/// Call once from initialize(), with the lift all the way down: this zeroes
/// the lift's rotation sensor, and every level height is measured up from
/// there. (initDevices() does.)
void initialize();

/// Call every opcontrol() loop tick, right after controller.update(). Reads
/// the controller and drives the intake, claw, and lift, then sends the
/// tick's status line to the controller screen (at most one write).
void update(sapphirelib::input::Controller& controller);

/// Stops the claw and intake and brakes the lift, right away. For disabled(),
/// and for just before something runs a blocking routine from opcontrol()
/// (driver.cpp's B+DOWN): nothing calls update() until that routine ends, and
/// every motor would otherwise keep its last command for the whole routine —
/// a lift climbing at 12V would stay at 12V into its hard stop.
///
/// Nothing else changes: the claw's deploy state, the lift's level and the
/// scoring mode are kept, and the next update() commands everything from
/// them again — the lift back on its PID loop at its current level. (After
/// a pause long enough to count as a restart, update() also drops any
/// half-finished score or re-seat, as it always has.)
void stop();

/// Logs the lift to `logger`: its PID's every step (channel "lift") and
/// what each update actually commanded (channel "lift.act": target and
/// position in degrees, the volts sent after gravity feedforward and the
/// clamp, and which branch of the control law ran — see
/// docs/TELEMETRY_FORMAT.md). Call from initialize(), before opcontrol can
/// run: it attaches the PID's observer and the lift's step listener, neither
/// of which is synchronized with update(). Without it, nothing is logged and
/// nothing else changes.
void attachTelemetry(sapphirelib::telemetry::Logger& logger);

} // namespace robot::macros
