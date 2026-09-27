/**
 * \file robot_macros.hpp
 *
 * Temporary driver macros for the intake, claw, and lift — a stand-in until
 * these get rebuilt as proper SapphireLib subsystems. Robot-specific, so it
 * sits outside include/sapphirelib and is kept out of the library archive
 * (see EXCLUDE_SRC_FROM_LIB in the Makefile).
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

#include "pros/misc.hpp"

namespace robot_macros {

/// Call once from initialize(), with the lift all the way down: this zeroes
/// the lift's rotation sensor, and every level height is measured up from
/// there.
void initialize();

/// Call every opcontrol() loop tick. Reads the controller and drives the
/// intake, claw, and lift.
void update(pros::Controller& controller);

}  // namespace robot_macros
