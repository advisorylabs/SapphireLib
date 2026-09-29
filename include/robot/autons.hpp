/**
 * \file robot/autons.hpp
 *
 * The robot's autonomous routines. Each one, and the list that puts it on the
 * brain screen, lives in autons.cpp, so adding a routine touches one file.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include "sapphirelib/gui/auton_selector_page.hpp"

namespace robot {

/// Adds every routine to the Auton tab, in the order they're listed there.
/// The first one is selected by default and runs if nobody picks one, so it
/// stays harmless. buildScreen() calls this.
void registerAutons(sapphirelib::gui::AutonSelectorPage& selector);

} // namespace robot
