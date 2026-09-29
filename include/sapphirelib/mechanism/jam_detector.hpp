/**
 * \file sapphirelib/mechanism/jam_detector.hpp
 *
 * Anti-jam for an intake or any other roller: notices a motor that's being
 * told to spin but isn't turning, and answers with a short reverse pulse to
 * shake the piece loose before going back to what the driver asked for. The
 * decision is pure — velocity and "now" come in as arguments — so it's
 * host-tested (tests/mechanism/jam_detector_test.cpp); mechanism::Roller is
 * the PROS shell that feeds it a motor group's readings.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::mechanism {

/// When a roller counts as jammed, and how it tries to clear itself. At
/// namespace scope so it can be a `= {}` default argument (see
/// PositionConfig for why).
struct JamConfig {
    /// Off by default, and then JamDetector::update() just passes the
    /// command through untouched.
    bool enabled = false;

    /// Only a command at least this strong (either direction) is watched: a
    /// roller run slowly on purpose is supposed to be slow.
    double minCommandVolts = 4.0;

    /// Turning slower than this, in RPM either way, counts as not moving.
    double stallRpm = 5.0;

    /// How long it must be commanded yet not moving before it counts as
    /// jammed. Must be longer than the roller takes to spin up from rest (or
    /// to turn around), or every start looks like a jam.
    std::uint32_t stallMs = 250;

    /// The unjamming pulse, in volts (a positive number), applied opposite
    /// to the command's direction...
    double reverseVolts = 6.0;

    /// ...for this long, then back to the command.
    std::uint32_t reverseMs = 200;
};

/// Decides, tick by tick, what to actually send a roller: the command, or a
/// reverse pulse after the roller has been commanded at >= minCommandVolts
/// yet turned slower than stallRpm for stallMs. After the pulse it goes back
/// to the command and the stall timer starts over, so a piece that's still
/// stuck gets another pulse stallMs later.
///
/// The stall timer starts over whenever the command drops below
/// minCommandVolts or changes direction, since the roller then needs a fresh
/// spin-up. The same goes for a pulse already in progress: it ends at once,
/// because a driver who let go of the intake (or switched to outtaking) no
/// longer wants it reversing on its own. Calls more than 100ms apart also
/// start over, as timing across a stretch nobody was watching means nothing.
///
/// A non-finite velocity (PROS reports an unplugged motor as infinity)
/// never counts as stalled, so a missing motor can't trigger pulses.
///
/// Call update() every tick with the tick's one "now" (util/timing.hpp);
/// between calls it can't do anything. Not thread-safe.
class JamDetector {
public:
    explicit JamDetector(JamConfig config = {});

    /// The volts to actually apply this tick, for `commandVolts` requested
    /// and the roller turning at `velocityRpm` (either sign).
    double update(double commandVolts, double velocityRpm, std::uint32_t nowMs);

    /// In a reverse pulse right now.
    bool reversing() const;

    /// Forgets any pulse in progress and the stall timer, as if the roller had
    /// just been stopped. Roller::stop() calls this.
    void reset();

    const JamConfig& config() const;

private:
    double pulseVolts() const;

    JamConfig config_;
    GapDetector gap_;
    /// "Commanded in direction_, yet slower than stallRpm", and since when.
    TimedFlag slow_;
    /// Running while a reverse pulse is.
    Stopwatch pulse_;
    /// The watched command's sign: +1, -1, or 0 when it's below
    /// minCommandVolts.
    int direction_ = 0;
};

} // namespace sapphirelib::mechanism
