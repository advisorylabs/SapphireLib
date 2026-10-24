#pragma once

#include <cstddef>
#include <vector>

#include "sapphirelib/input/button_tracker.hpp"

namespace sapphirelib::input {

/**
 * @brief The buttons a driver held, from one moment on
 */
struct TimedButtons {
    /** when, in seconds, on whatever clock plays them back */
    double tS = 0.0;
    /** a bit for every button held from then on (see maskOf()) */
    ButtonMask buttons = 0;
};

/**
 * @brief Plays recorded buttons back a tick at a time, on a clock that may pause or run unevenly
 *
 * Each advance() moves on to the buttons recorded for that moment, but never past a change it
 * hasn't played yet: a state the driver held for a single tick still gets a tick of its own (a
 * tick late if the clock jumped over it), so no press or release is lost, and code that reacts
 * to presses (ButtonTracker's edges, the driver macros) sees the same sequence the driver made.
 * It catches up a change a tick, so it never falls far behind. Pure: no PROS calls
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::input::ButtonPlayback buttons(plan.buttons); // from trajectoryFromRecording()
 * while (queue.isBusy()) {
 *     playback.replay(buttons.advance(drivetrain().trajectoryProgress().timeS));
 *     macros::update(playback);
 *     pros::delay(20);
 * }
 * @endcode
 */
class ButtonPlayback {
public:
    /**
     * @brief Construct a new ButtonPlayback
     *
     * @param changes the buttons from each moment they changed, in time order. Before the first,
     * nothing is held
     */
    explicit ButtonPlayback(std::vector<TimedButtons> changes);

    /**
     * @brief Move on to the buttons at a moment, one change at most
     *
     * @param clockS the playback clock, in seconds. A clock that goes backward holds still
     * @return ButtonMask the buttons to play this tick
     */
    ButtonMask advance(double clockS);

    /**
     * @brief Get the buttons the latest advance() played
     */
    ButtonMask current() const;

    /**
     * @brief Whether every change has been played
     */
    bool finished() const;

private:
    std::vector<TimedButtons> changes_;
    // the next change to play
    std::size_t next_ = 0;
    ButtonMask current_ = 0;
};

} // namespace sapphirelib::input
