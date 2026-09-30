#pragma once

#include <cstdint>

#include "sapphirelib/util/timing.hpp"

namespace sapphirelib::mechanism {

/**
 * @brief When a roller counts as jammed, and how it clears itself
 */
struct JamConfig {
    /** whether anti-jam is on. false by default, which passes the command through */
    bool enabled = false;

    /** only commands at least this strong are watched, in volts. 4 by default */
    double minCommandVolts = 4.0;

    /** slower than this counts as not moving, in RPM. 5 by default */
    double stallRpm = 5.0;

    /**
     * how long it must be commanded but not moving to count as jammed, in milliseconds. 250 by
     * default. Must be longer than the roller takes to spin up
     */
    std::uint32_t stallMs = 250;

    /** reverse pulse voltage, opposite the command's direction. 6 by default */
    double reverseVolts = 6.0;

    /** how long the reverse pulse lasts, in milliseconds. 200 by default */
    std::uint32_t reverseMs = 200;
};

/**
 * @brief Decides each tick whether to send a roller its command or a reverse pulse
 *
 * After the roller has been commanded at minCommandVolts or more but turned slower than stallRpm
 * for stallMs, it reverses for reverseMs, then goes back to the command. A command that drops
 * below minCommandVolts or changes direction starts over (and ends a pulse), and so do calls more
 * than 100ms apart. An unplugged motor (infinite velocity) never counts as stalled
 *
 * @note not thread-safe. Call update() every tick
 */
class JamDetector {
public:
    /**
     * @brief Construct a new JamDetector
     *
     * @param config jam settings
     */
    explicit JamDetector(JamConfig config = {});

    /**
     * @brief Get the voltage to send this tick
     *
     * @param commandVolts the voltage asked for
     * @param velocityRpm the roller's speed, in RPM, either sign
     * @param nowMs the current time, in milliseconds
     * @return double the voltage to send
     */
    double update(double commandVolts, double velocityRpm, std::uint32_t nowMs);

    /**
     * @brief Whether a reverse pulse is running
     */
    bool reversing() const;

    /**
     * @brief Forget any pulse and the stall timer, as if the roller just stopped
     */
    void reset();

    /**
     * @brief Get the jam settings
     */
    const JamConfig& config() const;

private:
    double pulseVolts() const;

    JamConfig config_;
    GapDetector gap_;
    // commanded in direction_ but slower than stallRpm, and since when
    TimedFlag slow_;
    // running while a reverse pulse is
    Stopwatch pulse_;
    // the watched command's sign: +1, -1, or 0 below minCommandVolts
    int direction_ = 0;
};

} // namespace sapphirelib::mechanism
