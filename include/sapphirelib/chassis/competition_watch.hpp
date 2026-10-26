#pragma once

#include <atomic>
#include <cstdint>

#include "pros/misc.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"

namespace sapphirelib::chassis {

static_assert(motion::kCompetitionDisabled == COMPETITION_DISABLED &&
              motion::kCompetitionAutonomous == COMPETITION_AUTONOMOUS &&
              motion::kCompetitionConnected == COMPETITION_CONNECTED);

/**
 * @brief The competition mode a motion started in, so the motion can stop if it changes, or if
 * something cancels it
 *
 * Every drivetrain motion keeps one and ends with ExitReason::interrupted once
 * motion::competitionModeChanged() says so, or once cancelMotions() is called after it started
 */
class CompetitionWatch {
public:
    /**
     * @brief Construct a new CompetitionWatch, taking the status now
     */
    CompetitionWatch() : start_(pros::competition::get_status()), cancels_(cancelCount().load()) {}

    /**
     * @brief Whether the mode has changed since construction, under competition control, or
     * cancelMotions() has been called since
     */
    bool changed() const {
        return motion::competitionModeChanged(start_, pros::competition::get_status()) ||
               cancelCount().load() != cancels_;
    }

    /**
     * @brief End every motion running now, as a competition mode change would. Safe from any task
     *
     * Motions started after the call aren't affected
     */
    static void cancelMotions() { cancelCount().fetch_add(1); }

private:
    // how many times cancelMotions() has been called
    static std::atomic<std::uint32_t>& cancelCount() {
        static std::atomic<std::uint32_t> count{0};
        return count;
    }

    std::uint8_t start_;
    std::uint32_t cancels_;
};

} // namespace sapphirelib::chassis
