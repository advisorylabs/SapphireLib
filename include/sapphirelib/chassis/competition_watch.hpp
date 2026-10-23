#pragma once

#include <cstdint>

#include "pros/misc.hpp"
#include "sapphirelib/motion/exit_tracker.hpp"

namespace sapphirelib::chassis {

static_assert(motion::kCompetitionDisabled == COMPETITION_DISABLED &&
              motion::kCompetitionAutonomous == COMPETITION_AUTONOMOUS &&
              motion::kCompetitionConnected == COMPETITION_CONNECTED);

/**
 * @brief The competition mode a motion started in, so the motion can stop if it changes
 *
 * Every drivetrain motion keeps one and ends with ExitReason::interrupted once
 * motion::competitionModeChanged() says so
 */
class CompetitionWatch {
public:
    /**
     * @brief Construct a new CompetitionWatch, taking the status now
     */
    CompetitionWatch() : start_(pros::competition::get_status()) {}

    /**
     * @brief Whether the mode has changed since construction, under competition control
     */
    bool changed() const {
        return motion::competitionModeChanged(start_, pros::competition::get_status());
    }

private:
    std::uint8_t start_;
};

} // namespace sapphirelib::chassis
