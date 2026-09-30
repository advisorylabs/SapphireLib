#pragma once

#include "sapphirelib/telemetry/channel.hpp"
#include "sapphirelib/tuning/characterization_runner.hpp"

namespace sapphirelib::telemetry {

/**
 * @brief Log every sample of an Auto-Tune characterization run to a channel
 *
 * Wraps the config so each sample also logs (volts, position), the same pair the runner records,
 * so the analyzer can refit the axis from the log. Segments show up as bursts of rows with gaps
 * between them. A segment cut short by its travel limit ends with one extra 0V row. The original
 * callbacks still run, unchanged
 *
 * @param config the run's config. Returned unchanged if measure or actuate is empty
 * @param channel a channel with two columns, volts first, like {"volts", "pos"}
 * @return tuning::CharacterizationConfig the wrapped config
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::telemetry::Channel& turnLog = logger().channel("char.turn", {"volts", "pos"});
 * // in the axis factory passed to PidTunerPage::addAxis()
 * return sapphirelib::telemetry::tapCharacterization(turnExperiment(), turnLog);
 * @endcode
 */
tuning::CharacterizationConfig tapCharacterization(tuning::CharacterizationConfig config,
                                                   Channel& channel);

/**
 * @brief tapCharacterization() for a mechanism run
 *
 * Also logs held samples, with NaN volts, so the pre-roll before each segment is in the log, and a
 * segment cut short by its limit ends with one NaN row
 *
 * @param config the run's config
 * @param channel a channel with two columns, volts first
 * @return tuning::MechanismCharacterizationConfig the wrapped config
 */
tuning::MechanismCharacterizationConfig
tapMechanismCharacterization(tuning::MechanismCharacterizationConfig config, Channel& channel);

} // namespace sapphirelib::telemetry
