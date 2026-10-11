#pragma once

#include <cstdint>

namespace sapphirelib::telemetry {

/**
 * @brief Why the logger is recording
 */
enum class RecordingReason : std::uint8_t {
    none,        // not recording
    startup,     // LoggerConfig::recordAtStart
    manual,      // Logger::startRecording(), e.g. the Home page's button
    competition, // competition control connected (LoggerConfig::recordUnderCompetition)
};

/**
 * @brief A request from Logger::startRecording() or stopRecording()
 */
enum class RecordingRequest : std::uint8_t { none, start, stop };

/**
 * @brief What RecordingPolicy::update() decided
 */
struct RecordingChange {
    /** a new recording starts now, in a new file */
    bool start = false;
    /** the recording ends now, and its file is closed */
    bool stop = false;
    /** why: the new recording's reason for a start, the ending one's for a stop */
    RecordingReason reason = RecordingReason::none;
};

/**
 * @brief When the logger records: a button outside matches, automatically in them
 *
 * - With recordAtStart, recording starts at the first update and runs until stopped.
 * - A start request starts a recording (if there isn't one); a stop request ends it.
 * - With recordUnderCompetition, connecting competition control (a field or a competition
 *   switch) starts a recording if there isn't one, and that recording ends once it's been
 *   disconnected for stopDelayMs. A recording that was already running when it connected
 *   (started by hand or at startup) is left for whoever started it to stop.
 * - A stop request during a match stops it for the rest of that connection.
 *
 * The delay keeps a match that loses its tether for a moment in one file.
 *
 * Pure, and one task owns it: the logger's sampler, which polls the competition status
 */
class RecordingPolicy {
public:
    /**
     * @brief Construct a new RecordingPolicy
     *
     * @param recordAtStart record from the first update
     * @param recordUnderCompetition record while competition control is connected
     * @param stopDelayMs how long competition control must stay disconnected before a recording
     * it started ends, in milliseconds
     */
    RecordingPolicy(bool recordAtStart, bool recordUnderCompetition, std::uint32_t stopDelayMs);

    /**
     * @brief Decide whether to start or stop recording. Call it every tick
     *
     * @param competitionConnected whether competition control is connected right now
     * @param request a pending start or stop request, applied before the competition rules
     * @param nowMs the time, in milliseconds
     * @return RecordingChange what changed. At most one of start and stop
     */
    RecordingChange update(bool competitionConnected, RecordingRequest request,
                           std::uint32_t nowMs);

    /**
     * @brief Whether a recording is running
     */
    bool recording() const;

    /**
     * @brief Why the running recording started. none when there isn't one
     */
    RecordingReason reason() const;

private:
    RecordingChange start(RecordingReason reason);
    RecordingChange stop();

    bool recordAtStart_;
    bool recordUnderCompetition_;
    std::uint32_t stopDelayMs_;

    bool started_ = false;
    bool recording_ = false;
    RecordingReason reason_ = RecordingReason::none;
    bool connected_ = false;
    // the running recording is the competition's to end
    bool heldByCompetition_ = false;
    bool disconnectPending_ = false;
    std::uint32_t disconnectedAtMs_ = 0;
};

/**
 * @brief Get a reason's name, as the "rec" telemetry events write it
 */
const char* recordingReasonName(RecordingReason reason);

} // namespace sapphirelib::telemetry
