#include "sapphirelib/telemetry/recording_policy.hpp"

namespace sapphirelib::telemetry {

RecordingPolicy::RecordingPolicy(bool recordAtStart, bool recordUnderCompetition,
                                 std::uint32_t stopDelayMs)
    : recordAtStart_(recordAtStart), recordUnderCompetition_(recordUnderCompetition),
      stopDelayMs_(stopDelayMs) {}

RecordingChange RecordingPolicy::update(bool competitionConnected, RecordingRequest request,
                                        std::uint32_t nowMs) {
    RecordingChange change;
    if (!started_) {
        started_ = true;
        if (recordAtStart_) change = start(RecordingReason::startup);
    }

    // a request wins over the competition rules this tick
    if (request == RecordingRequest::start && !recording_) {
        change = start(RecordingReason::manual);
    } else if (request == RecordingRequest::stop && recording_) {
        change = stop();
    }

    const bool connecting = competitionConnected && !connected_;
    const bool disconnecting = !competitionConnected && connected_;
    connected_ = competitionConnected;

    if (connecting) {
        disconnectPending_ = false;
        // a request this tick already decided; otherwise start one for the match
        if (recordUnderCompetition_ && !recording_ && request == RecordingRequest::none) {
            change = start(RecordingReason::competition);
            heldByCompetition_ = true;
        }
    } else if (disconnecting && heldByCompetition_) {
        disconnectPending_ = true;
        disconnectedAtMs_ = nowMs;
    }

    if (disconnectPending_ && !competitionConnected && recording_ &&
        nowMs - disconnectedAtMs_ >= stopDelayMs_) {
        change = stop();
    }
    return change;
}

bool RecordingPolicy::recording() const { return recording_; }

RecordingReason RecordingPolicy::reason() const { return reason_; }

RecordingChange RecordingPolicy::start(RecordingReason reason) {
    recording_ = true;
    reason_ = reason;
    heldByCompetition_ = false;
    disconnectPending_ = false;
    return RecordingChange{.start = true, .reason = reason};
}

RecordingChange RecordingPolicy::stop() {
    const RecordingReason ending = reason_;
    recording_ = false;
    reason_ = RecordingReason::none;
    heldByCompetition_ = false;
    disconnectPending_ = false;
    return RecordingChange{.stop = true, .reason = ending};
}

const char* recordingReasonName(RecordingReason reason) {
    switch (reason) {
        case RecordingReason::startup: return "startup";
        case RecordingReason::manual: return "manual";
        case RecordingReason::competition: return "competition";
        default: return "none";
    }
}

} // namespace sapphirelib::telemetry
