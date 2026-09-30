#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/telemetry/record.hpp"
#include "sapphirelib/telemetry/record_ring.hpp"

namespace sapphirelib::telemetry {

/**
 * @brief Settings for one telemetry channel. The defaults suit a 100Hz source
 */
struct ChannelOptions {
    /**
     * ring size in rows, rounded up to a power of two. 64 bytes each. 256 by default, 2.5 seconds
     * of a 100Hz source, which covers a slow SD write
     */
    std::size_t capacity = 256;

    /** digits after the decimal point, 0-6, trailing zeros trimmed. 4 by default */
    int decimals = 4;
};

/**
 * @brief One named stream of telemetry rows, like one PID, one pose, or one mechanism
 *
 * Made and owned by telemetry::Logger; keep the reference it returns and call record() on it.
 * Recording never waits: a row that can't be taken right now (the ring is full, or another task is
 * recording on the same channel) is dropped and counted in the file
 *
 * @note record from tasks above the logger's writer (priority 2), which every normal task is
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::telemetry::Channel& intakeLog = logger.channel("intake", {"volts", "rpm"});
 * // anywhere, any task
 * intakeLog.record({intakeVolts, intake.get_actual_velocity()});
 * @endcode
 */
class Channel {
public:
    /**
     * @brief Construct a new Channel. Logger does this
     *
     * @param schema the channel's name, kind, and columns
     * @param capacity ring size in rows
     * @param fileEpoch the owning Logger's count of files opened, see fileEpoch()
     */
    Channel(ChannelSchema schema, std::size_t capacity,
            const std::atomic<std::uint32_t>* fileEpoch = nullptr);

    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    /**
     * @brief Get the channel's name, kind, and columns
     */
    const ChannelSchema& schema() const;

    /**
     * @brief Log one row of values, timestamped now
     *
     * Values are in column order and stored as floats. Missing values are logged as NaN, extras
     * are ignored
     *
     * @param values the values
     * @return true the row was logged
     * @return false it was dropped, or this is an events channel
     */
    bool record(std::initializer_list<double> values);
    bool record(const double* values, std::size_t count);

    /**
     * @brief Log an event. For printf style messages, see Logger::event()
     *
     * @param tag the event tag, cut to 15 characters
     * @param message the message, cut to 191 characters. Unprintable characters become spaces
     * @return true the event was logged
     */
    bool recordEvent(const char* tag, const char* message);

    /**
     * @brief Commit a filled-in Record, stamping its time. For library use
     */
    bool commit(Record record);

    /**
     * @brief Get how many files the owning Logger has opened. 0 without a Logger
     *
     * PidProbe watches it to repeat a PID's config and gains at the top of every file
     */
    std::uint32_t fileEpoch() const;

    // writer task only

    /**
     * @brief Get the oldest unwritten Record, or nullptr. Valid until pop()
     */
    const Record* front();

    /**
     * @brief Get the Record `offset` places after front(), or nullptr. Valid until pop()
     */
    const Record* peek(std::size_t offset);

    /**
     * @brief Discard the oldest `count` Records
     */
    void pop(std::size_t count = 1);

    /**
     * @brief Get this channel's gate, for the writer's breakIfStuck() pass
     */
    ProducerGate& gate();

    /**
     * @brief Get the rows lost to a full ring. Safe from any task
     */
    std::uint32_t droppedFull() const;

    /**
     * @brief Get the rows lost to another task recording at the same moment. Safe from any task
     */
    std::uint32_t droppedContended() const;

    /**
     * @brief See RecordRing::resyncs()
     */
    std::uint32_t resyncs() const;

private:
    // commit count Records as one row (all or none), all with the same time
    bool commitAll(Record* records, std::size_t count);

    ChannelSchema schema_;
    // values an S row carries: the column count, or kPidValues for a pid channel (flags rides in
    // Record::flags)
    std::size_t valueCount_;
    RecordRing ring_;
    ProducerGate gate_;
    const std::atomic<std::uint32_t>* fileEpoch_;
};

/**
 * @brief Feeds every step of one PID into a pid channel
 *
 * Attach it with PID::setObserver(), or let Logger::pid() do both. It logs the PID's config and
 * gains at the top of each file, the gains again whenever they change, a row for every update(),
 * and one for every reset() that cleared state. All from the task running the PID
 */
class PidProbe final : public PidObserver {
public:
    /**
     * @brief Construct a new PidProbe
     *
     * @param channel the pid channel to log into
     * @param pid the PID it's for
     */
    PidProbe(Channel& channel, const PID& pid);

    void onPidUpdate(const PID& pid, const PidStep& step) override;
    void onPidReset(const PID& pid) override;

    /**
     * @brief Get the steps ignored because they came from a copy of the PID, which shares its
     * observer
     */
    std::uint32_t foreignSteps() const;

private:
    Channel& channel_;
    const PID* pid_;

    // loop task state. Only one task runs a PID at a time, so no synchronization is needed
    std::uint32_t announcedEpoch_ = 0;
    bool configLogged_ = false;
    bool gainsLogged_ = false;
    PIDGains loggedGains_;
    std::atomic<std::uint32_t> foreignSteps_{0};
};

} // namespace sapphirelib::telemetry
