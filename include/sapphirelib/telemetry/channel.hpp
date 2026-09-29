/**
 * \file sapphirelib/telemetry/channel.hpp
 *
 * A telemetry channel — one named stream of rows: one PID, one pose source,
 * one mechanism — and PidProbe, which feeds a PID's every step into one. Pure:
 * no PROS (timestamps come from sapphirelib::micros()), so it's unit-tested on
 * a desktop compiler with a fake clock (tests/telemetry/channel_test.cpp).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/telemetry/record.hpp"
#include "sapphirelib/telemetry/record_ring.hpp"

namespace sapphirelib::telemetry {

/// Per-channel settings, for designated initializers (`{.decimals = 3}`). The
/// defaults suit a 100Hz source.
struct ChannelOptions {
    /// Ring size in rows, rounded up to a power of two; 64 bytes each. It only
    /// has to cover the longest stretch the writer might go without draining:
    /// the default 256 is 2.5s of a 100Hz source, against a writer that drains
    /// every 20ms and an SD write that can stall for a couple of hundred.
    std::size_t capacity = 256;

    /// Digits after the decimal point in this channel's rows, 0-6, trailing
    /// zeros trimmed. Fixed-point rather than significant digits, so every
    /// value in a column resolves the same. Raise it for a channel whose
    /// values routinely sit below 0.001.
    int decimals = 4;
};

/// One named stream of telemetry rows. Made and owned by telemetry::Logger for
/// the life of the program; user code holds the reference Logger returns and
/// calls record() on it.
///
/// Any task may record into any channel, and recording never waits: a row that
/// can't be taken right now — the ring is full, or another task is mid-record
/// on this same channel — is dropped and counted, and the counts land in the
/// file's `D` rows. A control loop must never stall on logging, and a counted
/// gap is something a tuning app can see and step around.
///
/// One rule keeps that safe: record from tasks above the Logger's writer task
/// (priority 2) — every normal task is. The writer reclaims a channel whose
/// recorder seems to have died mid-record (see ProducerGate); a recorder at
/// priority 1-2 starved for 60ms+ could be mistaken for a dead one, which
/// costs at worst one torn row, never a crash.
class Channel {
public:
    /// `fileEpoch`, if given, is the owning Logger's count of files opened —
    /// see fileEpoch().
    Channel(ChannelSchema schema, std::size_t capacity,
            const std::atomic<std::uint32_t>* fileEpoch = nullptr);

    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    const ChannelSchema& schema() const;

    /// Logs one `S` row, values in column order, timestamped now. Doubles in,
    /// because that's what the rest of SapphireLib computes with (a braced list
    /// of doubles would be a narrowing error against float); stored as float.
    /// Missing values are logged as NaN, extras ignored. False if dropped — or
    /// if this is an events channel, which has no columns to fill.
    bool record(std::initializer_list<double> values);
    bool record(const double* values, std::size_t count);

    /// Logs an `E` row. `tag` is cut to kMaxEventTagChars (15) characters and
    /// `message` to kMaxEventMessageChars (191); a message longer than one
    /// Record's payload travels as several Records, committed together. The
    /// writer replaces anything unprintable, newlines included, with spaces.
    /// For printf-style messages see Logger::event().
    bool recordEvent(const char* tag, const char* message);

    /// Library use (PidProbe, Logger's own tasks): commits a filled-in Record,
    /// overwriting its tUs — a timestamp is only ever taken here, in one place,
    /// from one clock.
    bool commit(Record record);

    /// How many files the owning Logger has opened so far (0 before the first,
    /// and always 0 for a channel made without a Logger). PidProbe watches it
    /// to repeat a PID's config and gains at the top of every new file.
    std::uint32_t fileEpoch() const;

    // --- Writer task only --------------------------------------------------

    /// The oldest unwritten Record, or nullptr. Valid until pop().
    const Record* front();

    /// The Record `offset` places after front(), or nullptr — for gathering an
    /// event's continuations. Valid until pop().
    const Record* peek(std::size_t offset);

    /// Discards the oldest `count` Records.
    void pop(std::size_t count = 1);

    /// This channel's gate, for the writer's breakIfStuck() pass.
    ProducerGate& gate();

    /// Rows lost to a full ring / to another task recording on this channel at
    /// the same instant. Cumulative; safe from any task.
    std::uint32_t droppedFull() const;
    std::uint32_t droppedContended() const;

    /// See RecordRing::resyncs().
    std::uint32_t resyncs() const;

private:
    /// Commits `count` Records as one row (all or none), all stamped with the
    /// same time.
    bool commitAll(Record* records, std::size_t count);

    ChannelSchema schema_;
    /// Values an `S` row carries: the column count, or kPidValues for a pid
    /// channel (whose last column, flags, rides in Record::flags).
    std::size_t valueCount_;
    RecordRing ring_;
    ProducerGate gate_;
    const std::atomic<std::uint32_t>* fileEpoch_;
};

/// Feeds every step of one PID into a pid-kind Channel. Attach it with
/// PID::setObserver(), or let Logger::pid() do both.
///
/// Rows, all committed from the task running the PID:
///   - `C` and `G` ahead of the PID's first `S` in each file, so every file
///     stands on its own;
///   - `G` again on the first step after the gains change. PidTunerPage's +/-
///     buttons and Auto-Tune call setGains() from their own tasks; noticing the
///     change here, on the loop's own task, means neither ever touches the
///     channel — and neither needed to change;
///   - `S` for every update(), `R` for every reset() that cleared state.
class PidProbe final : public PidObserver {
public:
    PidProbe(Channel& channel, const PID& pid);

    void onPidUpdate(const PID& pid, const PidStep& step) override;
    void onPidReset(const PID& pid) override;

    /// Steps ignored because they came from a PID other than the one this probe
    /// was made for — a copy of it, which shares its observer pointer.
    std::uint32_t foreignSteps() const;

private:
    Channel& channel_;
    const PID* pid_;

    // Loop-task state. Only one task runs a given PID at a time (autonomous,
    // then opcontrol, then a tuner test — never two at once), so these need
    // no synchronization.
    std::uint32_t announcedEpoch_ = 0;
    bool configLogged_ = false;
    bool gainsLogged_ = false;
    PIDGains loggedGains_;
    std::atomic<std::uint32_t> foreignSteps_{0};
};

} // namespace sapphirelib::telemetry
