/**
 * \file sapphirelib/telemetry/record_ring.hpp
 *
 * The lock-free hand-off between telemetry producers (control loops) and the
 * SD writer task: a bounded ring of Records, and the try-lock gate that
 * serializes producers onto it. Pure — std::atomic only, no PROS — so it's
 * unit-tested on a desktop compiler (tests/telemetry/record_ring_test.cpp).
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "sapphirelib/telemetry/record.hpp"

namespace sapphirelib::telemetry {

// Everything below leans on these being plain loads, stores and CASes — a
// "lock-free" atomic that quietly fell back to a hidden lock would bring back
// exactly the problem the gate exists to avoid.
static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "telemetry needs lock-free 32-bit atomics");

/// Serializes producers onto one RecordRing without ever making one wait.
///
/// A mutex is the obvious tool and the wrong one here. PROS deletes the
/// autonomous/opcontrol task on every competition-mode change, wherever it
/// happens to be, and FreeRTOS never releases a mutex whose owner died — so a
/// mutex taken inside a PID update is, rarely but eventually, locked forever.
/// This gate is a try-lock instead: a producer that finds it held drops its
/// row (counted) rather than waiting, and the writer task calls breakIfStuck()
/// every pass to reclaim a gate whose holder died inside it.
///
/// Why that reclaim is safe: a holder keeps the gate for one short copy and
/// never blocks inside it, and every producer runs above the writer's
/// priority. On the V5's single user core, the writer only runs when no
/// higher-priority task is ready — so a gate it finds held, by the same entry,
/// across several of its passes belongs to a task that no longer exists.
class ProducerGate {
public:
    /// Takes the gate if it's free; never waits. On success `token` identifies
    /// this entry, for leave().
    bool tryEnter(std::uint32_t& token);

    /// Ends the entry `token` began. A no-op if breakIfStuck() already ended
    /// it: the holder was presumed dead, and the gate may belong to someone
    /// else by now.
    void leave(std::uint32_t token);

    /// Writer task only, once per pass. Force-releases the gate if it has been
    /// held by the same entry for `passes` consecutive calls; true if it did.
    bool breakIfStuck(int passes = 3);

    /// tryEnter() calls that found the gate held, cumulative.
    std::uint32_t contended() const;

private:
    friend struct ProducerGateTestAccess; // tests/telemetry/record_ring_test.cpp

    /// Even = free, odd = held; each enter and each leave advances it by one,
    /// so the value also names *which* entry holds it. That is what lets
    /// leave() and breakIfStuck() each refuse to end an entry the other
    /// already ended.
    std::atomic<std::uint32_t> state_{0};
    std::atomic<std::uint32_t> contended_{0};

    // breakIfStuck()'s watchdog state — writer task only.
    std::uint32_t watchedState_ = 0;
    int watchedPasses_ = 0;
};

/// Bounded FIFO of Records: any number of producers, serialized by a
/// ProducerGate, and exactly one consumer. Storage is allocated once, in the
/// constructor; push(), front(), peek() and pop() never allocate, block, or
/// lock.
///
/// Full means the *new* row is dropped and counted. Overwriting the oldest
/// instead would need the producer to move the consumer's tail — exactly the
/// cross-task write this design avoids — and a tuning app handles a counted
/// gap far better than silently reshuffled history.
class RecordRing {
public:
    /// `minCapacity` is rounded up to a power of two in [8, 32768].
    /// `startIndex` only exists so tests can exercise index wraparound.
    explicit RecordRing(std::size_t minCapacity, std::uint32_t startIndex = 0);

    RecordRing(const RecordRing&) = delete;
    RecordRing& operator=(const RecordRing&) = delete;

    std::size_t capacity() const;

    /// Producer side; the caller must hold this ring's ProducerGate. Copies
    /// `record` in, or returns false (counted in dropped()) if the ring is full.
    bool push(const Record& record);

    /// push() for a row that spans several Records (a long event): all of
    /// `count` go in, back to back, or none do. The consumer can never see
    /// part of the row, because the new head is published once, after the
    /// last copy. A refusal counts as one dropped row, not `count`.
    bool push(const Record* records, std::size_t count);

    /// Consumer side. The oldest unconsumed Record, or nullptr if none; valid
    /// until pop().
    const Record* front();

    /// Consumer side. The Record `offset` places after front() (peek(0) is
    /// front()), or nullptr if that many aren't in the ring — how the writer
    /// gathers an event's continuations. Valid until pop().
    const Record* peek(std::size_t offset);

    /// Consumer side. Discards the oldest `count` Records, which front()/peek()
    /// must already have shown to be there.
    void pop(std::size_t count = 1);

    /// Rows refused because the ring was full, cumulative. Safe from any task.
    std::uint32_t dropped() const;

    /// Times the consumer found the indices inconsistent — only possible if
    /// producers weren't serialized — and discarded the ring's contents to
    /// recover instead of reading garbage. Consumer side.
    std::uint32_t resyncs() const;

private:
    friend struct RecordRingTestAccess; // tests/telemetry/record_ring_test.cpp

    /// Records between tail and head — or 0 if the indices were inconsistent,
    /// after resyncing them (discarding what was there). Consumer side.
    std::uint32_t available();

    std::unique_ptr<Record[]> slots_;
    std::uint32_t mask_;
    std::atomic<std::uint32_t> head_; // next slot to write; producers only
    std::atomic<std::uint32_t> tail_; // next slot to read; consumer only
    std::atomic<std::uint32_t> dropped_{0};
    std::uint32_t resyncs_ = 0;
};

} // namespace sapphirelib::telemetry
