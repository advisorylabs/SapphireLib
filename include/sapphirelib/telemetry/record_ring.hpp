#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "sapphirelib/telemetry/record.hpp"

namespace sapphirelib::telemetry {

// everything below needs these to be real lock-free atomics, not a hidden lock
static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "telemetry needs lock-free 32-bit atomics");

/**
 * @brief Lets one producer at a time onto a RecordRing, without ever making one wait
 *
 * A mutex would be wrong here: PROS deletes competition tasks wherever they are on every mode
 * change, and a mutex whose owner died stays locked. This is a try-lock instead. A producer that
 * finds it held drops its row, and the writer calls breakIfStuck() every pass to reclaim a gate
 * whose holder died inside it. That's safe because holders never block inside the gate and every
 * producer runs above the writer's priority
 */
class ProducerGate {
public:
    /**
     * @brief Take the gate if it's free. Never waits
     *
     * @param token set to identify this entry, for leave()
     * @return true the gate was taken
     */
    bool tryEnter(std::uint32_t& token);

    /**
     * @brief Release the gate. Does nothing if breakIfStuck() already released this entry
     *
     * @param token the token from tryEnter()
     */
    void leave(std::uint32_t token);

    /**
     * @brief Force the gate open if the same entry has held it for `passes` calls. Writer only
     *
     * @param passes consecutive calls before forcing it. 3 by default
     * @return true the gate was forced open
     */
    bool breakIfStuck(int passes = 3);

    /**
     * @brief Get how many tryEnter() calls found the gate held
     */
    std::uint32_t contended() const;

private:
    friend struct ProducerGateTestAccess; // tests/telemetry/record_ring_test.cpp

    // even is free, odd is held. Each enter and leave advances it by one, so the value also names
    // which entry holds it, and leave() and breakIfStuck() can't both end the same entry
    std::atomic<std::uint32_t> state_{0};
    std::atomic<std::uint32_t> contended_{0};

    // breakIfStuck()'s state, writer only
    std::uint32_t watchedState_ = 0;
    int watchedPasses_ = 0;
};

/**
 * @brief Bounded queue of Records: many producers (through a ProducerGate), one consumer
 *
 * Storage is allocated once, in the constructor; push(), front(), peek(), and pop() never allocate,
 * block, or lock. When full, the new row is dropped and counted rather than overwriting the oldest
 */
class RecordRing {
public:
    /**
     * @brief Construct a new RecordRing
     *
     * @param minCapacity size in Records, rounded up to a power of two in [8, 32768]
     * @param startIndex starting index, only for testing index wraparound
     */
    explicit RecordRing(std::size_t minCapacity, std::uint32_t startIndex = 0);

    RecordRing(const RecordRing&) = delete;
    RecordRing& operator=(const RecordRing&) = delete;

    /**
     * @brief Get the size in Records
     */
    std::size_t capacity() const;

    /**
     * @brief Add a Record. The caller must hold the ring's ProducerGate
     *
     * @return false the ring was full, and the row was dropped
     */
    bool push(const Record& record);

    /**
     * @brief Add a row that spans several Records, all or none
     *
     * @return false the ring was full. Counts as one dropped row
     */
    bool push(const Record* records, std::size_t count);

    /**
     * @brief Get the oldest Record, or nullptr. Consumer only, valid until pop()
     */
    const Record* front();

    /**
     * @brief Get the Record `offset` places after front(), or nullptr. Consumer only
     */
    const Record* peek(std::size_t offset);

    /**
     * @brief Discard the oldest `count` Records. Consumer only
     */
    void pop(std::size_t count = 1);

    /**
     * @brief Get how many rows were dropped because the ring was full. Safe from any task
     */
    std::uint32_t dropped() const;

    /**
     * @brief Get how many times the consumer found the indices inconsistent and emptied the ring
     *
     * Only possible if producers weren't serialized. Consumer only
     */
    std::uint32_t resyncs() const;

private:
    friend struct RecordRingTestAccess; // tests/telemetry/record_ring_test.cpp

    // Records between tail and head, or 0 after resyncing inconsistent indices. Consumer only
    std::uint32_t available();

    std::unique_ptr<Record[]> slots_;
    std::uint32_t mask_;
    std::atomic<std::uint32_t> head_; // next slot to write, producers only
    std::atomic<std::uint32_t> tail_; // next slot to read, consumer only
    std::atomic<std::uint32_t> dropped_{0};
    std::uint32_t resyncs_ = 0;
};

} // namespace sapphirelib::telemetry
