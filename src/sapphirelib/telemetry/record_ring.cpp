#include "sapphirelib/telemetry/record_ring.hpp"

namespace sapphirelib::telemetry {

namespace {

constexpr std::size_t kMinCapacity = 8;
constexpr std::size_t kMaxCapacity = 32768;

std::size_t roundCapacity(std::size_t minCapacity) {
    std::size_t capacity = kMinCapacity;
    while (capacity < minCapacity && capacity < kMaxCapacity) capacity *= 2;
    return capacity;
}

} // namespace

// ProducerGate

bool ProducerGate::tryEnter(std::uint32_t& token) {
    std::uint32_t state = state_.load(std::memory_order_relaxed);
    // acquire pairs with the last holder's release in leave(), so this holder sees its writes
    if ((state & 1u) == 0 &&
        state_.compare_exchange_strong(state, state + 1, std::memory_order_acquire,
                                       std::memory_order_relaxed)) {
        token = state + 1;
        return true;
    }
    contended_.fetch_add(1, std::memory_order_relaxed);
    return false;
}

void ProducerGate::leave(std::uint32_t token) {
    std::uint32_t expected = token;
    // only fails if breakIfStuck() already ended this entry, and then the gate isn't ours
    state_.compare_exchange_strong(expected, token + 1, std::memory_order_release,
                                   std::memory_order_relaxed);
}

bool ProducerGate::breakIfStuck(int passes) {
    std::uint32_t state = state_.load(std::memory_order_acquire);
    if ((state & 1u) == 0) {
        watchedPasses_ = 0;
        return false;
    }
    if (watchedPasses_ == 0 || state != watchedState_) {
        // held by an entry we haven't watched yet: start counting. A live holder leaves long before
        watchedState_ = state;
        watchedPasses_ = 1;
        return false;
    }
    if (++watchedPasses_ < passes) return false;
    watchedPasses_ = 0;
    // only ends the exact entry we watched. If it left meanwhile, the CAS fails harmlessly
    return state_.compare_exchange_strong(state, state + 1, std::memory_order_acq_rel);
}

std::uint32_t ProducerGate::contended() const { return contended_.load(std::memory_order_relaxed); }

// RecordRing

RecordRing::RecordRing(std::size_t minCapacity, std::uint32_t startIndex)
    : slots_(new Record[roundCapacity(minCapacity)]),
      mask_(static_cast<std::uint32_t>(roundCapacity(minCapacity) - 1)), head_(startIndex),
      tail_(startIndex) {}

std::size_t RecordRing::capacity() const { return static_cast<std::size_t>(mask_) + 1; }

bool RecordRing::push(const Record& record) { return push(&record, 1); }

bool RecordRing::push(const Record* records, std::size_t count) {
    const std::uint32_t head = head_.load(std::memory_order_relaxed);
    const std::uint32_t tail = tail_.load(std::memory_order_acquire);
    // unsigned difference is exact across wraparound. More than capacity is full or inconsistent;
    // refuse either way, so a producer never writes over unread rows
    const std::uint32_t used = head - tail;
    if (count == 0 || used > mask_ + 1u || count > (mask_ + 1u) - used) {
        if (count != 0) dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
        slots_[(head + static_cast<std::uint32_t>(i)) & mask_] = records[i];
    }
    // publish once, last, so a producer deleted mid-copy only loses its own row
    head_.store(head + static_cast<std::uint32_t>(count), std::memory_order_release);
    return true;
}

std::uint32_t RecordRing::available() {
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    const std::uint32_t head = head_.load(std::memory_order_acquire);
    const std::uint32_t used = head - tail;
    if (used > mask_ + 1u) {
        // the head went backwards or leapt ahead, which only unserialized producers can do. Throw
        // away what's there rather than read torn slots
        tail_.store(head, std::memory_order_release);
        ++resyncs_;
        return 0;
    }
    return used;
}

const Record* RecordRing::front() { return peek(0); }

const Record* RecordRing::peek(std::size_t offset) {
    if (available() <= offset) return nullptr;
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    return &slots_[(tail + static_cast<std::uint32_t>(offset)) & mask_];
}

void RecordRing::pop(std::size_t count) {
    tail_.store(tail_.load(std::memory_order_relaxed) + static_cast<std::uint32_t>(count),
                std::memory_order_release);
}

std::uint32_t RecordRing::dropped() const { return dropped_.load(std::memory_order_relaxed); }

std::uint32_t RecordRing::resyncs() const { return resyncs_; }

} // namespace sapphirelib::telemetry
