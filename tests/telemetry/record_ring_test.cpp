// Host-side unit test for sapphirelib::telemetry::RecordRing and ProducerGate
// no PROS/embedded dependencies, so it builds and runs with a normal
// desktop compiler.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/record_ring_test.cpp src/sapphirelib/telemetry/record_ring.cpp -o record_ring_test && ./record_ring_test
// clang-format on

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "sapphirelib/telemetry/record_ring.hpp"

namespace sapphirelib::telemetry {

// Befriended by RecordRing / ProducerGate, to force states only a bug (or
// ~2^31 commits) could otherwise produce.
struct RecordRingTestAccess {
    static void setHead(RecordRing& ring, std::uint32_t head) { ring.head_.store(head); }
    static std::uint32_t head(RecordRing& ring) { return ring.head_.load(); }
    static std::uint32_t tail(RecordRing& ring) { return ring.tail_.load(); }
};

struct ProducerGateTestAccess {
    static void setState(ProducerGate& gate, std::uint32_t state) { gate.state_.store(state); }
    static std::uint32_t state(ProducerGate& gate) { return gate.state_.load(); }
};

} // namespace sapphirelib::telemetry

using sapphirelib::telemetry::ProducerGate;
using sapphirelib::telemetry::ProducerGateTestAccess;
using sapphirelib::telemetry::Record;
using sapphirelib::telemetry::RecordRing;
using sapphirelib::telemetry::RecordRingTestAccess;

namespace {

Record tagged(std::uint32_t tag) {
    Record record{};
    record.tUs = tag;
    record.values[0] = static_cast<float>(tag);
    return record;
}

void testCapacityRounding() {
    assert(RecordRing(0).capacity() == 8);
    assert(RecordRing(1).capacity() == 8);
    assert(RecordRing(8).capacity() == 8);
    assert(RecordRing(9).capacity() == 16);
    assert(RecordRing(300).capacity() == 512);
    assert(RecordRing(32768).capacity() == 32768);
    assert(RecordRing(1000000).capacity() == 32768);
}

void testFifoOrder() {
    RecordRing ring(8);
    assert(ring.front() == nullptr);
    for (std::uint32_t i = 0; i < 5; ++i) assert(ring.push(tagged(i)));
    for (std::uint32_t i = 0; i < 5; ++i) {
        const Record* front = ring.front();
        assert(front != nullptr && front->tUs == i && front->values[0] == static_cast<float>(i));
        ring.pop();
    }
    assert(ring.front() == nullptr);
    assert(ring.dropped() == 0);
}

void testFullDropsNewestAndRecovers() {
    RecordRing ring(8);
    for (std::uint32_t i = 0; i < 8; ++i) assert(ring.push(tagged(i)));
    // Full: the *new* row is refused and counted; history is untouched.
    assert(!ring.push(tagged(100)));
    assert(!ring.push(tagged(101)));
    assert(ring.dropped() == 2);
    assert(ring.front()->tUs == 0);
    ring.pop();
    assert(ring.push(tagged(8)));
    for (std::uint32_t i = 1; i <= 8; ++i) {
        assert(ring.front()->tUs == i);
        ring.pop();
    }
    assert(ring.front() == nullptr);
}

void testMultiRecordPushIsAllOrNothing() {
    RecordRing ring(8);
    for (std::uint32_t i = 0; i < 6; ++i) assert(ring.push(tagged(i)));
    const Record three[] = {tagged(10), tagged(11), tagged(12)};
    // Only two slots free: none of the three go in, and it's one dropped row.
    assert(!ring.push(three, 3));
    assert(ring.dropped() == 1);
    assert(ring.peek(6) == nullptr);
    assert(ring.push(three, 2));
    assert(ring.peek(7) != nullptr && ring.peek(7)->tUs == 11);
    assert(ring.peek(8) == nullptr);

    // peek() walks from the front; pop(n) discards n at once.
    assert(ring.peek(0) == ring.front());
    assert(ring.peek(3)->tUs == 3);
    ring.pop(6);
    assert(ring.front()->tUs == 10);
    assert(ring.peek(1)->tUs == 11);
    ring.pop(2);
    assert(ring.front() == nullptr);

    // A zero-length push is refused without counting a drop.
    assert(!ring.push(three, 0));
    assert(ring.dropped() == 1);
}

void testIndexWraparound() {
    // Start 16 short of the 32-bit wrap so the indices cross it mid-test.
    RecordRing ring(8, 0xFFFFFFF0u);
    std::uint32_t next = 0;
    std::uint32_t expect = 0;
    for (int round = 0; round < 20; ++round) {
        while (ring.push(tagged(next))) ++next; // fill to capacity every round
        assert(ring.dropped() == static_cast<std::uint32_t>(round + 1));
        for (int i = 0; i < 5; ++i) {
            assert(ring.front() != nullptr && ring.front()->tUs == expect);
            ring.pop();
            ++expect;
        }
    }
    // Well past the wrap by now, and still consistent.
    assert(RecordRingTestAccess::head(ring) < 0xFFFFFFF0u);
    while (ring.front() != nullptr) {
        assert(ring.front()->tUs == expect++);
        ring.pop();
    }
    assert(expect == next);
    assert(ring.resyncs() == 0);
}

void testResyncWhenHeadGoesBackwards() {
    RecordRing ring(8);
    for (std::uint32_t i = 0; i < 4; ++i) assert(ring.push(tagged(i)));
    ring.pop();
    // Only unserialized producers can do this; the consumer must recover
    // without reading a single slot.
    RecordRingTestAccess::setHead(ring, 0);
    // A producer refuses rather than write while the indices are inconsistent.
    assert(!ring.push(tagged(50)));
    assert(ring.front() == nullptr);
    assert(ring.resyncs() == 1);
    assert(RecordRingTestAccess::tail(ring) == RecordRingTestAccess::head(ring));
    // And the ring works normally afterwards.
    assert(ring.push(tagged(7)));
    assert(ring.front()->tUs == 7);
    ring.pop();
    assert(ring.front() == nullptr);
    assert(ring.resyncs() == 1);
}

void testGateEnterLeave() {
    ProducerGate gate;
    std::uint32_t token = 0;
    assert(gate.tryEnter(token));
    std::uint32_t other = 0;
    assert(!gate.tryEnter(other)); // held: refused, never waits
    assert(gate.contended() == 1);
    gate.leave(token);
    assert(gate.tryEnter(other));
    assert(other != token);
    gate.leave(other);
    assert(gate.contended() == 1);
    assert(!gate.breakIfStuck()); // free
}

void testBreakIfStuckOnlyAfterThreePasses() {
    ProducerGate gate;
    std::uint32_t token = 0;
    assert(gate.tryEnter(token));
    // A holder that never leaves: presumed dead on the third pass, not before.
    assert(!gate.breakIfStuck());
    assert(!gate.breakIfStuck());
    assert(gate.breakIfStuck());
    std::uint32_t next = 0;
    assert(gate.tryEnter(next)); // reclaimed
    gate.leave(next);
}

void testBreakIfStuckNeverOnAGateThatChangedHands() {
    ProducerGate gate;
    std::uint32_t token = 0;
    for (int i = 0; i < 10; ++i) {
        // Held at every pass, but by a different entry each time: a busy
        // channel, not a dead holder.
        assert(gate.tryEnter(token));
        assert(!gate.breakIfStuck());
        gate.leave(token);
    }
    // Free passes in between reset the count too.
    assert(gate.tryEnter(token));
    assert(!gate.breakIfStuck());
    assert(!gate.breakIfStuck());
    gate.leave(token);
    assert(!gate.breakIfStuck());
    assert(gate.tryEnter(token));
    assert(!gate.breakIfStuck());
    assert(!gate.breakIfStuck());
    assert(gate.breakIfStuck()); // the same entry, three passes running
}

void testStaleLeaveAfterReclaimIsANoOp() {
    ProducerGate gate;
    std::uint32_t stale = 0;
    assert(gate.tryEnter(stale));
    assert(!gate.breakIfStuck());
    assert(!gate.breakIfStuck());
    assert(gate.breakIfStuck());

    std::uint32_t fresh = 0;
    assert(gate.tryEnter(fresh));
    // The presumed-dead holder turns out to be alive after all and leaves: it
    // must not release the gate out from under the new holder...
    gate.leave(stale);
    std::uint32_t third = 0;
    assert(!gate.tryEnter(third));
    // ...and the new holder's own leave still works; nothing is wedged.
    gate.leave(fresh);
    assert(gate.tryEnter(third));
    gate.leave(third);
}

void testGateParityAcrossStateWraparound() {
    ProducerGate gate;
    ProducerGateTestAccess::setState(gate, 0xFFFFFFFEu); // free, one enter from the top
    std::uint32_t token = 0;
    assert(gate.tryEnter(token));
    assert(token == 0xFFFFFFFFu);
    std::uint32_t other = 0;
    assert(!gate.tryEnter(other));
    gate.leave(token);
    assert(ProducerGateTestAccess::state(gate) == 0); // wrapped, still even = free
    assert(gate.tryEnter(token));
    assert(token == 1);
    gate.leave(token);

    // A reclaim across the wrap works too.
    ProducerGateTestAccess::setState(gate, 0xFFFFFFFFu); // held by a dead entry
    assert(!gate.breakIfStuck());
    assert(!gate.breakIfStuck());
    assert(gate.breakIfStuck());
    assert(ProducerGateTestAccess::state(gate) == 0);
    assert(gate.tryEnter(token));
    gate.leave(token);
}

// Several producer threads hammer one gate + ring while a consumer drains.
// On a desktop's real cores this exercises the memory ordering the brain's
// single core never stresses: every record must arrive whole, in per-producer
// order, and every attempt must be accounted for as delivered or dropped.
void testConcurrentProducersNeverTearOrReorder() {
    constexpr int kProducers = 4;
    constexpr std::uint32_t kPerProducer = 20000;
    RecordRing ring(64);
    ProducerGate gate;
    std::atomic<int> running{kProducers};

    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p) {
        producers.emplace_back([&, p] {
            for (std::uint32_t seq = 0; seq < kPerProducer; ++seq) {
                Record record{};
                record.tUs = (static_cast<std::uint64_t>(p) << 32) | seq;
                // Every value derives from the sequence number, so a torn
                // copy shows up as a mismatch.
                for (float& value : record.values) value = static_cast<float>(seq);
                std::uint32_t token = 0;
                if (gate.tryEnter(token)) {
                    ring.push(record);
                    gate.leave(token);
                }
            }
            running.fetch_sub(1);
        });
    }

    std::uint64_t delivered = 0;
    std::int64_t lastSeq[kProducers];
    for (auto& seq : lastSeq) seq = -1;
    while (true) {
        const bool producersDone = running.load() == 0;
        const Record* record = ring.front();
        if (record == nullptr) {
            if (producersDone) break;
            continue;
        }
        const int producer = static_cast<int>(record->tUs >> 32);
        const auto seq = static_cast<std::uint32_t>(record->tUs & 0xFFFFFFFFu);
        assert(producer >= 0 && producer < kProducers);
        assert(static_cast<std::int64_t>(seq) > lastSeq[producer]);
        lastSeq[producer] = seq;
        for (float value : record->values) assert(value == static_cast<float>(seq));
        ring.pop();
        ++delivered;
    }
    for (auto& thread : producers) thread.join();
    while (ring.front() != nullptr) {
        ring.pop();
        ++delivered;
    }

    assert(ring.resyncs() == 0);
    assert(delivered + ring.dropped() + gate.contended() ==
           static_cast<std::uint64_t>(kProducers) * kPerProducer);
}

} // namespace

int main() {
    testCapacityRounding();
    testFifoOrder();
    testFullDropsNewestAndRecovers();
    testMultiRecordPushIsAllOrNothing();
    testIndexWraparound();
    testResyncWhenHeadGoesBackwards();
    testGateEnterLeave();
    testBreakIfStuckOnlyAfterThreePasses();
    testBreakIfStuckNeverOnAGateThatChangedHands();
    testStaleLeaveAfterReclaimIsANoOp();
    testGateParityAcrossStateWraparound();
    testConcurrentProducersNeverTearOrReorder();
    std::printf("record_ring_test: all tests passed\n");
    return 0;
}
