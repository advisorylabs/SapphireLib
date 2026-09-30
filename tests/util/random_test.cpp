// Host-side unit test for sapphirelib::Rng (header-only), no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// The golden values pin the exact sequence: the simulator's JavaScript port
// (tools/sim/js/mcl.js) is checked against the same numbers in
// tools/sim/test/mcl.test.js. Change one side, change the other.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/util/random_test.cpp -o random_test && ./random_test

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "sapphirelib/util/random.hpp"

using sapphirelib::Rng;

namespace {

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.17g, expected %.17g\n", label, actual, expected);
        assert(false);
    }
}

void testGoldenSequence() {
    Rng rng(42);
    const std::uint32_t expected[] = {2837322924u, 544945897u, 479756282u, 3500138142u,
                                      339756180u};
    for (const std::uint32_t value : expected) {
        const std::uint32_t got = rng.next();
        if (got != value) {
            std::printf("FAIL golden next(): got %u, expected %u\n", got, value);
            assert(false);
        }
    }
    Rng uniforms(7);
    expectNear(uniforms.uniform(), 0.23382771760225296, 0.0, "golden uniform 1");
    expectNear(uniforms.uniform(), 0.51223241887055337, 0.0, "golden uniform 2");
    Rng gaussians(7);
    expectNear(gaussians.gaussian(), -1.585033621793738, 1e-12, "golden gaussian 1");
    expectNear(gaussians.gaussian(), -0.19791852691893735, 1e-12, "golden gaussian 2");
}

void testSeedsGiveDifferentSequences() {
    Rng a(1);
    Rng b(2);
    int same = 0;
    for (int i = 0; i < 100; ++i) same += a.next() == b.next() ? 1 : 0;
    assert(same < 3);
}

void testReseedRestarts() {
    Rng rng(99);
    const std::uint32_t first = rng.next();
    rng.next();
    rng.reseed(99);
    assert(rng.next() == first);
}

void testUniformRangeAndMean() {
    Rng rng(3);
    double sum = 0.0;
    const int n = 200000;
    for (int i = 0; i < n; ++i) {
        const double u = rng.uniform();
        assert(u >= 0.0 && u < 1.0);
        sum += u;
    }
    expectNear(sum / n, 0.5, 0.005, "uniform mean");
}

void testGaussianMoments() {
    Rng rng(5);
    double sum = 0.0;
    double sumSquares = 0.0;
    const int n = 200000;
    for (int i = 0; i < n; ++i) {
        const double g = rng.gaussian();
        sum += g;
        sumSquares += g * g;
    }
    const double mean = sum / n;
    expectNear(mean, 0.0, 0.01, "gaussian mean");
    expectNear(std::sqrt(sumSquares / n - mean * mean), 1.0, 0.01, "gaussian stddev");
}

} // namespace

int main() {
    testGoldenSequence();
    testSeedsGiveDifferentSequences();
    testReseedRestarts();
    testUniformRangeAndMean();
    testGaussianMoments();
    std::printf("random_test: all tests passed\n");
    return 0;
}
