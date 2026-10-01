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

void testFastGaussianGolden() {
    // the ziggurat's tables and first draws. tools/sim/test/mcl.test.js checks the same numbers
    const sapphirelib::ZigguratTables& z = sapphirelib::zigguratTables();
    assert(z.k[0] == 15555140u);
    assert(z.k[1] == 0u);
    assert(z.k[2] == 12590646u);
    assert(z.k[64] == 16628623u);
    assert(z.k[127] == 15707337u);
    expectNear(z.w[0], 2.2131718675747815e-07, 1e-20, "golden w[0]");
    expectNear(z.w[127], 2.0519613360756637e-07, 1e-20, "golden w[127]");
    expectNear(z.f[1], 0.96359969312708615, 1e-14, "golden f[1]");
    expectNear(z.f[127], 0.0026696290838809228, 1e-16, "golden f[127]");
    Rng rng(7);
    const double expected[] = {0.25539103897165999,  -1.2118758829994307, -0.9925865667999193,
                               -0.15473818772236891, 0.79863601552382013, -0.84043943732603632};
    for (const double value : expected) expectNear(rng.fastGaussian(), value, 1e-12, "golden fast");
}

void testFastGaussianMatchesTheNormalDistribution() {
    // moments, and the tails, which the ziggurat draws by a separate path past 3.44
    Rng rng(12345);
    const int n = 2000000;
    double sum = 0.0;
    double sumSquares = 0.0;
    int over2 = 0;
    int pastTail = 0;
    for (int i = 0; i < n; ++i) {
        const double g = rng.fastGaussian();
        sum += g;
        sumSquares += g * g;
        if (std::fabs(g) > 2.0) ++over2;
        if (std::fabs(g) > sapphirelib::ZigguratTables::kTailStart) ++pastTail;
    }
    const double mean = sum / n;
    expectNear(mean, 0.0, 0.003, "fast gaussian mean");
    expectNear(std::sqrt(sumSquares / n - mean * mean), 1.0, 0.003, "fast gaussian stddev");
    // P(|x| > 2) = 0.0455, P(|x| > 3.4426) = 0.000576: within 4 standard errors
    expectNear(static_cast<double>(over2) / n, 0.0455003, 4 * 1.47e-4, "fast gaussian past 2");
    expectNear(static_cast<double>(pastTail) / n, 0.0005761, 4 * 1.7e-5, "fast gaussian tail");
}

} // namespace

int main() {
    testGoldenSequence();
    testSeedsGiveDifferentSequences();
    testReseedRestarts();
    testUniformRangeAndMean();
    testGaussianMoments();
    testFastGaussianGolden();
    testFastGaussianMatchesTheNormalDistribution();
    std::printf("random_test: all tests passed\n");
    return 0;
}
