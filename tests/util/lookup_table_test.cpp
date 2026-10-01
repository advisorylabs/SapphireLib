// Host-side unit test for sapphirelib::LookupTable (header-only), no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/util/lookup_table_test.cpp -o lookup_table_test && ./lookup_table_test

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "sapphirelib/util/lookup_table.hpp"

using sapphirelib::LookupTable;

namespace {

void expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: got %.17g, expected %.17g\n", label, actual, expected);
        assert(false);
    }
}

void testSamplesAreExact() {
    const LookupTable square(-2.0, 2.0, 4, [](double x) { return x * x; });
    expectNear(square.at(-2.0), 4.0, 0.0, "first sample");
    expectNear(square.at(0.25), 0.0625, 0.0, "a sample in the middle");
    expectNear(square.at(2.0), 4.0, 0.0, "last sample");
    // halfway between 0.25 and 0.5: the straight line, not the curve
    expectNear(square.at(0.375), (0.0625 + 0.25) / 2.0, 1e-15, "between samples");
}

void testEndsClamp() {
    const LookupTable line(0.0, 1.0, 8, [](double x) { return 3.0 * x + 1.0; });
    expectNear(line.at(-5.0), 1.0, 0.0, "below the table reads the first sample");
    expectNear(line.at(9.0), 4.0, 0.0, "above the table reads the last sample");
    expectNear(line.at(std::numeric_limits<double>::quiet_NaN()), 1.0, 0.0, "NaN reads the first");
    expectNear(line.minX(), 0.0, 0.0, "min x");
    expectNear(line.maxX(), 1.0, 0.0, "max x");
}

void testErrorBound() {
    // e^x from -32 to 0 at 64 samples to the unit: within step^2 / 8 = 3.1e-5, relatively
    const LookupTable expTable(-32.0, 0.0, 64, [](double x) { return std::exp(x); });
    double worst = 0.0;
    for (double x = -32.0; x <= 0.0; x += 0.00731) {
        worst = std::max(worst, std::fabs(expTable.at(x) - std::exp(x)) / std::exp(x));
    }
    std::printf("  exp table within %.2e of exp(), relatively\n", worst);
    assert(worst <= 3.1e-5);
}

} // namespace

int main() {
    testSamplesAreExact();
    testEndsClamp();
    testErrorBound();
    std::printf("lookup_table_test: all tests passed\n");
    return 0;
}
