// Host-side unit test for sapphirelib::wrapDegrees180 — no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/util/angle_test.cpp src/sapphirelib/util/angle.cpp -o angle_test && ./angle_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/util/angle.hpp"

using sapphirelib::wrapDegrees180;

namespace {

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-9) {
        std::printf("FAIL %s: got %f, expected %f\n", label, actual, expected);
        assert(false);
    }
}

void testInRangeValuesAreUnchanged() {
    expectNear(wrapDegrees180(0.0), 0.0, "0");
    expectNear(wrapDegrees180(90.0), 90.0, "90");
    expectNear(wrapDegrees180(-90.0), -90.0, "-90");
    expectNear(wrapDegrees180(180.0), 180.0, "180");
}

void testWrapsAcrossTheSeam() {
    expectNear(wrapDegrees180(181.0), -179.0, "181");
    expectNear(wrapDegrees180(-181.0), 179.0, "-181");
    expectNear(wrapDegrees180(350.0), -10.0, "350");
    expectNear(wrapDegrees180(-350.0), 10.0, "-350");
}

void testHandlesMultipleWraps() {
    expectNear(wrapDegrees180(360.0), 0.0, "360");
    expectNear(wrapDegrees180(719.0), -1.0, "719");
    expectNear(wrapDegrees180(-719.0), 1.0, "-719");
}

} // namespace

int main() {
    testInRangeValuesAreUnchanged();
    testWrapsAcrossTheSeam();
    testHandlesMultipleWraps();
    std::puts("angle_test: all assertions passed");
    return 0;
}
