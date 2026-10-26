// Host-side unit test for sapphirelib::motion::DirectedTurn, no PROS/embedded
// dependencies, so it builds and runs with a normal desktop compiler.
//
// Build & run:
//   g++ -std=c++20 -Iinclude tests/motion/turn_math_test.cpp \
//       src/sapphirelib/motion/turn_math.cpp src/sapphirelib/util/angle.cpp \
//       -o turn_math_test && ./turn_math_test

#include <cassert>
#include <cmath>
#include <cstdio>

#include "sapphirelib/motion/turn_math.hpp"

using sapphirelib::motion::DirectedTurn;
using sapphirelib::motion::TurnDirection;

namespace {

void expectNear(double actual, double expected, const char* label) {
    if (std::fabs(actual - expected) >= 1e-9) {
        std::printf("FAIL %s: got %f, expected %f\n", label, actual, expected);
        assert(false);
    }
}

void testShortestIsTheWrappedError() {
    DirectedTurn turn(10, 350, TurnDirection::shortest);
    expectNear(turn.errorDeg(350), 20, "short way across 0");
    expectNear(turn.errorDeg(100), -90, "short way back");
    assert(!turn.forcing());
}

void testClockwiseGoesTheLongWayUntilTheShortWayAgrees() {
    // from 0 to 270 the short way is 90 left; clockwise is 270 right
    DirectedTurn turn(270, 0, TurnDirection::clockwise);
    assert(turn.forcing());
    expectNear(turn.errorDeg(0), 270, "start: the long way, positive (right)");
    expectNear(turn.errorDeg(80), 190, "still the long way");
    // 89 to go is within 180: the short way takes over, and agrees
    expectNear(turn.errorDeg(181), 89, "short way from here");
    assert(!turn.forcing());
    // a small overshoot is corrected back, not turned all the way round again
    expectNear(turn.errorDeg(272), -2, "overshoot comes back");
}

void testCounterclockwiseIsNegative() {
    // from 0 to 90 the short way is right; counterclockwise is 270 left
    DirectedTurn turn(90, 0, TurnDirection::counterclockwise);
    assert(turn.forcing());
    expectNear(turn.errorDeg(0), -270, "the long way, negative (left)");
    expectNear(turn.errorDeg(260), -170, "within 180: short way");
    assert(!turn.forcing());
}

void testNearly180FollowsTheAskedDirection() {
    // 180.5 right is the long way by a hair: forced, it still goes right
    DirectedTurn right(180, 359.5, TurnDirection::clockwise);
    expectNear(right.errorDeg(359.5), 180.5, "right, the long way by a hair");
    DirectedTurn left(180, 0.5, TurnDirection::counterclockwise);
    expectNear(left.errorDeg(0.5), -180.5, "left, the long way by a hair");
}

void testAlreadyAShortWayTurnIsNotForced() {
    DirectedTurn turn(90, 0, TurnDirection::clockwise);
    assert(!turn.forcing());
    expectNear(turn.errorDeg(0), 90, "clockwise is already the short way");
}

void testJustPastTheTargetTurnsBack() {
    // 3 degrees past 90 the clockwise way: turning back the short way, not 357 more
    DirectedTurn turn(90, 93, TurnDirection::clockwise);
    assert(!turn.forcing());
    expectNear(turn.errorDeg(93), -3, "back the short way");
    // 10 past is more than the 5 degree allowance: forced the long way round
    DirectedTurn far(90, 100, TurnDirection::clockwise);
    assert(far.forcing());
    expectNear(far.errorDeg(100), 350, "all the way round");
}

} // namespace

int main() {
    testShortestIsTheWrappedError();
    testClockwiseGoesTheLongWayUntilTheShortWayAgrees();
    testCounterclockwiseIsNegative();
    testNearly180FollowsTheAskedDirection();
    testAlreadyAShortWayTurnIsNotForced();
    testJustPastTheTargetTurnsBack();
    std::puts("turn_math_test: all assertions passed");
    return 0;
}
