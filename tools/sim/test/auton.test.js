// Tests for auton.js, the simulator's port of the auton functions (sapphirelib/auton/auton.hpp):
// what a routine in autons.cpp can count on, checked against the simulated robot. Run:
// node --test tools/sim/test/*.test.js
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const SIM = require('../js/sim.js');
const A = require('../js/auton.js');
const { TOUR_CHAIN } = require('../js/routines.js');

// Runs routine(a, sim) as an autonomous routine with the robot really at `start`, calling
// during(sim) before every tick, and returns the finished simulation
function run(start, routine, { during = () => {}, tweak = () => {}, settings = {} } = {}) {
  const options = SIM.defaultOptions();
  tweak(options);
  const sim = new SIM.Simulation(options);
  sim.runRoutine({ name: 'test', start, *run(robot) {
    yield* routine(A.create(robot, settings), sim);
  } });
  while (sim.program && sim.world.timeMs < 60000) {
    during(sim);
    sim.tick();
  }
  assert.ok(!sim.program, 'the routine finished');
  return sim;
}

const START = { xIn: 0, yIn: -40, headingDeg: 0 };

test('runAsync(): the routine carries on while the motion drives, and waitUntilTraveled() catches it on the way', () => {
  let caughtAtIn = null;
  let stillDriving = null;
  const sim = run(START, function* (a, s) {
    a.setPose(0, -40, 0);
    a.runAsync(function* () { yield* a.moveToPoint(0, 0); });
    assert.ok(a.isMotionRunning());
    assert.ok(yield* a.waitUntilTraveled(12));
    caughtAtIn = s.world.truth.yIn + 40;
    stillDriving = a.isMotionRunning();
    yield* a.waitForMotion();
    // nothing running: the waits say so instead of waiting forever
    assert.equal(yield* a.waitUntilTraveled(1), false);
  });
  assert.ok(caughtAtIn >= 11 && caughtAtIn < 14, `caught ${caughtAtIn.toFixed(2)}in in`);
  assert.ok(stillDriving, 'still driving when caught');
  assert.ok(Math.abs(sim.world.truth.yIn) < 1, `ended at y ${sim.world.truth.yIn.toFixed(2)}`);
});

test('cancelMotion() stops the running motion where it is, and skips the rest of its block', () => {
  let cancelledAtIn = null;
  const sim = run(START, function* (a, s) {
    a.setPose(0, -40, 0);
    a.runAsync(function* () {
      yield* a.moveToPoint(0, 20);
      yield* a.turnTo(90);
    });
    assert.ok(yield* a.waitUntilNear(0, -20, 1));
    cancelledAtIn = s.world.truth.yIn;
    a.cancelMotion();
    assert.ok(!a.isMotionRunning());
    yield* a.delay(1000);
  });
  assert.ok(Math.abs(cancelledAtIn + 20) < 2, `cancelled at y ${cancelledAtIn.toFixed(2)}`);
  // braked to a stop a few inches on, far short of y = 20, and never turned
  assert.ok(sim.world.truth.yIn < -8, `stopped at y ${sim.world.truth.yIn.toFixed(2)}`);
  assert.ok(Math.abs(sim.world.truth.headingDeg) < 5, `heading ${sim.world.truth.headingDeg.toFixed(1)}`);
});

test('a motion called while one runs in the background waits for it', () => {
  const order = [];
  run(START, function* (a, s) {
    a.setPose(0, -40, 0);
    a.runAsync(function* () {
      yield* a.moveToPoint(0, -16);
      order.push(['background', s.world.timeMs]);
    });
    yield* a.turnTo(90);
    order.push(['routine', s.world.timeMs]);
  });
  assert.deepEqual(order.map(([who]) => who), ['background', 'routine']);
});

test('chaining ends a motion still moving, and the next starts from speed', () => {
  const square = (chain) => function* (a) {
    a.setPose(0, -40, 0);
    yield* a.moveToPoint(0, -16, { chain });
    yield* a.moveToPoint(24, -16);
  };
  const stopped = run(START, square(false));
  const chained = run(START, square(true));
  assert.ok(chained.world.timeMs < stopped.world.timeMs - 150,
    `chained ${chained.world.timeMs}ms vs ${stopped.world.timeMs}ms`);
  // and still lands
  assert.ok(Math.hypot(chained.world.truth.xIn - 24, chained.world.truth.yIn + 16) < 1);
});

test('turnTo() goes the way it\'s told', () => {
  const turn = (direction) => run(START, function* (a) {
    a.setPose(0, -40, 0);
    if (direction) yield* a.turnTo(270, direction);
    else yield* a.turnTo(270);
  }).world.truth.headingDeg;
  // the world's heading isn't wrapped: left is -90, right all the way round is +270
  assert.ok(Math.abs(turn(null) + 90) < 3, `the short way: ${turn(null)}`);
  assert.ok(Math.abs(turn('clockwise') - 270) < 3, `clockwise: ${turn('clockwise')}`);
  assert.ok(Math.abs(turn('counterclockwise') + 90) < 3, 'counterclockwise is already the short way');
});

test('maxVolts caps every wheel, and the move still lands', () => {
  let worst = 0;
  const sim = run(START, function* (a) {
    a.setPose(0, -40, 0);
    // slower, so longer than the 3s default to get there
    yield* a.moveToPoint(12, -10, { maxVolts: 4, timeoutMs: 8000 });
  }, { during: (s) => {
    const v = s.robot.drivetrain.applied;
    worst = Math.max(worst, Math.abs(v.forward) + Math.abs(v.strafe) + Math.abs(v.turn));
  } });
  assert.ok(worst <= 4 + 1e-9, `a wheel got ${worst.toFixed(2)}V`);
  assert.ok(worst > 3.5, 'and it used what it had');
  assert.ok(Math.hypot(sim.world.truth.xIn - 12, sim.world.truth.yIn + 10) < 1);
});

test('drive() and strafe() go straight, robot-relative', () => {
  const sim = run({ xIn: 0, yIn: -40, headingDeg: 90 }, function* (a) {
    a.setPose(0, -40, 90);
    yield* a.drive(24);   // facing +x
    yield* a.strafe(-12); // to its left: +y
  });
  assert.ok(Math.abs(sim.world.truth.xIn - 24) < 1, `x ${sim.world.truth.xIn.toFixed(2)}`);
  assert.ok(Math.abs(sim.world.truth.yIn + 28) < 1, `y ${sim.world.truth.yIn.toFixed(2)}`);
});

test('a wall reset puts odometry back on the wall the sensor sees', () => {
  // really 12in off the near wall, but told 3in further up the field
  const truth = { xIn: 0, yIn: -51.25, headingDeg: 0 };
  let before = null;
  let after = null;
  let ok = null;
  run(truth, function* (a) {
    a.setPose(0, -48.25, 0);
    yield* a.delay(100); // the sensors' first readings
    before = a.pose().yIn;
    ok = a.resetBack();
    after = a.pose().yIn;
  }, { tweak: (o) => { o.correctOdometry = false; } });
  assert.ok(ok, 'reset');
  assert.ok(Math.abs(before + 48.25) < 0.2);
  assert.ok(Math.abs(after - truth.yIn) < 0.5, `after the reset, y ${after.toFixed(2)}`);

  // near a corner, with Override's margin: skipped, nothing moved
  const corner = { xIn: 60, yIn: -51.25, headingDeg: 0 };
  run(corner, function* (a) {
    a.setPose(60, -48.25, 0);
    yield* a.delay(100);
    assert.equal(a.resetBack(), false);
    assert.ok(Math.abs(a.pose().yIn + 48.25) < 0.2);
  }, { tweak: (o) => { o.correctOdometry = false; }, settings: { wallReset: { cornerMarginIn: 18 } } });
});

test('waitUntilWaypoint() catches a background trajectory at its point, without it stopping', () => {
  // the Trajectory tour's point 4 is the stop facing the far wall, at (30, 24)
  let at = null;
  run(TOUR_CHAIN[0].waypoints[0], function* (a, s) {
    a.setPose(-48, -60, 0);
    a.runAsync(function* () { yield* a.followTrajectory(TOUR_CHAIN); });
    assert.ok(yield* a.waitUntilWaypoint(1));
    assert.ok(yield* a.waitUntilWaypoint(4));
    at = Object.assign({}, s.world.truth);
    assert.ok(a.isMotionRunning(), 'still driving the rest of it');
    yield* a.waitForMotion();
    // a point the path doesn't have
    a.runAsync(function* () { yield* a.followTrajectory(TOUR_CHAIN); });
    assert.equal(yield* a.waitUntilWaypoint(99), false);
  });
  assert.ok(Math.hypot(at.xIn - 30, at.yIn - 24) < 3, `at (${at.xIn.toFixed(1)}, ${at.yIn.toFixed(1)})`);
});
