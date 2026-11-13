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

test('minVolts: a chained motion reaches its exit at that speed, and hands it to the next', () => {
  // the robot's speed just after the chained move ends, the least it drove with on the way in,
  // and the volts it left running for the next motion
  const square = (minVolts) => {
    let speedInPerS = null;
    let lowest = Infinity;
    let leftRunning = null;
    let first = false;
    const sim = run(START, function* (a, s) {
      a.setPose(0, -40, 0);
      first = true;
      yield* a.moveToPoint(0, -16, { chain: true, minVolts });
      first = false;
      leftRunning = Math.hypot(s.robot.drivetrain.applied.forward, s.robot.drivetrain.applied.strafe);
      const from = Object.assign({}, s.world.truth);
      const fromMs = s.world.timeMs;
      yield* a.delay(10);
      speedInPerS = Math.hypot(s.world.truth.xIn - from.xIn, s.world.truth.yIn - from.yIn) /
        ((s.world.timeMs - fromMs) / 1000);
      yield* a.moveToPoint(24, -16);
    }, { during: (s) => {
      // minVolts is the translation's: forward, and the little sideways that steers it in
      const v = s.robot.drivetrain.applied;
      if (first) lowest = Math.min(lowest, Math.hypot(v.forward, v.strafe));
    } });
    return { sim, speedInPerS, lowest, leftRunning };
  };
  const bare = square(0);
  const kept = square(8);
  assert.ok(kept.lowest >= 8 - 1e-9, `drove in at ${kept.lowest.toFixed(2)}V`);
  assert.ok(kept.leftRunning >= 8 - 1e-9, `left ${kept.leftRunning.toFixed(2)}V running`);
  assert.ok(bare.leftRunning < 8, `without it, the PID had slowed to ${bare.leftRunning.toFixed(2)}V`);
  assert.ok(kept.speedInPerS > bare.speedInPerS + 5,
    `${kept.speedInPerS.toFixed(1)} in/s with it vs ${bare.speedInPerS.toFixed(1)} without`);
  // and the next motion still lands
  assert.ok(Math.hypot(kept.sim.world.truth.xIn - 24, kept.sim.world.truth.yIn + 16) < 1);
});

test('minVolts is held to maxVolts and the speed limit, and an unchained motion still settles', () => {
  let worst = 0;
  const sim = run(START, function* (a) {
    a.setPose(0, -40, 0);
    yield* a.moveToPoint(0, -20, { maxVolts: 4, minVolts: 8, timeoutMs: 8000 });
    a.setSpeedLimit(5);
    yield* a.turnTo(90, { minVolts: 8 });
    a.setSpeedLimit(12);
  }, { during: (s) => {
    const v = s.robot.drivetrain.applied;
    worst = Math.max(worst, Math.abs(v.forward) + Math.abs(v.strafe) + Math.abs(v.turn));
  } });
  assert.ok(worst <= 5 + 1e-9, `a wheel got ${worst.toFixed(2)}V`);
  // overshot at speed, then the PID brought it back
  assert.ok(Math.hypot(sim.world.truth.xIn, sim.world.truth.yIn + 20) < 1,
    `ended at (${sim.world.truth.xIn.toFixed(2)}, ${sim.world.truth.yIn.toFixed(2)})`);
  assert.ok(Math.abs(sim.world.truth.headingDeg - 90) < 3, `heading ${sim.world.truth.headingDeg.toFixed(1)}`);
});

test('minVolts on turnTo() and drive(): each pushes with at least it until there', () => {
  const lowest = { turn: Infinity, forward: Infinity };
  let leg = null;
  const sim = run(START, function* (a) {
    a.setPose(0, -40, 0);
    leg = 'turn';
    yield* a.turnTo(90, { chain: true, minVolts: 6 });
    leg = 'forward';
    yield* a.drive(20, { chain: true, minVolts: 6 });
    leg = null;
    yield* a.drive(0);
  }, { during: (s) => {
    const v = s.robot.drivetrain.applied;
    if (leg === 'turn') lowest.turn = Math.min(lowest.turn, Math.abs(v.turn));
    if (leg === 'forward') lowest.forward = Math.min(lowest.forward, v.forward);
  } });
  assert.ok(lowest.turn >= 6 - 1e-9, `turned at ${lowest.turn.toFixed(2)}V`);
  assert.ok(lowest.forward >= 6 - 1e-9, `drove at ${lowest.forward.toFixed(2)}V`);
  assert.ok(sim.world.truth.xIn > 15, `drove on to x ${sim.world.truth.xIn.toFixed(2)}`);
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

test('setSpeedLimit() caps the motions after it, below their own maxVolts, until it\'s lifted', () => {
  const worst = [0, 0];
  let leg = 0;
  const sim = run(START, function* (a) {
    a.setPose(0, -40, 0);
    a.setSpeedLimit(4);
    assert.equal(a.speedLimit(), 4);
    yield* a.moveToPose(12, -10, 0, { maxVolts: 8, timeoutMs: 8000 });
    a.setSpeedLimit(12);
    leg = 1;
    yield* a.moveToPoint(0, -40);
  }, { during: (s) => {
    const v = s.robot.drivetrain.applied;
    worst[leg] = Math.max(worst[leg], Math.abs(v.forward) + Math.abs(v.strafe) + Math.abs(v.turn));
  } });
  assert.ok(worst[0] <= 4 + 1e-9, `a wheel got ${worst[0].toFixed(2)}V under the limit`);
  assert.ok(worst[1] > 6, `and ${worst[1].toFixed(2)}V once it was lifted`);
  assert.ok(Math.hypot(sim.world.truth.xIn, sim.world.truth.yIn + 40) < 1);
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
