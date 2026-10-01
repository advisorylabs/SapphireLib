// Tests for the whole simulation: the robot program's odometry, localizer
// and motions running against the simulated world. Run:
// node --test tools/sim/test/*.test.js
//
// These are the claims the page makes when you show it to someone: MCL keeps
// the pose honest where odometry alone drifts, gets the robot back after a
// bump, ignores what it can't explain, and stays out of the way until
// setPose() puts odometry in the field frame.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const SIM = require('../js/sim.js');
const W = require('../js/world.js');
const R = require('../js/robot.js');
const { ROUTINES } = require('../js/routines.js');

function runRoutine(routine, tweak = () => {}, during = () => {}) {
  const options = SIM.defaultOptions();
  tweak(options);
  const sim = new SIM.Simulation(options);
  sim.runRoutine(routine);
  let worst = 0;
  let sum = 0;
  let ticks = 0;
  while (sim.program && sim.world.timeMs < 90000) {
    during(sim);
    sim.tick();
    const error = sim.errors().corrected;
    worst = Math.max(worst, error);
    sum += error;
    ticks++;
  }
  return { sim, end: sim.errors(), worst, mean: sum / ticks, finished: !sim.program };
}

test('every routine finishes within an inch, and MCL is closer on average than odometry alone', () => {
  for (const routine of ROUTINES) {
    const alone = runRoutine(routine, (o) => { o.correctOdometry = false; });
    const mcl = runRoutine(routine);
    assert.ok(alone.finished && mcl.finished, `${routine.id} finished`);
    assert.ok(mcl.end.corrected < 1.0, `${routine.id}: MCL ends ${mcl.end.corrected.toFixed(2)}in off`);
    // the average, not the worst or the end: a closed lap's odometry errors can cancel out by the
    // finish, and the worst moment can come before there's been anything to correct
    assert.ok(mcl.mean < 0.75 * alone.mean, `${routine.id}: mean ${mcl.mean} vs ${alone.mean}`);
  }
});

test('worn tracking wheels: odometry drifts, MCL holds', () => {
  const worn = (o) => {
    o.world.wheelDiameterErrorPct = 3;
    o.world.wheelSlipPct = 2;
    o.world.imuDriftDegPerMin = 3;
  };
  const alone = runRoutine(ROUTINES[0], (o) => { worn(o); o.correctOdometry = false; });
  const mcl = runRoutine(ROUTINES[0], worn);
  assert.ok(alone.worst > 3, `odometry alone worst ${alone.worst}`);
  assert.ok(mcl.worst < 2.5, `MCL worst ${mcl.worst}`);
  assert.ok(mcl.end.corrected < 1.0, `MCL end ${mcl.end.corrected}`);
});

test('a bump odometry never saw is corrected', () => {
  const bump = (sim) => { if (sim.world.timeMs === 6000) sim.world.bump(6, -4); };
  const alone = runRoutine(ROUTINES[0], (o) => { o.correctOdometry = false; }, bump);
  const mcl = runRoutine(ROUTINES[0], () => {}, bump);
  assert.ok(alone.end.corrected > 5, `odometry alone ends ${alone.end.corrected}`);
  assert.ok(mcl.end.corrected < 1.0, `MCL ends ${mcl.end.corrected}`);
});

test('field elements missing from the map are shrugged off as outliers', () => {
  const mcl = runRoutine(ROUTINES[0], (o) => { o.world.elements = W.exampleElements(); });
  assert.ok(mcl.worst < 2.0, `worst ${mcl.worst}`);
});

test('no corrections before setPose(), unless waitForSetPose is off', () => {
  for (const wait of [true, false]) {
    const options = SIM.defaultOptions();
    options.localizer = { waitForSetPose: wait };
    const sim = new SIM.Simulation(options);
    // like the robot at initialize(): really at (-48, -60), odometry says (0, 0), no setPose()
    sim.reset({ xIn: -48, yIn: -60, headingDeg: 0 });
    sim.robot.odometry.poseGeneration = 0;
    sim.robot.odometry.state.raw = { xIn: 0, yIn: 0, headingDeg: 0 };
    sim.robot.localizer.started = false;
    let corrected = false;
    for (let i = 0; i < 2000; ++i) {
      sim.setDriverInput(i % 800 < 400 ? 0.5 : -0.5, i % 600 < 300 ? 0.4 : -0.4, 0);
      sim.tick();
      corrected = corrected || sim.robot.localizer.status.correcting;
    }
    assert.equal(corrected, !wait, `waitForSetPose ${wait}`);
  }
});

test('stepping through updates by hand gives the same run as ticking', () => {
  const a = new SIM.Simulation(SIM.defaultOptions());
  const b = new SIM.Simulation(SIM.defaultOptions());
  a.runRoutine(ROUTINES[2]);
  b.runRoutine(ROUTINES[2]);
  for (let i = 0; i < 20; ++i) {
    b.beginLocalizerStep();
    b.weighLocalizerStep();
    b.finishLocalizerStep();
  }
  while (a.world.timeMs < b.world.timeMs) a.tick();
  assert.equal(a.world.timeMs, b.world.timeMs);
  assert.deepEqual(a.robot.odometry.getPose(), b.robot.odometry.getPose());
  assert.deepEqual(a.robot.localizer.status.estimate, b.robot.localizer.status.estimate);
});

test('reachedFinalApproach matches pure_pursuit_math_test.cpp', () => {
  assert.equal(R.reachedFinalApproach(0, 6, 0, 4), false);
  assert.equal(R.reachedFinalApproach(0, 6, 1, 4), false);
  assert.equal(R.reachedFinalApproach(5.9, 6, 2, 4), true);
  assert.equal(R.reachedFinalApproach(6.1, 6, 2, 4), false);
  assert.equal(R.reachedFinalApproach(3, 6, 3, 4), true);
  assert.equal(R.reachedFinalApproach(3, 6, 0, 2), true);
  assert.equal(R.reachedFinalApproach(3, 6, 0, 1), true);
});

test('findLookaheadPoint() stays on a lap\'s first side when its last passes beside it (pure_pursuit_math_test.cpp)', () => {
  const lap = [{ xIn: -30, yIn: -30 }, { xIn: -30, yIn: 30 }, { xIn: 30, yIn: 30 }, { xIn: 30, yIn: -30 },
    { xIn: -30, yIn: -30 }];
  const result = R.findLookaheadPoint(-28.86, -20.05, lap, 10, 0);
  assert.equal(result.segmentIndex, 0);
  assert.ok(Math.abs(result.point.xIn + 30) < 1e-9 && result.point.yIn > -11 && result.point.yIn < -9);
});

test('followPath() drives a closed lap instead of finishing where it starts', () => {
  const laps = ROUTINES.find((r) => r.id === 'laps');
  const options = SIM.defaultOptions();
  const sim = new SIM.Simulation(options);
  sim.runRoutine(laps);
  const corners = [[-30, 30], [30, 30], [30, -30]];
  const closest = corners.map(() => Infinity);
  while (sim.program && sim.world.timeMs < 90000) {
    sim.tick();
    corners.forEach(([x, y], i) => {
      closest[i] = Math.min(closest[i], Math.hypot(sim.world.truth.xIn - x, sim.world.truth.yIn - y));
    });
  }
  assert.ok(!sim.program, 'finished');
  // three 240in laps at cruise speed: well over ten seconds, not the instant finish it used to be
  assert.ok(sim.world.timeMs > 12000, `took ${sim.world.timeMs}ms`);
  for (const d of closest) assert.ok(d < 12, `came within ${d.toFixed(1)}in of every corner`);
  assert.ok(sim.errors().corrected < 1.5, `ended ${sim.errors().corrected.toFixed(2)}in off`);
});

test('tracking wheels follow OdometryConfig\'s sign: spinning in place leaves odometry still', () => {
  for (const flipped of [false, true]) {
    const options = SIM.defaultOptions();
    options.correctOdometry = false;
    Object.assign(options.world, { wheelDiameterErrorPct: 0, wheelSlipPct: 0, imuDriftDegPerMin: 0,
      imuScaleErrorPct: 0, imuNoiseDeg: 0, verticalWheelFlipped: flipped });
    const sim = new SIM.Simulation(options);
    sim.reset({ xIn: 0, yIn: 0, headingDeg: 0 });
    let worst = 0;
    for (let i = 0; i < 300; ++i) {
      sim.setDriverInput(0, 0, 0.5);
      sim.tick();
      const raw = sim.robot.odometry.snapshot().rawPose;
      worst = Math.max(worst, Math.hypot(raw.xIn, raw.yIn));
    }
    if (flipped) {
      // the wheel is on the other side from what the config says: a lap of phantom circles
      assert.ok(worst > 5, `flipped: odometry wandered ${worst.toFixed(2)}in`);
    } else {
      assert.ok(worst < 0.05, `odometry wandered ${worst.toFixed(3)}in on a spin in place`);
    }
  }
});

test('Auto-Tune designs the drive and turn gains from the axis models', () => {
  const designs = R.autoTuneGains(W.defaultSettings().models, 0.03);
  assert.ok(designs.drive.ok && designs.turn.ok);
  assert.ok(designs.drive.gains.kP > R.HAND_GAINS.drive.kP);
  assert.equal(designs.drive.gains.kI, 0);
  assert.ok(designs.drive.phaseMarginDeg >= 49.9);
});
