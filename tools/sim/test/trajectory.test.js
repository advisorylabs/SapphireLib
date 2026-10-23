// Tests for js/trajectory.js: the simulator's port of the trajectory planner.
// Run: node --test tools/sim/test/*.test.js
//
// The golden case is the one at the bottom of
// tests/motion/trajectory_generator_test.cpp: the same chain and models must
// plan the same trajectory, to well within what libm differences between C++
// and JavaScript can move it. The other cases repeat the C++ tests' claims.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const T = require('../js/trajectory.js');

// the simulator's default chassis (js/world.js)
const MODELS = {
  forward: { kS: 1.0, kV: 0.2, kA: 0.045 },
  strafe: { kS: 1.3, kV: 0.24, kA: 0.055 },
  turn: { kS: 0.6, kV: 0.045, kA: 0.0065 },
};

function close(actual, expected, tolerance, label) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: got ${actual}, expected ${expected} ± ${tolerance}`);
}

function wheelVolts(models, s) {
  const v = T.trajectoryFeedforward(models, s);
  return Math.abs(v.forward) + Math.abs(v.strafe) + Math.abs(v.turn);
}

function plan(sections) {
  const result = T.generateTrajectory(sections, MODELS);
  assert.ok(result.ok, result.error);
  return result.trajectory;
}

test('golden case matches tests/motion/trajectory_generator_test.cpp', () => {
  const t = plan([
    { waypoints: [{ xIn: -48, yIn: -60, headingDeg: 0 }, { xIn: -40, yIn: -30 }, { xIn: -20, yIn: -10, headingDeg: 45 }] },
    { waypoints: [{ xIn: 10, yIn: 0 }], constraints: { voltageBudget: 7.0 }, stopAtEnd: true },
    { waypoints: [{ xIn: -10, yIn: 20, headingDeg: 0 }] },
  ]);
  const golden = (actual, expected, label) => close(actual, expected, 1e-8 * Math.max(1, Math.abs(expected)), label);
  const s = t.samples;
  assert.equal(s.length, 243);
  golden(t.durationS(), 5.36486852518, 'duration');
  golden(t.lengthIn(), 119.887390117, 'length');
  golden(t.sectionStart(1).tS, 2.00665200769, 'section 1 start');
  golden(t.sectionStart(2).tS, 3.89911266776, 'section 2 start');
  const rows = [
    [40, 0.787062010217, 19.9137214934, -44.217670956, -40.4658270212, 9.40188960883, 31.2564846359,
      -1.28482452167, 31.0308703569, 3.74872200722, 34.7462129676],
    [100, 1.71232144608, 49.7843037335, -28.2350556326, -15.7742581088, 43.3608611768, 34.8779544272,
      2.39572562407, 34.6603808457, 3.88969210276, 15.4074679151],
    [160, 3.00004474926, 79.6548859736, -1.61302529974, -2.80653120002, 35.789193823, 15.1340479579,
      -3.5663346733, 11.7523101905, 9.53533495884, -16.6277431448],
    [223, 4.92289194887, 110.459299701, -3.33333333333, 13.3333333333, 1.35833455607, 24.6209472117,
      6.75910022089, 16.9920477058, -17.8174452817, -9.70397594546],
  ];
  for (const [k, tS, sIn, x, y, h, v, a, f, st, w] of rows) {
    const g = s[k];
    golden(g.tS, tS, `t[${k}]`);
    golden(g.sIn, sIn, `s[${k}]`);
    golden(g.xIn, x, `x[${k}]`);
    golden(g.yIn, y, `y[${k}]`);
    golden(g.headingDeg, h, `heading[${k}]`);
    golden(g.speedInPerS, v, `speed[${k}]`);
    golden(g.accelInPerS2, a, `accel[${k}]`);
    golden(g.forwardInPerS, f, `forward[${k}]`);
    golden(g.strafeInPerS, st, `strafe[${k}]`);
    golden(g.omegaDegPerS, w, `turn rate[${k}]`);
  }
});

test('golden branch matches tests/motion/trajectory_generator_test.cpp', () => {
  // the golden chain's first section ending at 25in/s, and a branch carrying on from it
  const trunkResult = T.generateTrajectory([{ waypoints: [{ xIn: -48, yIn: -60, headingDeg: 0 },
    { xIn: -40, yIn: -30 }, { xIn: -20, yIn: -10, headingDeg: 45 }] }], MODELS, { endSpeedInPerS: 25 });
  assert.ok(trunkResult.ok, trunkResult.error);
  const trunk = trunkResult.trajectory;
  const branchResult = T.generateTrajectory([{ waypoints: [{ xIn: 10, yIn: 0 }, { xIn: 20, yIn: 24, headingDeg: 90 }],
    constraints: { voltageBudget: 8 } }], MODELS, { from: trunk });
  assert.ok(branchResult.ok, branchResult.error);
  const branch = branchResult.trajectory;
  const golden = (actual, expected, label) => close(actual, expected, 1e-8 * Math.max(1, Math.abs(expected)), label);
  assert.equal(trunk.samples.length, 121);
  assert.equal(branch.samples.length, 119);
  golden(trunk.durationS(), 2.01067677543, 'trunk duration');
  golden(trunk.samples[120].speedInPerS, 25, 'junction speed');
  golden(branch.durationS(), 2.98658036529, 'branch duration');
  golden(branch.lengthIn(), 58.5257483556, 'branch length');
  const rows = [
    [0, 0, 0, -20, -10, 45, 25, -14.9181223458, 24.834295924, 2.87362940515, 0],
    [1, 0.019958081963, 0.495980918267, -19.6066287741, -9.69794477816, 45.0002704141, 24.7022628915,
      -14.6687615374, 24.4402120439, 3.588568964, 0.0402315927545],
    [15, 0.323834021373, 7.43971377401, -13.4156474867, -6.62058706667, 45.7570655131, 21.3950535901,
      -7.24651901054, 19.2287856045, 9.3809446378, 6.07615871889],
    [40, 0.946144341287, 19.8392367307, -1.30721735675, -3.99843596606, 54.8242569316, 19.3283698973,
      2.00030408028, 17.5955293187, 7.99895186146, 22.3853134531],
    [70, 1.69358260801, 34.7186642787, 12.0741504601, 1.85680027705, 75.1850970868, 17.2133970174,
      -4.85410762631, 14.5107632718, -9.25952407774, 23.1208557435],
  ];
  for (const [k, tS, sIn, x, y, h, v, a, f, st, w] of rows) {
    const g = branch.samples[k];
    golden(g.tS, tS, `t[${k}]`);
    golden(g.sIn, sIn, `s[${k}]`);
    golden(g.xIn, x, `x[${k}]`);
    golden(g.yIn, y, `y[${k}]`);
    golden(g.headingDeg, h, `heading[${k}]`);
    golden(g.speedInPerS, v, `speed[${k}]`);
    golden(g.accelInPerS2, a, `accel[${k}]`);
    golden(g.forwardInPerS, f, `forward[${k}]`);
    golden(g.strafeInPerS, st, `strafe[${k}]`);
    golden(g.omegaDegPerS, w, `turn rate[${k}]`);
  }
  assert.ok(!branch.samples[0].stop && !trunk.samples[120].stop);
  assert.ok(T.endsMoving(trunk) && !T.endsMoving(branch) && T.continuesFrom(branch, trunk));
});

test('a branch that can\'t take the junction speed says so', () => {
  const trunk = T.generateTrajectory([{ waypoints: [{ xIn: 0, yIn: 0, headingDeg: 0 }, { xIn: 0, yIn: 60 }] }], MODELS,
    { endSpeedInPerS: 30 }).trajectory;
  const r = T.generateTrajectory([{ waypoints: [{ xIn: 4, yIn: 61 }, { xIn: 8, yIn: 56 }] }], MODELS, { from: trunk });
  assert.ok(!r.ok);
  assert.ok(r.error.startsWith('can\'t carry on at 30.0in/s'), r.error);
  const none = T.generateTrajectory([{ waypoints: [{ xIn: 0, yIn: 10 }] }], MODELS, { from: new T.Trajectory() });
  assert.equal(none.error, 'the trajectory to continue from is empty');
});

test('the voltage budget holds at every sample, with every cap', () => {
  const limits = { voltageBudget: 9, forward: { maxVelocity: 40, maxAcceleration: 120 },
    strafe: { maxVelocity: 25, maxAcceleration: 80 }, turn: { maxVelocity: 200, maxAcceleration: 600 } };
  const t = plan([{ waypoints: [{ xIn: -48, yIn: -48, headingDeg: 0 }, { xIn: -20, yIn: -10 },
    { xIn: 10, yIn: 0, headingDeg: 90 }, { xIn: 30, yIn: 30 }, { xIn: 40, yIn: 60, headingDeg: 180 }],
  constraints: limits }]);
  for (const s of t.samples) {
    assert.ok(wheelVolts(MODELS, s) <= 9.15);
    assert.ok(Math.abs(s.forwardInPerS) <= 40 + 1e-6);
    assert.ok(Math.abs(s.strafeInPerS) <= 25 + 1e-6);
  }
});

test('strafing is slower than driving forward, by the strafe model', () => {
  const forward = plan([{ waypoints: [{ xIn: 0, yIn: 0, headingDeg: 0 }, { xIn: 0, yIn: 120 }] }]);
  const sideways = plan([{ waypoints: [{ xIn: 0, yIn: 0, headingDeg: 90 }, { xIn: 0, yIn: 120 }] }]);
  const top = (10 - MODELS.strafe.kS) / MODELS.strafe.kV;
  const fastest = Math.max(...sideways.samples.map((s) => -s.strafeInPerS));
  assert.ok(fastest <= top + 1e-9 && fastest > 0.99 * top);
  assert.ok(sideways.durationS() > 1.15 * forward.durationS());
});

test('sections: a slower section is braked for, a stop reaches 0', () => {
  const t = plan([
    { waypoints: [{ xIn: 0, yIn: 0, headingDeg: 0 }, { xIn: 0, yIn: 60 }] },
    { waypoints: [{ xIn: 0, yIn: 100 }], constraints: { forward: { maxVelocity: 20 } }, stopAtEnd: true },
    { waypoints: [{ xIn: 30, yIn: 100 }] },
  ]);
  assert.equal(t.sectionCount(), 3);
  close(t.sectionStart(1).sIn, 60, 1e-6, 'section 1 start');
  for (const s of t.samples) {
    if (s.sIn > 60 + 1e-9 && s.sIn < 100) assert.ok(s.speedInPerS <= 20 + 1e-9);
    if (Math.abs(s.sIn - 100) < 1e-6) assert.ok(s.stop && s.speedInPerS === 0);
  }
  close(t.sample(t.sectionStart(1).tS).speedInPerS, 20, 1, 'through the junction');
});

test('rejections say why', () => {
  const unmeasured = Object.assign({}, MODELS, { strafe: { kS: 0, kV: 0, kA: 0 } });
  let r = T.generateTrajectory([{ xIn: 0, yIn: 0, headingDeg: 0 }, { xIn: 0, yIn: 10 }], {}, unmeasured);
  assert.equal(r.error, 'the Strafe model isn\'t measured (kV and kA must be above 0): run Auto-Tune');
  r = T.generateTrajectory([{ xIn: 0, yIn: 0 }, { xIn: 0, yIn: 10 }], {}, MODELS);
  assert.equal(r.error, 'the first waypoint needs a heading (where the robot starts)');
  r = T.generateTrajectory([{ xIn: 0, yIn: 0, headingDeg: 0 }, { xIn: 0, yIn: 10 }], { voltageBudget: 0.5 }, MODELS);
  assert.equal(r.error, 'the voltage budget can\'t start the robot moving at 0.0in along the path');
  r = T.generateTrajectory([{ waypoints: [{ xIn: 0, yIn: 0, headingDeg: 0 }, { xIn: 0, yIn: 9 }] }, { waypoints: [] }],
    MODELS);
  assert.equal(r.error, 'section 1 has no waypoints');
  assert.ok(r.trajectory.empty());
});
