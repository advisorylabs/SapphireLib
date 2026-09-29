// Tests for js/tuning.js — offline tuning from logs. Run: node --test
// tools/analyzer/test/*.test.js
//
// The demo logs come from a simulation with known physics, so the fits can be
// checked against the truth — and the Auto-Tune refits against what the
// (simulated) robot itself reported, which they must match: same data, same
// math.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const slt = require('../js/slt.js');
const M = require('../js/model.js');
const T = require('../js/tuning.js');
const demo = require('../js/demo.js');

const PIT = slt.parse(demo.pitSession(), 'SL000041.CSV');
const MATCH = slt.parse(demo.match(), 'SL000042.CSV');

function near(actual, expected, fraction, label) {
  assert.ok(Math.abs(actual - expected) <= Math.abs(expected) * fraction,
    `${label}: ${actual} vs ${expected} (±${fraction * 100}%)`);
}

test('systems: the drivetrain axes and the lift, with their controllers', () => {
  const found = T.systems(MATCH);
  assert.deepEqual(found.map((s) => s.id), ['axis:Fwd', 'axis:Strafe', 'axis:Turn', 'mech:lift']);
  assert.deepEqual(found.find((s) => s.id === 'axis:Turn').controllers.map((c) => c.name),
    ['Turn', 'Hold']);
  const lift = found.find((s) => s.id === 'mech:lift');
  assert.equal(lift.controllers[0].loopS, 0.02);
  assert.equal(lift.gravity.kind, 'constant');
});

test('Auto-Tune refits reproduce what the robot reported, and the truth', () => {
  const truth = {
    Fwd: [1.0, 0.2, 0.045, 0],
    Strafe: [1.3, 0.24, 0.055, 0],
    Turn: [0.6, 0.045, 0.0065, 0],
    Lift: [0.7, 0.017, 0.002, 2.2],
  };
  for (const system of T.systems(PIT)) {
    assert.equal(system.autotuneRuns.length, 1, system.label);
    const run = system.autotuneRuns[0];
    const fit = T.fitAutotune(system, run);
    assert.ok(fit.ok, system.label);
    const r = run.reported;
    // Logged values are rounded to 4 decimals; the robot fitted unrounded ones.
    near(fit.model.motion.kS, r.kS, 0.01, `${system.label} kS vs robot`);
    near(fit.model.motion.kV, r.kV, 0.005, `${system.label} kV vs robot`);
    near(fit.model.motion.kA, r.kA, 0.01, `${system.label} kA vs robot`);
    const [kS, kV, kA, kG] = truth[system.axisName];
    near(fit.model.motion.kV, kV, 0.03, `${system.label} kV vs truth`);
    near(fit.model.motion.kA, kA, 0.08, `${system.label} kA vs truth`);
    assert.ok(Math.abs(fit.model.motion.kS - kS) < 0.12, `${system.label} kS vs truth`);
    if (kG) assert.ok(Math.abs(fit.model.kG - kG) < 0.1, `kG ${fit.model.kG}`);
    assert.ok(Math.abs(fit.delayS - (system.kind === 'mechanism' ? 0.02 : 0.03)) < 0.012);
  }
});

test('passive fits from ordinary driving recover the drivetrain axes', () => {
  for (const system of T.systems(MATCH).filter((s) => s.kind === 'drive')) {
    const fit = T.fitPassive(system, T.driveSamples(MATCH, system, 37.5, 142.5));
    assert.ok(fit.ok && fit.samplesUsed > 1000, system.label);
    const kV = { Fwd: 0.2, Strafe: 0.24, Turn: 0.045 }[system.axisName];
    near(fit.model.motion.kV, kV, 0.06, `${system.label} kV`);
  }
});

test('the lift: passive kG sits between empty and loaded; the robot ran no feedforward', () => {
  const lift = T.systems(MATCH).find((s) => s.kind === 'mechanism');
  const fit = T.fitPassive(lift, T.mechanismSamples(MATCH, lift, 37.5, 90));
  assert.ok(fit.ok);
  assert.ok(fit.model.kG > 2.2 && fit.model.kG < 5.4, `kG ${fit.model.kG}`);
  assert.equal(T.inferredGravityVolts(MATCH, lift, 37.5, 142.5), 0);
});

test('what-if: designed gains plus fitted feedforward track the match\'s targets better', () => {
  const lift = T.systems(MATCH).find((s) => s.kind === 'mechanism');
  const fit = T.fitPassive(lift, T.mechanismSamples(MATCH, lift, 37.5, 90));
  const window = { t0: 40, t1: 90 };
  const current = T.replayMechanism(MATCH, lift, fit, Object.assign({
    gains: T.gainsAt(MATCH, lift.controllers[0], 40), gravityVolts: 0 }, window));
  const d = T.design(fit, lift.controllers[0]);
  assert.ok(d.ok);
  const proposed = T.replayMechanism(MATCH, lift, fit, Object.assign({
    gains: d.gains, gravityVolts: fit.model.kG }, window));
  assert.ok(current.metrics.meanHoldError > 10, `sags: ${current.metrics.meanHoldError}`);
  assert.ok(Math.abs(proposed.metrics.meanHoldError) < current.metrics.meanHoldError / 3,
    `holds: ${proposed.metrics.meanHoldError}`);
  assert.ok(proposed.metrics.withinFraction > current.metrics.withinFraction);
  // The replay with the gains the robot had should look like what it did.
  let diff = 0;
  for (let i = 0; i < current.x.length; ++i) diff += Math.abs(current.x[i] - current.recorded[i]);
  assert.ok(diff / current.x.length < 40, `model vs recording: ${diff / current.x.length}`);
});

test('what-if: turn steps replay from their recorded first error', () => {
  const turn = T.systems(MATCH).find((s) => s.axisName === 'Turn');
  const fit = T.fitPassive(turn, T.driveSamples(MATCH, turn, 37.5, 142.5));
  const controller = turn.controllers[0];
  const steps = T.replaySteps(MATCH, turn, fit, controller, {
    gains: T.gainsAt(MATCH, controller, 20), t0: 20, t1: 35 });
  assert.ok(steps.length >= 2);
  assert.ok(Math.abs(steps[0].e0 - 90) < 1, `first turn: ${steps[0].e0}`);
  const d = T.design(fit, controller);
  const better = T.replaySteps(MATCH, turn, fit, controller, { gains: d.gains, t0: 20, t1: 35 });
  assert.ok(better[0].metrics.overshootFraction < steps[0].metrics.overshootFraction,
    'the designed gains overshoot less than the hand-picked ones');
});

test('C++ snippets name what to paste where', () => {
  const lift = T.systems(PIT).find((s) => s.kind === 'mechanism');
  const fit = T.fitAutotune(lift, lift.autotuneRuns[0]);
  const d = T.design(fit, lift.controllers[0]);
  const text = T.mechanismSnippet(lift, fit, d.gains, fit.model.kG, 'SL000041.CSV');
  assert.match(text, /constexpr sapphirelib::PIDGains kLiftGains\{\.kP = [\d.]+, \.kI = 0\.0, \.kD = [\d.]+\};/);
  assert.match(text, /constexpr double kLiftGravityVolts = 2\.2\d*;/);
  const fits = {};
  const designs = {};
  for (const s of T.systems(PIT).filter((x) => x.kind === 'drive')) {
    fits[s.axisName] = T.fitAutotune(s, s.autotuneRuns[0]);
    for (const c of s.controllers) designs[c.name] = T.design(fits[s.axisName], c);
  }
  const drive = T.driveSnippet(fits, designs, 'SL000041.CSV');
  assert.match(drive, /drivetrain\(\)\.setAxisModels\(\{/);
  assert.match(drive, /\.turn = \{\.kS = /);
  assert.match(drive, /headingHoldPID\(\)\.setGains/);
});
