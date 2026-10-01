// Tests for the MCL auto-tuner (js/tuner.js), the log recorder
// (js/recorder.js), and calibrating from logs (tools/analyzer/js/mclcal.js).
// Run: node --test tools/sim/test/*.test.js
//
// Calibration is checked the only way it can be, on logs whose world is known
// exactly: the simulator records a run as the robot would log it, and
// mclcal.js has to find that world again. The search is checked by starting it
// somewhere bad and seeing it climb out, on seeds it never saw.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const T = require('../js/tuner.js');
const W = require('../js/world.js');
const L = require('../js/mcl.js');
const cal = require('../../analyzer/js/mclcal.js');
const TF = require('../../analyzer/js/tunefile.js');

const KNOWN = { sensorNoiseScale: 2, sensorDropoutPct: 8, sensorOutlierPct: 5, sensorDelayMs: 40,
  wheelDiameterErrorPct: 2.5 };

function logsIn(world, { seeds = [11, 12, 13], extra = {} } = {}) {
  return ['tour', 'laps', 'sprints'].map((routine, i) =>
    T.recordedLog(Object.assign({ id: routine, routine, world }, extra), T.defaultConfig(), seeds[i], 900001 + i));
}

test('the tuner and the robot agree on every default', () => {
  const sim = T.defaultConfig();
  for (const s of TF.LOCALIZER_SETTINGS) assert.equal(sim[s.key], s.default, s.key);
  assert.equal(sim.periodMs, TF.DEFAULT_PERIOD_MS);
});

test('a recorded run reads like a robot log: channels, meta, beams', () => {
  const [log] = logsIn({});
  assert.equal(log.meta['mcl.filter.particleCount'], '300');
  assert.equal(log.meta['mcl.sensor2'], '-7,0,180');
  for (const name of ['odom', 'mcl', 'mcl.beams', 'chassis']) assert.ok(log.get(name), name);
  assert.deepEqual(log.get('mcl').columns, ['x', 'y', 'spread', 'neff', 'used', 'agree', 'correcting', 'corr_x',
    'corr_y', 'us', 'raw_x', 'raw_y']);
  const beams = log.get('mcl.beams');
  assert.ok(beams.length > 300);
  // expected is worked out for every sensor, a reading only where there was one
  let missing = 0;
  for (let i = 0; i < beams.length; ++i) {
    for (let s = 0; s < 4; ++s) {
      assert.ok(!Number.isNaN(beams.cols[`e${s}`][i]));
      if (Number.isNaN(beams.cols[`m${s}`][i])) missing++;
    }
  }
  assert.ok(missing > 0, 'out-of-range sensors log no reading');
  assert.equal(log.events.filter((e) => e.tag === 'rec').length, 2);
});

test('calibration finds the world a log came from', () => {
  const c = cal.calibrate(logsIn(KNOWN));
  assert.ok(c.ok, c.problems.join('; '));
  assert.ok(Math.abs(c.world.sensorNoiseScale - 2) < 0.3, `noise scale ${c.world.sensorNoiseScale}`);
  assert.ok(Math.abs(c.world.sensorDropoutPct - 8) < 2.5, `dropouts ${c.world.sensorDropoutPct}%`);
  assert.ok(Math.abs(c.world.sensorOutlierPct - 5) < 2, `outliers ${c.world.sensorOutlierPct}%`);
  assert.ok(Math.abs(c.world.wheelDiameterErrorPct - 2.5) < 0.6, `wheel scale ${c.world.wheelDiameterErrorPct}%`);
  // a wheel that reads 2.5% long is really 1/1.025 the configured diameter
  assert.ok(Math.abs(c.odometry.verticalDiameterScale - 1 / 1.025) < 0.008);
  // outliers in this world are things nearer than the wall
  assert.ok(c.outliers.shortShare > 0.9);
  // the robot compensates 30ms of the ~56ms readings really are old: what's left shows, partly
  assert.ok(c.latency.ok && c.latency.measuredMs > 36, `latency at least ${c.latency.measuredMs}ms`);
  // sensors mounted where the program thinks: no bias
  for (const s of c.sensors) assert.ok(Math.abs(s.medianIn) < 0.3, `sensor ${s.index} bias ${s.medianIn}`);
  assert.ok(c.trajectories.length >= 3);
});

test('a clean world calibrates as clean', () => {
  const c = cal.calibrate(logsIn({ sensorOutlierPct: 0, sensorDropoutPct: 0, wheelDiameterErrorPct: 0 }));
  assert.ok(c.world.sensorOutlierPct < 0.5, `outliers ${c.world.sensorOutlierPct}%`);
  assert.ok(c.world.sensorDropoutPct < 0.5, `dropouts ${c.world.sensorDropoutPct}%`);
  assert.ok(Math.abs(c.world.wheelDiameterErrorPct) < 0.5, `wheel scale ${c.world.wheelDiameterErrorPct}%`);
});

test('a sensor mounted an inch off from what the code says reads as a bias', () => {
  // the back sensor's face is really 8in behind center; the program thinks 7, so it reads an inch
  // short of what the map says. The estimate gives a little ground to it, so about half the inch
  // shows, all of it on the sensor that's wrong
  const believed = W.defaultSettings().sensors;
  const real = { sensors: believed.map((m, i) => (i === 2 ? L.mount(-8, 0, 180) : m)) };
  const c = cal.calibrate(logsIn(real, { extra: { localizerMounts: believed } }));
  const back = c.sensors.find((s) => s.index === 2);
  assert.ok(back.medianIn < -0.35, `back ${back.medianIn}`);
  for (const s of c.sensors.filter((x) => x.index !== 2)) {
    assert.ok(Math.abs(s.medianIn) < 0.3, `sensor ${s.index} ${s.medianIn}`);
  }
});

test('replaying a logged path walks the robot along it, and MCL tracks it', () => {
  const [log] = logsIn({});
  const [trajectory] = cal.trajectoriesOf(log, cal.settingsOf(log));
  const routine = T.replayRoutine(trajectory, 15);
  const metrics = T.runCase({ id: 'replay', routine, world: {} }, T.defaultConfig(), 3);
  assert.ok(metrics.finished);
  assert.ok(Math.abs(metrics.seconds - 15) < 0.2, `ran ${metrics.seconds}s`);
  assert.ok(metrics.meanIn < 1.0, `mean error ${metrics.meanIn}`);

  // the world's truth is the path itself
  const world = new W.World(W.defaultSettings(), trajectory.start);
  world.setScript(routine.script);
  world.step(5000);
  const at = routine.script.find((s) => s.tMs >= 5000);
  assert.ok(Math.hypot(world.truth.xIn - at.xIn, world.truth.yIn - at.yIn) < 0.2);
});

test('the sensor delay is matched by simulating, not taken from the leftover', () => {
  const steps = T.calibrateWorld(logsIn(KNOWN));
  let step;
  let runs = 0;
  while (!(step = steps.next()).done) runs++;
  const { latency, world, start } = step.value;
  assert.ok(runs > 20);
  assert.ok(latency.ok, 'matched');
  // the true delay is 40ms; a few minutes of logs pin it to within about +-10
  assert.ok(Math.abs(latency.delayMs - 40) < 20, `matched ${latency.delayMs}ms`);
  assert.ok(Math.abs(latency.delayMs - 40) < 3 * latency.stderrMs + 5, `${latency.delayMs} +- ${latency.stderrMs}`);
  assert.equal(world.sensorDelayMs, Math.round(latency.delayMs));
  // the search starts from what the robot was running
  assert.equal(start['filter.particleCount'], 300);
});

test('the search climbs out of a bad start, and it holds on seeds it never saw', () => {
  const bad = Object.assign(T.defaultConfig(), {
    'filter.motionNoise.perInch': 0.01, 'filter.beam.outlierProbability': 0.015, 'filter.beam.minSigmaIn': 0.25,
  });
  const cases = T.standardCases().filter((c) => c.id === 'worn' || c.id === 'crowded');
  const result = T.finish(T.search({
    start: bad, cases, budget: 24, seeds: [7, 11], holdoutSeeds: [101, 202],
    params: ['filter.motionNoise.perInch', 'filter.beam.outlierProbability', 'filter.beam.minSigmaIn'],
  }));
  assert.ok(result.bestScore < 0.85, `score ${result.bestScore}`);
  assert.equal(result.holdout.verdict, 'better', JSON.stringify(result.holdout));
  const perInch = result.changes.find((c) => c.key === 'filter.motionNoise.perInch');
  assert.ok(perInch && perInch.to > 0.02, 'doubts odometry more');
  assert.ok(result.evaluations <= 24);
  assert.equal(result.history[0].score, 1);
  for (let i = 1; i < result.history.length; ++i) assert.ok(result.history[i].score <= result.history[i - 1].score);

  // and it's a TUNE.CFG the robot accepts
  const profile = TF.revise(TF.emptyProfile(), T.tuneChanges(result.best, TF.emptyProfile()),
    { source: 'sim tuner' }).profile;
  const back = TF.parse(TF.format(profile));
  assert.ok(back.ok, back.error);
  assert.equal(back.profile.mcl['filter.motionNoise.perInch'], result.best['filter.motionNoise.perInch']);
});

test('settings that cost CPU have to pay for it', () => {
  const start = T.defaultConfig();
  assert.equal(T.cpuRatio(start, start), 1);
  assert.equal(T.cpuRatio(Object.assign({}, start, { 'filter.particleCount': 600 }), start), 2);
  assert.equal(T.cpuRatio(Object.assign({}, start, { periodMs: 100 }), start), 0.5);
});

test('paired comparisons cancel shared noise', () => {
  const a = [{ errors: [1.0, 2.0, 0.5] }, { errors: [0.8, 0.8] }];
  const b = [{ errors: [1.1, 2.2, 0.55] }, { errors: [0.88, 0.88] }];
  const p = T.paired(a, b);
  assert.ok(Math.abs(p.mean - Math.log(1 / 1.1)) < 1e-12);
  assert.ok(p.se < 1e-12);
  assert.equal(p.n, 5);
});

test('a recorded run logs mcl.state every update and mcl.pts every 5th, like telemetry.cpp', () => {
  const log = T.recordedLog({ id: 'tour', routine: 'tour', world: {} }, T.defaultConfig(), 3, 900050);
  const mcl = log.channels.find((c) => c.name === 'mcl');
  const state = log.channels.find((c) => c.name === 'mcl.state');
  const points = log.channels.find((c) => c.name === 'mcl.pts');
  assert.ok(mcl && state && points);
  assert.equal(state.length, mcl.length);
  // four rows of six particles at every 5th update, starting with the first
  assert.equal(points.length, 4 * Math.ceil(mcl.length / 5));
  // offsets from the estimate: a converged cloud is within a few inches of it
  let far = 0;
  for (let i = 0; i < points.length; ++i) {
    for (let k = 0; k < 6; ++k) if (Math.hypot(points.cols[`dx${k}`][i], points.cols[`dy${k}`][i]) > 6) far++;
  }
  assert.ok(far < 0.02 * 6 * points.length, `${far} particles more than 6in from the estimate`);
  // correcting rows say nothing blocked them; the others say why
  for (let i = 0; i < mcl.length; ++i) assert.equal(mcl.cols.correcting[i] === 1, state.cols.blocked[i] === 0);
  assert.ok(state.cols.fit.some((v) => v > 0.2));
});
