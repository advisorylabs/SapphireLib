// Tests for js/mcl.js: the port of the robot's localization math. Run:
// node --test tools/sim/test/*.test.js
//
// The golden values come from the C++ unit tests (tests/util/random_test.cpp,
// tests/localization/*_test.cpp, tests/odom/odometry_math_test.cpp), so the
// simulator is held to exactly what the robot computes.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const L = require('../js/mcl.js');

function near(actual, expected, tolerance, label) {
  assert.ok(Math.abs(actual - expected) <= tolerance,
    `${label}: ${actual} vs ${expected} (±${tolerance})`);
}

function fourSensors() {
  return [L.mount(7, 0, 0), L.mount(0, 7, 90), L.mount(-7, 0, 180), L.mount(0, -7, 270)];
}

// readingsFrom() in particle_filter_test.cpp, exact readings only
function exactReadings(map, mounts, x, y, heading) {
  const beam = L.beamModel();
  return mounts.map((m) => {
    const ray = L.sensorRay(x, y, heading, m);
    const d = map.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
    return L.reading(d, d <= beam.maxRangeIn);
  });
}

test('Rng: the same sequence as random.hpp (random_test.cpp)', () => {
  const rng = new L.Rng(42);
  for (const value of [2837322924, 544945897, 479756282, 3500138142, 339756180]) {
    assert.equal(rng.next(), value);
  }
  const u = new L.Rng(7);
  assert.equal(u.uniform(), 0.23382771760225296);
  assert.equal(u.uniform(), 0.51223241887055337);
  const g = new L.Rng(7);
  near(g.gaussian(), -1.585033621793738, 1e-12, 'gaussian 1');
  near(g.gaussian(), -0.19791852691893735, 1e-12, 'gaussian 2');
});

test('angle wrapping matches angle.cpp', () => {
  assert.equal(L.wrapDegrees180(190), -170);
  assert.equal(L.wrapDegrees180(-180), 180);
  assert.equal(L.wrapDegrees180(540), 180);
  assert.equal(L.wrapDegrees360(-10), 350);
  assert.equal(L.wrapDegrees360(725), 5);
});

test('FieldMap: raycasts (field_map_test.cpp)', () => {
  const map = L.FieldMap.centered();
  const half = L.kVrcFieldSizeIn / 2;
  near(map.castRayAtHeadingIn(0, 0, 0), half, 1e-9, 'facing +y');
  near(map.castRayAtHeadingIn(10, -20, 0), half + 20, 1e-9, 'off center, +y');
  near(map.castRayAtHeadingIn(10, -20, 90), half - 10, 1e-9, 'off center, +x');
  const box = L.FieldMap.centered(140);
  box.addBox(-5, 20, 5, 30);
  near(box.castRayAtHeadingIn(0, 0, 0), 20, 1e-9, 'box in front');
  near(box.castRayAtHeadingIn(6, 0, 0), 70, 1e-9, 'beside the box');
  near(box.castRayAtHeadingIn(5, 0, 0), 20, 1e-9, 'box corner');
  assert.equal(new L.FieldMap(0, 0, 10, 10).castRayAtHeadingIn(-5, 5, 270), Infinity);
});

test('sensor model (sensor_model_test.cpp)', () => {
  const ray = L.sensorRay(0, 0, 90, L.mount(7, 2, 0));
  near(ray.xIn, 7, 1e-9, 'x');
  near(ray.yIn, -2, 1e-9, 'y');
  const beam = L.beamModel();
  near(L.readingSigmaIn(beam, L.reading(40, true, 1.5)), 2.5, 1e-9, 'sigma');
  const peak = 0.9 / (Math.sqrt(2 * Math.PI) * 2.0) + 0.1 / 78.0;
  near(L.readingLogLikelihood(40, 40, 2.0, beam), Math.log(peak), 1e-12, 'peak');
  near(L.readingLogLikelihood(40, Infinity, 2.0, beam), Math.log(0.1 / 78), 1e-12, 'no hit');
  const good = L.distanceReadingFromMm(1016, 63, 20, beam);
  assert.ok(good.valid);
  near(good.distanceIn, 40, 1e-12, '1016mm');
  assert.ok(!L.distanceReadingFromMm(9999, 63, 20, beam).valid);
  assert.ok(!L.distanceReadingFromMm(2147483647, 2147483647, 20, beam).valid);
  assert.ok(!L.distanceReadingFromMm(1500, 10, 20, beam).valid);
  assert.ok(L.distanceReadingFromMm(150, 0, 20, beam).valid);
  const now = L.compensateLatency(L.reading(30, true, 0), 40, 0.03);
  near(now.distanceIn, 28.8, 1e-12, 'latency');
  near(now.extraSigmaIn, 0.6, 1e-12, 'latency sigma');
});

test('correctionStep (odometry_math_test.cpp)', () => {
  let step = L.correctionStep(3, -4, 0.5);
  near(step.dxIn, 0.3, 1e-12, 'dx');
  near(step.dyIn, -0.4, 1e-12, 'dy');
  step = L.correctionStep(0.3, -0.4, 1);
  assert.deepEqual(step, { dxIn: 0.3, dyIn: -0.4 });
  step = L.correctionStep(30, 40, Infinity);
  assert.deepEqual(step, { dxIn: 30, dyIn: 40 });
});

test('computeOdometryDelta (odometry_math_test.cpp)', () => {
  let d = L.computeOdometryDelta(90, 90, 10, 0, 0, 0);
  near(d.dxIn, 10, 1e-9, 'forward at 90');
  near(d.dyIn, 0, 1e-9, 'forward at 90');
  // pure rotation with the right offset doesn't move
  const offset = 3;
  d = L.computeOdometryDelta(0, 90, 0, offset * Math.PI / 2, 0, offset);
  near(d.dxIn, 0, 1e-9, 'rotation');
  near(d.dyIn, 0, 1e-9, 'rotation');
});

test('ParticleFilter: the golden seeded run (particle_filter_test.cpp testGoldenRun)', () => {
  const map = L.FieldMap.centered();
  const filter = new L.ParticleFilter(map, fourSensors(), { particleCount: 64, seed: 1234 });
  filter.reset(-20, 15, 3.0);
  for (let step = 0; step < 6; ++step) {
    filter.predict(0.4, -0.2, 3.0);
    const readings = exactReadings(map, fourSensors(), -18 + 0.4 * step, 16 - 0.2 * step, 25 + 3 * step);
    readings[2].valid = step % 3 !== 1;
    filter.weigh(25 + 3 * step, readings);
    filter.resampleIfNeeded();
  }
  const e = filter.estimate();
  near(e.xIn, -16.074287648507, 1e-9, 'x');
  near(e.yIn, 14.950802327907, 1e-9, 'y');
  near(e.spreadIn, 1.064527667393, 1e-9, 'spread');
  near(e.effectiveParticles, 62.107761325610, 1e-9, 'effective particles');
  near(filter.particles[0].xIn, -16.682482235779, 1e-9, 'particle 0 x');
  near(filter.particles[0].yIn, 17.062945229742, 1e-9, 'particle 0 y');
});

test('ParticleFilter: converges on the truth from a wide start', () => {
  const map = L.FieldMap.centered();
  const filter = new L.ParticleFilter(map, fourSensors());
  filter.reset(0, 0, 8);
  const readings = exactReadings(map, fourSensors(), 6, -4, 30);
  for (let i = 0; i < 20; ++i) {
    filter.predict(0, 0, 0);
    assert.ok(filter.weigh(30, readings));
    filter.resampleIfNeeded();
  }
  const e = filter.estimate();
  assert.ok(Math.hypot(e.xIn - 6, e.yIn + 4) < 0.3);
});

test('ParticleFilter: checkSensors flags a blocked sensor', () => {
  const map = L.FieldMap.centered(140);
  const filter = new L.ParticleFilter(map, fourSensors());
  const readings = exactReadings(map, fourSensors(), 0, 0, 0);
  readings[0].distanceIn = 20;
  readings[3].valid = false;
  const { agreeing, checks } = filter.checkSensors(0, 0, 0, readings, 3.0);
  assert.equal(agreeing, 2);
  assert.ok(checks[0].used && !checks[0].agrees);
  near(checks[0].expectedIn, 63, 1e-9, 'front expected');
  assert.ok(!checks[3].used);
});
