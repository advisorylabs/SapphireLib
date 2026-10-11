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

test('Rng.fastGaussian: the same ziggurat as random.hpp (random_test.cpp)', () => {
  const z = L.zigguratTables();
  assert.deepEqual([z.k[0], z.k[1], z.k[2], z.k[64], z.k[127]], [15555140, 0, 12590646, 16628623, 15707337]);
  near(z.w[0], 2.2131718675747815e-07, 1e-20, 'w[0]');
  near(z.w[127], 2.0519613360756637e-07, 1e-20, 'w[127]');
  near(z.f[1], 0.96359969312708615, 1e-14, 'f[1]');
  near(z.f[127], 0.0026696290838809228, 1e-16, 'f[127]');
  const rng = new L.Rng(7);
  for (const value of [0.25539103897165999, -1.2118758829994307, -0.9925865667999193, -0.15473818772236891,
    0.79863601552382013, -0.84043943732603632]) {
    near(rng.fastGaussian(), value, 1e-12, 'fastGaussian');
  }
  // and it is a normal distribution, tails included (the tail past 3.44 is its own path)
  const r = new L.Rng(12345);
  const n = 400000;
  let sum = 0;
  let squares = 0;
  let pastTail = 0;
  for (let i = 0; i < n; ++i) {
    const v = r.fastGaussian();
    sum += v;
    squares += v * v;
    if (Math.abs(v) > 3.442619855899) pastTail++;
  }
  near(sum / n, 0, 0.006, 'mean');
  near(Math.sqrt(squares / n), 1, 0.006, 'standard deviation');
  near(pastTail / n, 0.0005761, 4 * 3.8e-5, 'tail');
});

test('LookupTable: samples, straight lines between, clamped ends (lookup_table_test.cpp)', () => {
  const square = new L.LookupTable(-2, 2, 4, (x) => x * x);
  assert.equal(square.at(-2), 4);
  assert.equal(square.at(0.25), 0.0625);
  near(square.at(0.375), (0.0625 + 0.25) / 2, 1e-15, 'between samples');
  const line = new L.LookupTable(0, 1, 8, (x) => 3 * x + 1);
  assert.equal(line.at(-5), 1);
  assert.equal(line.at(9), 4);
  assert.equal(line.at(NaN), 1);
  const exp = new L.LookupTable(-32, 0, 64, Math.exp);
  let worst = 0;
  for (let x = -32; x <= 0; x += 0.00731) worst = Math.max(worst, Math.abs(exp.at(x) - Math.exp(x)) / Math.exp(x));
  assert.ok(worst <= 3.1e-5, `exp table off by ${worst}`);
});

test('ParallelRayCaster: castRayIn() for parallel rays (field_map_test.cpp)', () => {
  const map = L.FieldMap.centered();
  map.addBox(-5, 20, 5, 30);
  map.addBox(60, -10, 80, 10);
  map.addSegment({ x1In: -40, y1In: -40, x2In: -20, y2In: -50 });
  const caster = new L.ParallelRayCaster();
  let worst = 0;
  for (let heading = 0; heading < 360; heading += 15) {
    const rad = heading * Math.PI / 180;
    const dirs = [[Math.sin(rad), Math.cos(rad)]];
    if (heading % 90 === 0) dirs.push([[0, 1], [1, 0], [0, -1], [-1, 0]][heading / 90]);
    for (const [dirX, dirY] of dirs) {
      caster.aim(map, dirX, dirY);
      for (let x = -90.3; x < 90; x += 7.7) {
        for (let y = -90.13; y < 90; y += 9.1) {
          const expected = map.castRayIn(x, y, dirX, dirY);
          const got = caster.castIn(x, y);
          if (!Number.isFinite(expected)) assert.equal(got, Infinity, `(${x}, ${y}) heading ${heading}`);
          else worst = Math.max(worst, Math.abs(got - expected));
        }
      }
    }
  }
  assert.ok(worst < 1e-9, `off by ${worst}`);
});

test('ReadingScorer: readingLogLikelihood() by lookup table (sensor_model_test.cpp)', () => {
  let worst = 0;
  for (const outlierProbability of [0, 0.001, 0.1, 0.5, 1]) {
    for (const sigma of [0.05, 0.6, 2, 5]) {
      const beam = L.beamModel({ outlierProbability });
      for (let m = 1; m < 80; m += 7.3) {
        const scorer = new L.ReadingScorer(m, sigma, beam);
        for (let e = 0; e < 90; e += 0.37) {
          const exact = L.readingLogLikelihood(m, e, sigma, beam);
          const fast = scorer.logLikelihood(e);
          if (exact < -700) assert.ok(fast < -690);
          else worst = Math.max(worst, Math.abs(fast - exact));
        }
      }
    }
  }
  assert.ok(worst < 1e-5, `off by ${worst}`);
  assert.equal(new L.ReadingScorer().logLikelihood(12), 0);
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
  near(e.xIn, -16.107423809228, 1e-9, 'x');
  near(e.yIn, 14.987853574406, 1e-9, 'y');
  near(e.spreadIn, 1.075714917576, 1e-9, 'spread');
  near(e.effectiveParticles, 62.293936038009, 1e-9, 'effective particles');
  near(filter.particles[0].xIn, -16.741012233836, 1e-9, 'particle 0 x');
  near(filter.particles[0].yIn, 16.985146738946, 1e-9, 'particle 0 y');
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

test('sampleParticles: picks by weight, like particle_filter_test.cpp', () => {
  const particles = [];
  for (let i = 0; i < 10; ++i) particles.push({ xIn: i, yIn: 0, weight: 0.5 / 9 });
  particles[3].weight = 0.5;
  const picks = L.sampleParticles(particles, 8);
  assert.equal(picks.filter((p) => p.xIn === 3).length, 4);
  for (const p of particles) p.weight = 0.1;
  assert.deepEqual(L.sampleParticles(particles, 5).map((p) => p.xIn), [0, 2, 4, 6, 8]);
  assert.deepEqual(L.sampleParticles([], 3), []);
});

test('ParticleFilter.recoveryFit: primed by the first weigh, and a bump drops it', () => {
  const map = L.FieldMap.centered();
  const filter = new L.ParticleFilter(map, fourSensors(), { seed: 4 });
  filter.reset(0, 0, 0.5);
  filter.weigh(0, exactReadings(map, fourSensors(), 0, 0, 0));
  const first = filter.recoveryFit();
  assert.ok(first.latest > 0.3 && first.latest <= 1);
  assert.equal(first.fast, first.latest);
  assert.equal(first.slow, first.latest);
  filter.weigh(0, exactReadings(map, fourSensors(), 8, 0, 0));
  const bumped = filter.recoveryFit();
  assert.ok(bumped.latest < 0.5 * first.latest && bumped.fast < bumped.slow);
});
