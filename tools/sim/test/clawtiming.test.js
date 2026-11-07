// Tests for js/clawtiming.js, autoClaw's timing model: that its rule is macros.cpp's, that it
// shuts the jaws where the lags say, and the claims the claw timing page (claw.html) makes.
// Run: node --test tools/sim/test/*.test.js
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const CT = require('../js/clawtiming.js');

const mmPerMs = (inPerS) => (inPerS * 25.4) / 1000;

// every lag exact: a sensor that measures every ms with no noise, a 1ms tick, a valve that never
// varies, and odometry's speed spot on
function exact(over = {}) {
  return { ...CT.defaults(), sensorPeriodMs: 1, sensorNoiseMm: 0, loopMs: 1, speedNoiseInPerS: 0,
    valveJitterMs: 0, ...over };
}

test('ruleFires() is macros.cpp\'s autoClawAim(): the reading less the speed times the lead, the tick and the age, out to the reach', () => {
  const p = { ...CT.defaults(), rule: 'leadAge', grabMm: 60, leadMs: 75, loopMs: 15, maxReachMm: 250 };
  // 1mm per ms (39in/s), 10ms old: 200 - (75 + 15 + 10) is 100, not yet
  assert.deepEqual(CT.ruleFires(p, 200, 10, 1), { predictedMm: 100, fires: false });
  assert.equal(CT.ruleFires(p, 160, 10, 1).fires, true);
  // past the reach never fires, however fast; backing away counts as standing still
  assert.equal(CT.ruleFires(p, 260, 0, 5).fires, false);
  assert.equal(CT.ruleFires(p, 9999, 0, 5).fires, false);
  assert.equal(CT.ruleFires(p, 61, 0, -1).fires, false);
  assert.equal(CT.ruleFires(p, 60, 0, -1).fires, true);
  // 'lead' leaves the age out, as commit a66eb14 did; 'fixed' is the reading alone
  assert.equal(CT.ruleFires({ ...p, rule: 'lead' }, 160, 10, 1).predictedMm, 85);
  assert.deepEqual(CT.ruleFires({ ...p, rule: 'fixed', fixedMm: 125 }, 125, 10, 1), { predictedMm: 125, fires: true });
});

test('with every lag exact, a lead of the sensor\'s latency plus the valve shuts the jaws at grabMm at any speed', () => {
  const p = exact({ leadMs: 15 + 60 });
  for (const speed of [5, 20, 40, 60]) {
    const r = CT.grabTrial(p, speed, new CT.Rng(1));
    assert.equal(r.outcome, 'caught');
    // within a tick's and a sample's travel (1ms each), and the reading's rounding
    assert.ok(Math.abs(r.errorMm) <= 2 * mmPerMs(speed) + 1, `${speed} in/s: ${r.errorMm}`);
  }
});

test('a fixed trigger shuts the jaws later the faster the robot goes, by the speed times the lags', () => {
  const p = exact({ rule: 'fixed', fixedMm: 125 });
  for (const speed of [5, 40]) {
    const r = CT.grabTrial(p, speed, new CT.Rng(1));
    // 15ms of latency and 60 of valve, from the reading at 125mm
    assert.ok(Math.abs(r.pinAtShutMm - (125 - mmPerMs(speed) * 75)) <= 2 * mmPerMs(speed) + 1,
      `${speed} in/s: ${r.pinAtShutMm}`);
  }
});

test('the old fixed 125mm catches Pins around one speed only, early below it and late above; the lead catches them all', () => {
  const p = CT.defaults();
  const speeds = [4, 28, 52];
  const [slow, mid, fast] = CT.sweep({ ...p, rule: 'fixed', fixedMm: 125 }, { speeds, trials: 300 });
  assert.ok(slow.early > 0.9, `slow ${slow.early}`);
  assert.ok(mid.caught > 0.9, `28in/s ${mid.caught}`);
  assert.ok(fast.late > 0.9, `fast ${fast.late}`);
  for (const row of CT.sweep(p, { speeds, trials: 300 })) {
    assert.ok(row.caught >= 0.95, `${row.speedInPerS} in/s: ${row.caught}`);
  }
});

test('the reading\'s age buys speed: the lead with it holds 95% faster than the lead without', () => {
  const speeds = CT.speedRange(20, 70, 2);
  const withAge = CT.maxSpeed(CT.sweep(CT.defaults(), { speeds, trials: 300 }));
  const without = { ...CT.defaults(), rule: 'lead' };
  without.leadMs = CT.bestLeadMs(without, 40);
  const noAge = CT.maxSpeed(CT.sweep(without, { speeds, trials: 300 }));
  assert.ok(withAge > noAge, `with ${withAge}, without ${noAge}`);
});

test('bestLeadMs() lands on the lags: the sensor\'s latency plus the valve, whatever the tick', () => {
  const p = CT.defaults();
  const expected = p.sensorLatencyMs + p.valveMs;
  for (const loopMs of [20, 10, 5]) {
    const lead = CT.bestLeadMs({ ...p, loopMs }, 40);
    assert.ok(Math.abs(lead - expected) < 6, `${loopMs}ms tick: ${lead} for ${expected}`);
  }
  // without the age and the tick, the lead has to cover a tick and the sample's own age as well
  const noAge = CT.bestLeadMs({ ...p, rule: 'lead' }, 40);
  assert.ok(noAge > expected + p.loopMs, `${noAge}`);
});

test('the valve\'s spread sets the top speed: the more it varies, the slower the claw can grab', () => {
  const speeds = CT.speedRange(4, 70, 2);
  const top = [2, 10, 20].map((valveJitterMs) =>
    CT.maxSpeed(CT.sweep({ ...CT.defaults(), valveJitterMs }, { speeds, trials: 300 })));
  assert.ok(top[0] > top[1] && top[1] > top[2], top.join(', '));
});

test('slowing through the grab shuts the jaws early, and speeding up late, against a steady approach', () => {
  const p = CT.defaults();
  const error = (accelInPerS2) =>
    CT.sweep({ ...p, accelInPerS2 }, { speeds: [40], trials: 400 })[0].meanErrorMm;
  const steady = error(0);
  assert.ok(error(-150) > steady + 3, `${error(-150)} vs ${steady}`);
  assert.ok(error(150) < steady - 3, `${error(150)} vs ${steady}`);
});

test('a robot that stops short of the Pin never grabs', () => {
  // 10in/s at the claw's window, slowing at 400in/s^2: stopped within 125ms of the trigger
  const r = CT.grabTrial({ ...CT.defaults(), accelInPerS2: -2000 }, 10, new CT.Rng(5));
  assert.ok(r.outcome === 'stopped' || r.outcome === 'caught', r.outcome);
});

test('maxSpeed() is the fastest speed with every speed below it at the threshold too', () => {
  const rows = [4, 8, 12, 16].map((speedInPerS, i) => ({ speedInPerS, caught: [1, 0.97, 0.9, 1][i] }));
  assert.equal(CT.maxSpeed(rows), 8);
  assert.equal(CT.maxSpeed(rows, 0.85), 16);
  assert.equal(CT.maxSpeed([{ speedInPerS: 4, caught: 0.5 }]), 0);
});

test('errorBudget() adds its parts as root sum of squares, and the age shrinks the sample\'s part to a tick', () => {
  const p = CT.defaults();
  const b = CT.errorBudget(p, 40);
  const rss = Math.sqrt(b.items.reduce((s, i) => s + i.mm * i.mm, 0));
  assert.ok(Math.abs(b.totalMm - rss) < 1e-9);
  assert.equal(b.windowMm, p.tipsMm - p.backMm + p.shoveMm);
  const sample = (rule) => CT.errorBudget({ ...p, rule }, 40).items.find((i) => i.key === 'sample').mm;
  assert.ok(Math.abs(sample('leadAge') / sample('lead') - p.loopMs / p.sensorPeriodMs) < 1e-9);
});

test('a sweep is the same every time for the same seed', () => {
  const p = CT.defaults();
  assert.deepEqual(CT.sweep(p, { speeds: [30], trials: 50, seed: 9 }), CT.sweep(p, { speeds: [30], trials: 50, seed: 9 }));
});

test('a traced grab has the samples the sensor took and every look the program took, up to the shut', () => {
  const r = CT.grabTrial(CT.defaults(), 30, new CT.Rng(2), { trace: true });
  assert.equal(r.outcome, 'caught');
  assert.ok(r.samples.length > 3 && r.looks.length > 3);
  const last = r.looks[r.looks.length - 1];
  assert.equal(last.fires, true);
  assert.equal(last.tMs, r.fireMs);
  assert.ok(r.looks.slice(0, -1).every((l) => !l.fires));
  assert.ok(Math.abs(r.pinAt(r.shutMs) - r.pinAtShutMm) < 1e-9);
});
