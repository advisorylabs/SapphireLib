// Tests for js/macros.js, the simulator's port of the lift, claw and extender calls autonomous
// routines make (include/robot/macros.hpp): the numbers src/robot/macros.cpp would give, and the
// Pins going where the claw puts them. Run: node --test tools/sim/test/*.test.js
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const SIM = require('../js/sim.js');
const A = require('../js/auton.js');
const M = require('../js/macros.js');
const F = require('../js/field.js');

const T = F.kTilePitchIn;

// Runs routine(a, m, sim) with the robot really at `start`, and returns the finished simulation
function run(start, routine, { preload = true, clawRule = null } = {}) {
  const sim = new SIM.Simulation(SIM.defaultOptions());
  sim.runRoutine({ name: 'test', start, *run(robot) {
    const a = A.create(robot);
    const m = M.create(robot, a, { preload, clawRule });
    a.setPose(start.xIn, start.yIn, start.headingDeg);
    yield* routine(a, m, sim);
  } });
  while (sim.program && sim.world.timeMs < 30000) sim.tick();
  assert.ok(!sim.program, 'the routine finished');
  return sim;
}

const kClear = { xIn: 0, yIn: -40, headingDeg: 0 };

test('liftScoreDeg() dips 30 degrees past halfway to the level below, as macros.cpp\'s scoreDipDeg()', () => {
  const dips = [];
  run(kClear, function* (a, m) {
    for (const level of [0, 1, 2, 3, 6]) {
      m.setLiftLevel(m.Goal.alliance, level);
      dips.push(m.liftScoreDeg());
    }
    // MEDIUM has no L1: it goes to L2, 425 degrees, and dips toward the bottom
    m.setLiftLevel(m.Goal.medium, 1);
    dips.push(m.liftScoreDeg());
    yield* a.delay(10);
  });
  // ALLIANCE's ladder is 0, 225, 370, 635, 910, 1170, 1430
  assert.deepEqual(dips, [0, 82.5, 267.5, 472.5, 1270, 182.5]);
});

test('the lift climbs to its level on its own loop, and waitForLift() says when it\'s there', () => {
  let arrived = null;
  let atMs = null;
  let early = null;
  const sim = run(kClear, function* (a, m, s) {
    m.setLiftLevel(m.Goal.alliance, 3); // 635 degrees
    early = yield* m.waitForLift(100);
    arrived = yield* m.waitForLift(1500);
    atMs = s.world.timeMs;
  });
  const st = sim.robot.macros.status();
  assert.equal(early, false);
  assert.equal(arrived, true);
  assert.ok(Math.abs(st.liftDeg - 635) <= 15, `${st.liftDeg}`);
  // full power up most of the way: well under a second
  assert.ok(atMs > 200 && atMs < 800, `there at ${atMs} ms`);
  assert.equal(st.level, 3);
});

test('down at the bottom, the lift rests on its stop', () => {
  const sim = run(kClear, function* (a, m) {
    m.setLiftLevel(m.Goal.alliance, 2);
    yield* m.waitForLift(1500);
    m.setLiftLevel(m.Goal.alliance, 0);
    yield* a.delay(1500);
  });
  assert.ok(sim.robot.macros.liftAtBottom());
});

test('opening the claw over a Goal with the lift up stacks the Pin at the Goal\'s next level', () => {
  // the claw holds its Pin kClawReachIn ahead: stopped that far east of the left red Goal, facing it
  const onGoal = { xIn: -2 * T + M.kClawReachIn, yIn: -T, headingDeg: 270 };
  const sim = run(onGoal, function* (a, m) {
    m.setLiftLevel(m.Goal.alliance, 1);
    yield* m.waitForLift(1500);
    m.setLiftDeg(m.liftScoreDeg());
    yield* m.waitForLift(1000);
    m.setClawClosed(false);
    yield* a.delay(200);
  });
  const scored = sim.robot.macros.scored();
  assert.equal(scored.length, 1);
  assert.equal(scored[0].goal, 'red goal (left)');
  assert.equal(scored[0].level, 1);
  assert.equal(scored[0].liftLevel, 1);
  assert.ok(scored[0].offIn < 0.05, `${scored[0].offIn}`);
});

test('a neutral Goal starts with its Pin, so the first one scored on it is its L2', () => {
  const onGoal = { xIn: -T, yIn: 2 * T - M.kClawReachIn, headingDeg: 0 };
  const sim = run(onGoal, function* (a, m) {
    m.setLiftLevel(m.Goal.medium, 2);
    yield* m.waitForLift(1500);
    m.setClawClosed(false);
    yield* a.delay(200);
  });
  const [p] = sim.robot.macros.scored();
  assert.equal(p.goal, 'neutral goal (top)');
  assert.equal(p.level, 2);
});

test('opening the claw with the lift down, or away from a Goal, leaves the Pin standing on the floor', () => {
  const sim = run(kClear, function* (a, m) {
    m.setClawClosed(false);
    yield* a.delay(200);
    yield* a.drive(-6);
  });
  assert.equal(sim.robot.macros.scored().length, 0);
  const dropped = sim.robot.macros.pins().find((p) => p.byRoutine);
  assert.equal(dropped.state, 'standing');
  assert.ok(Math.abs(dropped.yIn - (-40 + M.kClawReachIn)) < 0.2, `${dropped.yIn}`);
});

test('autoClaw: slow, and the claw fires early enough to shut on a Pin, then full speed again', () => {
  // facing the yellow Pin at (-T, -T) from 11in north, the claw open and empty
  const start = { xIn: -T, yIn: -T + 11, headingDeg: 180 };
  let limitDuring = null;
  const sim = run(start, function* (a, m) {
    m.setAutoClaw(true);
    limitDuring = a.speedLimit();
    // armed once the sensor has seen the claw empty for 400ms
    yield* a.delay(500);
    a.runAsync(function* () { yield* a.drive(7); });
    yield* a.waitUntil(() => m.clawClosed(), 3000);
    yield* a.waitForMotion();
    yield* a.delay(400);
  }, { preload: false });
  const m = sim.robot.macros;
  assert.equal(limitDuring, M.kClawSpeedVolts);
  assert.ok(m.clawClosed());
  assert.ok(m.pins().some((p) => p.state === 'held' && p.color === 'yellow'));
  // driving onto it, it fired on a reading further out than kGrabMm, aiming the jaws at kGrabMm:
  // the reading less the speed times the lead and the reading's age
  const grab = m.status().lastGrab;
  assert.ok(grab.readingMm > M.kGrabMm, `fired at ${grab.readingMm}mm`);
  assert.ok(grab.aimMm <= M.kGrabMm && grab.aimMm > M.kGrabMm - 15, `aimed at ${grab.aimMm}mm`);
  const aim = grab.readingMm - grab.approachMmPerMs * (M.kClawLeadMs + M.kAutonClawPollMs + grab.ageMs);
  assert.ok(Math.abs(aim - grab.aimMm) < 1e-9);
  assert.equal(m.autoClaw(), false);
  assert.equal(sim.robot.events.some((e) => /autoClaw off \(grabbed\)/.test(e.text)), true);
});

// From 20in out, drives onto the yellow Pin with autoClaw on and the drive capped at `volts`
// rather than clawSpeed's, on through where the Pin stands (the simulator's Pins don't stop the
// robot), so it's still at speed as it grabs; says how fast it went, what the sensor read as it
// fired, and whether the claw ended up holding the Pin
function grabAt(volts, clawRule = null) {
  const start = { xIn: -T, yIn: -T + 20, headingDeg: 180 };
  const sim = run(start, function* (a, m) {
    m.setAutoClaw(true);
    a.setSpeedLimit(volts);
    yield* a.delay(500);
    a.runAsync(function* () { yield* a.drive(32); });
    yield* a.waitUntil(() => m.clawClosed(), 3000);
    yield* a.waitForMotion();
    yield* a.delay(400);
  }, { preload: false, clawRule });
  const m = sim.robot.macros;
  const grab = m.status().lastGrab;
  return { inPerS: (grab.approachMmPerMs * 1000) / 25.4, readingMm: grab.readingMm,
    held: m.pins().some((p) => p.state === 'held' && p.color === 'yellow') };
}

test('autoClaw\'s lead: the faster the robot drives onto a Pin, the further out it fires, and it holds the Pin at every speed', () => {
  const grabs = [3, 5, 8, 12].map((volts) => grabAt(volts));
  for (const g of grabs) assert.ok(g.held, `missed at ${g.inPerS.toFixed(1)} in/s`);
  for (let i = 1; i < grabs.length; ++i) {
    assert.ok(grabs[i].inPerS > grabs[i - 1].inPerS + 5, `${grabs[i].inPerS} after ${grabs[i - 1].inPerS}`);
    assert.ok(grabs[i].readingMm > grabs[i - 1].readingMm, `${grabs[i].readingMm} after ${grabs[i - 1].readingMm}`);
  }
  // full speed reaches 40in/s, twice what clawSpeed's 5V does
  assert.ok(grabs[3].inPerS > 40, `${grabs[3].inPerS}`);
});

test('without the lead, a fixed trigger is right for one speed only: set for a slow grab it shuts behind the Pin at full speed, and set for full speed, in front of it slowly', () => {
  // about 95ms from a reading to the jaws shut: 24mm of travel at 3V, 100mm at 12V
  const setSlow = { rule: 'fixed', fixedMm: M.kGrabMm + 25 };
  const setFast = { rule: 'fixed', fixedMm: M.kGrabMm + 100 };
  assert.ok(grabAt(3, setSlow).held);
  assert.equal(grabAt(12, setSlow).held, false);
  assert.ok(grabAt(12, setFast).held);
  assert.equal(grabAt(3, setFast).held, false);
});

test('setAutoClaw(true, mm) grabs at that reading, whatever the speed, as a routine tuned at its own approach wants', () => {
  const start = { xIn: -T, yIn: -T + 20, headingDeg: 180 };
  const grabs = [3, 12].map((volts) => {
    const sim = run(start, function* (a, m) {
      m.setAutoClaw(true, 85);
      a.setSpeedLimit(volts);
      yield* a.delay(500);
      a.runAsync(function* () { yield* a.drive(32); });
      yield* a.waitUntil(() => m.clawClosed(), 3000);
      yield* a.waitForMotion();
    }, { preload: false });
    return sim.robot.macros.status().lastGrab;
  });
  for (const g of grabs) {
    // the first sample at 85mm or closer: within a sample period's travel of it
    const stepMm = g.approachMmPerMs * 33 + 1;
    assert.ok(g.readingMm <= 85 && g.readingMm > 85 - stepMm, `fired at ${g.readingMm}mm`);
  }
  // full speed: the lead would have fired well out past 85mm
  assert.ok(grabs[1].approachMmPerMs * (M.kClawLeadMs + M.kAutonClawPollMs) + M.kGrabMm > 120);
});

test('autoClaw waits for the claw to have been empty: started on a Pin, it doesn\'t shut on it', () => {
  // the Pin already between the open jaws, as just after letting go of it
  const onPin = { xIn: -T, yIn: -T + M.kClawReachIn, headingDeg: 180 };
  const sim = run(onPin, function* (a, m) {
    m.setAutoClaw(true);
    yield* a.delay(1000);
  }, { preload: false });
  assert.equal(sim.robot.macros.clawClosed(), false);
  assert.ok(sim.robot.macros.pinInClaw());
});
