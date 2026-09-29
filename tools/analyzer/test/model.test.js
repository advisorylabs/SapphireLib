// Tests for js/model.js — the analyzer's ports of the robot's tuning and
// control math. Run: node --test tools/analyzer/test/
//
// The golden case is the one in tests/tuning/characterization_math_test.cpp:
// the same deterministic data (no RNG, no libm — only + - * / and compares,
// so both languages build it bit for bit) must give the C++ numbers. That's
// what keeps the analyzer's refits honest about what Auto-Tune would do.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const M = require('../js/model.js');

function close(actual, expected, relative, label) {
  const tolerance = Math.abs(expected) * relative;
  assert.ok(Math.abs(actual - expected) <= tolerance,
    `${label}: got ${actual}, expected ${expected} ± ${tolerance}`);
}

// --- The golden case (keep in step with goldenRun() in the C++ test) ----------

function goldenNoise(i, seed) {
  return (((i * 7919 + seed * 104729) % 97) - 48) * 1e-4;
}

function goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, direction, volts, ramp, start, limit,
  maxTicks, seed) {
  const commands = [];
  const run = [];
  let x = start;
  let v = 0;
  for (let i = 0; i < maxTicks; ++i) {
    const reading = x + goldenNoise(i, seed);
    if (i >= holdTicks && (direction > 0 ? reading >= limit : reading <= limit)) break;
    let command = NaN;
    if (i >= holdTicks) {
      const t = (i - holdTicks) * 0.01;
      command = direction * (ramp ? Math.min(4.0 * t, volts) : volts);
    }
    run.push({ timeMs: i * 10, volts: command, position: reading });
    commands.push(command);
    const applied = i - delayTicks >= 0 ? commands[i - delayTicks] : NaN;
    if (Number.isNaN(applied)) {
      v = 0;
      continue;
    }
    for (let s = 0; s < 20; ++s) {
      const load = applied - kG;
      const friction = Math.abs(v) > 1e-3 ? kS * M.signOf(v)
        : M.signOf(load) * Math.min(Math.abs(load), kS);
      v += ((load - friction - kV * v) / kA) * 0.0005;
      x += v * 0.0005;
    }
  }
  return run;
}

function goldenData(kS, kV, kA, kG, delayTicks, holdTicks, lower, upper) {
  return {
    ramps: [
      goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, 1, 8, true, lower, upper, 300, 1),
      goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, -1, 8, true, upper, lower, 300, 2),
    ],
    steps: [
      goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, 1, 6, false, lower, upper, 200, 3),
      goldenRun(kS, kV, kA, kG, delayTicks, holdTicks, -1, 6, false, upper, lower, 200, 4),
    ],
  };
}

test('drive axis fit matches the C++ golden case', () => {
  const drive = M.characterizeAxis(goldenData(1.1, 0.14, 0.035, 0, 3, 0, -1000, 1000), 2.0);
  assert.ok(drive.ok);
  assert.equal(drive.fit.samplesUsed, 820);
  close(drive.fit.model.kS, 1.0763486609024009, 1e-12, 'kS');
  close(drive.fit.model.kV, 0.14048089905604738, 1e-12, 'kV');
  close(drive.fit.model.kA, 0.035163511411799429, 1e-12, 'kA');
  close(drive.fit.rSquared, 0.99998910846074862, 1e-12, 'R2');
  close(drive.delayS, 0.031434814419046525, 1e-12, 'delay');
});

test('lift fit (constant gravity, held pre-roll) matches the C++ golden case', () => {
  const lift = M.characterizeMechanism(goldenData(0.6, 0.017, 0.002, 2.0, 2, 10, 20, 700),
    M.gravityShape('constant'), 5.0);
  assert.ok(lift.ok);
  assert.equal(lift.fit.samplesUsed, 776);
  close(lift.fit.model.motion.kS, 0.56109754370967291, 1e-12, 'kS');
  close(lift.fit.model.motion.kV, 0.017076500587205532, 1e-12, 'kV');
  close(lift.fit.model.motion.kA, 0.0021459253802502522, 1e-12, 'kA');
  close(lift.fit.model.kG, 2.011164031791643, 1e-12, 'kG');
  close(lift.fit.rSquared, 0.9998817832001935, 1e-12, 'R2');
  close(lift.delayS, 0.018001926627715409, 1e-12, 'delay');
});

// --- Gain design (gain_design_test.cpp's cases) ---------------------------------

const kModel = M.feedforward(1.1, 0.14, 0.035);

test('pole placement formulas', () => {
  const design = M.designPositionGains(kModel, M.responseSpec(0.6));
  assert.ok(design.ok && !design.limitedByDelay);
  const omega = M.normalizedSettleTime(1.0) / 0.6;
  close(design.naturalFrequency, omega, 1e-12, 'omega');
  close(design.gains.kP, kModel.kA * omega * omega, 1e-12, 'kP');
  close(design.gains.kD, 2 * omega * kModel.kA - kModel.kV, 1e-12, 'kD');
  assert.equal(design.gains.kI, 0);
  close(design.staticErrorBound, kModel.kS / design.gains.kP, 1e-12, 'friction band');
  const critical = M.normalizedSettleTime(1.0);
  close((1 + critical) * Math.exp(-critical), 0.02, 0.01, 'critical damping settle');
});

test('delay limits the design to the phase margin asked for', () => {
  const design = M.designPositionGains(kModel, M.responseSpec(0.3, 1.0, 50), 0.05);
  assert.ok(design.ok && design.limitedByDelay && design.settleTimeS > 0.3);
  assert.ok(Math.abs(design.phaseMarginDeg - 50) < 0.5);
  assert.ok(!M.designPositionGains(M.feedforward(), M.responseSpec()).ok, 'unmeasured model');
});

// --- PID (pid_test.cpp's semantics) ----------------------------------------------

test('PID: continuous-time gains, flags, anti-windup, derivative on measurement', () => {
  const pid = new M.PID({ gains: { kP: 2, kI: 1, kD: 0.5 }, nominalDtS: 0.01 });
  let out = pid.update(10, 0);
  assert.equal(pid.lastStep.flags & M.FLAG.firstStep, M.FLAG.firstStep);
  close(out, 2 * 10 + 1 * 0.1, 1e-12, 'first step has no derivative');
  out = pid.update(10, 1);
  close(pid.lastStep.dTerm, 0.5 * (9 - 10) / 0.01, 1e-12, 'derivative per second');
  assert.equal(pid.lastStep.flags, 0);

  const limited = new M.PID({ gains: { kP: 5, kI: 10 }, outputLimit: 12 });
  limited.update(100, 0);
  assert.ok(limited.lastStep.flags & M.FLAG.saturated);
  assert.ok(limited.lastStep.flags & M.FLAG.integralHeld);
  assert.equal(limited.integral, 0, 'saturated integration rolled back');

  const dom = new M.PID({ gains: { kP: 0, kD: 1 }, derivativeOnMeasurement: true });
  dom.update(0, 0);
  dom.update(50, 0);
  assert.ok(dom.lastStep.dTerm === 0, 'a target step kicks nothing');
  assert.ok(dom.reset() && !dom.reset(), 'reset reports whether it cleared state');
});

// --- Mechanism law and thermal curve ---------------------------------------------

test('position law: track with gravity, seat and rest at the floor, brake without a sensor', () => {
  const pid = new M.PID({ gains: { kP: 0.1 }, nominalDtS: 0.02 });
  const config = {
    gravity: { constantVolts: 1.5, cosineVolts: 0, horizontalPosition: 0, armDegreesPerUnit: 1 },
    seat: { enabled: true, floor: 0, seatVolts: 2, restBand: 3 },
    maxVolts: 12,
  };
  let cmd = M.computePositionCommand(pid, config, 100, 40);
  assert.equal(cmd.law, M.LAW.track);
  close(cmd.volts, 0.1 * 60 + 1.5, 1e-12, 'track volts');
  cmd = M.computePositionCommand(pid, config, 0, 40);
  assert.equal(cmd.law, M.LAW.seat);
  assert.ok(cmd.volts <= -2);
  assert.equal(M.computePositionCommand(pid, config, 0, 2).law, M.LAW.rest);
  cmd = M.computePositionCommand(pid, config, 100, NaN);
  assert.ok(cmd.law === M.LAW.sensorLost && cmd.brake);
  assert.equal(M.LAW_NAMES[M.LAW.external], 'external');
});

test('thermal derating follows the V5 steps', () => {
  assert.equal(M.thermalPowerFraction(40), 1);
  assert.equal(M.thermalPowerFraction(57), 0.5);
  assert.equal(M.thermalPowerFraction(62), 0.25);
  assert.equal(M.thermalPowerFraction(80), 0);
  assert.equal(M.thermalPowerFraction(Infinity), 1, 'an unplugged motor is not a hot one');
  close(M.thermalPowerFraction(55), 0.75, 1e-12, 'ramps across the step');
});

// --- Simulation: fit → design → closed loop ---------------------------------------

test('a design on a simulated lift holds its targets once gravity is cancelled', () => {
  const truth = M.mechanismModel(M.feedforward(0.6, 0.017, 0.002), 2.0, M.gravityShape('constant'));
  const design = M.designPositionGains(truth.motion, M.responseSpec(0.5), 0.03);
  const pid = new M.PID({ gains: design.gains, outputLimit: 12, derivativeOnMeasurement: true,
    nominalDtS: 0.02 });
  const config = { gravity: M.gravityFeedforwardOf(truth), maxVolts: 12 };
  const run = M.simulateClosedLoop({
    model: truth, delayS: 0.02, periodS: 0.02, durationS: 2, x0: 100,
    targetAt: () => 400,
    controller: (target, x) => M.computePositionCommand(pid, config, target, x).volts,
  });
  const metrics = M.stepMetrics(run.t, run.x, 100, 400, 15, run.u);
  assert.ok(metrics.settled && metrics.settleS < 0.9, `settles: ${metrics.settleS}`);
  assert.ok(metrics.overshootFraction < 0.05, `no real overshoot: ${metrics.overshootFraction}`);
  assert.ok(metrics.finalError <= design.staticErrorBound + 1, `final ${metrics.finalError}`);

  // Without the feedforward the same loop sags by about kG/kP.
  const sagging = new M.PID({ gains: design.gains, outputLimit: 12, nominalDtS: 0.02 });
  const noFf = { gravity: { constantVolts: 0, cosineVolts: 0, horizontalPosition: 0,
    armDegreesPerUnit: 1 }, maxVolts: 12 };
  const sag = M.simulateClosedLoop({
    model: truth, delayS: 0.02, periodS: 0.02, durationS: 2, x0: 100, targetAt: () => 400,
    controller: (target, x) => M.computePositionCommand(sagging, noFf, target, x).volts,
  });
  const sagError = 400 - sag.x[sag.x.length - 1];
  assert.ok(sagError > 0.5 * (2.0 / design.gains.kP), `sags below: ${sagError}`);
});

test('open-loop replay reproduces the plant it was fitted from', () => {
  const truth = M.mechanismModel(M.feedforward(1.1, 0.14, 0.035));
  const sim = new M.PlantSim(truth, 0.03, 0, 0);
  const samples = [];
  for (let i = 0; i < 200; ++i) {
    const volts = i < 10 ? 0 : 6;
    samples.push({ t: i * 0.01, volts, position: sim.x });
    sim.command(volts);
    sim.advance(0.01);
  }
  const predicted = M.replayOpenLoop(truth, 0.03, samples);
  for (let i = 0; i < samples.length; ++i) {
    assert.ok(Math.abs(predicted[i] - samples[i].position) < 1e-9, `sample ${i}`);
  }
});
