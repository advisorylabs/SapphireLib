#!/usr/bin/env node
/*
 * Notebook experiments: every number on Section G's Test pages comes from here.
 *
 *   node notebook/build/experiments.js        -> notebook/build/data/*.json
 *
 * Everything runs on the repo's own simulator (tools/sim) and the analyzer's
 * ports of the library's PID, axis model and gain design (tools/analyzer/js/model.js).
 * Nothing here is measured on the robot.
 */
'use strict';

const fs = require('fs');
const path = require('path');

const root = path.join(__dirname, '..', '..');
const SIM = require(path.join(root, 'tools/sim/js/sim.js'));
const R = require(path.join(root, 'tools/sim/js/robot.js'));
const A = require(path.join(root, 'tools/sim/js/auton.js'));
const M = require(path.join(root, 'tools/analyzer/js/model.js'));
const CT = require(path.join(root, 'tools/sim/js/clawtiming.js'));
const { ROUTINES } = require(path.join(root, 'tools/sim/js/routines.js'));

const outDir = path.join(__dirname, 'data');
fs.mkdirSync(outDir, { recursive: true });
const save = (name, data) => {
  fs.writeFileSync(path.join(outDir, name + '.json'), JSON.stringify(data, null, 1));
  console.log('wrote', name);
};
const mean = (xs) => xs.reduce((a, b) => a + b, 0) / xs.length;
const round = (x, n = 3) => Number(x.toFixed(n));

// ---------------------------------------------------------------- simulator runs

function run(routine, tweak = () => {}, during = () => {}, sampleEveryMs = 0) {
  const options = SIM.defaultOptions();
  tweak(options);
  const sim = new SIM.Simulation(options);
  sim.runRoutine(routine);
  let worst = 0;
  let sum = 0;
  let ticks = 0;
  const series = [];
  const track = [];
  while (sim.program && sim.world.timeMs < 90000) {
    during(sim);
    sim.tick();
    const e = sim.errors();
    worst = Math.max(worst, e.corrected);
    sum += e.corrected;
    ticks++;
    if (sampleEveryMs && sim.world.timeMs % sampleEveryMs === 0) {
      series.push({ t: sim.world.timeMs / 1000, raw: round(e.raw), corrected: round(e.corrected) });
      track.push({ t: sim.world.timeMs / 1000, x: round(sim.world.truth.xIn, 2), y: round(sim.world.truth.yIn, 2) });
    }
  }
  return { sim, end: sim.errors().corrected, worst, mean: sum / ticks, finished: !sim.program,
    timeS: sim.world.timeMs / 1000, series, track };
}

const CONDITIONS = {
  Normal: { world: () => {}, during: () => {} },
  'Worn wheels': {
    world: (o) => { o.world.wheelDiameterErrorPct = 3; o.world.wheelSlipPct = 2; o.world.imuDriftDegPerMin = 3; },
    during: () => {},
  },
  '6 in bump': {
    world: () => {},
    during: (sim) => { if (sim.world.timeMs === 6000) sim.world.bump(6, -4); },
  },
  Defender: {
    world: (o) => { o.world.defender.enabled = true; o.world.defender.patrol = true; },
    during: () => {},
  },
};
const SEEDS = [1, 2, 3, 4, 5];
const byId = (id) => ROUTINES.find((r) => r.id === id);

function mclVsOdom() {
  const rows = [];
  for (const [condition, c] of Object.entries(CONDITIONS)) {
    for (const id of ['tour', 'laps', 'sprints']) {
      const cell = { condition, routine: byId(id).name, odom: { mean: [], end: [] }, mcl: { mean: [], end: [] } };
      for (const seed of SEEDS) {
        for (const correct of [false, true]) {
          const r = run(byId(id), (o) => { c.world(o); o.world.seed = seed; o.correctOdometry = correct; }, c.during);
          const into = correct ? cell.mcl : cell.odom;
          into.mean.push(r.mean);
          into.end.push(r.end);
        }
      }
      rows.push({ condition, routine: cell.routine,
        odomMean: round(mean(cell.odom.mean), 2), odomEnd: round(mean(cell.odom.end), 2),
        mclMean: round(mean(cell.mcl.mean), 2), mclEnd: round(mean(cell.mcl.end), 2),
        odomMeanSeeds: cell.odom.mean.map((x) => round(x, 2)), mclMeanSeeds: cell.mcl.mean.map((x) => round(x, 2)) });
      console.log(condition, id, rows[rows.length - 1].odomMean, rows[rows.length - 1].mclMean);
    }
  }
  save('mcl_vs_odom', { seeds: SEEDS, rows });
}

function driftSeries() {
  // one worn-wheels Field tour with a 6 in bump at 6 s, seed 1: odometry alone against MCL, every 100 ms
  const c = CONDITIONS['Worn wheels'];
  const bump = CONDITIONS['6 in bump'].during;
  const out = {};
  for (const correct of [false, true]) {
    const r = run(byId('tour'), (o) => { c.world(o); o.world.seed = 1; o.correctOdometry = correct; }, bump, 100);
    out[correct ? 'mcl' : 'odom'] = r.series.map((s) => ({ t: s.t, e: s.corrected }));
    if (!correct) out.track = r.track;
  }
  save('drift_series', out);
}

function particleCount() {
  const c = CONDITIONS['Worn wheels'];
  const rows = [];
  for (const count of [50, 100, 300, 1000]) {
    const means = [];
    const ends = [];
    const recover = [];
    for (const seed of SEEDS) {
      let bumpedAt = null;
      let backAt = null;
      const r = run(byId('tour'),
        (o) => { c.world(o); o.world.seed = seed; o.localizer = { filter: { particleCount: count } }; },
        (sim) => {
          if (sim.world.timeMs === 6000) { sim.world.bump(6, -4); bumpedAt = 6000; }
          if (bumpedAt !== null && backAt === null && sim.world.timeMs > 6050 && sim.errors().corrected < 1.0) {
            backAt = sim.world.timeMs;
          }
        });
      means.push(r.mean);
      ends.push(r.end);
      recover.push(backAt === null ? NaN : (backAt - 6000) / 1000);
      if (seed === 1) rows.particleCheck = r.sim.robot.localizer.config.filter.particleCount;
    }
    rows.push({ particles: count, mean: round(mean(means), 2), end: round(mean(ends), 2),
      recoverS: round(mean(recover.filter(Number.isFinite)), 2), configured: rows.particleCheck });
    console.log('particles', count, rows[rows.length - 1]);
  }
  save('particle_count', { seeds: SEEDS, rows });
}

// ---------------------------------------------------------------- PID features

const FORWARD = M.mechanismModel(M.feedforward(1.0, 0.2, 0.045));
const TURN = M.mechanismModel(M.feedforward(0.6, 0.045, 0.0065));
const DELAY = 0.03;

function closedLoop({ model = FORWARD, pid, periodS = 0.01, durationS = 3, targetAt, limitsAt = null }) {
  const sim = new M.PlantSim(model, DELAY, 0, 0, null);
  const n = Math.ceil(durationS / periodS);
  const t = [];
  const x = [];
  const u = [];
  for (let i = 0; i < n; ++i) {
    const now = i * periodS;
    if (limitsAt) sim.limits = limitsAt(now);
    const volts = Math.max(-12, Math.min(12, pid.update(targetAt(now), sim.x, periodS)));
    t.push(round(now, 3));
    x.push(round(sim.x, 3));
    u.push(round(volts, 3));
    sim.command(volts);
    sim.advance(periodS);
  }
  return { t, x, u };
}

function pidAntiWindup() {
  // drive to 24 in; a wall at 3 in holds the robot for the first 1.5 s, then is removed
  const gains = { kP: 0.9, kI: 1.5, kD: 0.06 };
  const out = { gains, wallUntilS: 1.5, target: 24 };
  const cases = {
    guard: { gains, outputLimit: 12, integralLimit: 0 },
    none: { gains, outputLimit: 0, integralLimit: 0 },
  };
  for (const [name, config] of Object.entries(cases)) {
    const pid = new M.PID(config);
    const r = closedLoop({ pid, durationS: 9, targetAt: () => 24,
      limitsAt: (now) => (now < 1.5 ? [-1, 3] : null) });
    const peak = Math.max(...r.x);
    let settle = null;
    for (let i = r.t.length - 1; i >= 0; --i) {
      if (Math.abs(r.x[i] - 24) > 0.5) { settle = r.t[i] + 0.01; break; }
    }
    out[name] = { t: r.t.filter((_, i) => i % 2 === 0), x: r.x.filter((_, i) => i % 2 === 0),
      overshootIn: round(peak - 24, 2), settleS: round(settle - 1.5, 2) };
    console.log('windup', name, out[name].overshootIn, out[name].settleS);
  }
  save('pid_windup', out);
}

function pidDerivativeKick() {
  // heading hold: the driver nudges the target 5 degrees at t = 0.2 s
  const design = M.designPositionGains(TURN.motion, M.responseSpec(0.5, 1.0, 50), DELAY);
  const gains = design.gains;
  const out = { gains: { kP: round(gains.kP, 4), kD: round(gains.kD, 5) }, nudgeDeg: 5 };
  for (const onMeasurement of [false, true]) {
    const pid = new M.PID({ gains, outputLimit: 0, derivativeOnMeasurement: onMeasurement });
    const sim = new M.PlantSim(TURN, DELAY, 0, 0, null);
    const t = [];
    const raw = [];
    const x = [];
    for (let i = 0; i < 120; ++i) {
      const now = i * 0.01;
      const volts = pid.update(now >= 0.2 ? 5 : 0, sim.x, 0.01);
      t.push(round(now, 2));
      raw.push(round(volts, 2));
      x.push(round(sim.x, 3));
      sim.command(volts);
      sim.advance(0.01);
    }
    out[onMeasurement ? 'onMeasurement' : 'onError'] = { t, volts: raw, x, peakVolts: round(Math.max(...raw.map(Math.abs)), 1) };
    console.log('kick', onMeasurement, out[onMeasurement ? 'onMeasurement' : 'onError'].peakVolts);
  }
  save('pid_kick', out);
}

function pidLoopPeriod() {
  // gains designed once; run at 10 ms and at 20 ms with per-second gains, and at 20 ms the way a
  // per-tick PID (integral += error, derivative = error - last) tuned at 10 ms would behave
  const base = M.designPositionGains(FORWARD.motion, M.responseSpec(0.6, 1.0, 50), DELAY).gains;
  const gains = { kP: base.kP, kI: 0.8, kD: base.kD };
  const cases = [
    { name: 'Per-second gains, 10 ms loop', periodS: 0.01, gains },
    { name: 'Per-second gains, 20 ms loop', periodS: 0.02, gains },
    { name: 'Per-tick gains, 20 ms loop', periodS: 0.02, gains: { kP: gains.kP, kI: gains.kI * 0.5, kD: gains.kD * 2 } },
    { name: 'Per-tick gains, 5 ms loop', periodS: 0.005, gains: { kP: gains.kP, kI: gains.kI * 2, kD: gains.kD * 0.5 } },
  ];
  const out = { gains: { kP: round(gains.kP, 3), kI: gains.kI, kD: round(gains.kD, 4) }, cases: [] };
  for (const c of cases) {
    const pid = new M.PID({ gains: c.gains, outputLimit: 12 });
    const r = closedLoop({ pid, periodS: c.periodS, durationS: 2.5, targetAt: () => 24 });
    const m = M.stepMetrics(r.t, r.x, 0, 24, 0.5, r.u);
    out.cases.push({ name: c.name, settleS: round(m.settleS, 2), overshootIn: round(m.overshoot, 2),
      t: r.t, x: r.x });
    console.log('period', c.name, round(m.settleS, 2), round(m.overshoot, 2));
  }
  save('pid_period', out);
}

// ---------------------------------------------------------------- the three autotuners

function stepTest(gains, sizeIn, durationS = 3) {
  const pid = new M.PID({ gains, outputLimit: 12 });
  const r = closedLoop({ pid, durationS, targetAt: () => sizeIn });
  const m = M.stepMetrics(r.t, r.x, 0, sizeIn, 0.5, r.u);
  let cost = 0;
  for (let i = 0; i < r.t.length; ++i) cost += Math.abs(sizeIn - r.x[i]) * 0.01;
  return { r, m, cost: cost + 5 * m.overshoot };
}

function autotuners() {
  const out = {};

  // 1. Twiddle: nudge one gain at a time, keep the change if a 24 in test drive scored better
  {
    const p = [0.3, 0.0, 0.0];
    const dp = [0.3, 0.3, 0.03];
    const keys = ['kP', 'kI', 'kD'];
    const gainsOf = (v) => ({ kP: Math.max(0, v[0]), kI: Math.max(0, v[1]), kD: Math.max(0, v[2]) });
    let runs = 0;
    let unstable = 0;
    const history = [];
    const trial = (v) => {
      const s = stepTest(gainsOf(v), 24);
      runs++;
      const bad = s.m.overshootFraction > 0.15 || !s.m.settled || s.m.settleS > 2.5;
      if (bad) unstable++;
      history.push({ run: runs, cost: round(s.cost, 2), overshootIn: round(s.m.overshoot, 2), bad });
      return s.cost;
    };
    let best = trial(p);
    while (dp[0] + dp[1] + dp[2] > 0.02 && runs < 200) {
      for (let i = 0; i < 3; ++i) {
        p[i] += dp[i];
        let cost = trial(p);
        if (cost < best) { best = cost; dp[i] *= 1.1; continue; }
        p[i] -= 2 * dp[i];
        cost = trial(p);
        if (cost < best) { best = cost; dp[i] *= 1.1; continue; }
        p[i] += dp[i];
        dp[i] *= 0.9;
      }
    }
    out.twiddle = { gains: gainsOf(p), runs, unstableRuns: unstable, history,
      robotSeconds: runs * 3, keys };
  }

  // 2. Ziegler-Nichols relay: +/-6 V around the target, gains from the oscillation's period and size
  {
    const sim = new M.PlantSim(FORWARD, DELAY, 0, 0, null);
    const d = 6;
    const t = [];
    const x = [];
    for (let i = 0; i < 800; ++i) {
      sim.command(sim.x < 0 ? d : -d);
      sim.advance(0.01);
      t.push(round(i * 0.01, 2));
      x.push(round(sim.x, 3));
    }
    const crossings = [];
    for (let i = 401; i < x.length; ++i) if (x[i - 1] < 0 && x[i] >= 0) crossings.push(t[i]);
    const periods = [];
    for (let i = 1; i < crossings.length; ++i) periods.push(crossings[i] - crossings[i - 1]);
    const Tu = mean(periods);
    const tail = x.slice(400);
    const amp = (Math.max(...tail) - Math.min(...tail)) / 2;
    const Ku = (4 * d) / (Math.PI * amp);
    out.relay = { gains: { kP: 0.6 * Ku, kI: (1.2 * Ku) / Tu, kD: 0.075 * Ku * Tu },
      Ku: round(Ku, 3), TuS: round(Tu, 3), amplitudeIn: round(amp, 3), runs: 1, robotSeconds: 8,
      t: t.slice(0, 500), x: x.slice(0, 500) };
  }

  // 3. Model + pole placement: the axis model, then gains for a chosen settle time and damping
  {
    const design = M.designPositionGains(FORWARD.motion, M.responseSpec(0.6, 1.0, 50), DELAY);
    out.model = { gains: design.gains, runs: 4, robotSeconds: 10,
      design: { ok: design.ok, phaseMarginDeg: round(design.phaseMarginDeg ?? NaN, 1),
        settleTimeS: round(design.settleTimeS ?? NaN, 2) } };
  }

  // the same three test drives for each: 6, 24 and 48 in
  for (const name of ['twiddle', 'relay', 'model']) {
    const g = out[name].gains;
    out[name].gains = { kP: round(g.kP, 3), kI: round(g.kI, 3), kD: round(g.kD, 4) };
    out[name].tests = [6, 24, 48].map((size) => {
      const s = stepTest(g, size, 4);
      return { sizeIn: size, settleS: round(s.m.settleS, 2), overshootIn: round(s.m.overshoot, 2),
        finalErrorIn: round(s.m.finalError, 2), t: s.r.t.filter((_, i) => i % 2 === 0), x: s.r.x.filter((_, i) => i % 2 === 0) };
    });
    console.log(name, out[name].gains, out[name].runs,
      out[name].tests.map((x) => `${x.sizeIn}in settle ${x.settleS}s over ${x.overshootIn}`).join(' | '));
  }
  save('autotuners', out);
}

// ---------------------------------------------------------------- pure pursuit lookahead

function distanceToPath(x, y, pts) {
  let best = Infinity;
  for (let i = 1; i < pts.length; ++i) {
    const [ax, ay] = pts[i - 1];
    const [bx, by] = pts[i];
    const dx = bx - ax;
    const dy = by - ay;
    const s = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)));
    best = Math.min(best, Math.hypot(x - (ax + s * dx), y - (ay + s * dy)));
  }
  return best;
}

const perfectWorld = (o) => {
  o.correctOdometry = false;
  Object.assign(o.world, { wheelDiameterErrorPct: 0, wheelSlipPct: 0, imuDriftDegPerMin: 0, imuScaleErrorPct: 0, imuNoiseDeg: 0 });
};

function lookahead() {
  const pts = [[-48, -48], [-48, 0], [-12, 24], [-12, 48], [36, 48], [36, 0], [0, -30]];
  const rows = [];
  for (const L of [3, 6, 9, 12, 18, 24]) {
    const routine = {
      id: 'la', name: 'la', start: { xIn: -48, yIn: -48, headingDeg: 0 },
      *run(robot) {
        const a = A.create(robot);
        a.setPose(-48, -48, 0);
        yield* a.followPath(pts, { lookaheadIn: L, maxVolts: 9, timeoutMs: 20000 });
      },
    };
    const options = SIM.defaultOptions();
    perfectWorld(options);
    const sim = new SIM.Simulation(options);
    sim.runRoutine(routine);
    let sum = 0;
    let worst = 0;
    let n = 0;
    let wiggle = 0;
    let lastDir = null;
    let lastX = sim.world.truth.xIn;
    let lastY = sim.world.truth.yIn;
    const track = [];
    while (sim.program && sim.world.timeMs < 40000) {
      sim.tick();
      const { xIn, yIn } = sim.world.truth;
      const d = distanceToPath(xIn, yIn, pts);
      sum += d;
      worst = Math.max(worst, d);
      n++;
      const step = Math.hypot(xIn - lastX, yIn - lastY);
      if (step > 0.02) {
        const dir = Math.atan2(yIn - lastY, xIn - lastX);
        if (lastDir !== null) {
          let dd = dir - lastDir;
          while (dd > Math.PI) dd -= 2 * Math.PI;
          while (dd < -Math.PI) dd += 2 * Math.PI;
          wiggle += Math.abs(dd);
        }
        lastDir = dir;
        lastX = xIn;
        lastY = yIn;
      }
      if (sim.world.timeMs % 100 === 0) track.push([round(xIn, 1), round(yIn, 1)]);
    }
    rows.push({ lookaheadIn: L, meanIn: round(sum / n, 2), worstIn: round(worst, 2),
      timeS: round(sim.world.timeMs / 1000, 2), finished: !sim.program,
      turningDeg: round((wiggle * 180) / Math.PI, 0), track });
    console.log('lookahead', L, rows[rows.length - 1].meanIn, rows[rows.length - 1].worstIn,
      rows[rows.length - 1].timeS, rows[rows.length - 1].turningDeg, !sim.program);
  }
  save('lookahead', { path: pts, rows });
}

// ---------------------------------------------------------------- diagonal: moveToPoint vs trajectory

function diagonal() {
  const start = [-48, -48];
  const end = [24, 0];
  const out = { start, end };
  const routines = {
    moveToPoint: function* (a) { yield* a.moveToPoint(end[0], end[1], { timeoutMs: 8000 }); },
    trajectory: function* (a) {
      yield* a.followTrajectory([{ waypoints: [{ xIn: start[0], yIn: start[1], headingDeg: 0 },
        { xIn: end[0], yIn: end[1], headingDeg: 0 }] }]);
    },
  };
  for (const [name, body] of Object.entries(routines)) {
    const routine = {
      id: name, name, start: { xIn: start[0], yIn: start[1], headingDeg: 0 },
      *run(robot) {
        const a = A.create(robot);
        a.setPose(start[0], start[1], 0);
        yield* body(a);
      },
    };
    const options = SIM.defaultOptions();
    perfectWorld(options);
    const sim = new SIM.Simulation(options);
    sim.runRoutine(routine);
    let worst = 0;
    let sum = 0;
    let n = 0;
    const track = [];
    while (sim.program && sim.world.timeMs < 20000) {
      sim.tick();
      const { xIn, yIn } = sim.world.truth;
      const d = distanceToPath(xIn, yIn, [start, end]);
      worst = Math.max(worst, d);
      sum += d;
      n++;
      if (sim.world.timeMs % 50 === 0) track.push([round(xIn, 2), round(yIn, 2)]);
    }
    const t = sim.world.truth;
    out[name] = { worstIn: round(worst, 2), meanIn: round(sum / n, 2), timeS: round(sim.world.timeMs / 1000, 2),
      endErrorIn: round(Math.hypot(t.xIn - end[0], t.yIn - end[1]), 2), finished: !sim.program, track };
    console.log('diagonal', name, out[name].worstIn, out[name].meanIn, out[name].timeS, out[name].endErrorIn);
  }
  save('diagonal', out);
}

// ---------------------------------------------------------------- autoClaw timing

function claw() {
  const speeds = CT.speedRange(4, 60, 4);
  const p = CT.defaults();
  const noAge = { ...p, rule: 'lead' };
  noAge.leadMs = CT.bestLeadMs(noAge, 40);
  const rules = {
    'Fixed trigger at 125 mm': { ...p, rule: 'fixed', fixedMm: 125 },
    'Lead by speed': noAge,
    'Lead by speed + reading age + tick': p,
  };
  const out = { speeds, defaults: { grabMm: p.grabMm, leadMs: p.leadMs, loopMs: p.loopMs,
    sensorPeriodMs: p.sensorPeriodMs, sensorLatencyMs: p.sensorLatencyMs }, rules: {} };
  for (const [name, params] of Object.entries(rules)) {
    const rows = CT.sweep(params, { speeds, trials: 400 });
    out.rules[name] = rows.map((r) => ({ speed: r.speedInPerS, caught: round(r.caught, 3), early: round(r.early, 3),
      late: round(r.late, 3), meanErrorMm: round(r.meanErrorMm, 1) }));
    console.log('claw', name, rows.map((r) => Math.round(r.caught * 100)).join(' '));
  }
  save('claw', out);
}

const which = process.argv.slice(2);
const all = { mclVsOdom, driftSeries, particleCount, pidAntiWindup, pidDerivativeKick, pidLoopPeriod, autotuners,
  lookahead, diagonal, claw };
for (const [name, fn] of Object.entries(all)) {
  if (which.length && !which.includes(name)) continue;
  console.log('---', name);
  fn();
}
