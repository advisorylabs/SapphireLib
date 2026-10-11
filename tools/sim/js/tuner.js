/*
 * SapphireLib simulator: tuner.js
 *
 * Auto-tunes the Monte Carlo localizer: searches LocalizerConfig for the
 * settings that keep the corrected pose closest to the truth, across a suite
 * of simulated runs, instead of anyone nudging sliders by hand.
 *
 * Two stages, the same search:
 *
 *   1. The standard suite: the simulator's own scenarios (a clean robot,
 *      worn tracking wheels, bumps, noisy sensors, a crowded field), each
 *      driven by a routine from routines.js. Settings that do well across all
 *      of them are a sound start for any robot.
 *   2. Your robot: calibrate the simulated world from the robot's logs
 *      (mclcal.js measures its sensor noise, outliers, dropouts and tracking
 *      wheel scale; the sensor latency is matched here, by simulating), then
 *      search again on that world, including replays of the paths the robot
 *      really drove. Run it after every few practice sessions: each round of
 *      logs makes the simulated robot more like the real one.
 *
 * The search is a coordinate pattern search: try each setting a step up and
 * down, keep any change that lowers the score, and halve the steps once a
 * sweep finds nothing. Every candidate runs on the same random seeds (common
 * random numbers), so two settings are compared on identical noise, and the
 * winner is checked on seeds it never saw (the holdout) before anyone trusts
 * it.
 *
 * The score is relative: each case's error with the candidate settings over
 * its error with the starting settings, averaged over the cases, so 1.0 is
 * "no better" and 0.8 is "20% less error". A case's error is the corrected
 * pose's mean distance from the truth, plus half its 95th percentile, a
 * quarter of where it ended, and half a second per second a bump took to
 * recover. Settings that cost more CPU (particles, update rate) pay
 * cpuWeight per extra share of the starting CPU.
 *
 * Generators throughout, yielding after every simulated run, so the page can
 * stay responsive (app.js runs a slice per frame) and Node can just iterate.
 *
 * Plain script: window.SIM.tuner in a browser, require('./tuner.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./world.js'), require('./robot.js'), require('./routines.js'),
      require('./sim.js'), require('./recorder.js'), require('../../analyzer/js/slt.js'),
      require('../../analyzer/js/mclcal.js'), require('../../analyzer/js/tunefile.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.tuner = factory(root.SIM.world, root.SIM.robot, root.SIM.routines, root.SIM.sim, root.SIM.recorder,
      root.SA.slt, root.SA.mclcal, root.SA.tunefile);
  }
})(function (W, R, RT, S, REC, slt, cal, TF) {
  'use strict';

  const { ROUTINES } = RT;

  /**
   * What the search may change. `scale` is how it steps: 'log' multiplies,
   * 'lin' adds. `on` is whether it's searched by default; the rest are
   * either measured instead (sensorLatencyMs, from logs) or trade something
   * the score can't see (particleCount and periodMs cost CPU, which the
   * cpuWeight term charges for, and maxCorrectionRateInPerS trades how fast
   * a correction lands against how hard the motions feel it).
   */
  const PARAMS = [
    { key: 'filter.motionNoise.perInch', label: 'Motion noise per inch', min: 0.005, max: 0.3, scale: 'log', on: true },
    { key: 'filter.motionNoise.baseIn', label: 'Motion noise standing still', min: 0.002, max: 0.3, scale: 'log', on: true },
    { key: 'filter.motionNoise.perDegreeIn', label: 'Motion noise per degree', min: 0.0005, max: 0.05, scale: 'log', on: true },
    { key: 'filter.beam.outlierProbability', label: 'Outlier probability', min: 0.01, max: 0.5, scale: 'log', on: true },
    { key: 'filter.beam.sigmaFraction', label: 'Sensor noise, share of distance', min: 0.01, max: 0.2, scale: 'log', on: true },
    { key: 'filter.beam.minSigmaIn', label: 'Sensor noise floor', min: 0.2, max: 3, scale: 'log', on: true },
    { key: 'filter.resampleThreshold', label: 'Resample threshold', min: 0.1, max: 0.9, scale: 'lin', on: true },
    { key: 'filter.recovery.triggerRatio', label: 'Recovery trigger', min: 0.4, max: 0.95, scale: 'lin', on: true },
    { key: 'filter.recovery.radiusIn', label: 'Recovery radius', min: 6, max: 48, scale: 'log', on: true },
    { key: 'filter.recovery.maxFraction', label: 'Recovery share', min: 0.02, max: 0.4, scale: 'log', on: true },
    { key: 'maxCorrectionSpreadIn', label: 'Correct only below spread', min: 0.5, max: 8, scale: 'log', on: true },
    { key: 'agreementSigmas', label: 'Agreement threshold', min: 1.5, max: 6, scale: 'log', on: true },
    { key: 'minAgreeingSensors', label: 'Agreeing sensors needed', min: 1, max: 3, scale: 'lin', whole: true, on: true },
    { key: 'sensorLatencyMs', label: 'Latency compensation', min: 0, max: 120, scale: 'lin', on: false },
    { key: 'maxCorrectionRateInPerS', label: 'Correction speed', min: 1, max: 16, scale: 'log', on: false },
    { key: 'filter.particleCount', label: 'Particles', min: 100, max: 1500, scale: 'log', whole: true, on: false },
    { key: 'periodMs', label: 'Update period', min: 20, max: 100, scale: 'lin', whole: true, on: false },
  ];

  const BOOLEAN_KEYS = new Set(['filter.recovery.enabled', 'correctOdometry', 'waitForSetPose']);
  const DEFAULT_SEEDS = [7, 11, 13, 17];
  const HOLDOUT_SEEDS = [101, 202, 303, 404];

  // --- Configs: flat { path: value } maps, like TUNE.CFG's mcl.* lines -------------------

  function getPath(object, path) {
    return path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), object);
  }

  /** The robot program's defaults, flat, with periodMs. */
  function defaultConfig() {
    const nested = R.localizerConfig({});
    const out = {};
    for (const s of TF.LOCALIZER_SETTINGS) {
      const value = getPath(nested, s.key);
      out[s.key] = typeof value === 'boolean' ? (value ? 1 : 0) : value;
    }
    out.periodMs = TF.DEFAULT_PERIOD_MS;
    return out;
  }

  /** A flat config as the overrides R.localizerConfig() and Simulation take. */
  function nest(flat) {
    const out = {};
    for (const [key, raw] of Object.entries(flat)) {
      if (key === 'periodMs' || raw === undefined) continue;
      const value = BOOLEAN_KEYS.has(key) ? raw !== 0 : raw;
      const parts = key.split('.');
      let o = out;
      for (let i = 0; i < parts.length - 1; ++i) o = o[parts[i]] || (o[parts[i]] = {});
      o[parts[parts.length - 1]] = value;
    }
    return out;
  }

  function configKey(config) {
    return JSON.stringify(Object.keys(config).sort().map((k) => [k, config[k]]));
  }

  /** Relative CPU: particles per millisecond of update period, against the start's. */
  function cpuRatio(config, start) {
    return (config['filter.particleCount'] / config.periodMs) / (start['filter.particleCount'] / start.periodMs);
  }

  // --- Cases ------------------------------------------------------------------------------

  /** The standard suite: the simulator's scenarios, each with a routine. */
  function standardCases() {
    return [
      { id: 'clean', name: 'Clean robot, field tour', routine: 'tour', world: {} },
      { id: 'worn', name: 'Worn tracking wheels, square laps', routine: 'laps',
        world: { wheelDiameterErrorPct: 3, wheelSlipPct: 2, imuDriftDegPerMin: 3 } },
      { id: 'bumps', name: 'Two bumps, field tour', routine: 'tour', world: {},
        bumps: [{ atMs: 6000, dxIn: 6, dyIn: -4 }, { atMs: 15000, dxIn: -5, dyIn: 5 }] },
      { id: 'noisy', name: 'Noisy sensors, sprints', routine: 'sprints',
        world: { sensorNoiseScale: 3, sensorDropoutPct: 15 } },
      { id: 'crowded', name: 'Crowded field, field tour', routine: 'tour', world: {}, elements: true,
        defender: { enabled: true, patrol: true, xIn: 0, yIn: 30 } },
    ];
  }

  /** A routine that walks the robot along a logged path (see World.setScript()). */
  function replayRoutine(trajectory, maxSeconds = 40) {
    const samples = trajectory.samples.filter((s) => s.tMs <= maxSeconds * 1000);
    const endMs = samples[samples.length - 1].tMs;
    return {
      id: 'replay',
      name: trajectory.name,
      description: 'A path the robot really drove, from its log.',
      start: { xIn: samples[0].xIn, yIn: samples[0].yIn, headingDeg: samples[0].headingDeg },
      script: samples,
      *run(robot) {
        robot.odometry.setPose(this.start);
        while (robot.world.timeMs < endMs) yield;
      },
    };
  }

  /**
   * The suite for your robot, from calibrateWorld(): two routines and up to
   * maxReplays of its own paths, all in the calibrated world.
   */
  function robotCases(world, trajectories = [], { maxReplays = 3, maxSeconds = 40 } = {}) {
    const cases = [
      { id: 'robot-tour', name: 'Your robot: field tour', routine: 'tour', world },
      { id: 'robot-sprints', name: 'Your robot: sprints', routine: 'sprints', world },
    ];
    trajectories.slice(0, maxReplays).forEach((t, i) => {
      cases.push({ id: `replay-${i}`, name: `Replay: ${t.name}`, routine: replayRoutine(t, maxSeconds), world });
    });
    return cases;
  }

  function routineOf(c) {
    return typeof c.routine === 'string' ? ROUTINES.find((r) => r.id === c.routine) : c.routine;
  }

  /** A Simulation set up for one case, seed and config, with its routine started. */
  function simulationFor(c, config, seed) {
    const options = S.defaultOptions();
    const world = Object.assign({}, c.world || {});
    if (world.models) world.models = JSON.parse(JSON.stringify(world.models));
    Object.assign(options.world, world);
    options.world.seed = seed;
    if (c.elements) options.world.elements = W.exampleElements().map((e) => Object.assign(e, { inWorld: true }));
    if (c.defender) Object.assign(options.world.defender, c.defender);
    if (c.localizerMounts) options.localizerMounts = c.localizerMounts;
    options.localizer = nest(config);
    // a different filter seed per run seed, so a setting can't win on one lucky draw
    options.localizer.filter = Object.assign(options.localizer.filter || {}, { seed });
    options.localizerPeriodMs = config.periodMs || TF.DEFAULT_PERIOD_MS;
    const sim = new S.Simulation(options);
    sim.runRoutine(routineOf(c));
    return sim;
  }

  /**
   * The true path of case `c` at `seed`, driven once by its routine with
   * `config`: { script, start, bumps }. The script leaves out the bumps'
   * shoves, and `bumps` says how far each really moved the robot (a wall can
   * stop one short); runCase() shoves it again at the same moments.
   *
   * Every candidate the search tries then replays this one path, open loop.
   * Driven closed loop, each candidate would steer a slightly different
   * path, the world's random draws (slip, sensor noise, dropouts) would land
   * differently, and that noise is bigger than most settings' effects. On
   * one path they're identical for every candidate, so only the localizer
   * differs.
   */
  function referencePath(c, config, seed, { limitMs = 90000 } = {}) {
    const sim = simulationFor(c, config, seed);
    const pending = (c.bumps || []).slice();
    const offset = { xIn: 0, yIn: 0 };
    const bumps = [];
    const script = [];
    const t0 = sim.world.timeMs;
    const record = () => {
      const t = sim.world.truth;
      script.push({ tMs: sim.world.timeMs - t0, xIn: t.xIn - offset.xIn, yIn: t.yIn - offset.yIn,
        headingDeg: t.headingDeg });
    };
    record();
    while (sim.program && sim.world.timeMs < limitMs) {
      if (pending.length && sim.world.timeMs >= pending[0].atMs) {
        const b = pending.shift();
        const before = { xIn: sim.world.truth.xIn, yIn: sim.world.truth.yIn };
        sim.world.bump(b.dxIn, b.dyIn);
        const moved = { atMs: b.atMs, dxIn: sim.world.truth.xIn - before.xIn, dyIn: sim.world.truth.yIn - before.yIn };
        offset.xIn += moved.dxIn;
        offset.yIn += moved.dyIn;
        bumps.push(moved);
      }
      sim.tick();
      record();
    }
    const first = script[0];
    return { script, start: { xIn: first.xIn, yIn: first.yIn, headingDeg: first.headingDeg }, bumps };
  }

  /**
   * Drives every case that isn't already a replay once per seed, keeping its
   * path for runCase() (see referencePath()). One generator step per run.
   */
  function* prepare(cases, seeds, config) {
    for (const c of cases) {
      if (routineOf(c).script) continue;
      if (!c.paths) c.paths = new Map();
      for (const seed of seeds) {
        if (c.paths.has(seed)) continue;
        c.paths.set(seed, referencePath(c, config, seed));
        yield { type: 'prepare', caseId: c.id, seed };
      }
    }
  }

  /**
   * Runs one case with one config and seed: { meanIn, p95In, endIn,
   * worstIn, recoveryS, finished, seconds }. On the case's reference path
   * for that seed when prepare() has made one, by its routine otherwise.
   */
  function runCase(c, config, seed, { limitMs = 90000 } = {}) {
    const path = c.paths ? c.paths.get(seed) : null;
    const driven = path
      ? Object.assign({}, c, { routine: replayRoutine({ name: routineOf(c).name, samples: path.script }, Infinity),
        bumps: path.bumps })
      : c;
    const sim = simulationFor(driven, config, seed);
    const bumps = (driven.bumps || []).slice();
    const errors = [];
    let recoveries = 0;
    let recoverySum = 0;
    let bumpAtMs = null;
    while (sim.program && sim.world.timeMs < limitMs) {
      if (bumps.length && sim.world.timeMs >= bumps[0].atMs) {
        const b = bumps.shift();
        sim.world.bump(b.dxIn, b.dyIn);
        if (bumpAtMs !== null) recoverySum += 6; // the last one never came back
        bumpAtMs = sim.world.timeMs;
      }
      sim.tick();
      const e = sim.errors().corrected;
      errors.push(e);
      if (bumpAtMs !== null && sim.world.timeMs - bumpAtMs > 50 && e < 1.0) {
        recoverySum += (sim.world.timeMs - bumpAtMs) / 1000;
        recoveries++;
        bumpAtMs = null;
      } else if (bumpAtMs !== null && sim.world.timeMs - bumpAtMs > 6000) {
        recoverySum += 6;
        recoveries++;
        bumpAtMs = null;
      }
    }
    if (bumpAtMs !== null) {
      recoverySum += 6;
      recoveries++;
    }
    const sorted = Float64Array.from(errors).sort();
    const mean = errors.reduce((a, b) => a + b, 0) / Math.max(1, errors.length);
    return {
      meanIn: mean,
      p95In: sorted[Math.min(sorted.length - 1, Math.floor(0.95 * sorted.length))] || 0,
      endIn: errors.length ? errors[errors.length - 1] : 0,
      worstIn: sorted.length ? sorted[sorted.length - 1] : 0,
      recoveryS: recoveries ? recoverySum / recoveries : 0,
      finished: !sim.program,
      seconds: sim.world.timeMs / 1000,
    };
  }

  /** One run's error, in inch-equivalents: see the file comment. */
  function caseError(m) {
    return m.meanIn + 0.5 * m.p95In + 0.25 * m.endIn + 0.5 * m.recoveryS + (m.finished ? 0 : 10);
  }

  function averageMetrics(list) {
    const out = {};
    for (const key of ['meanIn', 'p95In', 'endIn', 'worstIn', 'recoveryS', 'seconds']) {
      out[key] = list.reduce((a, m) => a + m[key], 0) / list.length;
    }
    out.finished = list.every((m) => m.finished);
    out.error = list.reduce((a, m) => a + caseError(m), 0) / list.length;
    return out;
  }

  /**
   * Every case at every seed: [{ id, name, metrics, errors }], `errors` being
   * each seed's caseError() in seed order. One generator step per run.
   */
  function* measure(config, cases, seeds) {
    const out = [];
    for (const c of cases) {
      const runs = [];
      for (const seed of seeds) {
        runs.push(runCase(c, config, seed));
        yield { type: 'run', caseId: c.id, seed };
      }
      out.push({ id: c.id, name: c.name, metrics: averageMetrics(runs), errors: runs.map(caseError) });
    }
    return out;
  }

  /**
   * How `a` compares with `b` run for run (same case, same seed), as the
   * mean log of the error ratio and its standard error: { mean, se, n }.
   * Negative is better. Paired, so the noise both share cancels; what's left
   * says whether a difference is bigger than chance.
   */
  function paired(a, b) {
    const logs = [];
    for (let i = 0; i < a.length; ++i) {
      for (let j = 0; j < a[i].errors.length; ++j) {
        logs.push(Math.log(Math.max(1e-6, a[i].errors[j]) / Math.max(1e-6, b[i].errors[j])));
      }
    }
    const n = logs.length;
    const mean = logs.reduce((x, y) => x + y, 0) / n;
    const variance = n > 1 ? logs.reduce((x, y) => x + (y - mean) * (y - mean), 0) / (n - 1) : 0;
    return { mean, se: Math.sqrt(variance / n), n };
  }

  /**
   * The score of `measured` against `baseline` (both from measure()): see the
   * file comment. Any case more than 5% worse than it started also adds half
   * the excess, so the search can't buy a better average by giving up one
   * kind of match: a setting that only wins when sensors are clean is no
   * use on a crowded field.
   */
  function score(measured, baseline, config, start, cpuWeight) {
    let sum = 0;
    let worse = 0;
    for (let i = 0; i < measured.length; ++i) {
      const ratio = measured[i].metrics.error / Math.max(1e-6, baseline[i].metrics.error);
      sum += ratio;
      worse += Math.max(0, ratio - 1.05);
    }
    const cpu = cpuRatio(config, start);
    return sum / measured.length + 0.5 * worse + cpuWeight * Math.max(0, cpu - 1);
  }

  // --- The search ------------------------------------------------------------------------

  function initialStep(p) {
    if (p.scale === 'log') return 1.6;
    if (p.whole) return Math.max(1, Math.round((p.max - p.min) / 6));
    return (p.max - p.min) / 5;
  }

  function shrink(p, step) {
    if (p.scale === 'log') return Math.sqrt(step);
    if (p.whole) return Math.max(1, Math.round(step / 2));
    return step / 2;
  }

  function stepIsTiny(p, step) {
    if (p.scale === 'log') return step < 1.04;
    if (p.whole) return step <= 1;
    return step < (p.max - p.min) / 40;
  }

  function moved(p, value, direction, step) {
    let v;
    if (p.scale === 'log') {
      const base = value > 0 ? value : p.min;
      v = direction > 0 ? base * step : base / step;
    } else {
      v = value + direction * step;
    }
    v = Math.min(Math.max(v, p.min), p.max);
    if (p.key === 'filter.particleCount') return Math.round(v / 10) * 10;
    if (p.whole) return Math.round(v);
    return Number(v.toPrecision(3));
  }

  /**
   * Searches `params` (keys from PARAMS) from `start` (a flat config) over
   * `cases`. A change is kept only when it lowers the score by minGain and,
   * run for run against the current best, by more than `confidence`
   * standard errors: the noise between seeds is as big as most settings'
   * effects, and a search that kept every lucky draw would end up tuned to
   * its seeds. Yields { type: 'prepare' | 'run' | 'eval' | 'holdout', ... } as it goes
   * and returns { start, best, bestScore, evaluations, history, perCase,
   * holdout, changes }.
   */
  function* search({ start = defaultConfig(), cases = standardCases(), params = PARAMS.filter((p) => p.on).map((p) => p.key),
    budget = 100, seeds = DEFAULT_SEEDS, holdoutSeeds = HOLDOUT_SEEDS, cpuWeight = 0.15, minGain = 0.01,
    confidence = 1.0 } = {}) {
    const chosen = PARAMS.filter((p) => params.includes(p.key));
    const memo = new Map();
    let evaluations = 0;
    const history = [];

    // one true path per case and seed, which every candidate replays
    yield* prepare(cases, seeds.concat(holdoutSeeds), start);
    const baseline = yield* measure(start, cases, seeds);
    memo.set(configKey(start), 1);
    evaluations++;
    let best = Object.assign({}, start);
    let bestScore = 1;
    let bestMeasured = baseline;
    history.push({ evaluation: evaluations, score: bestScore });
    yield { type: 'eval', evaluations, budget, score: 1, bestScore, accepted: true, key: null, best };

    const steps = new Map(chosen.map((p) => [p.key, initialStep(p)]));
    let stalled = false;
    while (evaluations < budget && !stalled) {
      let improved = false;
      for (const p of chosen) {
        if (evaluations >= budget) break;
        for (const direction of [1, -1]) {
          if (evaluations >= budget) break;
          const value = moved(p, best[p.key], direction, steps.get(p.key));
          if (value === best[p.key]) continue;
          const candidate = Object.assign({}, best, { [p.key]: value });
          const key = configKey(candidate);
          if (memo.has(key)) continue;
          const measured = yield* measure(candidate, cases, seeds);
          evaluations++;
          const s = score(measured, baseline, candidate, start, cpuWeight);
          memo.set(key, s);
          const vsBest = paired(measured, bestMeasured);
          const accepted = s < bestScore * (1 - minGain) && vsBest.mean < -confidence * vsBest.se;
          if (accepted) {
            best = candidate;
            bestScore = s;
            bestMeasured = measured;
            improved = true;
          }
          history.push({ evaluation: evaluations, score: bestScore });
          yield { type: 'eval', evaluations, budget, score: s, bestScore, accepted, key: p.key, value,
            gainPct: -100 * vsBest.mean, sePct: 100 * vsBest.se, best };
          if (accepted) break; // keep going this way next sweep, from the new best
        }
      }
      if (!improved) {
        let anyLeft = false;
        for (const p of chosen) {
          const next = shrink(p, steps.get(p.key));
          steps.set(p.key, next);
          if (!stepIsTiny(p, next)) anyLeft = true;
        }
        stalled = !anyLeft;
      }
    }

    // checked on seeds the search never saw
    const holdoutStart = yield* measure(start, cases, holdoutSeeds);
    const holdoutBest = yield* measure(best, cases, holdoutSeeds);
    yield { type: 'holdout' };
    const holdoutScore = score(holdoutBest, holdoutStart, best, start, cpuWeight);
    const holdoutPaired = paired(holdoutBest, holdoutStart);
    // better on seeds it never saw, by more than chance: trust it. Otherwise it's a coin flip, and
    // the starting settings are as good as this suite can tell
    const verdict = holdoutPaired.mean < -2 * holdoutPaired.se ? 'better'
      : holdoutPaired.mean < 0 ? 'marginal' : 'none';

    const changes = [];
    for (const p of PARAMS) {
      if (best[p.key] !== start[p.key]) changes.push({ key: p.key, label: p.label, from: start[p.key], to: best[p.key] });
    }
    return {
      start,
      best,
      bestScore,
      evaluations,
      history,
      perCase: cases.map((c, i) => ({ id: c.id, name: c.name, start: baseline[i].metrics, best: bestMeasured[i].metrics,
        holdoutStart: holdoutStart[i].metrics, holdoutBest: holdoutBest[i].metrics })),
      holdout: { score: holdoutScore, gainPct: -100 * holdoutPaired.mean, sePct: 100 * holdoutPaired.se, verdict },
      changes,
    };
  }

  /** Runs a generator to the end, synchronously, and returns its result (Node, tests). */
  function finish(generator) {
    for (;;) {
      const step = generator.next();
      if (step.done) return step.value;
    }
  }

  // --- Calibrating the world from logs ---------------------------------------------------------

  /** A simulated run of `c`, recorded as the robot would log it, and parsed back. */
  function recordedLog(c, config, seed, fileIndex) {
    const sim = simulationFor(c, config, seed);
    sim.recorder = new REC.LogRecorder(sim, { fileIndex, name: routineOf(c).name });
    while (sim.program && sim.world.timeMs < 90000) sim.tick();
    return slt.parse(sim.recorder.text(), `SL${fileIndex}.CSV`);
  }

  /**
   * The simulated world that reproduces the robot's logs. mclcal.js measures
   * the noise, outliers, dropouts and wheel scale directly. Latency it can't:
   * a stale reading also drags the estimate back with it, so the leftover
   * error it measures is only part of the delay. So the sensors' delay is
   * matched instead: simulate the robot's own paths (or the routines, with
   * no paths) with its own localizer settings at several delays, measure each
   * the same way, and take the delay whose measurement matches the robot's.
   * Its uncertainty is mostly the robot's: about +-8ms from a few minutes of
   * logs, less with more.
   *
   * The localizer's own sensorLatencyMs isn't set from this: compensating
   * also widens each reading's uncertainty, and in the simulator matching
   * the full delay often does worse than leaving some of it. The search
   * decides that, on the calibrated world.
   *
   * Yields after each simulated run; returns { world, start, trajectories,
   * latency: { ok, delayMs, stderrMs, curve, robotMs }, calibration }.
   */
  function* calibrateWorld(logs, { delays = [0, 15, 30, 45, 60, 80, 100], seeds = [5, 6, 7] } = {}) {
    const calibration = cal.calibrate(logs);
    const world = Object.assign({}, calibration.world);
    // the chassis Auto-Tune last measured on the robot, where the logs have it
    const chassis = calibration.chassis || {};
    if (chassis.forward || chassis.strafe || chassis.turn) {
      const models = W.defaultSettings().models;
      for (const axis of ['forward', 'strafe', 'turn']) {
        if (chassis[axis]) models[axis] = { kS: chassis[axis].kS, kV: chassis[axis].kV, kA: chassis[axis].kA };
      }
      world.models = models;
      if (Number.isFinite(chassis.delayS)) world.delayS = chassis.delayS;
    }
    const start = Object.assign({}, calibration.settings);
    const trajectories = calibration.trajectories;
    const latency = { ok: false, robotMs: calibration.latency.ok ? calibration.latency.measuredMs : NaN, curve: [] };
    if (calibration.latency.ok) {
      const cases = trajectories.length
        ? trajectories.slice(0, 3).map((t, i) => ({ id: `replay-${i}`, routine: replayRoutine(t, 40), world }))
        : [{ id: 'tour', routine: 'tour', world }, { id: 'laps', routine: 'laps', world },
          { id: 'sprints', routine: 'sprints', world }];
      for (const delayMs of delays) {
        const logsAt = [];
        for (const c of cases) {
          for (const seed of seeds) {
            logsAt.push(recordedLog(Object.assign({}, c, { world: Object.assign({}, world, { sensorDelayMs: delayMs }) }),
              start, seed, 990000 + logsAt.length));
            yield { type: 'calibrate', delayMs };
          }
        }
        const measured = cal.calibrate(logsAt).latency;
        if (measured.ok) latency.curve.push({ delayMs, measuredMs: measured.measuredMs });
      }
      const line = invert(latency.curve, latency.robotMs);
      latency.delayMs = line.delayMs;
      latency.stderrMs = calibration.latency.stderrMs / line.slope;
      latency.ok = Number.isFinite(latency.delayMs);
      if (latency.ok) world.sensorDelayMs = Math.round(latency.delayMs);
    }
    return { world, start, trajectories, latency, calibration };
  }

  /**
   * The delay whose measurement is `target`: a straight line fitted through
   * the curve (each point is itself a noisy measurement, so a line is
   * steadier than joining the dots), solved for the target. Points past
   * 80ms are left out: there, readings start missing the beam's latency
   * window altogether and the measurement flattens. { delayMs, slope }.
   */
  function invert(curve, target) {
    const pts = curve.filter((p) => p.delayMs <= 80);
    if (pts.length < 2 || !Number.isFinite(target)) return { delayMs: NaN, slope: NaN };
    const n = pts.length;
    const mx = pts.reduce((a, p) => a + p.delayMs, 0) / n;
    const my = pts.reduce((a, p) => a + p.measuredMs, 0) / n;
    let sxy = 0;
    let sxx = 0;
    for (const p of pts) {
      sxy += (p.delayMs - mx) * (p.measuredMs - my);
      sxx += (p.delayMs - mx) * (p.delayMs - mx);
    }
    const slope = sxy / sxx;
    if (!(slope > 0.05)) return { delayMs: NaN, slope };
    return { delayMs: Math.max(0, mx + (target - my) / slope), slope };
  }

  /** The search's result as the C++ LocalizerConfig fields it changed, for localizer(). */
  function cppSnippet(config) {
    const defaults = defaultConfig();
    const lines = [];
    for (const s of TF.LOCALIZER_SETTINGS) {
      if (config[s.key] === undefined || config[s.key] === defaults[s.key]) continue;
      lines.push(`setLocalizerSetting(config, "${s.key}", ${TF.num(config[s.key])});`);
    }
    if (config.periodMs !== defaults.periodMs) lines.push(`// and localizer().startTask(${config.periodMs});`);
    if (!lines.length) return '// the defaults: nothing to change';
    return ['// in localizer(), before constructing it:', 'LocalizerConfig config;', ...lines].join('\n');
  }

  /** The config's mcl.* changes against `profile`, ready for tunefile.revise(). */
  function tuneChanges(config, profile) {
    const current = TF.localizerSettings(profile);
    const changes = [];
    for (const s of TF.LOCALIZER_SETTINGS) {
      if (config[s.key] !== undefined && config[s.key] !== current[s.key]) {
        changes.push({ key: `mcl.${s.key}`, value: config[s.key] });
      }
    }
    if (config.periodMs !== undefined && config.periodMs !== current.periodMs) {
      changes.push({ key: 'mcl.periodMs', value: config.periodMs });
    }
    return changes;
  }

  return {
    PARAMS,
    DEFAULT_SEEDS,
    HOLDOUT_SEEDS,
    defaultConfig,
    nest,
    cpuRatio,
    standardCases,
    robotCases,
    replayRoutine,
    referencePath,
    prepare,
    runCase,
    caseError,
    measure,
    paired,
    search,
    finish,
    calibrateWorld,
    recordedLog,
    invert,
    cppSnippet,
    tuneChanges,
  };
});
