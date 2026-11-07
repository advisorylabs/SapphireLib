/*
 * SapphireLib simulator: clawtiming.js
 *
 * autoClaw's timing, one approach at a time: the robot drives its open claw
 * onto a standing Pin, the claw's distance sensor reads it now and then, the
 * program looks at the latest reading every tick and fires the piston, and
 * the jaws shut some time later. Whether they shut with the Pin between them
 * comes down to how far the Pin travelled in the meantime, which is speed
 * times every lag in the chain:
 *
 *   the Pin at d ──(sensor latency)── reading readable ──(up to a sample
 *   period, then up to a tick)── the program sees it ──(valve and piston)──
 *   jaws shut
 *
 * At a steady speed the lags add up to a fixed time, so a fixed trigger
 * distance is right for one speed only: faster, the claw shuts late (the Pin
 * shoved by the back of the claw, or knocked over); slower, early. Firing
 * when the reading, less the distance still to cover in that time, reaches
 * where the jaws should shut fixes every speed at once (the lead rule). What
 * still limits the speed is the part of the timing that varies from grab to
 * grab: where the sensor's sample falls, where the tick falls, the valve, the
 * sensor's noise. Those spread the grabs out by speed times their spread,
 * and the claw's window doesn't grow with speed.
 *
 * The rules, src/robot/macros.cpp's autoClawAim() and the two before it:
 *   fixed      reading <= fixedMm (kPieceDetectMm, before the lead)
 *   lead       reading - speed * leadMs <= grabMm (commit a66eb14's reach)
 *   leadAge    reading - speed * (leadMs + tick + age) <= grabMm, where age
 *              is how long the reading has stood unchanged (the Pin kept
 *              coming while the sensor hadn't measured again) and tick is
 *              the program's period: half of it for when the reading
 *              arrived since the last look, half for where between two
 *              looks the Pin crosses. So leadMs is just the sensor's
 *              latency plus the valve and piston (macros.cpp's)
 * The two lead rules only fire on readings at maxReachMm or closer.
 *
 * Distances are as the claw's sensor reads them, in mm: what it reads with
 * the Pin against the back of the claw (backMm), just between the jaw tips
 * (tipsMm), and so on, so every number here is one you can read off the
 * sensor with a Pin held where it says. Speeds are in in/s toward the Pin.
 *
 * Plain script: window.SIM.clawtiming in a browser, require('./clawtiming.js')
 * in Node. The claw timing page (claw.html) draws it; macros.js uses its
 * sensor model and rule for the simulator's claw.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.clawtiming = factory();
  }
})(function () {
  'use strict';

  const kMmPerIn = 25.4;
  // pros::Distance::get_distance() with nothing in range
  const kNoObjectMm = 9999;
  const kMaxRangeMm = 2000;

  /**
   * The settings. The sensor's are world.js's for every distance sensor (V5 spec, half of it as
   * the standard deviation); the claw's geometry is macros.js's claw (a guess: a Pin 1.6in across,
   * caught anywhere from 1.5in behind to 1in ahead of the jaws' middle, 3.4in ahead of the
   * sensor); the valve is macros.js's kPistonMs, plus a guess at its spread; the rule is
   * macros.cpp's. Measure yours (docs/NEW_CHASSIS.md).
   */
  function defaults() {
    return {
      // the claw's distance sensor
      sensorPeriodMs: 33, // how often it measures (about 30Hz)
      sensorLatencyMs: 15, // from measuring to the reading being readable
      sensorNoiseMm: 7.5, // 1 standard deviation, up close

      // the program
      loopMs: 20, // how often it looks: driver control's tick (kDriverLoopMs); autonomous's task is 10
      speedNoiseInPerS: 0.5, // odometry's speed, 1 standard deviation

      // the valve and piston: from firing to the jaws shut on the Pin
      valveMs: 60,
      valveJitterMs: 5, // 1 standard deviation, tank pressure and all

      // the claw, as the sensor reads a Pin standing there
      backMm: 28, // against the back of the claw: no nearer
      tipsMm: 91, // just between the jaw tips: any further out and they shut in front of it
      shoveMm: 10, // how far the back of the claw can push a Pin before it tips over

      // the rule
      rule: 'leadAge', // 'fixed', 'lead' or 'leadAge'
      fixedMm: 125, // 'fixed': macros.cpp's kPieceDetectMm before the lead
      grabMm: 60, // the lead rules: where the jaws should shut on the Pin (kGrabMm)
      leadMs: 75, // the lead rules: from the sensor measuring to the jaws shut (kClawLeadMs)
      maxReachMm: 250, // the lead rules: never fire on a reading further out than this
      maxAgeMs: 50, // 'leadAge': the most a reading counts as aged (kClawReadingMaxAgeMs)

      // the approach
      accelInPerS2: 0, // speeding up (+) or slowing (-) through the grab
    };
  }

  /** mulberry32: a small seeded generator, with normals by Box-Muller. */
  class Rng {
    constructor(seed) {
      this.s = seed >>> 0;
      this.spare = null;
    }

    uniform() {
      this.s = (this.s + 0x6d2b79f5) >>> 0;
      let t = this.s;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    }

    normal() {
      if (this.spare !== null) {
        const s = this.spare;
        this.spare = null;
        return s;
      }
      let u = 0;
      while (u === 0) u = this.uniform();
      const v = this.uniform();
      const r = Math.sqrt(-2 * Math.log(u));
      this.spare = r * Math.sin(2 * Math.PI * v);
      return r * Math.cos(2 * Math.PI * v);
    }
  }

  /**
   * Whether the rule fires on this tick's look: the reading (mm), how long it has stood unchanged
   * (ms), and odometry's speed toward the Pin (mm per ms). Returns the distance the rule compares
   * with its trigger, and whether it fires. The same as macros.cpp's autoClawFires().
   */
  function ruleFires(p, readingMm, ageMs, speedMmPerMs) {
    if (p.rule === 'fixed') return { predictedMm: readingMm, fires: readingMm <= p.fixedMm };
    if (readingMm > p.maxReachMm) return { predictedMm: readingMm, fires: false };
    const ahead = p.rule === 'leadAge' ? p.leadMs + p.loopMs + ageMs : p.leadMs;
    const predictedMm = readingMm - Math.max(0, speedMmPerMs) * ahead;
    return { predictedMm, fires: predictedMm <= p.grabMm };
  }

  /** Which way a grab went, by where the Pin was as the jaws shut. */
  function outcomeOf(p, pinAtShutMm) {
    if (pinAtShutMm > p.tipsMm) return 'early';
    if (pinAtShutMm < p.backMm - p.shoveMm) return 'late';
    return 'caught';
  }

  /**
   * The robot's approach: `speedInPerS` toward the Pin as it reaches the middle of the claw's
   * window, speeding up or slowing at accelInPerS2 all the way (never backing up: slowing, it
   * stops; speeding up, it was standing still before). Time 0 is far enough out that the sensor
   * has read the Pin beyond any trigger, or where the robot started from rest. Returns the Pin's
   * distance (mm) and the robot's speed (mm per ms) at each time, and when it stops.
   */
  function approach(p, speedInPerS) {
    const vc = (speedInPerS * kMmPerIn) / 1000; // mm per ms, at the window's middle
    const a = (p.accelInPerS2 * kMmPerIn) / 1e6; // mm per ms^2
    // time measured from the window's middle (tau), over which the robot moves
    const tauLo = a > 0 ? -vc / a : -Infinity;
    const tauHi = a < 0 ? -vc / a : Infinity;
    const clampTau = (tau) => Math.min(Math.max(tau, tauLo), tauHi);
    const travel = (tau) => {
      const t = clampTau(tau);
      return vc * t + 0.5 * a * t * t;
    };
    const middleMm = windowMiddleMm(p);
    const trigger = p.rule === 'fixed' ? p.fixedMm : Math.max(p.maxReachMm, p.grabMm);
    // far enough out that a sample period, its latency and a tick go by before the trigger
    const margin = (u) => trigger + 60 + u * (p.sensorPeriodMs + p.sensorLatencyMs + p.loopMs + 40);
    const tauFor = (mm) => {
      // the tau, before the middle, at which the Pin was mm out (bisection: travel only grows)
      const want = middleMm - mm;
      if (want <= travel(tauLo)) return Number.isFinite(tauLo) ? tauLo : want / Math.max(vc, 1e-9);
      let lo = -1e6;
      let hi = 0;
      for (let i = 0; i < 80; ++i) {
        const mid = (lo + hi) / 2;
        if (travel(mid) < want) lo = mid;
        else hi = mid;
      }
      return hi;
    };
    let tau0 = tauFor(margin(vc));
    tau0 = tauFor(margin(Math.max(vc, vc + a * clampTau(tau0))));
    const speedAt = (t) => Math.max(0, vc + a * clampTau(t + tau0));
    const pinAt = (t) => middleMm - travel(t + tau0);
    return { startMm: pinAt(0), pinAt, speedAt, stopMs: Number.isFinite(tauHi) ? tauHi - tau0 : Infinity };
  }

  /**
   * One approach at `speedInPerS`, with randomness from `rng`. The Pin starts out beyond any
   * trigger, the sensor and the program's ticks at random phases. With `trace`, it also returns
   * every look the program took and every sample the sensor took, for drawing.
   *
   * Returns { outcome: 'caught' | 'early' | 'late' | 'missed' | 'stopped', fireMs, readingMm,
   * ageMs, predictedMm, shutMs, pinAtShutMm, errorMm (pinAtShutMm - grabMm, or - fixedMm for the
   * fixed rule) }. 'missed': the Pin went past without the rule firing; 'stopped': the robot
   * stopped short of it.
   */
  function grabTrial(p, speedInPerS, rng, { trace = false } = {}) {
    const motion = approach(p, speedInPerS);
    const { startMm, pinAt, speedAt, stopMs } = motion;

    const phase = rng.uniform() * p.sensorPeriodMs;
    const tickPhase = rng.uniform() * p.loopMs;
    const samples = new Map();
    const sampleAt = (k) => {
      if (!samples.has(k)) {
        const t = phase + k * p.sensorPeriodMs;
        const mm = Math.round(pinAt(t) + rng.normal() * p.sensorNoiseMm);
        samples.set(k, { tMs: t, mm: mm > kMaxRangeMm ? kNoObjectMm : Math.max(0, mm), trueMm: pinAt(t) });
      }
      return samples.get(k);
    };

    const looks = trace ? [] : null;
    let last = null;
    let changedMs = 0;
    const result = { outcome: 'missed', fireMs: null, readingMm: null, ageMs: null, predictedMm: null,
      shutMs: null, pinAtShutMm: null, errorMm: null };
    const lateEdge = p.backMm - p.shoveMm;
    for (let j = 0; ; ++j) {
      const t = tickPhase + j * p.loopMs;
      const k = Math.floor((t - p.sensorLatencyMs - phase) / p.sensorPeriodMs);
      const readingMm = k < 0 ? kNoObjectMm : sampleAt(k).mm;
      if (readingMm !== last) {
        last = readingMm;
        changedMs = t;
      }
      const ageMs = Math.min(t - changedMs, p.maxAgeMs ?? Infinity);
      const speed = Math.max(0, speedAt(t) + (rng.normal() * p.speedNoiseInPerS * kMmPerIn) / 1000);
      const r = ruleFires(p, readingMm, ageMs, speed);
      if (looks) looks.push({ tMs: t, readingMm, ageMs, predictedMm: r.predictedMm, fires: r.fires, trueMm: pinAt(t) });
      if (r.fires) {
        const shutMs = t + Math.max(0, p.valveMs + rng.normal() * p.valveJitterMs);
        const pinAtShutMm = pinAt(shutMs);
        Object.assign(result, { outcome: outcomeOf(p, pinAtShutMm), fireMs: t, readingMm, ageMs,
          predictedMm: r.predictedMm, shutMs, pinAtShutMm,
          errorMm: pinAtShutMm - (p.rule === 'fixed' ? p.fixedMm : p.grabMm) });
        break;
      }
      if (pinAt(t) < lateEdge - 40) break;
      if (t > stopMs + 100) {
        result.outcome = 'stopped';
        break;
      }
    }
    if (trace) {
      const endMs = (result.shutMs ?? looks[looks.length - 1].tMs) + 60;
      result.looks = looks;
      result.samples = [];
      for (let k = 0; phase + k * p.sensorPeriodMs <= endMs; ++k) result.samples.push(sampleAt(k));
      result.startMm = startMm;
      result.pinAt = pinAt;
      result.endMs = endMs;
    }
    return result;
  }

  /**
   * Many approaches at each speed. Returns one row per speed: { speedInPerS, n, caught, early,
   * late, missed, stopped (fractions), meanErrorMm, sdErrorMm (of the grabs that fired) }.
   */
  function sweep(p, { speeds, trials = 400, seed = 1 } = {}) {
    return speeds.map((speedInPerS, i) => {
      const rng = new Rng(seed * 7919 + i * 104729);
      const counts = { caught: 0, early: 0, late: 0, missed: 0, stopped: 0 };
      let sum = 0;
      let sum2 = 0;
      let fired = 0;
      for (let n = 0; n < trials; ++n) {
        const r = grabTrial(p, speedInPerS, rng);
        counts[r.outcome]++;
        if (r.errorMm !== null) {
          sum += r.errorMm;
          sum2 += r.errorMm * r.errorMm;
          fired++;
        }
      }
      const mean = fired ? sum / fired : NaN;
      const row = { speedInPerS, n: trials, meanErrorMm: mean,
        sdErrorMm: fired > 1 ? Math.sqrt(Math.max(0, sum2 / fired - mean * mean)) : NaN };
      for (const key of Object.keys(counts)) row[key] = counts[key] / trials;
      return row;
    });
  }

  /**
   * The fastest speed of a sweep at which, and at every speed below which, at least `threshold`
   * of the grabs caught their Pin. 0 if even the slowest didn't.
   */
  function maxSpeed(rows, threshold = 0.95) {
    let best = 0;
    for (const row of rows) {
      if (row.caught < threshold) break;
      best = row.speedInPerS;
    }
    return best;
  }

  /** Speeds from `from` to `to` in steps of `step`, in/s. */
  function speedRange(from, to, step) {
    const out = [];
    for (let v = from; v <= to + 1e-9; v += step) out.push(Math.round(v * 100) / 100);
    return out;
  }

  /** Where the claw's window is middle: the best grabMm for these settings. */
  function windowMiddleMm(p) {
    return (p.backMm - p.shoveMm + p.tipsMm) / 2;
  }

  /**
   * The leadMs that, at `speedInPerS`, puts the grabs' average where the rule aims (grabMm): the
   * lead that's right for these lags. The average miss is speed times the difference between
   * the lead and the real lags, so a couple of corrections land on it.
   */
  function bestLeadMs(p, speedInPerS, { trials = 600, seed = 3 } = {}) {
    const u = (speedInPerS * kMmPerIn) / 1000;
    let lead = p.leadMs;
    for (let i = 0; i < 4; ++i) {
      const [row] = sweep({ ...p, leadMs: lead }, { speeds: [speedInPerS], trials, seed });
      if (!Number.isFinite(row.meanErrorMm)) break;
      // shut early (positive error): fire later, with less lead
      lead = Math.max(0, lead - row.meanErrorMm / u);
    }
    return lead;
  }

  /**
   * Where the spread of the grabs comes from at `speedInPerS`, each as the standard deviation of
   * where the Pin is when the jaws shut (mm), and all of them together. Lags the lead covers move
   * every grab the same way, so they're not here: only what varies from one grab to the next.
   * Uniform spreads (sample, tick) are their width over root 12.
   */
  function errorBudget(p, speedInPerS) {
    const u = (speedInPerS * kMmPerIn) / 1000;
    const root12 = Math.sqrt(12);
    // a reading's age when the program sees it: anywhere in a sample period, or with leadAge, the
    // program counts it from when it saw the reading change, so only the tick's worth is left
    const sampleSpanMs = p.rule === 'leadAge' ? Math.min(p.loopMs, p.sensorPeriodMs) : p.sensorPeriodMs;
    const lead = p.rule === 'fixed' ? 0 : p.leadMs;
    const items = [
      { key: 'sample', label: 'Sensor sample timing', mm: (u * sampleSpanMs) / root12 },
      { key: 'tick', label: 'Program tick', mm: (u * p.loopMs) / root12 },
      { key: 'valve', label: 'Valve and piston', mm: u * p.valveJitterMs },
      { key: 'noise', label: 'Sensor noise', mm: p.sensorNoiseMm },
      { key: 'speed', label: 'Speed estimate', mm: ((p.speedNoiseInPerS * kMmPerIn) / 1000) * lead },
    ];
    const total = Math.sqrt(items.reduce((s, i) => s + i.mm * i.mm, 0));
    return { items, totalMm: total, windowMm: p.tipsMm - (p.backMm - p.shoveMm) };
  }

  return {
    kMmPerIn,
    kNoObjectMm,
    defaults,
    Rng,
    ruleFires,
    outcomeOf,
    grabTrial,
    sweep,
    maxSpeed,
    speedRange,
    windowMiddleMm,
    bestLeadMs,
    errorBudget,
  };
});
