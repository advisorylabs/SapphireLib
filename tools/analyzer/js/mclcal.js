/*
 * SapphireLib telemetry analyzer: mclcal.js
 *
 * Measures, from the robot's own logs, what Monte Carlo localization is up
 * against: how noisy the distance sensors really are, how often something
 * blocks one, how often one reads nothing with a wall in range, how stale a
 * reading is when it's used, whether a sensor reads consistently long or
 * short (a mount measured wrong), and how far off the tracking wheels' scale
 * is. All from two channels src/robot/telemetry.cpp records every update:
 *
 *   mcl        the estimate, its spread and agreement, and raw odometry
 *   mcl.beams  each sensor's reading (m), what the map says it should read
 *              from the estimate (e), and its closing speed (v)
 *
 * The residual m - e, over updates where the estimate is trustworthy, is the
 * sensor's error plus a little of the estimate's. Binned by distance, its
 * robust spread is the noise; its tails are outliers; regressed on v it
 * gives the latency still uncompensated (a reading taken L seconds before
 * it's used reads v·L long, and the localizer already took sensorLatencyMs
 * off). The estimate minus raw odometry is how far odometry has drifted;
 * its growth per inch driven, in the robot's frame, is the tracking wheels'
 * scale error.
 *
 * The simulator (tools/sim, its Tune tab) uses the result to make its world
 * behave like the real robot, replays the logged paths through it, and tunes
 * the localizer for that. test (tools/sim/test/calibrate.test.js) runs it on
 * simulated logs whose world is known, and checks it finds that world.
 *
 * Plain script: window.SA.mclcal in a browser, require('./mclcal.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./slt.js'), require('./tunefile.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.mclcal = factory(root.SA.slt, root.SA.tunefile);
  }
})(function (slt, TF) {
  'use strict';

  const kMadToSigma = 1.4826;
  const kBinIn = 8;
  const kMinBin = 25;
  const kMinLatencySpeed = 3; // in/s
  const kWindowS = 3;
  // the simulator's sensor: noise = scale * 0.5 * max(15mm, 5%), sampled every 33ms (world.js)
  const kSimFloorIn = 15 / 25.4;
  const kSimSensorPeriodMs = 33;

  function median(values) {
    if (!values.length) return NaN;
    const sorted = Float64Array.from(values).sort();
    const mid = sorted.length >> 1;
    return sorted.length % 2 ? sorted[mid] : 0.5 * (sorted[mid - 1] + sorted[mid]);
  }

  function percentile(values, p) {
    if (!values.length) return NaN;
    const sorted = Float64Array.from(values).sort();
    return sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))];
  }

  function robustSigma(values) {
    const m = median(values);
    return kMadToSigma * median(values.map((v) => Math.abs(v - m)));
  }

  /** The localizer settings a log ran with: its #meta lines over the defaults. */
  function settingsOf(log) {
    const out = {};
    for (const s of TF.LOCALIZER_SETTINGS) {
      const text = log.meta[`mcl.${s.key}`];
      const value = text === undefined ? NaN : Number(text);
      out[s.key] = Number.isFinite(value) ? value : s.default;
    }
    const period = Number(log.meta['mcl.periodMs']);
    out.periodMs = Number.isFinite(period) ? period : TF.DEFAULT_PERIOD_MS;
    return out;
  }

  /** The sensor mounts from #meta mcl.sensorN: [{ forwardIn, rightIn, facingDeg }]. */
  function mountsOf(log) {
    const out = [];
    for (let i = 0; ; ++i) {
      const text = log.meta[`mcl.sensor${i}`];
      if (text === undefined) break;
      const [forwardIn, rightIn, facingDeg] = text.split(',').map(Number);
      out.push({ forwardIn, rightIn, facingDeg });
    }
    return out;
  }

  /** The configured sensor noise at a distance: readingSigmaIn() without latency. */
  function configuredSigma(settings, distanceIn) {
    return Math.max(settings['filter.beam.minSigmaIn'], settings['filter.beam.sigmaFraction'] * distanceIn);
  }

  /** Heading rate at t, degrees per second, from odom's heading around it. */
  function turnRateAt(odom, t) {
    const i = odom.indexAt(t);
    if (i < 1 || i + 1 >= odom.length) return 0;
    const dh = ((odom.cols.heading[i + 1] - odom.cols.heading[i - 1] + 540) % 360) - 180;
    const dt = odom.t[i + 1] - odom.t[i - 1];
    return dt > 0 ? dh / dt : 0;
  }

  // --- One log's samples -----------------------------------------------------------------

  /**
   * Every reading the log has, against what the map said, flagged trusted
   * when its update's estimate was one the localizer would correct with.
   */
  function readingsOf(log, settings) {
    const mcl = log.get('mcl');
    const beams = log.get('mcl.beams');
    if (!mcl || !beams || !mcl.has('spread') || !beams.has('m0')) return [];
    const odom = log.get('odom');
    const out = [];
    const maxRange = settings['filter.beam.maxRangeIn'];
    const agreed = (k) => k >= 0 && k < mcl.length && mcl.cols.agree[k] >= Math.max(1, settings.minAgreeingSensors);
    for (let i = 0; i < beams.length; ++i) {
      const t = beams.t[i];
      const j = mcl.indexAt(t + 1e-4);
      if (j < 0 || Math.abs(mcl.t[j] - t) > 0.002) continue;
      // trusted by the updates either side, not this one: a dropped or blocked reading lowers
      // its own update's agreement, and judging by that would hide exactly those readings
      const trusted = mcl.cols.spread[j] <= settings.maxCorrectionSpreadIn && agreed(j - 1) && agreed(j + 1);
      const spinning = odom ? Math.abs(turnRateAt(odom, t)) > 0.8 * settings.maxTurnRateDegPerS : false;
      for (let s = 0; s < 4; ++s) {
        if (!beams.has(`m${s}`)) break;
        const m = beams.cols[`m${s}`][i];
        const e = beams.cols[`e${s}`][i];
        const v = beams.cols[`v${s}`][i];
        if (!Number.isFinite(e) || e > maxRange) continue; // no wall in range: nothing to learn
        out.push({ t, s, m, e, v, trusted, spinning });
      }
    }
    return out;
  }

  /**
   * Raw odometry's drift against the estimate, over windows of about
   * kWindowS: { fwd, lat, driftFwd, driftLat } per window, in the robot's
   * frame. A tracking wheel that reads 2% long drifts -2% of what it reads.
   */
  function driftWindowsOf(log, settings) {
    const mcl = log.get('mcl');
    const odom = log.get('odom');
    if (!mcl || !odom || !mcl.has('raw_x')) return [];
    const c = mcl.cols;
    const windows = [];
    let w = null;
    for (let j = 1; j < mcl.length; ++j) {
      const dt = mcl.t[j] - mcl.t[j - 1];
      const trusted = (k) => c.spread[k] <= settings.maxCorrectionSpreadIn && c.agree[k] >= 2;
      if (!(dt > 0 && dt < 0.12) || !trusted(j) || !trusted(j - 1)) {
        w = null;
        continue;
      }
      const rawDx = c.raw_x[j] - c.raw_x[j - 1];
      const rawDy = c.raw_y[j] - c.raw_y[j - 1];
      const driftDx = (c.x[j] - c.raw_x[j]) - (c.x[j - 1] - c.raw_x[j - 1]);
      const driftDy = (c.y[j] - c.raw_y[j]) - (c.y[j - 1] - c.raw_y[j - 1]);
      const h = odom.valueAt('heading', 0.5 * (mcl.t[j] + mcl.t[j - 1]), 0.05);
      if (!Number.isFinite(h) || !Number.isFinite(rawDx)) {
        w = null;
        continue;
      }
      const rad = (h * Math.PI) / 180;
      const fx = Math.sin(rad);
      const fy = Math.cos(rad);
      if (!w) {
        w = { start: mcl.t[j - 1], fwd: 0, lat: 0, driftFwd: 0, driftLat: 0 };
        windows.push(w);
      }
      w.fwd += rawDx * fx + rawDy * fy;
      w.lat += rawDx * fy - rawDy * fx;
      w.driftFwd += driftDx * fx + driftDy * fy;
      w.driftLat += driftDx * fy - driftDy * fx;
      if (mcl.t[j] - w.start >= kWindowS) w = null;
    }
    return windows;
  }

  /**
   * Paths to replay in the simulator: enabled stretches where the localizer
   * trusted its estimate most of the time (so the pose is in field
   * coordinates), as { name, start, samples: [{ tMs, xIn, yIn, headingDeg }] }
   * with headings unwrapped.
   */
  function trajectoriesOf(log, settings, { maxSeconds = 60, fieldHalfIn = 70.25 - 7 } = {}) {
    const odom = log.get('odom');
    const mcl = log.get('mcl');
    if (!odom || !mcl) return [];
    const out = [];
    const name = (log.meta.file || log.fileName || 'log').replace(/\.csv$/i, '');
    for (const session of slt.sessions(log)) {
      for (const [part, span] of [['auton', session.auton], ['driver', session.driver]]) {
        if (!span) continue;
        const [m0, m1] = mcl.range(span[0], span[1]);
        let trusted = 0;
        for (let j = m0; j < m1; ++j) {
          if (mcl.cols.spread[j] <= settings.maxCorrectionSpreadIn && mcl.cols.agree[j] >= 2) trusted++;
        }
        if (m1 - m0 < 40 || trusted < 0.6 * (m1 - m0)) continue;
        const [i0, i1] = odom.range(span[0], Math.min(span[1], span[0] + maxSeconds));
        const samples = [];
        let heading = null;
        let inside = true;
        for (let i = i0; i < i1; ++i) {
          const x = odom.cols.x[i];
          const y = odom.cols.y[i];
          const h = odom.cols.heading[i];
          if (![x, y, h].every(Number.isFinite)) continue;
          if (Math.abs(x) > fieldHalfIn || Math.abs(y) > fieldHalfIn) inside = false;
          heading = heading === null ? h : heading + ((((h - heading) % 360) + 540) % 360) - 180;
          samples.push({ tMs: Math.round((odom.t[i] - odom.t[i0]) * 1000), xIn: x, yIn: y, headingDeg: heading });
        }
        if (!inside || samples.length < 200) continue;
        out.push({ name: `${name} ${session.label}, ${part}`, start: { xIn: samples[0].xIn, yIn: samples[0].yIn,
          headingDeg: samples[0].headingDeg }, samples });
      }
    }
    return out;
  }

  // --- Fits --------------------------------------------------------------------------------

  /** sigma(d) = max(minSigma, fraction * d), by weighted least squares over the bins. */
  function fitSigmaModel(bins) {
    let best = null;
    for (let minSigma = 0.05; minSigma <= 4.0001; minSigma += 0.025) {
      for (let fraction = 0; fraction <= 0.2001; fraction += 0.001) {
        let cost = 0;
        for (const b of bins) {
          const err = b.sigmaIn - Math.max(minSigma, fraction * b.dIn);
          cost += b.n * err * err;
        }
        if (!best || cost < best.cost) best = { cost, minSigmaIn: minSigma, sigmaFraction: fraction };
      }
    }
    return best;
  }

  function fitNoise(readings, settings) {
    const inliers = readings.filter((r) => r.trusted && !r.spinning && Number.isFinite(r.m) &&
      Math.abs(r.m - r.e) <= Math.max(4, 6 * configuredSigma(settings, r.e)));
    const byBin = new Map();
    for (const r of inliers) {
      const bin = Math.floor(r.e / kBinIn);
      if (!byBin.has(bin)) byBin.set(bin, []);
      byBin.get(bin).push(r);
    }
    const bins = [];
    for (const [bin, rs] of [...byBin.entries()].sort((a, b) => a[0] - b[0])) {
      if (rs.length < kMinBin) continue;
      bins.push({ dIn: median(rs.map((r) => r.e)), sigmaIn: robustSigma(rs.map((r) => r.m - r.e)), n: rs.length,
        lo: bin * kBinIn, hi: (bin + 1) * kBinIn });
    }
    if (bins.length < 2) return { ok: false, bins, n: inliers.length, why: 'too few readings with a wall in range' };
    const model = fitSigmaModel(bins);
    // the simulator's noise is scale * 0.5 * max(15mm, 5%): least squares through the origin
    let num = 0;
    let den = 0;
    for (const b of bins) {
      const unit = 0.5 * Math.max(kSimFloorIn, 0.05 * b.dIn);
      num += b.n * b.sigmaIn * unit;
      den += b.n * unit * unit;
    }
    return { ok: true, bins, n: inliers.length, minSigmaIn: model.minSigmaIn, sigmaFraction: model.sigmaFraction,
      simNoiseScale: num / den, sigmaAt: (d) => Math.max(model.minSigmaIn, model.sigmaFraction * d) };
  }

  function fitOutliers(readings, noise) {
    const valid = readings.filter((r) => r.trusted && !r.spinning && Number.isFinite(r.m));
    if (valid.length < 200 || !noise.ok) return { ok: false, n: valid.length, why: 'too few readings' };
    let outliers = 0;
    let short = 0;
    let detectable = 0;
    for (const r of valid) {
      const band = Math.max(2, 4 * noise.sigmaAt(r.e));
      if (Math.abs(r.m - r.e) > band) {
        outliers++;
        if (r.m < r.e) short++;
      }
      // how likely the simulator's outlier (anywhere between the sensor and the wall) lands
      // outside the band, so the simulator's rate can be set to produce the same count
      detectable += Math.max(0, (r.e - band - 1) / Math.max(r.e - 1, 1e-6));
    }
    const rate = outliers / valid.length;
    const seen = detectable / valid.length;
    return { ok: true, n: valid.length, rate, shortShare: outliers ? short / outliers : 0,
      simPct: seen > 0.05 ? Math.min(50, (100 * rate) / seen) : 100 * rate };
  }

  function fitDropouts(readings, settings) {
    // a wall comfortably in range, and not mid-spin (the localizer skips those on purpose)
    const inRange = readings.filter((r) => r.trusted && !r.spinning && r.e <= settings['filter.beam.maxRangeIn'] - 4);
    if (inRange.length < 200) return { ok: false, n: inRange.length, why: 'too few readings' };
    const missing = inRange.filter((r) => !Number.isFinite(r.m)).length;
    const rate = missing / inRange.length;
    return { ok: true, n: inRange.length, rate, simPct: 100 * rate };
  }

  /**
   * Each sensor's offset (a mount measured wrong reads long or short all the
   * time) and the latency left uncompensated (a reading v·L long at closing
   * speed v), fitted together: residual = offset[sensor] + slope · v. One at
   * a time, they leak into each other: a sensor that's mostly driven away
   * from (the back one, on a robot that drives forward) reads short from
   * latency alone, which a plain median calls a mount error. Alternates a
   * robust offset per sensor (median) with a least-squares slope, which
   * settles in a few rounds. { offsets: Map, slope, stderr, n, moving }.
   */
  function fitOffsetsAndLatency(readings, noise) {
    const pts = readings.filter((r) => r.trusted && !r.spinning && Number.isFinite(r.m) && Number.isFinite(r.v) &&
      noise.ok && Math.abs(r.m - r.e) <= Math.max(2, 4 * noise.sigmaAt(r.e)));
    const moving = pts.filter((p) => Math.abs(p.v) >= kMinLatencySpeed);
    const offsets = new Map();
    let slope = 0;
    for (let round = 0; round < 6; ++round) {
      for (let s = 0; s < 4; ++s) {
        const mine = pts.filter((p) => p.s === s).map((p) => p.m - p.e - slope * p.v);
        offsets.set(s, mine.length ? median(mine) : 0);
      }
      let svv = 0;
      let svr = 0;
      for (const p of moving) {
        svv += p.v * p.v;
        svr += p.v * (p.m - p.e - offsets.get(p.s));
      }
      slope = svv > 0 ? svr / svv : 0;
    }
    let ss = 0;
    let svv = 0;
    for (const p of moving) {
      const r = p.m - p.e - offsets.get(p.s) - slope * p.v;
      ss += r * r;
      svv += p.v * p.v;
    }
    const stderr = svv > 0 ? Math.sqrt(ss / Math.max(1, moving.length - 1) / svv) : Infinity;
    const counts = new Map([0, 1, 2, 3].map((s) => [s, pts.filter((p) => p.s === s).length]));
    return { offsets, counts, slope, stderr, n: pts.length, moving: moving.length };
  }

  function fitLatency(joint, settings) {
    const configuredMs = settings.sensorLatencyMs;
    if (joint.moving < 100) {
      return { ok: false, n: joint.moving, configuredMs, why: 'too little driving toward or away from walls' };
    }
    // seconds of latency left uncompensated, as a lower bound (see calibrate())
    const measuredMs = configuredMs + 1000 * joint.slope;
    return { ok: true, n: joint.moving, configuredMs, measuredMs, stderrMs: 1000 * joint.stderr,
      simDelayMs: Math.max(0, measuredMs - kSimSensorPeriodMs / 2) };
  }

  function sensorBiases(readings, mounts, joint) {
    const out = [];
    for (let s = 0; s < 4; ++s) {
      const mine = readings.filter((r) => r.s === s);
      if (!mine.length) continue;
      const valid = mine.filter((r) => r.trusted && Number.isFinite(r.m)).length;
      const n = joint.counts.get(s);
      out.push({ index: s, mount: mounts[s] || null, n, medianIn: n ? joint.offsets.get(s) : NaN,
        agreeShare: valid ? n / valid : 0 });
    }
    return out;
  }

  function fitOdometryScale(windows) {
    let sff = 0;
    let sdf = 0;
    let sll = 0;
    let sdl = 0;
    for (const w of windows) {
      sff += w.fwd * w.fwd;
      sdf += w.driftFwd * w.fwd;
      sll += w.lat * w.lat;
      sdl += w.driftLat * w.lat;
    }
    // drift per inch odometry read, along each wheel; true travel is (1 + slope) times what was read
    const travelIn = windows.reduce((sum, w) => sum + Math.hypot(w.fwd, w.lat), 0);
    if (travelIn < 200) return { ok: false, windows: windows.length, travelIn, why: 'too little driving' };
    const fwd = sff > 400 ? sdf / sff : NaN;
    const lat = sll > 400 ? sdl / sll : NaN;
    const err = (slope) => (Number.isFinite(slope) ? (-100 * slope) / (1 + slope) : NaN);
    const known = [err(fwd), err(lat)].filter(Number.isFinite);
    return {
      ok: known.length > 0,
      windows: windows.length,
      travelIn,
      // a wheel that reads 2% long: +2 here, and its real diameter is 1/1.02 of the configured one
      verticalReadsLongPct: err(fwd),
      horizontalReadsLongPct: err(lat),
      verticalDiameterScale: Number.isFinite(fwd) ? 1 + fwd : NaN,
      horizontalDiameterScale: Number.isFinite(lat) ? 1 + lat : NaN,
      simWheelErrorPct: known.length ? known.reduce((a, b) => a + b, 0) / known.length : NaN,
    };
  }

  function cpuOf(logs) {
    const updateUs = [];
    const samp = [];
    const fmt = [];
    for (const log of logs) {
      const mcl = log.get('mcl');
      if (mcl && mcl.has('us')) {
        for (const us of mcl.cols.us) if (us > 0) updateUs.push(us);
      }
      for (const h of log.health) {
        if (typeof h.values.samp_us === 'number') samp.push(h.values.samp_us);
        if (typeof h.values.fmt_us === 'number') fmt.push(h.values.fmt_us);
      }
    }
    const mean = (a) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);
    const periodMs = logs.length ? settingsOf(logs[0]).periodMs : TF.DEFAULT_PERIOD_MS;
    const mclUs = mean(updateUs);
    return {
      mclUpdateUs: { mean: mclUs, p95: percentile(updateUs, 0.95), n: updateUs.length },
      // share of one CPU each takes; an upper bound, since preemption counts as busy
      mclPct: Number.isFinite(mclUs) ? (100 * mclUs) / (periodMs * 1000) : NaN,
      loggerPct: samp.length ? (mean(samp) + mean(fmt)) / 1e4 : NaN,
      samplerUsPerS: mean(samp),
      formatUsPerS: mean(fmt),
    };
  }

  /**
   * The chassis Auto-Tune last measured on the robot, from the logs' `tune`
   * events: { forward, strafe, turn, delayS }, each axis { kS, kV, kA } or
   * missing.
   */
  function chassisOf(logs) {
    const latest = {};
    const delays = [];
    const names = { Fwd: 'forward', Strafe: 'strafe', Turn: 'turn' };
    for (const log of logs) {
      for (const m of slt.tuneEvents(log).models) {
        const axis = names[m.axis];
        if (!axis || !(m.kV > 0 && m.kA > 0)) continue;
        latest[axis] = { kS: m.kS, kV: m.kV, kA: m.kA };
        if (m.delayS > 0) delays.push(m.delayS);
      }
    }
    if (delays.length) latest.delayS = delays.reduce((a, b) => a + b, 0) / delays.length;
    return latest;
  }

  // --- All of it ---------------------------------------------------------------------------

  /**
   * Calibrates from one or more parsed logs (slt.parse()). Returns every
   * measurement with its sample count and whether it had enough data (ok),
   * the simulator world settings they come to (`world`), paths to replay,
   * the chassis Auto-Tune measured, and the CPU the localizer and logger
   * took.
   *
   * Nothing here is a LocalizerConfig change on its own: the settings that
   * suit this world are the simulator's search to find (tuner.js). Latency
   * especially: latency.measuredMs is a lower bound (a stale reading drags
   * the estimate back with it, hiding part of the delay), and
   * world.sensorDelayMs from it is only a first guess, which the simulator
   * replaces by matching (tuner.js calibrateWorld()).
   */
  function calibrate(logs) {
    logs = [].concat(logs);
    const problems = [];
    const usable = logs.filter((log) => log.get('mcl.beams') && log.get('mcl') && log.get('mcl').has('raw_x'));
    if (!usable.length) {
      problems.push('No mcl.beams channel: these logs predate per-sensor MCL logging, or MCL wasn\'t running.');
    }
    const settings = usable.length ? settingsOf(usable[0]) : TF.localizerSettings(TF.emptyProfile());
    const mounts = usable.length ? mountsOf(usable[0]) : [];
    let readings = [];
    let windows = [];
    let trajectories = [];
    for (const log of usable) {
      const own = settingsOf(log);
      readings = readings.concat(readingsOf(log, own));
      windows = windows.concat(driftWindowsOf(log, own));
      trajectories = trajectories.concat(trajectoriesOf(log, own));
    }
    const noise = fitNoise(readings, settings);
    const outliers = fitOutliers(readings, noise);
    const dropouts = fitDropouts(readings, settings);
    const joint = fitOffsetsAndLatency(readings, noise);
    const latency = fitLatency(joint, settings);
    const sensors = sensorBiases(readings, mounts, joint);
    const odometry = fitOdometryScale(windows);

    const world = {};
    if (noise.ok) world.sensorNoiseScale = round(noise.simNoiseScale, 2);
    if (outliers.ok) world.sensorOutlierPct = round(outliers.simPct, 1);
    if (dropouts.ok) world.sensorDropoutPct = round(dropouts.simPct, 1);
    if (latency.ok) world.sensorDelayMs = Math.round(latency.simDelayMs);
    if (odometry.ok) world.wheelDiameterErrorPct = round(odometry.simWheelErrorPct, 2);

    return {
      ok: usable.length > 0 && noise.ok,
      problems,
      logs: usable.length,
      settings,
      mounts,
      readings: { total: readings.length, trusted: readings.filter((r) => r.trusted).length },
      noise: { ...noise, sigmaAt: undefined },
      outliers,
      dropouts,
      latency,
      sensors,
      odometry,
      cpu: cpuOf(logs),
      chassis: chassisOf(logs),
      trajectories,
      world,
    };
  }

  function round(value, digits) {
    const k = 10 ** digits;
    return Math.round(value * k) / k;
  }

  return { calibrate, settingsOf, mountsOf, readingsOf, driftWindowsOf, trajectoriesOf, chassisOf, fitSigmaModel };
});
