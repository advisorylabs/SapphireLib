/*
 * SapphireLib telemetry analyzer: tuning.js
 *
 * Offline tuning from logs: the practical half of "a tuner in the analyzer".
 * The analyzer can't drive the robot, but it doesn't need to: a log already
 * holds what Auto-Tune measures (its char.* runs, and in any match the
 * volts every mechanism was sent next to where it went). So this module:
 *
 *   1. finds the tunable systems in a log: the drivetrain's axes and every
 *      position mechanism (a pid channel "X" with an "X.act" channel);
 *   2. extracts fit data from Auto-Tune runs, or passively from ordinary
 *      driving over any time range;
 *   3. fits the model with the robot's own math (model.js), designs gains for
 *      each controller with the robot's own pole placement, and
 *   4. replays what the robot was asked to do through the fitted plant, with
 *      the gains it had and with the new ones, so a design is judged against
 *      real targets before it's ever flashed; and
 *   5. writes the C++ to paste into the robot program.
 *
 * Plain script: window.SA.tuning in a browser, require('./tuning.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./slt.js'), require('./model.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.tuning = factory(root.SA.slt, root.SA.model);
  }
})(function (slt, M) {
  'use strict';

  // The robot program's own specs (src/robot/tuning.cpp), as defaults.
  const DRIVE_AXES = [
    { axis: 'Fwd', char: 'char.fwd', column: 'fwd_v', unit: 'in', minSpeed: 2 },
    { axis: 'Strafe', char: 'char.strafe', column: 'strafe_v', unit: 'in', minSpeed: 2 },
    { axis: 'Turn', char: 'char.turn', column: 'turn_v', unit: 'deg', minSpeed: 5 },
  ];
  const DRIVE_CONTROLLERS = [
    { name: 'Drive', pid: 'drive', axis: 'Fwd', spec: M.responseSpec(0.6, 1, 50), threshold: 1 },
    { name: 'Turn', pid: 'turn', axis: 'Turn', spec: M.responseSpec(0.5, 1, 50), threshold: 2 },
    { name: 'Hold', pid: 'hold', axis: 'Turn', spec: M.responseSpec(0.9, 1, 50), threshold: 2 },
  ];
  const MECHANISM_SPEC = M.responseSpec(0.5, 1, 50);

  function capitalize(name) {
    return name.charAt(0).toUpperCase() + name.slice(1);
  }

  /**
   * The tunable systems in `log`: [{ id, label, kind: 'drive'|'mechanism',
   * unit, minSpeed, gravity, charChannel, axisName, controllers: [{ name, pid,
   * spec, threshold }], autotuneRuns: [...] }].
   */
  function systems(log) {
    const out = [];
    const hasPassiveDrive = log.get('chassis') && log.get('odom');
    for (const a of DRIVE_AXES) {
      const runs = slt.characterizationRuns(log, a.char, a.axis);
      if (runs.length === 0 && !hasPassiveDrive) continue;
      out.push({
        id: `axis:${a.axis}`,
        label: `${a.axis} axis`,
        kind: 'drive',
        unit: a.unit,
        minSpeed: a.minSpeed,
        gravity: M.gravityShape('none'),
        charChannel: a.char,
        axisName: a.axis,
        column: a.column,
        autotuneRuns: runs,
        controllers: DRIVE_CONTROLLERS.filter((c) => c.axis === a.axis && log.get(c.pid))
          .map((c) => Object.assign({ loopS: c.pid === 'hold' ? 0.02 : 0.01 }, c)),
      });
    }
    for (const act of log.channels) {
      if (!act.name.endsWith('.act') || !['target', 'pos', 'volts', 'law'].every((c) => act.has(c))) {
        continue;
      }
      const base = act.name.slice(0, -4);
      const pid = log.get(base);
      const axisName = capitalize(base);
      const charName = `char.${base}`;
      const runs = slt.characterizationRuns(log, charName, axisName);
      const loopS = pid && pid.config ? pid.config.nominal_dt_s : 0.02;
      out.push({
        id: `mech:${base}`,
        label: capitalize(base),
        kind: 'mechanism',
        unit: 'deg',
        minSpeed: 10,
        gravity: M.gravityShape('constant'),
        charChannel: charName,
        axisName,
        actChannel: act.name,
        autotuneRuns: runs,
        controllers: pid ? [{ name: axisName, pid: base, spec: MECHANISM_SPEC, threshold: 15,
          loopS }] : [],
      });
    }
    return out;
  }

  // --- Fit data ---------------------------------------------------------------------

  /** Splits { t, volts, position } samples into runs at time gaps. */
  function toRuns(samples, maxGapS) {
    const runs = [];
    let current = null;
    let t0 = 0;
    let last = null;
    for (const s of samples) {
      if (!Number.isFinite(s.position)) {
        current = null;
        last = null;
        continue;
      }
      if (!current || s.t - last > maxGapS) {
        current = [];
        runs.push(current);
        t0 = s.t;
      }
      current.push({ timeMs: Math.round((s.t - t0) * 1000), volts: s.volts, position: s.position });
      last = s.t;
    }
    return runs.filter((r) => r.length > 12);
  }

  /**
   * Passive fit data for a mechanism over [t0, t1] from its X.act rows: the
   * volts it was sent against where it went. Braked and external rows (law
   * no sensor, off, external) have no known volts, so they're NaN, which
   * the fit skips.
   */
  function mechanismSamples(log, system, t0, t1) {
    const act = log.get(system.actChannel);
    const [i0, i1] = act.range(t0, t1);
    const out = [];
    const { volts, pos, law } = act.cols;
    for (let i = i0; i < i1; ++i) {
      const braked = law[i] === M.LAW.sensorLost || law[i] === M.LAW.off || law[i] === M.LAW.external;
      out.push({ t: act.t[i], volts: braked ? NaN : volts[i], position: pos[i] });
    }
    return out;
  }

  /**
   * Passive fit data for a drivetrain axis over [t0, t1]: the axis volts from
   * "chassis" against the odometry pose, resolved into the robot's own frame
   * (distance along forward, along strafe, or cumulative heading). Ticks where
   * the corner mix could have clipped (|fwd|+|strafe|+|turn| > 12, which only
   * autonomous's raw PID output reaches) are NaN: what the motors actually
   * got there isn't known.
   */
  function driveSamples(log, system, t0, t1) {
    const chassis = log.get('chassis');
    const odom = log.get('odom');
    // raw odometry where the log has it: the localizer's correction easing in moves the corrected
    // pose at up to 4in/s with no volts behind it, which reads as motion
    const xs = odom.has('raw_x') ? odom.cols.raw_x : odom.cols.x;
    const ys = odom.has('raw_y') ? odom.cols.raw_y : odom.cols.y;
    const [i0, i1] = odom.range(t0, t1);
    const out = [];
    let along = 0;
    let heading = null;
    let prev = null;
    for (let i = i0; i < i1; ++i) {
      const t = odom.t[i];
      const j = chassis.indexAt(t + 0.002);
      if (j < 0 || Math.abs(chassis.t[j] - t) > 0.006) {
        prev = null;
        continue;
      }
      const f = chassis.cols.fwd_v[j];
      const s = chassis.cols.strafe_v[j];
      const r = chassis.cols.turn_v[j];
      const clipped = Math.abs(f) + Math.abs(s) + Math.abs(r) > 12;
      const x = xs[i];
      const y = ys[i];
      const h = odom.cols.heading[i];
      if (prev === null) {
        heading = h;
      } else {
        heading += ((((h - prev.h) % 360) + 540) % 360) - 180;
        const mid = (((prev.h + h) / 2) * Math.PI) / 180;
        const dx = x - prev.x;
        const dy = y - prev.y;
        if (system.axisName === 'Fwd') along += dx * Math.sin(mid) + dy * Math.cos(mid);
        if (system.axisName === 'Strafe') along += dx * Math.cos(mid) - dy * Math.sin(mid);
      }
      prev = { x, y, h };
      const volts = system.axisName === 'Fwd' ? f : system.axisName === 'Strafe' ? s : r;
      out.push({ t, volts: clipped ? NaN : volts, position: system.axisName === 'Turn' ? heading : along });
    }
    return out;
  }

  // --- Fitting ------------------------------------------------------------------------

  /**
   * Refits one Auto-Tune run exactly as the robot did (characterizeAxis() or
   * characterizeMechanism() on its CharacterizationData). Returns { ok, model:
   * MechanismModel, rSquared, samplesUsed, delayS, source }.
   */
  function fitAutotune(system, run, gravity = system.gravity) {
    const result = system.kind === 'mechanism' || gravity.kind !== 'none'
      ? M.characterizeMechanism(run.data, gravity, system.minSpeed)
      : (() => {
        const r = M.characterizeAxis(run.data, system.minSpeed);
        return { ok: r.ok, fit: { ok: r.fit.ok, model: M.mechanismModel(r.fit.model),
          rSquared: r.fit.rSquared, samplesUsed: r.fit.samplesUsed }, delayS: r.delayS };
      })();
    return {
      ok: result.ok,
      model: result.fit.model,
      rSquared: result.fit.rSquared,
      samplesUsed: result.fit.samplesUsed,
      delayS: result.delayS,
      source: `Auto-Tune run ${run.index + 1}`,
      reported: run.reported,
      runs: run.data.ramps.concat(run.data.steps),
    };
  }

  /**
   * Fits passively recorded data. With no clean steps to time a delay from,
   * the delay is found by trying each shift and keeping the best-explaining
   * one: the same regression, asked "which lag makes this make sense".
   */
  function fitPassive(system, samples, gravity = system.gravity, maxShift = 10) {
    const period = medianPeriod(samples);
    const runs = toRuns(samples, Math.max(0.1, 4 * period));
    let best = null;
    for (let shift = 0; shift <= maxShift; ++shift) {
      const fit = M.fitMechanism(runs, gravity, system.minSpeed, 3, shift, 0);
      if (!M.ffValid(fit.model.motion)) continue;
      if (!best || fit.rSquared > best.fit.rSquared + 1e-4) best = { fit, shift };
    }
    if (!best) {
      const fit = M.fitMechanism(runs, gravity, system.minSpeed, 3, 0, 0);
      return { ok: false, model: fit.model, rSquared: fit.rSquared, samplesUsed: fit.samplesUsed,
        delayS: 0, source: 'match data', runs, period };
    }
    return {
      ok: best.fit.rSquared >= 0.6,
      model: best.fit.model,
      rSquared: best.fit.rSquared,
      samplesUsed: best.fit.samplesUsed,
      delayS: best.shift * period,
      source: 'match data',
      runs,
      period,
    };
  }

  function medianPeriod(samples) {
    const gaps = [];
    for (let i = 1; i < samples.length && gaps.length < 2000; ++i) {
      const g = samples[i].t - samples[i - 1].t;
      if (g > 0 && g < 0.2) gaps.push(g);
    }
    gaps.sort((a, b) => a - b);
    return gaps.length ? gaps[gaps.length >> 1] : 0.01;
  }

  // --- Design ---------------------------------------------------------------------------

  /**
   * Designs one controller from a fit, as PidTunerPage does, including the
   * extra latency a loop slower than the characterization's sampling adds.
   */
  function design(fit, controller, spec = controller.spec, samplePeriodS = 0.01) {
    const slower = Math.max(0, (controller.loopS || 0.01) - samplePeriodS);
    return M.designPositionGains(fit.model.motion, spec, fit.delayS + 0.5 * slower);
  }

  /**
   * The gravity volts a mechanism was actually running with, inferred from
   * its log: what it sent (X.act volts) minus what its PID output, on the
   * tracking rows where neither was clamped.
   */
  function inferredGravityVolts(log, system, t0, t1) {
    const act = log.get(system.actChannel);
    const pid = log.get(system.controllers[0] ? system.controllers[0].pid : '');
    if (!act || !pid) return NaN;
    const [i0, i1] = act.range(t0, t1);
    const diffs = [];
    for (let i = i0; i < i1 && diffs.length < 4000; ++i) {
      if (act.cols.law[i] !== M.LAW.track || Math.abs(act.cols.volts[i]) >= 11.9) continue;
      const j = pid.indexAt(act.t[i]);
      if (j < 0 || act.t[i] - pid.t[j] > 0.005) continue;
      if (pid.cols.flags[j] & 1) continue;
      diffs.push(act.cols.volts[i] - pid.cols.out[j]);
    }
    if (diffs.length < 10) return NaN;
    diffs.sort((a, b) => a - b);
    return diffs[diffs.length >> 1];
  }

  /** The gains a controller had at time t (its last G row), or null. */
  function gainsAt(log, controller, t) {
    const ch = log.get(controller.pid);
    if (!ch) return null;
    const g = ch.gainsAt(t) || ch.gains[0];
    return g ? { kP: g.kP, kI: g.kI, kD: g.kD } : null;
  }

  function pidConfigOf(log, controller, gains) {
    const ch = log.get(controller.pid);
    const c = ch && ch.config ? ch.config : null;
    return M.pidConfig({
      gains,
      integralLimit: c ? c.integral_limit : 0,
      outputLimit: c ? c.output_limit : 12,
      slewRate: c ? c.slew_rate : 0,
      derivativeOnMeasurement: c ? c.derivative_on_measurement === 1 : false,
      nominalDtS: c ? c.nominal_dt_s : controller.loopS || 0.01,
    });
  }

  // --- What-if replays -----------------------------------------------------------------

  /**
   * Replays a mechanism's recorded targets over [t0, t1] through the fitted
   * plant under `gains` and `gravityVolts` (constant feedforward, as
   * GravityFeedforward::constantVolts, or the fit's shape if `useFitShape`),
   * with the seat-and-rest law at the floor. Returns { t, x, u, target,
   * recorded, metrics }, `recorded` being the logged position at each step.
   */
  function replayMechanism(log, system, fit, { gains, gravityVolts, t0, t1, seat = null,
    maxVolts = 12 }) {
    const act = log.get(system.actChannel);
    const [i0, i1] = act.range(t0, t1);
    if (i1 - i0 < 2) return null;
    const controller = system.controllers[0];
    const pid = new M.PID(pidConfigOf(log, controller, gains));
    const period = controller.loopS || 0.02;
    // The mechanism's travel as the log saw it: its hard stops, near enough.
    let floor = Infinity;
    let low = Infinity;
    let high = -Infinity;
    for (let i = i0; i < i1; ++i) {
      floor = Math.min(floor, act.cols.target[i]);
      if (Number.isFinite(act.cols.pos[i])) {
        low = Math.min(low, act.cols.pos[i]);
        high = Math.max(high, act.cols.pos[i]);
      }
    }
    const span = Math.max(1, high - low);
    const config = {
      gravity: { constantVolts: gravityVolts, cosineVolts: 0, horizontalPosition: 0, armDegreesPerUnit: 1 },
      seat: seat || { enabled: true, floor: Math.max(0, floor), seatVolts: 0, restBand: 2 },
      maxVolts,
    };
    if (fit.model.gravity.kind === 'cosine') {
      const ff = M.gravityFeedforwardOf(M.mechanismModel(fit.model.motion, gravityVolts, fit.model.gravity));
      config.gravity = ff;
    }
    const start = act.t[i0];
    const targetAt = (t) => act.valueAt('target', start + t);
    const run = M.simulateClosedLoop({
      model: fit.model,
      delayS: fit.delayS,
      periodS: period,
      durationS: act.t[i1 - 1] - start,
      x0: act.cols.pos[i0],
      limits: [low, high + 0.25 * span],
      targetAt,
      controller: (target, x) => {
        const cmd = M.computePositionCommand(pid, config, target, x);
        return cmd.brake ? null : cmd.volts;
      },
    });
    const recorded = new Float64Array(run.t.length);
    for (let k = 0; k < run.t.length; ++k) recorded[k] = act.valueAt('pos', start + run.t[k]);
    return {
      t: Float64Array.from(run.t, (t) => t + start),
      x: run.x,
      u: run.u,
      target: run.target,
      recorded,
      metrics: trackingMetrics(run.t, run.x, run.target, run.u, controller.threshold),
    };
  }

  /**
   * How well a trajectory tracked its targets: RMS error, the fraction of time
   * within `band`, the fraction at full volts, and where it came to rest,
   * the error at the end of each steady target it had stopped moving at
   * (targets at the lowest level, where it rests on its floor, excluded).
   */
  function trackingMetrics(t, x, target, u, band) {
    let sq = 0;
    let n = 0;
    let within = 0;
    let saturated = 0;
    let lowest = Infinity;
    for (let i = 0; i < t.length; ++i) {
      if (!Number.isFinite(x[i])) continue;
      const e = target[i] - x[i];
      sq += e * e;
      n++;
      if (Math.abs(e) <= band) within++;
      if (Number.isFinite(u[i]) && Math.abs(u[i]) >= 11.9) saturated++;
      lowest = Math.min(lowest, target[i]);
    }
    const rests = [];
    let segmentStart = 0;
    for (let i = 1; i <= t.length; ++i) {
      if (i < t.length && target[i] === target[i - 1]) continue;
      const last = i - 1;
      const back = Math.max(segmentStart, last - 10);
      const stopped = Number.isFinite(x[last]) && Number.isFinite(x[back]) &&
        Math.abs(x[last] - x[back]) < 0.02 * Math.max(1, Math.abs(target[last] - lowest));
      if (t[last] - t[segmentStart] >= 0.4 && target[last] > lowest && stopped) {
        rests.push(target[last] - x[last]);
      }
      segmentStart = i;
    }
    return {
      rmsError: n ? Math.sqrt(sq / n) : NaN,
      withinFraction: n ? within / n : 0,
      saturatedFraction: n ? saturated / n : 0,
      meanHoldError: rests.length ? rests.reduce((a, b) => a + b, 0) / rests.length : NaN,
      holds: rests.length,
    };
  }

  /**
   * Replays each recorded step response of a drivetrain controller that
   * starts within [t0, t1] through the fitted axis: from the response's first
   * error to zero, error folded into the target as the robot's loops do. A
   * response runs to its natural end (the next reset), whatever t1 is.
   * Continuous loops (a "response" longer than `maxResponseS`, like heading
   * hold under a driver) have no step to replay and are skipped. Returns
   * [{ start, duration, e0, recorded: { t, err }, sim: { t, err, u }, metrics }].
   */
  function replaySteps(log, system, fit, controller, { gains, t0, t1, maxResponses = 12,
    maxResponseS = 10 }) {
    const ch = log.get(controller.pid);
    if (!ch) return [];
    const flags = ch.cols.flags;
    const out = [];
    let i = ch.range(t0, t1)[0];
    // Back up to the start of the response containing t0, if t0 is mid-response.
    while (i > 0 && !(flags[i] & slt.FIRST_STEP)) i--;
    while (i < ch.length && out.length < maxResponses) {
      const first = i;
      let last = i;
      while (last + 1 < ch.length && !(flags[last + 1] & slt.FIRST_STEP)) last++;
      i = last + 1;
      const start = ch.t[first];
      if (start > t1) break;
      if (start < t0 - 1e-9) continue;
      const length = ch.t[last] - start;
      if (length > maxResponseS) continue;
      const duration = Math.min(4, length + 0.4);
      const e0 = ch.cols.err[first];
      if (Math.abs(e0) < 2 * controller.threshold || duration < 0.2) continue;
      const pid = new M.PID(pidConfigOf(log, controller, gains));
      const sim = M.simulateClosedLoop({
        model: fit.model, delayS: fit.delayS, periodS: controller.loopS || 0.01, durationS: duration,
        x0: 0, targetAt: () => e0, controller: (target, x) => pid.update(target - x, 0),
      });
      const [a, b] = ch.range(start, start + duration);
      out.push({
        start,
        duration,
        e0,
        recorded: { t: Float64Array.from(ch.t.subarray(a, b), (t) => t - start),
          err: ch.cols.err.slice(a, b) },
        sim: { t: sim.t, err: Float64Array.from(sim.x, (x) => e0 - x), u: sim.u },
        metrics: M.stepMetrics(sim.t, sim.x, 0, e0, controller.threshold, sim.u),
      });
    }
    return out;
  }

  // --- C++ to paste -------------------------------------------------------------------

  function num(value, digits = 4) {
    if (!Number.isFinite(value)) return '0.0';
    if (value === 0) return '0.0';
    const abs = Math.abs(value);
    const text = abs < 0.001 ? value.toExponential(3) : value.toPrecision(digits);
    return text.includes('.') || text.includes('e') ? text : `${text}.0`;
  }

  function gainsLiteral(g) {
    return `{.kP = ${num(g.kP)}, .kI = ${num(g.kI)}, .kD = ${num(g.kD)}}`;
  }

  function modelComment(fit) {
    const m = fit.model;
    const g = m.gravity.kind !== 'none' ? ` kG ${num(m.kG, 3)} (${m.gravity.kind})` : '';
    return `kS ${num(m.motion.kS, 3)} kV ${num(m.motion.kV, 3)} kA ${num(m.motion.kA, 3)}${g}, ` +
      `delay ${Math.round(fit.delayS * 1000)} ms, R² ${fit.rSquared.toFixed(3)}`;
  }

  /** C++ for a mechanism: its gains and gravity volts, as macros.cpp declares them. */
  function mechanismSnippet(system, fit, gains, gravityVolts, sourceName) {
    const name = system.label;
    const lines = [
      `// ${name}: designed by the telemetry analyzer from ${sourceName} (${fit.source})`,
      `//   model ${modelComment(fit)}`,
    ];
    if (fit.model.gravity.kind === 'cosine') {
      lines.push(`//   gravity: cosine, horizontal at ${num(fit.model.gravity.horizontalPosition, 4)}`);
      lines.push(`constexpr sapphirelib::PIDGains k${name}Gains${gainsLiteral(gains)};`);
      lines.push(`// PositionConfig: .gravity = {.cosineVolts = ${num(gravityVolts, 3)}, ` +
        `.horizontalPosition = ${num(fit.model.gravity.horizontalPosition, 4)}, ` +
        `.armDegreesPerUnit = ${num(fit.model.gravity.armDegreesPerUnit, 4)}}`);
    } else {
      lines.push(`constexpr sapphirelib::PIDGains k${name}Gains${gainsLiteral(gains)};`);
      lines.push(`constexpr double k${name}GravityVolts = ${num(gravityVolts, 3)};`);
    }
    return lines.join('\n');
  }

  /** C++ for the drivetrain: each controller's config and the axis models. */
  function driveSnippet(fits, designs, sourceName) {
    const lines = [`// Drivetrain: designed by the telemetry analyzer from ${sourceName}`];
    for (const [name, d] of Object.entries(designs)) {
      if (!d || !d.ok) continue;
      const which = name === 'Drive' ? 'drivePIDConfig' : name === 'Turn' ? 'turnPIDConfig'
        : 'headingHoldPID() (setGains)';
      lines.push(`// ${name} → ${which}: settles in ${d.settleTimeS.toFixed(2)}s, phase margin ` +
        `${d.phaseMarginDeg.toFixed(0)}°${d.limitedByDelay ? ' (slowed for delay)' : ''}, ` +
        `friction band ±${d.staticErrorBound.toPrecision(2)}`);
      if (name === 'Hold') {
        lines.push(`drivetrain().headingHoldPID().setGains(${gainsLiteral(d.gains)});`);
      } else {
        lines.push(`PID::Config{.gains = ${gainsLiteral(d.gains)}, .outputLimit = 12.0},`);
      }
    }
    const models = [];
    for (const [axis, key] of [['Fwd', 'forward'], ['Strafe', 'strafe'], ['Turn', 'turn']]) {
      const fit = fits[axis];
      if (!fit || !fit.ok) continue;
      const m = fit.model.motion;
      models.push(`    .${key} = {.kS = ${num(m.kS)}, .kV = ${num(m.kV)}, .kA = ${num(m.kA)}},` +
        ` // ${modelComment(fit)}`);
    }
    if (models.length) {
      lines.push('// Axis models, for DriverInputMode::velocity (initDevices()):');
      lines.push('drivetrain().setAxisModels({');
      lines.push(...models);
      lines.push('});');
    }
    return lines.join('\n');
  }

  return {
    DRIVE_CONTROLLERS,
    systems,
    toRuns,
    mechanismSamples,
    driveSamples,
    fitAutotune,
    fitPassive,
    design,
    inferredGravityVolts,
    gainsAt,
    pidConfigOf,
    replayMechanism,
    replaySteps,
    trackingMetrics,
    mechanismSnippet,
    driveSnippet,
  };
});
