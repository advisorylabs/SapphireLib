/*
 * SapphireLib telemetry analyzer: refine.js
 *
 * Fine-tunes the drivetrain's PIDs from what they really did. Auto-Tune
 * designs gains from a model it measures in a few seconds of open-loop
 * ramps and steps; real motions, with the robot loaded, carpet worn and the
 * battery sagging, don't always match it. Every motion the robot logs is a
 * step response with known gains, so:
 *
 *   1. collect every step response of a controller across the logs: its
 *      gains, its first error, and the error over time, as recorded;
 *   2. replay each one through the axis model with the gains it ran with,
 *      and find the model (inertia and damping scaled, delay changed) whose
 *      replays match what the robot really did: closed-loop identification;
 *   3. design the controller again from that model, exactly as the robot's
 *      PidTunerPage designs it, and replay the same steps with the new gains
 *      to predict what changes;
 *   4. say why: what the responses did against what the design promised,
 *      and what about the model was off.
 *
 * The analyzer proposes; nothing reaches the robot until someone accepts it
 * and saves a TUNE.CFG (tunefile.js). A change is capped at half or double
 * each gain per round, so a bad batch of logs can only move things so far,
 * and needs a few runs' worth of steps before it says anything at all.
 *
 * history() is the other half: every loaded run, in order, with the gains
 * each controller ran, how its responses went, and what changed them
 * (TUNE.CFG's revision, Auto-Tune on the robot, the PID page by hand, or a
 * rebuild), so the effect of each round shows up in the next run's numbers.
 *
 * Plain script: window.SA.refine in a browser, require('./refine.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./slt.js'), require('./model.js'), require('./tuning.js'),
      require('./tunefile.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.refine = factory(root.SA.slt, root.SA.model, root.SA.tuning, root.SA.tunefile);
  }
})(function (slt, M, T, TF) {
  'use strict';

  /**
   * The controllers refined from step responses, with the robot's own specs
   * (src/robot/tuning.cpp). `signed` is whether the logged error keeps its
   * sign: turns do; the point motions' distance to the target doesn't.
   */
  const CONTROLLERS = [
    { name: 'Turn', pid: 'turn', axis: 'Turn', modelKey: 'turn', spec: M.responseSpec(0.5, 1, 50), threshold: 2,
      loopS: 0.01, signed: true, unit: '°' },
    { name: 'Drive', pid: 'drive', axis: 'Fwd', modelKey: 'fwd', spec: M.responseSpec(0.6, 1, 50), threshold: 1,
      loopS: 0.01, signed: false, unit: 'in' },
  ];

  const kMaxResponseS = 6;
  const kMinResponseS = 0.25;
  const kReplayS = 3;
  const kMaxResponses = 16;
  const kMinResponses = 3;
  const kMaxGainStep = 2; // per round, each gain moves at most this factor

  // --- Step responses ----------------------------------------------------------------

  /**
   * A controller's step responses in one log: [{ start, e0, gains, config,
   * t, err, metrics, file }], t from the response's start and err as logged
   * (absolute for unsigned controllers). Continuous loops, tiny steps, and
   * responses that never got halfway (blocked, not dynamics) are left out.
   */
  function responsesOf(log, controller) {
    const ch = log.get(controller.pid);
    if (!ch || ch.kind !== 'pid' || !ch.length) return [];
    const flags = ch.cols.flags;
    const out = [];
    let i = 0;
    while (i < ch.length) {
      while (i < ch.length && !(flags[i] & slt.FIRST_STEP)) i++;
      if (i >= ch.length) break;
      const first = i;
      let last = i;
      while (last + 1 < ch.length && !(flags[last + 1] & slt.FIRST_STEP)) last++;
      i = last + 1;
      // a reset (R row) inside ends the response there
      const start = ch.t[first];
      const reset = ch.resets.find((r) => r > start + 1e-6 && r <= ch.t[last]);
      if (reset !== undefined) while (last > first && ch.t[last] >= reset) last--;
      const length = ch.t[last] - start;
      if (length > kMaxResponseS || length < kMinResponseS) continue;
      const e0Raw = ch.cols.err[first];
      const e0 = controller.signed ? e0Raw : Math.abs(e0Raw);
      if (!(Math.abs(e0) >= 3 * controller.threshold)) continue;
      const n = Math.min(last - first + 1, Math.round(kReplayS / 0.01) + 1);
      const t = new Float64Array(n);
      const err = new Float64Array(n);
      for (let k = 0; k < n; ++k) {
        t[k] = ch.t[first + k] - start;
        err[k] = controller.signed ? ch.cols.err[first + k] : Math.abs(ch.cols.err[first + k]);
      }
      let closest = Infinity;
      for (const e of err) closest = Math.min(closest, Math.abs(e));
      if (closest > 0.5 * Math.abs(e0)) continue;
      const g = ch.gainsAt(start) || ch.gains[0];
      if (!g) continue;
      out.push({
        start,
        e0,
        gains: { kP: g.kP, kI: g.kI, kD: g.kD },
        config: ch.config,
        t,
        err,
        metrics: measuredMetrics(t, err, e0, controller),
        file: log.meta.file || log.fileName || '',
      });
    }
    return out;
  }

  /**
   * What a recorded response did: overshoot as a share of the step (for an
   * unsigned error, how far it came back up after first getting close), how
   * long until it last left the exit band, and where it ended.
   */
  function measuredMetrics(t, err, e0, controller) {
    const band = controller.threshold;
    let overshoot = 0;
    if (controller.signed) {
      const dir = Math.sign(e0);
      for (const e of err) overshoot = Math.max(overshoot, -e * dir);
    } else {
      let reached = false;
      let low = Infinity;
      for (const e of err) {
        if (!reached && e < 0.25 * Math.abs(e0)) reached = true;
        if (reached) {
          low = Math.min(low, e);
          if (e - low > band) overshoot = Math.max(overshoot, e);
        }
      }
    }
    let lastOutside = -1;
    for (let k = 0; k < err.length; ++k) if (Math.abs(err[k]) > band) lastOutside = k;
    const settled = lastOutside < err.length - 1;
    return {
      overshootFraction: overshoot / Math.abs(e0),
      settleS: lastOutside < 0 ? 0 : t[Math.min(lastOutside + 1, t.length - 1)],
      settled,
      finalError: Math.abs(err[err.length - 1]),
    };
  }

  // --- Replaying a response through a model -------------------------------------------------

  function pidFor(response, controller) {
    const c = response.config;
    return new M.PID(M.pidConfig({
      gains: response.gains,
      integralLimit: c ? c.integral_limit : 0,
      outputLimit: c ? c.output_limit : 12,
      slewRate: c ? c.slew_rate : 0,
      derivativeOnMeasurement: c ? c.derivative_on_measurement === 1 : false,
      nominalDtS: c ? c.nominal_dt_s : controller.loopS,
    }));
  }

  /** The error over time of `response`'s step through `model` with `gains` (its own by default). */
  function replay(response, model, delayS, controller, gains = response.gains, durationS = null) {
    const pid = pidFor(Object.assign({}, response, { gains }), controller);
    const period = pid.config.nominalDtS || controller.loopS;
    const span = durationS ?? (response.t.length ? response.t[response.t.length - 1] + period : kReplayS);
    const run = M.simulateClosedLoop({
      model: M.mechanismModel(model),
      delayS,
      periodS: period,
      durationS: span,
      x0: 0,
      targetAt: () => response.e0,
      controller: (target, x) => pid.update(target - x, 0),
    });
    const err = Float64Array.from(run.x, (x) => (controller.signed ? response.e0 - x : Math.abs(response.e0 - x)));
    return { t: run.t, err, u: run.u };
  }

  /** Mean squared mismatch between a replay and the recording, as a share of the step. */
  function mismatch(responses, model, delayS, controller) {
    let sum = 0;
    let count = 0;
    for (const r of responses) {
      const sim = replay(r, model, delayS, controller);
      const scale = r.e0 * r.e0;
      for (let k = 0; k < r.t.length; ++k) {
        // the replay steps at the loop period; the log may have jitter, so take the nearest step
        const j = Math.min(sim.t.length - 1, Math.round(r.t[k] / (sim.t[1] - sim.t[0] || controller.loopS)));
        const d = r.err[k] - sim.err[j];
        sum += (d * d) / scale;
        count++;
      }
    }
    return count ? sum / count : Infinity;
  }

  /**
   * The model that best explains the recorded responses: the base model's
   * inertia (kA) and damping (kV) scaled, and its delay changed, by a coarse
   * grid then a finer one around the best. kS stays: friction is what the
   * open-loop measurement gets right.
   */
  function identify(responses, base, controller) {
    const scaled = (a, v) => M.feedforward(base.model.kS, base.model.kV * v, base.model.kA * a);
    const evaluate = (a, v, d) => mismatch(responses, scaled(a, v), d, controller);
    let best = { a: 1, v: 1, d: base.delayS, cost: evaluate(1, 1, base.delayS) };
    const baseCost = best.cost;
    const consider = (a, v, d) => {
      const cost = evaluate(a, v, d);
      if (cost < best.cost) best = { a, v, d, cost };
    };
    for (const a of [0.5, 0.7, 1, 1.4, 2, 2.8]) {
      for (const v of [0.7, 1, 1.4]) {
        for (const d of [0, 0.025, 0.05, 0.075, 0.1, 0.15]) consider(a, v, d);
      }
    }
    for (let round = 0; round < 2; ++round) {
      const { a: a0, v: v0, d: d0 } = best;
      const step = round === 0 ? [0.84, 1, 1.19] : [0.92, 1, 1.09];
      const dStep = round === 0 ? 0.0125 : 0.005;
      for (const fa of step) {
        for (const fv of step) {
          for (const dd of [-dStep, 0, dStep]) consider(a0 * fa, v0 * fv, Math.max(0, d0 + dd));
        }
      }
    }
    return {
      model: scaled(best.a, best.v),
      delayS: best.d,
      kAScale: best.a,
      kVScale: best.v,
      // RMS mismatch as a share of the step: before (the base model) and after
      baseRms: Math.sqrt(baseCost),
      rms: Math.sqrt(best.cost),
    };
  }

  // --- The base model -----------------------------------------------------------------

  /**
   * What the robot designed its gains from, most authoritative first: the
   * newest Auto-Tune measurement in the logs (a `tune,model` event), the
   * loaded TUNE.CFG's model, or a fit to the logs' own driving.
   */
  function baseModel(logs, controller, profile = null) {
    let latest = null;
    for (const log of logs) {
      for (const m of slt.tuneEvents(log).models) {
        if (m.axis === controller.axis && m.kV > 0 && m.kA > 0 && (!latest || m.t >= latest.t || log !== latest.log)) {
          latest = { log, t: m.t, model: M.feedforward(m.kS, m.kV, m.kA), delayS: m.delayS || 0.03,
            source: `Auto-Tune on the robot (${log.meta.file || log.fileName})` };
        }
      }
    }
    if (latest) return latest;
    const fromFile = profile && profile.models[controller.modelKey];
    if (fromFile) {
      return { model: M.feedforward(fromFile.kS, fromFile.kV, fromFile.kA), delayS: 0.03,
        source: 'TUNE.CFG (delay assumed 30ms)' };
    }
    for (const log of logs.slice().reverse()) {
      const system = T.systems(log).find((s) => s.axisName === controller.axis);
      if (!system) continue;
      const fit = T.fitPassive(system, T.driveSamples(log, system, log.start, log.end));
      if (fit.ok) {
        return { model: fit.model.motion, delayS: fit.delayS, source: `a fit to the driving in ${log.meta.file || log.fileName}` };
      }
    }
    return null;
  }

  // --- Refining -----------------------------------------------------------------------

  function median(values) {
    if (!values.length) return NaN;
    const s = values.slice().sort((a, b) => a - b);
    const m = s.length >> 1;
    return s.length % 2 ? s[m] : 0.5 * (s[m - 1] + s[m]);
  }

  /** Each recorded e0 replayed through `model` with `gains`: median overshoot and settle time. */
  function predict(responses, model, delayS, controller, gains) {
    const overshoot = [];
    const settle = [];
    for (const r of responses) {
      const sim = replay(r, model, delayS, controller, gains, kReplayS);
      const m = measuredMetrics(sim.t, sim.err, r.e0, controller);
      overshoot.push(m.overshootFraction);
      settle.push(m.settled ? m.settleS : kReplayS);
    }
    return { overshootFraction: median(overshoot), settleS: median(settle) };
  }

  function capGains(current, designed) {
    const capped = { kP: designed.kP, kI: designed.kI, kD: designed.kD };
    let wasCapped = false;
    for (const k of ['kP', 'kD']) {
      if (!(current[k] > 0)) continue;
      const lo = current[k] / kMaxGainStep;
      const hi = current[k] * kMaxGainStep;
      if (capped[k] < lo || capped[k] > hi) {
        capped[k] = Math.min(Math.max(capped[k], lo), hi);
        wasCapped = true;
      }
    }
    return { gains: capped, capped: wasCapped };
  }

  function pct(fraction) {
    return `${Math.round(100 * fraction)}%`;
  }

  function sec(s) {
    return `${s.toFixed(2)}s`;
  }

  /**
   * Refines one controller from every log: { ok, why, controller, base,
   * responses, measured, identified, current, designed, proposed, capped,
   * predicted: { now, proposed }, change: 'retune' | 'none', reasons }.
   * `responses` can be given to restrict it (say, to the runs since the last
   * change); otherwise it's the most recent kMaxResponses across the logs.
   */
  function refine(logs, controller, { profile = null, responses = null } = {}) {
    const all = responses || logs.flatMap((log) => responsesOf(log, controller));
    const recent = all.slice(-kMaxResponses);
    const result = { ok: false, controller, responses: recent, reasons: [] };
    if (recent.length < kMinResponses) {
      result.why = `${recent.length} usable ${controller.name.toLowerCase()} step${recent.length === 1 ? '' : 's'} ` +
        `in these logs; it needs at least ${kMinResponses} (motions of ${3 * controller.threshold}${controller.unit} or more).`;
      return result;
    }
    const base = baseModel(logs, controller, profile);
    if (!base) {
      result.why = 'No model to start from: run Auto-Tune on the robot with the SD card in, or open a log of ordinary driving.';
      return result;
    }
    result.base = base;
    const current = recent[recent.length - 1].gains;
    result.current = current;
    result.measured = {
      overshootFraction: median(recent.map((r) => r.metrics.overshootFraction)),
      settleS: median(recent.map((r) => (r.metrics.settled ? r.metrics.settleS : r.t[r.t.length - 1]))),
      unsettled: recent.filter((r) => !r.metrics.settled).length,
    };
    const identified = identify(recent, base, controller);
    result.identified = identified;
    const designed = M.designPositionGains(identified.model, controller.spec,
      identified.delayS + 0.5 * Math.max(0, controller.loopS - 0.01));
    result.designed = designed;
    if (!designed.ok) {
      result.why = 'The refined model can\'t be designed for (it came out invalid): the steps may not be plant dynamics.';
      return result;
    }
    const { gains: proposed, capped } = capGains(current, designed.gains);
    result.proposed = proposed;
    result.capped = capped;
    result.predicted = {
      // what the robot's own model said these steps would do, with the gains they ran
      promised: predict(recent, base.model, base.delayS, controller, current),
      now: predict(recent, identified.model, identified.delayS, controller, current),
      proposed: predict(recent, identified.model, identified.delayS, controller, proposed),
    };
    result.ok = true;

    // Worth changing only if the model was off (the refined one explains the steps clearly
    // better), the new gains would do clearly better on the same steps without making them
    // much slower, and they're actually different. Judged on the recorded steps, not the spec:
    // a 180° turn at 12V can't settle in 0.5s whatever the gains
    const promised = result.predicted.promised;
    const now = result.predicted.now;
    const next = result.predicted.proposed;
    const modelOff = identified.baseRms > 0.03 && identified.rms < 0.7 * identified.baseRms;
    const overshootGain = now.overshootFraction - next.overshootFraction;
    const better = (overshootGain > 0.03 || next.settleS < 0.9 * now.settleS) &&
      (next.settleS <= 1.25 * now.settleS || overshootGain > 0.08);
    const gainsMove = Math.abs(proposed.kP / current.kP - 1) > 0.05 ||
      Math.abs((proposed.kD + 1e-9) / (current.kD + 1e-9) - 1) > 0.1;
    result.change = modelOff && better && gainsMove ? 'retune' : 'none';
    // what can be saved: new gains and their model, or (gains fine, model off) the model alone
    result.modelOff = modelOff;
    result.unexplained = !modelOff && identified.baseRms > 0.08;
    result.saves = result.change === 'retune' ? 'gains' : modelOff ? 'model' : null;

    // why, in words
    const m = result.measured;
    const r = result.reasons;
    r.push(`${recent.length} recorded steps overshot ${pct(m.overshootFraction)} and settled in ${sec(m.settleS)} ` +
      `(median)${m.unsettled ? `, ${m.unsettled} never settling` : ''}. The model their gains were designed from ` +
      `said ${pct(promised.overshootFraction)} and ${sec(promised.settleS)}.`);
    const parts = [];
    if (Math.abs(identified.delayS - base.delayS) >= 0.01) {
      parts.push(`${Math.round(identified.delayS * 1000)}ms of delay, not ${Math.round(base.delayS * 1000)}ms`);
    }
    if (Math.abs(identified.kAScale - 1) >= 0.12) parts.push(`${identified.kAScale.toFixed(2)}× the inertia (kA)`);
    if (Math.abs(identified.kVScale - 1) >= 0.12) parts.push(`${identified.kVScale.toFixed(2)}× the damping (kV)`);
    const unexplained = result.unexplained;
    if (modelOff && parts.length) {
      r.push(`Replayed through the model with the gains each ran, they match best with ${parts.join(' and ')} ` +
        `than ${base.source} measured: mismatch ${pct(identified.baseRms)} → ${pct(identified.rms)} of the step.`);
    } else if (unexplained) {
      r.push(`Neither the model from ${base.source} nor any rescaling of it explains them (mismatch ` +
        `${pct(identified.rms)} of the step): something besides its constants is at work, like a blocked turn, ` +
        'contact with another robot, or a different load.');
    } else {
      r.push(`The model from ${base.source} already explains them (mismatch ${pct(identified.baseRms)} of the step).`);
    }
    if (result.change === 'retune') {
      r.push(`Designed again for that: kP ${TF.num(current.kP)} → ${TF.num(proposed.kP)}, kD ${TF.num(current.kD)} → ` +
        `${TF.num(proposed.kD)}${capped ? ' (capped at 2× per round; the next round can go further)' : ''}` +
        `${designed.limitedByDelay ? ', slowed to keep its phase margin with the delay' : ''}. ` +
        `On these same steps: overshoot ${pct(now.overshootFraction)} → ${pct(next.overshootFraction)}, ` +
        `settle ${sec(now.settleS)} → ${sec(next.settleS)}.`);
    } else if (unexplained) {
      r.push('No change: new gains can\'t fix what the model doesn\'t describe. Look at these steps in PID responses.');
    } else if (!modelOff) {
      r.push('Its gains are doing what they were designed to: no change.');
    } else {
      r.push(`A design from the refined model wouldn't do clearly better on these steps (overshoot ` +
        `${pct(now.overshootFraction)} → ${pct(next.overshootFraction)}, settle ${sec(now.settleS)} → ${sec(next.settleS)}): ` +
        'no change. Big turns at full power are limited by the motors, not the gains; the model is still worth keeping.');
    }
    return result;
  }

  /**
   * TUNE.CFG changes for accepted refinements: a retune's gains and the
   * refined model they were designed from; for gains that are fine on a model
   * that was off, the model alone (velocity driving uses it). Nothing for a
   * refinement that found nothing to save.
   */
  function tuneChanges(results) {
    const changes = [];
    for (const r of results) {
      if (!r.ok || !r.saves) continue;
      if (r.saves === 'gains') changes.push({ key: `pid.${r.controller.pid}`, value: r.proposed });
      const m = r.identified.model;
      changes.push({ key: `model.${r.controller.modelKey}`, value: { kS: m.kS, kV: m.kV, kA: m.kA } });
    }
    return changes;
  }

  // --- History -------------------------------------------------------------------------

  /**
   * Every run, oldest first, with what each controller ran and how it went:
   * [{ run, file, tune: { status, rev }, controllers: [{ name, pid, gains:
   * [{ t, kP, kI, kD, source }], responses, overshoot, settleS, timeouts }],
   * mcl: { correctionIn, correctingShare, updateUs }, cpu: { loggerPct },
   * changes: [{ controller, from, to, source }] }]. `source` is what changed
   * the gains: TUNE.CFG's revision, Auto-Tune on the robot, the PID page, or
   * a rebuild.
   */
  function history(runs) {
    const ordered = runs.slice().sort((a, b) => (slt.fileIndex(a.meta.file || a.fileName) ?? 0) -
      (slt.fileIndex(b.meta.file || b.fileName) ?? 0));
    const rows = [];
    let previous = null;
    for (const run of ordered) {
      const tuneEvents = slt.tuneEvents(run).models;
      const motions = slt.motions(run);
      const row = {
        run,
        file: run.files.join(' + '),
        tune: { status: run.meta.tune || null, rev: run.meta['tune.rev'] ? Number(run.meta['tune.rev']) : null },
        sessions: slt.sessions(run).map((s) => s.label),
        autoTuned: tuneEvents.length > 0,
        controllers: [],
        changes: [],
      };
      for (const c of [...CONTROLLERS, { name: 'Hold', pid: 'hold' }, { name: 'Lift', pid: 'lift' }]) {
        const ch = run.get(c.pid);
        if (!ch || !ch.gains.length) continue;
        const sets = [];
        for (const g of ch.gains) {
          const last = sets[sets.length - 1];
          if (last && last.kP === g.kP && last.kI === g.kI && last.kD === g.kD) continue;
          let source = null;
          if (last) {
            const tuned = tuneEvents.some((e) => e.t <= g.t && g.t - e.t < 10);
            source = tuned ? 'Auto-Tune on the robot' : 'PID page, by hand';
          }
          sets.push({ t: g.t, kP: g.kP, kI: g.kI, kD: g.kD, source });
        }
        const controller = CONTROLLERS.find((x) => x.pid === c.pid);
        const responses = controller ? responsesOf(run, controller) : [];
        const kinds = { turn: ['turnToHeading'], drive: ['driveDistance', 'moveToPoint', 'moveToPose'] }[c.pid] || [];
        row.controllers.push({
          name: c.name,
          pid: c.pid,
          gains: sets,
          responses: responses.length,
          overshoot: median(responses.map((r) => r.metrics.overshootFraction)),
          settleS: median(responses.map((r) => (r.metrics.settled ? r.metrics.settleS : r.t[r.t.length - 1]))),
          timeouts: motions.filter((mo) => kinds.includes(mo.kind) && mo.reason === 'timeout').length,
        });
        // between runs: the first gains here against the last ones there
        const before = previous && previous.controllers.find((x) => x.pid === c.pid);
        if (before && before.gains.length) {
          const from = before.gains[before.gains.length - 1];
          const to = sets[0];
          if (from.kP !== to.kP || from.kI !== to.kI || from.kD !== to.kD) {
            const revNow = row.tune.rev;
            const revThen = previous.tune.rev;
            let source;
            if (revNow !== null && revNow !== revThen) {
              source = `TUNE.CFG r${revNow}`;
            } else if (previous.autoTuned) {
              // Auto-Tune set gains on the robot last run, and a restart lost them
              source = 'a restart: last run\'s Auto-Tune gains weren\'t kept (save them in TUNE.CFG)';
            } else {
              source = 'a rebuild (the code\'s gains changed)';
            }
            row.changes.push({ controller: c.name, from, to, source });
          }
        }
        for (const g of sets.slice(1)) row.changes.push({ controller: c.name, from: null, to: g, source: g.source });
      }
      const mcl = run.get('mcl');
      if (mcl && mcl.length) {
        let corr = 0;
        let correcting = 0;
        let us = 0;
        let usCount = 0;
        for (let i = 0; i < mcl.length; ++i) {
          corr += Math.hypot(mcl.cols.corr_x[i], mcl.cols.corr_y[i]);
          if (mcl.cols.correcting[i] > 0) correcting++;
          if (mcl.has('us') && mcl.cols.us[i] > 0) {
            us += mcl.cols.us[i];
            usCount++;
          }
        }
        row.mcl = { correctionIn: corr / mcl.length, correctingShare: correcting / mcl.length,
          updateUs: usCount ? us / usCount : NaN };
      }
      const samp = run.health.map((h) => h.values.samp_us).filter((v) => typeof v === 'number');
      const fmt = run.health.map((h) => h.values.fmt_us).filter((v) => typeof v === 'number');
      if (samp.length) {
        row.cpu = { loggerPct: (samp.reduce((a, b) => a + b, 0) / samp.length + fmt.reduce((a, b) => a + b, 0) /
          Math.max(1, fmt.length)) / 1e4 };
      }
      rows.push(row);
      previous = row;
    }
    return rows;
  }

  return { CONTROLLERS, responsesOf, measuredMetrics, replay, mismatch, identify, baseModel, predict, refine,
    tuneChanges, history };
});
