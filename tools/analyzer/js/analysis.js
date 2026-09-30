/*
 * SapphireLib telemetry analyzer: analysis.js
 *
 * Turns a parsed log into findings: what went wrong, when, on which system,
 * and what probably caused it: the "why did the robot fade in the last 30
 * seconds" answer a team would otherwise dig out of charts by hand. Every
 * finding carries a time to jump the replay to and the channels worth
 * charting next to it.
 *
 * Thresholds are the V5's published numbers where there are any (motor
 * derating starts at 55°C) and conservative rules of thumb elsewhere; they're
 * all in THRESHOLDS so a team can adjust them to their robot.
 *
 * Plain script: window.SA.analysis in a browser, require('./analysis.js') in
 * Node. Depends on slt.js and model.js.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./slt.js'), require('./model.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.analysis = factory(root.SA.slt, root.SA.model);
  }
})(function (slt, model) {
  'use strict';

  const THRESHOLDS = {
    motorWarmC: 50, // approaching derating
    motorDerateC: 55, // the V5 halves its power here
    stallAmps: 1.8,
    stallRpm: 5,
    stallMinS: 0.5,
    stallLongS: 3,
    stallShareOfEnabled: 0.15,
    disconnectMinS: 0.15,
    speedPerVoltDrop: 0.25, // late vs early, as a fraction
    batteryWarnV: 11.0,
    batteryCriticalV: 10.0,
    batteryStartPct: 60,
    batteryHotC: 50,
    controllerDropMinS: 0.1,
    oscillationSwings: 6,
    responseMaxS: 10, // longer "responses" are continuously running loops
    loopGapS: 0.1,
    trackingBand: 20, // mechanism units (degrees for a lift on a rotation sensor)
    trackingMinS: 1.5,
    stuckWindowS: 1.0, // "stuck" = moved less than trackingBand in this long
    sagMin: 3,
    odomJumpIn: 6,
    slowWriteUs: 250000,
  };

  const SEVERITY_RANK = { critical: 0, warning: 1, info: 2 };

  /** Which pid channels each motion kind drives (docs/TELEMETRY_FORMAT.md). */
  const MOTION_PIDS = {
    driveDistance: ['drive'],
    turnToHeading: ['turn'],
    moveToPoint: ['drive', 'turn'],
    moveToPose: ['drive', 'turn'],
    followPath: ['drive', 'turn'],
  };

  /**
   * What a V5 motor still delivers at a *reported* temperature. Motors report
   * in 5°C steps, so this is the published step (50% at 55°C, 25% at 60°C,
   * 12.5% at 65°C, off at 70°C) rather than model.thermalPowerFraction()'s
   * smoothed curve, which is for live control.
   */
  function reportedDerating(tempC) {
    if (tempC >= 70) return 0;
    if (tempC >= 65) return 0.125;
    if (tempC >= 60) return 0.25;
    if (tempC >= 55) return 0.5;
    return 1;
  }

  // --- Time -------------------------------------------------------------------------

  function clock(seconds) {
    const s = Math.max(0, seconds);
    const minutes = Math.floor(s / 60);
    const rest = s - minutes * 60;
    return `${minutes}:${rest < 10 ? '0' : ''}${rest.toFixed(1)}`;
  }

  /**
   * A time the way someone who watched the match would say it: "Match 1,
   * driver 1:20.4", "Practice 2, 0:12.0", or program time.
   */
  function timeLabel(sessions, t) {
    for (const s of sessions) {
      if (t < s.start - 0.001 || t > s.end + 0.001) continue;
      if (s.auton && t >= s.auton[0] - 0.001 && t <= s.auton[1] + 0.001) {
        return `${s.label}, auton ${clock(t - s.auton[0])}`;
      }
      if (s.driver && t >= s.driver[0] - 0.001 && t <= s.driver[1] + 0.001) {
        return `${s.label}, driver ${clock(t - s.driver[0])}`;
      }
      return `${s.label}, ${clock(t - s.start)}`;
    }
    return `t=${t.toFixed(1)}s`;
  }

  // --- Helpers ----------------------------------------------------------------------

  function motorChannels(log) {
    return log.channels.filter((c) => c.kind === 'samples' && c.has('temp') && c.has('amps') &&
      c.has('volts') && c.has('rpm'));
  }

  function mechanismChannels(log) {
    return log.channels.filter((c) => c.kind === 'samples' && c.has('target') && c.has('pos') &&
      c.has('volts') && c.has('law'));
  }

  function shortName(name) {
    return name.replace(/^motor\./, '');
  }

  /** Enabled (autonomous/opcontrol) stretches, or the whole log if none are logged. */
  function enabledPeriods(log) {
    const list = slt.periods(log).filter((p) => p.mode !== 'disabled');
    if (slt.phases(log).length === 0) return [{ mode: 'unknown', comp: false, start: log.start, end: log.end }];
    return list;
  }

  function isEnabled(periods, t) {
    return periods.some((p) => t >= p.start && t <= p.end);
  }

  function median(values) {
    if (values.length === 0) return NaN;
    const sorted = values.slice().sort((a, b) => a - b);
    const mid = sorted.length >> 1;
    return sorted.length % 2 ? sorted[mid] : 0.5 * (sorted[mid - 1] + sorted[mid]);
  }

  /**
   * Runs of consecutive rows (within [i0, i1)) where `test(i)` holds, as
   * { start, end, first, last } in seconds/indices; a run's end is the time
   * of the first row after it (or its last row's time).
   */
  function episodes(channel, i0, i1, test) {
    const out = [];
    let open = null;
    for (let i = i0; i < i1; ++i) {
      if (test(i)) {
        if (!open) open = { first: i, start: channel.t[i] };
        open.last = i;
      } else if (open) {
        open.end = channel.t[i];
        out.push(open);
        open = null;
      }
    }
    if (open) {
      open.end = channel.t[open.last];
      out.push(open);
    }
    return out;
  }

  // --- The analysis ----------------------------------------------------------------

  /**
   * Analyzes `log` over [range[0], range[1]] (default: all of it). Returns
   * { findings, motors, battery, pids, mechanisms, motions, logger, sessions }.
   * Findings are sorted most severe first, then by time; each is { severity:
   * 'critical'|'warning'|'info', t, end, system, title, detail, channels }.
   */
  function analyze(log, options = {}) {
    const range = options.range || [log.start, log.end];
    const [t0, t1] = range;
    const sessions = slt.sessions(log);
    const periods = enabledPeriods(log);
    const findings = [];
    const when = (t) => timeLabel(sessions, t);
    const add = (f) => {
      if (f.t !== null && f.t !== undefined && (f.t < t0 - 0.001 || f.t > t1 + 0.001)) return;
      findings.push(Object.assign({ end: null, channels: [] }, f));
    };

    const report = {
      findings,
      sessions,
      motors: analyzeMotors(log, t0, t1, periods, add, when),
      battery: analyzeBattery(log, t0, t1, sessions, periods, add, when),
      pids: analyzePids(log, t0, t1, add, when),
      mechanisms: analyzeMechanisms(log, t0, t1, add, when),
      motions: null,
      logger: analyzeLogger(log, add, when),
    };
    analyzeDriver(log, t0, t1, periods, add, when);
    analyzeDevices(log, add, when);
    analyzeOdometry(log, t0, t1, add, when);
    report.motions = analyzeMotions(log, t0, t1, report, add, when);
    analyzeEnding(log, add, when);
    mergeDuplicates(findings);

    findings.sort((a, b) => SEVERITY_RANK[a.severity] - SEVERITY_RANK[b.severity] ||
      (a.t ?? Infinity) - (b.t ?? Infinity));
    findings.forEach((f, i) => {
      f.id = i;
    });
    return report;
  }

  /**
   * A motor that drops out shows up twice (its channel goes NaN, and the
   * Diagnostics page logs the port lost), so fold the device event into the
   * motor finding that starts at the same moment.
   */
  function mergeDuplicates(findings) {
    for (let i = findings.length - 1; i >= 0; --i) {
      const device = findings[i];
      if (device.kind !== 'device-lost') continue;
      const motor = findings.find((f) => f.kind === 'motor-disconnect' &&
        Math.abs(f.t - device.t) < 0.5);
      if (!motor) continue;
      motor.detail += ` The Diagnostics page saw it too: "${device.system}" on port ` +
        `${device.port}${device.found ? ` (found: ${device.found})` : ''}.`;
      motor.system = `${motor.system} (${device.system})`;
      findings.splice(i, 1);
    }
  }

  function analyzeMotors(log, t0, t1, periods, add, when) {
    const T = THRESHOLDS;
    const summaries = [];
    const enabledTotal = periods.reduce((sum, p) =>
      sum + Math.max(0, Math.min(p.end, t1) - Math.max(p.start, t0)), 0);

    for (const ch of motorChannels(log)) {
      const [i0, i1] = ch.range(t0, t1);
      const name = shortName(ch.name);
      const temp = ch.cols.temp;
      const amps = ch.cols.amps;
      const volts = ch.cols.volts;
      const rpm = ch.cols.rpm;
      const faults = ch.cols.faults;
      const summary = { name, channel: ch.name, peakTemp: NaN, peakAt: null, firstWarm: null,
        firstDerate: null, timeAboveDerate: 0, stallS: 0, meanAmps: NaN, disconnects: [],
        neverAnswered: false, rpmPerVoltEarly: NaN, rpmPerVoltLate: NaN, faultBits: 0 };
      summaries.push(summary);
      if (i1 <= i0) continue;

      const dead = (i) => Number.isNaN(volts[i]) && Number.isNaN(temp[i]) && Number.isNaN(amps[i]);
      let answered = 0;
      let ampSum = 0;
      let ampCount = 0;
      for (let i = i0; i < i1; ++i) {
        if (dead(i)) continue;
        answered++;
        if (Number.isFinite(temp[i]) && !(temp[i] <= summary.peakTemp)) {
          summary.peakTemp = temp[i];
          summary.peakAt = ch.t[i];
        }
        if (Number.isFinite(temp[i]) && temp[i] >= T.motorWarmC && summary.firstWarm === null) {
          summary.firstWarm = ch.t[i];
        }
        if (Number.isFinite(temp[i]) && temp[i] >= T.motorDerateC) {
          if (summary.firstDerate === null) summary.firstDerate = ch.t[i];
          const next = i + 1 < ch.length ? ch.t[i + 1] : ch.t[i];
          summary.timeAboveDerate += Math.max(0, Math.min(next, t1) - ch.t[i]);
        }
        if (Number.isFinite(amps[i])) {
          ampSum += Math.abs(amps[i]);
          ampCount++;
        }
        if (faults && Number.isFinite(faults[i])) summary.faultBits |= faults[i];
      }
      summary.meanAmps = ampCount ? ampSum / ampCount : NaN;

      if (answered === 0) {
        summary.neverAnswered = true;
        add({ severity: 'info', t: null, system: name, title: `${name}: never answered`,
          detail: `No motor answered on ${ch.name}'s port in this range: unplugged, or the port ` +
            'in config.hpp is a placeholder. If it should be there, check its cable.',
          channels: [ch.name] });
        continue;
      }

      // Disconnects: the logger writes whole NaN rows for a motor that stops
      // answering. Only ones that start after it had been answering count.
      let seenAlive = false;
      for (const ep of episodes(ch, i0, i1, dead)) {
        if (!seenAlive && ep.first === i0) continue;
        seenAlive = true;
        const cameBack = ep.last + 1 < i1;
        const duration = cameBack ? ep.end - ep.start : ch.t[i1 - 1] - ep.start;
        if (cameBack && duration < T.disconnectMinS) continue;
        summary.disconnects.push({ start: ep.start, end: cameBack ? ep.end : null });
        const enabled = isEnabled(periods, ep.start);
        add({ severity: enabled ? 'critical' : 'warning', t: ep.start, end: cameBack ? ep.end : null,
          system: name, kind: 'motor-disconnect',
          title: cameBack ? `${name} disconnected for ${duration.toFixed(1)}s`
            : `${name} disconnected and never came back`,
          detail: `The motor stopped answering at ${when(ep.start)}` +
            (cameBack ? ` and came back ${duration.toFixed(1)}s later` : '') +
            '. Whatever it drives had no power meanwhile. Usually a loose or damaged cable, or ' +
            'a port knocked by a collision. Reseat and check it.',
          channels: [ch.name] });
      }
      for (let i = i0; i < i1 && !seenAlive; ++i) if (!dead(i)) seenAlive = true;

      // Temperature.
      if (summary.firstDerate !== null) {
        const fraction = reportedDerating(summary.peakTemp);
        add({ severity: 'critical', t: summary.firstDerate, system: name,
          title: `${name} overheated (${summary.peakTemp.toFixed(0)}°C)`,
          detail: `It reached ${T.motorDerateC}°C at ${when(summary.firstDerate)} and stayed there ` +
            `${summary.timeAboveDerate.toFixed(0)}s. A V5 motor cuts its own current limit as it ` +
            `heats (to half at 55°C, ${Math.round(fraction * 100)}% at its peak of ` +
            `${summary.peakTemp.toFixed(0)}°C) and reports nothing up the command path, so ` +
            'whatever it drives quietly lost torque: slower to accelerate, weaker holding a load. ' +
            'Look for stalls or holding at high current before this (it heats fastest then), ' +
            'lower hold voltages, or rest it between matches.',
          channels: [ch.name] });
      } else if (summary.firstWarm !== null) {
        add({ severity: 'warning', t: summary.firstWarm, system: name,
          title: `${name} running hot (${summary.peakTemp.toFixed(0)}°C)`,
          detail: `It reached ${summary.peakTemp.toFixed(0)}°C (at ${when(summary.peakAt)}). ` +
            'At 55°C a V5 motor halves its own power; this one is one busy match from it.',
          channels: [ch.name] });
      }
      if (summary.faultBits & 2) {
        add({ severity: 'critical', t: null, system: name, title: `${name}: motor driver fault`,
          detail: 'The motor reported a driver fault (fault bit 2): an electrical problem inside ' +
            'the motor or its cable. Swap the motor if it repeats.', channels: [ch.name] });
      }

      // Stalls: high current, not turning.
      const stalled = (i) => Number.isFinite(amps[i]) && Math.abs(amps[i]) >= T.stallAmps &&
        Number.isFinite(rpm[i]) && Math.abs(rpm[i]) < T.stallRpm;
      const stalls = episodes(ch, i0, i1, stalled).filter((ep) => ep.end - ep.start >= T.stallMinS);
      summary.stallS = stalls.reduce((sum, ep) => sum + (ep.end - ep.start), 0);
      const longest = stalls.reduce((a, b) => (!a || b.end - b.start > a.end - a.start ? b : a), null);
      if (summary.stallS > 0 && (summary.stallS >= T.stallShareOfEnabled * enabledTotal ||
          (longest && longest.end - longest.start >= T.stallLongS))) {
        let ampsDuring = 0;
        let n = 0;
        for (const ep of stalls) {
          for (let i = ep.first; i <= ep.last; ++i) {
            ampsDuring += Math.abs(amps[i]);
            n++;
          }
        }
        const share = enabledTotal > 0 ? summary.stallS / enabledTotal : 0;
        add({ severity: 'warning', t: longest.start, end: longest.end, system: name,
          title: `${name} stalled ${summary.stallS.toFixed(0)}s`,
          detail: `${stalls.length} stall${stalls.length === 1 ? '' : 's'} totalling ` +
            `${summary.stallS.toFixed(1)}s (${Math.round(share * 100)}% of enabled time) at ` +
            `${(ampsDuring / n).toFixed(1)}A while barely turning; the longest ` +
            `${(longest.end - longest.start).toFixed(1)}s from ${when(longest.start)}. Stalled ` +
            'current is nearly all heat. If it\'s holding something on purpose, hold with less ' +
            'voltage; if not, something is jammed or it\'s pushing a hard stop.',
          channels: [ch.name] });
      }

      // Speed per volt, early vs late: a motor delivering less speed for the
      // same volts is derating (hot) or dragging (friction).
      const ratios = [];
      for (let i = i0; i < i1; ++i) {
        if (!Number.isFinite(volts[i]) || !Number.isFinite(rpm[i])) continue;
        if (Math.abs(volts[i]) < 4 || Math.abs(rpm[i]) < 20) continue;
        if (Math.sign(volts[i]) !== Math.sign(rpm[i])) continue;
        ratios.push({ t: ch.t[i], ratio: Math.abs(rpm[i]) / Math.abs(volts[i]), temp: temp[i] });
      }
      if (ratios.length >= 60) {
        const third = Math.floor(ratios.length / 3);
        const early = ratios.slice(0, third);
        const late = ratios.slice(ratios.length - third);
        summary.rpmPerVoltEarly = median(early.map((r) => r.ratio));
        summary.rpmPerVoltLate = median(late.map((r) => r.ratio));
        const drop = 1 - summary.rpmPerVoltLate / summary.rpmPerVoltEarly;
        if (drop >= T.speedPerVoltDrop) {
          const tempEarly = median(early.map((r) => r.temp).filter(Number.isFinite));
          const tempLate = median(late.map((r) => r.temp).filter(Number.isFinite));
          add({ severity: 'warning', t: late[0].t, system: name,
            title: `${name} lost ${Math.round(drop * 100)}% of its speed per volt`,
            detail: `${summary.rpmPerVoltEarly.toFixed(1)} rpm/V early in the range, ` +
              `${summary.rpmPerVoltLate.toFixed(1)} rpm/V late, while its temperature went ` +
              `${Number.isFinite(tempEarly) ? tempEarly.toFixed(0) : '?'}→` +
              `${Number.isFinite(tempLate) ? tempLate.toFixed(0) : '?'}°C. Hot: thermal ` +
              'derating. Not hot: friction (a rubbing gear, a bent shaft) or a load that grew.',
            channels: [ch.name] });
        }
      }
    }
    return summaries;
  }

  function analyzeBattery(log, t0, t1, sessions, periods, add, when) {
    const T = THRESHOLDS;
    const ch = log.get('batt');
    const out = { start: NaN, end: NaN, min: NaN, minAt: null, pctStart: NaN, pctEnd: NaN,
      peakAmps: NaN, peakTemp: NaN };
    if (!ch || !ch.has('volts')) return out;
    const [i0, i1] = ch.range(t0, t1);
    if (i1 <= i0) return out;
    const volts = ch.cols.volts;
    const pct = ch.cols.pct;
    const amps = ch.cols.amps;
    const temp = ch.cols.temp;
    for (let i = i0; i < i1; ++i) {
      if (Number.isFinite(volts[i])) {
        if (Number.isNaN(out.start)) out.start = volts[i];
        out.end = volts[i];
        if (isEnabled(periods, ch.t[i]) && !(volts[i] >= out.min)) {
          out.min = volts[i];
          out.minAt = ch.t[i];
        }
      }
      if (pct && Number.isFinite(pct[i])) {
        if (Number.isNaN(out.pctStart)) out.pctStart = pct[i];
        out.pctEnd = pct[i];
      }
      if (amps && Number.isFinite(amps[i]) && !(Math.abs(amps[i]) <= out.peakAmps)) {
        out.peakAmps = Math.abs(amps[i]);
      }
      if (temp && Number.isFinite(temp[i]) && !(temp[i] <= out.peakTemp)) out.peakTemp = temp[i];
    }
    if (out.min < T.batteryCriticalV) {
      add({ severity: 'critical', t: out.minAt, system: 'Battery',
        title: `Battery sagged to ${out.min.toFixed(2)}V`,
        detail: `At ${when(out.minAt)}. Sag this deep under load starves every motor at once and ` +
          'can brown the brain out. Check the battery\'s charge and age, and what was drawing ' +
          'current then (the motors\' amps).', channels: ['batt'] });
    } else if (out.min < T.batteryWarnV) {
      add({ severity: 'warning', t: out.minAt, system: 'Battery',
        title: `Battery sagged to ${out.min.toFixed(2)}V`,
        detail: `At ${when(out.minAt)}. Every motor gets less when the battery sags; a fresher ` +
          'battery would give more.', channels: ['batt'] });
    }
    if (pct) {
      for (const s of sessions) {
        if (s.kind !== 'match' || s.start < t0 - 0.001 || s.start > t1) continue;
        const startPct = ch.valueAt('pct', s.start + 1, 5);
        if (Number.isFinite(startPct) && startPct < T.batteryStartPct) {
          add({ severity: 'warning', t: s.start, system: 'Battery',
            title: `${s.label} started at ${startPct.toFixed(0)}% battery`,
            detail: 'Start matches on a charged battery. A V5 battery at low charge sags more ' +
              'under the same load.', channels: ['batt'] });
        }
      }
    }
    if (out.peakTemp >= T.batteryHotC) {
      add({ severity: 'warning', t: null, system: 'Battery',
        title: `Battery reached ${out.peakTemp.toFixed(0)}°C`,
        detail: 'A hot battery delivers less and ages faster; let it cool before charging.',
        channels: ['batt'] });
    }
    return out;
  }

  function analyzeDriver(log, t0, t1, periods, add, when) {
    const ch = log.get('driver');
    if (!ch || !ch.has('connected')) return;
    const [i0, i1] = ch.range(t0, t1);
    const connected = ch.cols.connected;
    for (const ep of episodes(ch, i0, i1, (i) => connected[i] === 0)) {
      const duration = ep.end - ep.start;
      if (duration < THRESHOLDS.controllerDropMinS || !isEnabled(periods, ep.start)) continue;
      add({ severity: 'critical', t: ep.start, end: ep.end, system: 'Controller',
        title: `Controller disconnected for ${duration.toFixed(1)}s`,
        detail: `From ${when(ep.start)}, the robot got no driver input (every stick reads 0). ` +
          'Usually the radio: check its cable and mounting, keep it away from metal and motors, ' +
          'and check the controller\'s battery.', channels: ['driver'] });
    }
  }

  function analyzeDevices(log, add, when) {
    for (const { t, tag, msg } of log.events) {
      if (tag !== 'device') continue;
      const parts = msg.split(',');
      const keys = slt.parseKeys(parts.slice(1));
      const label = keys.label || `port ${keys.port}`;
      if (parts[0] === 'lost') {
        const back = log.events.find((e) => e.tag === 'device' && e.t > t &&
          e.msg.startsWith('back,') && e.msg.includes(`port=${keys.port},`));
        add({ severity: 'critical', t, end: back ? back.t : null, system: label,
          kind: 'device-lost', port: keys.port, found: keys.found,
          title: `${label} unplugged${back ? ` for ${(back.t - t).toFixed(1)}s` : ''}`,
          detail: `The Diagnostics check for port ${keys.port} failed at ${when(t)}` +
            (keys.found ? ` (found: ${keys.found})` : '') +
            (back ? `, and it came back at ${when(back.t)}.` : ', and it never came back.'),
          channels: [] });
      } else if (parts[0] === 'missing') {
        add({ severity: 'warning', t, system: label, title: `${label} missing at startup`,
          detail: `Port ${keys.port} didn't have the expected device when the program started` +
            (keys.found ? ` (found: ${keys.found})` : '') + '.', channels: [] });
      }
    }
  }

  function analyzeOdometry(log, t0, t1, add, when) {
    const ch = log.get('odom');
    if (!ch || !ch.has('x')) return;
    const [i0, i1] = ch.range(t0, t1);
    const x = ch.cols.x;
    const y = ch.cols.y;
    let jumps = 0;
    let first = null;
    let biggest = 0;
    for (let i = Math.max(i0, 1); i < i1; ++i) {
      if (ch.t[i] - ch.t[i - 1] > 0.05) continue;
      const d = Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
      if (d > THRESHOLDS.odomJumpIn) {
        jumps++;
        if (first === null) first = ch.t[i];
        biggest = Math.max(biggest, d);
      }
    }
    if (jumps > 0) {
      add({ severity: 'warning', t: first, system: 'Odometry',
        title: `Odometry jumped ${biggest.toFixed(0)}in${jumps > 1 ? ` (${jumps} times)` : ''}`,
        detail: `The pose moved farther in one update than a robot can (first at ${when(first)}). ` +
          'A tracking wheel or IMU glitch, or a setPose() mid-motion. Every motion after it ' +
          'drove to the wrong place.', channels: ['odom'] });
    }
  }

  /** Per-response metrics for every pid channel, plus oscillation and loop-stall findings. */
  function analyzePids(log, t0, t1, add, when) {
    const out = [];
    for (const ch of log.channels.filter((c) => c.kind === 'pid')) {
      const [i0, i1] = ch.range(t0, t1);
      const err = ch.cols.err;
      const flags = ch.cols.flags;
      const outCol = ch.cols.out;
      const dt = ch.cols.dt;
      const responses = [];
      let current = null;
      let saturated = 0;
      const gaps = [];
      for (let i = i0; i < i1; ++i) {
        if (!current || (flags[i] & slt.FIRST_STEP)) {
          current = { first: i, last: i, start: ch.t[i], swings: 0, saturated: 0 };
          responses.push(current);
        } else {
          const gap = ch.t[i] - ch.t[i - 1];
          if (gap > Math.max(5 * dt[i], THRESHOLDS.loopGapS)) gaps.push({ t: ch.t[i - 1], gap });
          // A swing is a zero crossing after an excursion worth the name: a
          // tenth of the step, so noise dithering around zero isn't counted.
          current.peak = Math.max(current.peak || 0, Math.abs(err[i - 1]));
          if (Math.sign(err[i]) !== Math.sign(err[i - 1]) && err[i] !== 0) {
            if (current.peak >= 0.1 * Math.abs(err[current.first])) current.swings++;
            current.peak = 0;
          }
        }
        current.last = i;
        if (flags[i] & 1) {
          current.saturated++;
          saturated++;
        }
      }
      for (const r of responses) {
        r.end = ch.t[r.last];
        r.rows = r.last - r.first + 1;
        r.initialError = err[r.first];
        r.finalError = err[r.last];
        r.saturatedFraction = r.saturated / r.rows;
        let peak = 0;
        for (let i = r.first; i <= r.last; ++i) peak = Math.max(peak, Math.abs(outCol[i]));
        r.peakOut = peak;
        r.gains = ch.gainsAt(r.start);
        // Only step responses: a loop that runs all the time (driver heading
        // hold) never "starts" a response, and its error crossing zero is just
        // it following the driver.
        const stepLike = r.end - r.start <= THRESHOLDS.responseMaxS &&
          Math.abs(r.initialError) > 1e-6;
        if (stepLike && r.swings >= THRESHOLDS.oscillationSwings && r.rows > 20) {
          // Decaying swings are a loop settling; growing or steady ones aren't.
          const tail = Math.max(r.first, r.last - Math.floor(r.rows / 3));
          let tailPeak = 0;
          for (let i = tail; i <= r.last; ++i) tailPeak = Math.max(tailPeak, Math.abs(err[i]));
          const initial = Math.max(Math.abs(r.initialError), 1e-9);
          if (tailPeak > 0.15 * initial) {
            r.oscillating = true;
            add({ severity: 'warning', t: r.start, end: r.end, system: ch.name,
              title: `${ch.name} PID oscillated (${r.swings} swings)`,
              detail: `The error crossed zero ${r.swings} times from ${when(r.start)} without ` +
                'dying out' + (r.gains ? ` (kP ${fmt(r.gains.kP)}, kD ${fmt(r.gains.kD)})` : '') +
                '. Too much kP, too little kD, or more delay than the gains allow. The Tune tab ' +
                'can design gains from a measured model.', channels: [ch.name] });
          }
        }
      }
      if (gaps.length > 0) {
        const worst = gaps.reduce((a, b) => (b.gap > a.gap ? b : a));
        add({ severity: gaps.length > 3 ? 'warning' : 'info', t: worst.t, system: ch.name,
          title: `${ch.name} loop stalled ${gaps.length}× (worst ${(worst.gap * 1000).toFixed(0)}ms)`,
          detail: 'Its control loop went much longer than its period between steps mid-response: ' +
            'something blocked the task running it (a long device call, a busy loop at higher ' +
            'priority). The robot drove blind for that long.', channels: [ch.name] });
      }
      out.push({ name: ch.name, responses, saturatedFraction: i1 > i0 ? saturated / (i1 - i0) : 0,
        gaps });
    }
    return out;
  }

  function analyzeMechanisms(log, t0, t1, add, when) {
    const T = THRESHOLDS;
    const out = [];
    for (const ch of mechanismChannels(log)) {
      const base = ch.name.replace(/\.act$/, '');
      const [i0, i1] = ch.range(t0, t1);
      const target = ch.cols.target;
      const pos = ch.cols.pos;
      const volts = ch.cols.volts;
      const law = ch.cols.law;
      const summary = { name: base, channel: ch.name, sensorLostS: 0, shortfalls: [], sag: NaN,
        holds: 0 };
      out.push(summary);

      for (const ep of episodes(ch, i0, i1, (i) => law[i] === model.LAW.sensorLost)) {
        summary.sensorLostS += ep.end - ep.start;
        add({ severity: 'critical', t: ep.start, end: ep.end, system: base,
          title: `${base} sensor lost for ${(ep.end - ep.start).toFixed(1)}s`,
          detail: `Its position sensor stopped answering at ${when(ep.start)}, so it braked in ` +
            'place until the reading came back. Check the sensor\'s cable and port.',
          channels: [ch.name] });
      }

      // Couldn't reach its target: well off it, and no longer getting closer.
      // (Far off but moving is just a long move.)
      const off = (i) => law[i] === model.LAW.track && Number.isFinite(pos[i]) &&
        Math.abs(target[i] - pos[i]) > T.trackingBand;
      for (const ep of episodes(ch, i0, i1, off)) {
        if (ep.end - ep.start < T.trackingMinS) continue;
        let stuckFrom = null;
        let full = 0;
        let rows = 0;
        let worst = 0;
        let j = ep.first;
        for (let i = ep.first; i <= ep.last; ++i) {
          while (ch.t[j] < ch.t[i] - T.stuckWindowS) j++;
          if (ch.t[i] - ch.t[ep.first] < T.stuckWindowS) continue;
          if (Math.abs(pos[i] - pos[j]) < T.trackingBand) {
            if (stuckFrom === null) stuckFrom = ch.t[j];
            rows++;
            if (Math.abs(volts[i]) >= 11.5) full++;
            worst = Math.max(worst, Math.abs(target[i] - pos[i]));
          }
        }
        if (stuckFrom === null) continue;
        const atFull = full / rows > 0.5;
        // Stuck with volts to spare is the loop settling short under load;
        // the sag check below reports that once, rather than once per hold.
        if (!atFull) continue;
        summary.shortfalls.push({ start: stuckFrom, end: ep.end, worst, atFull });
        add({ severity: 'critical', t: stuckFrom, end: ep.end, system: base,
          title: `${base} stuck ${worst.toFixed(0)} short of its target`,
          detail: `From ${when(stuckFrom)} it stopped getting closer, still more than ` +
            `${T.trackingBand} off for ${(ep.end - stuckFrom).toFixed(1)}s, at full power. ` +
            'Overloaded, jammed, or its motors are derating: check their temperatures at this ' +
            'moment.',
          channels: [ch.name, base] });
      }

      // Sag: where it came to rest under a steady target, is it consistently
      // below? That's gravity the loop isn't cancelling.
      const errors = [];
      let holdStart = null;
      for (let i = i0 + 1; i < i1; ++i) {
        const steady = law[i] === model.LAW.track && target[i] === target[i - 1] &&
          Number.isFinite(pos[i]) && Math.abs(pos[i] - pos[i - 1]) < 0.2;
        if (!steady) {
          holdStart = null;
          continue;
        }
        if (holdStart === null) holdStart = ch.t[i];
        if (ch.t[i] - holdStart >= 0.3 && Math.abs(target[i] - pos[i]) < 3 * T.trackingBand) {
          errors.push(target[i] - pos[i]);
        }
      }
      summary.holds = errors.length;
      if (errors.length >= 20) {
        summary.sag = median(errors);
        if (summary.sag >= T.sagMin) {
          add({ severity: 'warning', t: null, system: base,
            title: `${base} settles ${summary.sag.toFixed(1)} below its targets`,
            detail: 'Holding still at a steady target, it rests consistently low: gravity the loop ' +
              'isn\'t cancelling, so kP·error is holding the weight up. Add gravity feedforward ' +
              '(the Tune tab measures kG from this log) rather than raising kP.',
            channels: [ch.name] });
        }
      }
      const external = episodes(ch, i0, i1, (i) => law[i] === model.LAW.external);
      if (external.length > 0) {
        add({ severity: 'info', t: external[0].start, system: base,
          title: `Auto-Tune drove the ${base} (${external.length} run${external.length > 1 ? 's' : ''})`,
          detail: 'Its loop stood aside while a characterization run drove the motors. The Tune ' +
            'tab refits those runs.', channels: [ch.name] });
      }
    }
    return out;
  }

  function fmt(value) {
    if (!Number.isFinite(value)) return String(value);
    const abs = Math.abs(value);
    return abs !== 0 && (abs < 0.01 || abs >= 1000) ? value.toExponential(2) : value.toFixed(3);
  }

  function analyzeMotions(log, t0, t1, report, add, when) {
    const list = slt.motions(log).filter((m) => m.start >= t0 - 0.001 && m.start <= t1);
    for (const m of list) {
      if (m.reason === 'timeout') {
        // What was the robot doing when it gave up?
        const context = [];
        const hot = report.motors
          .filter((mo) => !mo.neverAnswered)
          .map((mo) => ({ name: mo.name, temp: log.get(mo.channel).valueAt('temp', m.end, 1) }))
          .filter((mo) => mo.temp >= THRESHOLDS.motorWarmC);
        if (hot.length) context.push(`hot motors then: ${hot.map((h) => `${h.name} ${h.temp}°C`).join(', ')}`);
        const batt = log.get('batt');
        const volts = batt ? batt.valueAt('volts', m.end, 2) : NaN;
        if (Number.isFinite(volts)) context.push(`battery ${volts.toFixed(1)}V`);
        // Only the loops this kind of motion runs (docs/TELEMETRY_FORMAT.md),
        // and only before it ended: the next motion's first step can share
        // its end's millisecond.
        let saturatedAtEnd = false;
        for (const name of MOTION_PIDS[m.kind] || []) {
          const ch = log.get(name);
          if (!ch) continue;
          const [a, b] = ch.range(Math.max(m.start, m.end - 0.3), m.end - 1e-6);
          for (let i = a; i < b; ++i) if (ch.cols.flags[i] & 1) saturatedAtEnd = true;
        }
        const unit = m.kind === 'turnToHeading' ? '°' : 'in';
        add({ severity: 'warning', t: m.start, end: m.end, system: 'Drivetrain',
          title: `${m.kind} timed out ${m.error !== null ? `${m.error.toFixed(1)}${unit} short` : ''}`,
          detail: `Started ${when(m.start)}, gave up after ${m.ms} ms. ` +
            (saturatedAtEnd ? 'It was still at full power when it gave up: pushing against ' +
              'something, or asking for more than the robot has. '
              : 'Its output wasn\'t saturated: the loop settled short of the threshold (friction ' +
                'against a small kP, or a threshold tighter than the robot can hold). ') +
            (context.length ? `(${context.join('; ')})` : ''),
          channels: ['drive', 'turn', 'odom'] });
      } else if (m.reason === 'aborted') {
        add({ severity: 'info', t: m.start, system: 'Drivetrain', title: `${m.kind} aborted`,
          detail: 'It never started: no odometry set, or an empty path. The terminal log says which.',
          channels: [] });
      } else if (m.end === null) {
        add({ severity: 'info', t: m.start, end: m.stop, system: 'Drivetrain',
          title: `${m.kind} cut short`,
          detail: `The competition state changed mid-motion (${when(m.stop)}), which ends the ` +
            'autonomous task wherever it is. Expected at the end of autonomous; anywhere else, ' +
            'the routine ran long.', channels: [] });
      }
    }
    return list;
  }

  function analyzeLogger(log, add, when) {
    const out = { files: log.files.length, skipped: log.skipped, drops: 0, faults: 0,
      slowestWriteUs: 0 };
    let lastFaults = 0;
    for (const h of log.health) {
      const v = h.values;
      if (v.faults > lastFaults) {
        add({ severity: 'critical', t: h.t, system: 'SD log',
          title: 'SD card write failed', detail: `At ${when(h.t)} a write to the card failed ` +
            '(pulled, full, or failing card). Rows were lost until a new file opened.', channels: [] });
        lastFaults = v.faults;
      }
      out.faults = Math.max(out.faults, v.faults || 0);
      out.drops = Math.max(out.drops, v.drops || 0);
      out.slowestWriteUs = Math.max(out.slowestWriteUs, v.wmax_us || 0);
    }
    if (out.drops > 0) {
      const byChannel = new Map();
      for (const d of log.drops) byChannel.set(d.name, d.full + d.contended);
      const list = [...byChannel.entries()].filter(([, n]) => n > 0)
        .map(([name, n]) => `${name} ${n}`).join(', ');
      add({ severity: out.drops > 100 ? 'warning' : 'info', t: null, system: 'SD log',
        title: `${out.drops} telemetry rows dropped`,
        detail: `Rows the logger couldn't keep up with (${list || 'see D rows'}). Charts have small ` +
          'gaps there; nothing on the robot was slowed.', channels: [] });
    }
    if (out.slowestWriteUs > THRESHOLDS.slowWriteUs) {
      add({ severity: 'info', t: null, system: 'SD log',
        title: `Slowest SD write ${(out.slowestWriteUs / 1000).toFixed(0)}ms`,
        detail: 'A slow card, or a folder with many files. Empty /sl now and then.', channels: [] });
    }
    for (const { t, tag, msg } of log.events) {
      if (tag !== 'sd') continue;
      if (msg.startsWith('dir_missing')) {
        add({ severity: 'warning', t, system: 'SD log', title: 'Log folder missing',
          detail: 'Logs are going to the card\'s root. Make an "sl" folder at the root on a ' +
            'computer (the V5 can\'t create folders).', channels: [] });
      } else if (msg.startsWith('reopened')) {
        add({ severity: 'warning', t, system: 'SD log', title: 'Log reopened after an SD fault',
          detail: `A new file continued the run after a write failure (${msg}).`, channels: [] });
      }
    }
    if (log.skipped > 0) {
      add({ severity: 'info', t: null, system: 'SD log', title: `${log.skipped} lines skipped`,
        detail: 'Lines that didn\'t parse (usually a torn write at power-off) were skipped.',
        channels: [] });
    }
    return out;
  }

  function analyzeEnding(log, add, when) {
    const list = slt.phases(log);
    const last = list[list.length - 1];
    if (!last || last.mode === 'disabled' || !last.comp) return;
    add({ severity: 'critical', t: log.end, system: 'Robot',
      title: `Log ends mid-${last.mode === 'autonomous' ? 'autonomous' : 'match'}`,
      detail: `The last thing logged was ${when(log.end)}, still enabled under competition ` +
        'control. The field disables the robot at the end of a match and the logger writes that ' +
        'to the card at once, so a log that just stops means the brain lost power or the program ' +
        'crashed. Check the battery connection and the power cable, and the terminal for a crash.',
      channels: ['batt'] });
  }

  return {
    THRESHOLDS,
    reportedDerating,
    analyze,
    timeLabel,
    clock,
    motorChannels,
    mechanismChannels,
    shortName,
    median,
    episodes,
    fmt,
  };
});
