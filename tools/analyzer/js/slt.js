/*
 * SapphireLib telemetry analyzer: slt.js
 *
 * Reads SLT v1 telemetry files (docs/TELEMETRY_FORMAT.md) into a columnar
 * model the rest of the analyzer charts and analyzes. Parsing follows
 * tools/telemetry/slt_read.py, the reference reader, rule for rule: refuse any
 * major version but 1, discard a torn final line, skip (and count) any line
 * that fails to parse, ignore anything unrecognized, and match channels by
 * name. test/slt.test.js runs slt_read.py's self-test checks against the same
 * golden file.
 *
 * On top of the reference reader: typed-array columns and time lookups,
 * motion/auton pairing, competition sessions (matches) from phase events,
 * Auto-Tune characterization runs, and merging the files one program run was
 * split across after an SD fault.
 *
 * Plain script: window.SA.slt in a browser, require('./slt.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.slt = factory();
  }
})(function () {
  'use strict';

  const FIRST_STEP = 8;
  const CONFIG_KEYS = ['integral_limit', 'output_limit', 'slew_rate', 'derivative_on_measurement',
    'nominal_dt_s'];
  const PID_COLUMNS = ['target', 'meas', 'err', 'p', 'i', 'd', 'u_raw', 'out', 'dt', 'flags'];
  const NUMBER = /^[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?$/;
  const INTEGER = /^-?\d+$/;

  class ParseError extends Error {}

  function num(text) {
    if (text === 'nan') return NaN;
    if (text === 'inf') return Infinity;
    if (text === '-inf') return -Infinity;
    if (!NUMBER.test(text)) throw new ParseError(`not a number: ${text}`);
    return Number(text);
  }

  function int(text) {
    if (!INTEGER.test(text)) throw new ParseError(`not an integer: ${text}`);
    return Number(text);
  }

  /** Index of the last element <= value in a sorted array; -1 if none. */
  function lastAtOrBefore(sorted, value) {
    let lo = 0;
    let hi = sorted.length - 1;
    let found = -1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (sorted[mid] <= value) {
        found = mid;
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    return found;
  }

  /** Index of the first element >= value; sorted.length if none. */
  function firstAtOrAfter(sorted, value) {
    let lo = 0;
    let hi = sorted.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (sorted[mid] < value) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  class Channel {
    constructor(id, name, kind, decimals, columns) {
      this.id = id;
      this.name = name;
      this.kind = kind; // 'samples' | 'pid' | 'events'
      this.decimals = decimals;
      this.columns = columns;
      this.gains = []; // { t, kP, kI, kD }
      this.config = null;
      this.resets = []; // t of each R row (s)
      this.dropped = [0, 0]; // latest D row: full, contended
      this._t = [];
      this._v = columns.map(() => []);
      this.length = 0;
      this.t = new Float64Array(0); // seconds since program start
      this.cols = {};
    }

    _push(tUs, values) {
      this._t.push(tUs);
      for (let i = 0; i < values.length; ++i) this._v[i].push(values[i]);
    }

    _finalize() {
      if (this._t === null) return;
      this.length = this._t.length;
      this.t = Float64Array.from(this._t, (us) => us / 1e6);
      this.cols = {};
      this.columns.forEach((name, i) => {
        this.cols[name] = Float64Array.from(this._v[i]);
      });
      this._t = null;
      this._v = null;
    }

    has(column) {
      return this.columns.includes(column);
    }

    column(name) {
      const values = this.cols[name];
      if (!values) throw new Error(`channel ${this.name} has no column ${name}`);
      return values;
    }

    /** Index of the last row at or before `tSec`; -1 if none. */
    indexAt(tSec) {
      return lastAtOrBefore(this.t, tSec);
    }

    /**
     * The column's value as of `tSec` (the last row at or before it), or NaN
     * if there's none within `maxAgeS`.
     */
    valueAt(name, tSec, maxAgeS = Infinity) {
      const i = this.indexAt(tSec);
      if (i < 0 || tSec - this.t[i] > maxAgeS) return NaN;
      return this.cols[name][i];
    }

    /** [first, end) row indices with t0 <= t <= t1. */
    range(t0, t1) {
      return [firstAtOrAfter(this.t, t0), lastAtOrBefore(this.t, t1) + 1];
    }

    /** (kP, kI, kD) in effect at tSec, or null before the first G row. */
    gainsAt(tSec) {
      let current = null;
      for (const g of this.gains) {
        if (g.t > tSec) break;
        current = g;
      }
      return current;
    }
  }

  class Log {
    constructor() {
      this.meta = {};
      this.channels = []; // registration order
      this.byId = new Map();
      this.byName = new Map();
      this.events = []; // { t, tag, msg }
      this.health = []; // { t, values }
      this.drops = []; // { t, id, name, full, contended }
      this.skipped = 0;
      this.firstUs = null;
      this.lastUs = 0;
      this.files = [];
      this.bytes = 0;
    }

    get start() {
      return this.firstUs === null ? 0 : this.firstUs / 1e6;
    }

    get end() {
      return this.lastUs / 1e6;
    }

    get(name) {
      return this.byName.get(name) || null;
    }

    _stamp(text) {
      const t = int(text);
      if (t > this.lastUs) this.lastUs = t;
      if (this.firstUs === null || t < this.firstUs) this.firstUs = t;
      return t;
    }

    _addChannel(channel) {
      this.byId.set(channel.id, channel);
      const existing = this.byName.get(channel.name);
      if (!existing) {
        this.byName.set(channel.name, channel);
        this.channels.push(channel);
      }
    }
  }

  function parseLine(log, line) {
    const comma = line.indexOf(',');
    const kind = comma < 0 ? line : line.slice(0, comma);
    const rest = comma < 0 ? '' : line.slice(comma + 1);
    switch (kind) {
      case 'S': {
        const parts = rest.split(',');
        const channel = log.byId.get(int(parts[0]));
        if (!channel) throw new ParseError('unknown id');
        if (parts.length - 2 !== channel.columns.length) throw new ParseError('row width');
        const tUs = log._stamp(parts[1]);
        const values = new Array(parts.length - 2);
        for (let i = 2; i < parts.length; ++i) values[i - 2] = num(parts[i]);
        channel._push(tUs, values);
        break;
      }
      case 'G': {
        const parts = rest.split(',');
        if (parts.length !== 5) throw new ParseError('gains width');
        const channel = log.byId.get(int(parts[0]));
        if (!channel) throw new ParseError('unknown id');
        const t = log._stamp(parts[1]) / 1e6;
        channel.gains.push({ t, kP: num(parts[2]), kI: num(parts[3]), kD: num(parts[4]) });
        break;
      }
      case 'C': {
        const parts = rest.split(',');
        if (parts.length !== 2 + CONFIG_KEYS.length) throw new ParseError('config width');
        const channel = log.byId.get(int(parts[0]));
        if (!channel) throw new ParseError('unknown id');
        log._stamp(parts[1]);
        const config = {};
        CONFIG_KEYS.forEach((key, i) => {
          config[key] = num(parts[2 + i]);
        });
        channel.config = config;
        break;
      }
      case 'R': {
        const parts = rest.split(',');
        if (parts.length !== 2) throw new ParseError('reset width');
        const channel = log.byId.get(int(parts[0]));
        if (!channel) throw new ParseError('unknown id');
        channel.resets.push(log._stamp(parts[1]) / 1e6);
        break;
      }
      case 'E': {
        const first = rest.indexOf(',');
        if (first < 0) throw new ParseError('event width');
        const second = rest.indexOf(',', first + 1);
        const t = log._stamp(rest.slice(0, first)) / 1e6;
        const tag = second < 0 ? rest.slice(first + 1) : rest.slice(first + 1, second);
        const msg = second < 0 ? '' : rest.slice(second + 1);
        log.events.push({ t, tag, msg });
        break;
      }
      case 'D': {
        const parts = rest.split(',');
        if (parts.length !== 4) throw new ParseError('drops width');
        const t = log._stamp(parts[0]) / 1e6;
        const channel = log.byId.get(int(parts[1]));
        if (!channel) throw new ParseError('unknown id');
        channel.dropped = [int(parts[2]), int(parts[3])];
        log.drops.push({ t, id: channel.id, name: channel.name, full: channel.dropped[0],
          contended: channel.dropped[1] });
        break;
      }
      case 'H': {
        const parts = rest.split(',');
        const t = log._stamp(parts[0]) / 1e6;
        const values = {};
        for (const pair of parts.slice(1)) {
          const eq = pair.indexOf('=');
          if (eq < 0) throw new ParseError('health pair');
          values[pair.slice(0, eq)] = int(pair.slice(eq + 1));
        }
        log.health.push({ t, values });
        break;
      }
      case '#chan': {
        const parts = rest.split(',');
        if (parts.length < 4) throw new ParseError('chan width');
        const kindName = parts[2];
        log._addChannel(new Channel(int(parts[0]), parts[1], kindName, int(parts[3]),
          parts.slice(4)));
        break;
      }
      case '#meta': {
        const eq = rest.indexOf(',');
        if (eq < 0) throw new ParseError('meta width');
        log.meta[rest.slice(0, eq)] = rest.slice(eq + 1);
        break;
      }
      default:
        // Future row types and '#' directives are skipped by design.
        break;
    }
  }

  /**
   * A Log from a whole file's text (or an ArrayBuffer/Uint8Array of it).
   * Throws if it isn't SLT v1; skips (and counts) bad lines otherwise.
   */
  function parse(data, fileName = '') {
    let text = data;
    if (typeof text !== 'string') {
      text = new TextDecoder('ascii').decode(data);
    }
    const lines = text.split('\n');
    // The last piece is "" after a final newline, or a line torn by power
    // loss. Either way it isn't a record.
    lines.pop();
    if (lines.length === 0 || !/^#SLT,1(,|$)/.test(lines[0])) {
      throw new Error(`${fileName || 'file'}: not an SLT v1 telemetry file`);
    }
    const log = new Log();
    log.bytes = text.length;
    for (let i = 1; i < lines.length; ++i) {
      try {
        parseLine(log, lines[i]);
      } catch (error) {
        if (!(error instanceof ParseError)) throw error;
        log.skipped++;
      }
    }
    for (const channel of log.byId.values()) channel._finalize();
    log.fileName = fileName || log.meta.file || 'log';
    log.files = [log.fileName];
    return log;
  }

  // --- Merging files --------------------------------------------------------------

  /** The file index in "SL000042.CSV", or null. */
  function fileIndex(name) {
    const match = /SL(\d{6})\.CSV$/i.exec(name || '');
    return match ? Number(match[1]) : null;
  }

  function reopenedFrom(log) {
    for (const e of log.events) {
      if (e.tag !== 'sd') continue;
      const m = /^reopened,prev=([^,]+)/.exec(e.msg);
      if (m) return m[1];
    }
    return null;
  }

  /** Appends `next` (a later file of the same program run) onto `into`. */
  function append(into, next) {
    for (const channel of next.channels) {
      let target = into.get(channel.name);
      if (!target) {
        target = new Channel(channel.id, channel.name, channel.kind, channel.decimals,
          channel.columns.slice());
        target._finalize();
        into.channels.push(target);
        into.byName.set(target.name, target);
      }
      const concat = (a, b) => {
        const out = new Float64Array(a.length + b.length);
        out.set(a);
        out.set(b, a.length);
        return out;
      };
      target.t = concat(target.t, channel.t);
      for (const column of target.columns) {
        target.cols[column] = concat(target.cols[column],
          channel.cols[column] || new Float64Array(channel.length).fill(NaN));
      }
      target.length = target.t.length;
      target.gains = target.gains.concat(channel.gains);
      target.resets = target.resets.concat(channel.resets);
      if (!target.config) target.config = channel.config;
      target.dropped = channel.dropped;
    }
    into.events = into.events.concat(next.events);
    into.health = into.health.concat(next.health);
    into.drops = into.drops.concat(next.drops);
    into.skipped += next.skipped;
    into.lastUs = Math.max(into.lastUs, next.lastUs);
    into.firstUs = Math.min(into.firstUs, next.firstUs);
    into.bytes += next.bytes;
    into.files = into.files.concat(next.files);
  }

  /**
   * Groups parsed Logs into program runs. A file that opened after an SD
   * fault on another (its `sd,reopened,prev=<name>` event) continues that
   * file's run and is merged into it; every other file is a run of its own.
   * Runs come back in file order.
   */
  function mergeRuns(logs) {
    const sorted = logs.slice().sort((a, b) => {
      const ia = fileIndex(a.meta.file || a.fileName);
      const ib = fileIndex(b.meta.file || b.fileName);
      if (ia !== null && ib !== null && ia !== ib) return ia - ib;
      return (a.fileName || '').localeCompare(b.fileName || '');
    });
    const runs = [];
    const byFile = new Map();
    for (const log of sorted) {
      const prev = reopenedFrom(log);
      const owner = prev ? byFile.get(prev.toUpperCase()) : null;
      if (owner) {
        append(owner, log);
        byFile.set((log.meta.file || log.fileName).toUpperCase(), owner);
      } else {
        runs.push(log);
        byFile.set((log.meta.file || log.fileName).toUpperCase(), log);
      }
    }
    for (const run of runs) run.events.sort((a, b) => a.t - b.t);
    return runs;
  }

  // --- Events: motions, autons, phases, sessions, tuning -------------------------

  function parseKeys(parts) {
    const keys = {};
    for (const part of parts) {
      const eq = part.indexOf('=');
      if (eq < 0) continue;
      const value = part.slice(eq + 1);
      keys[part.slice(0, eq)] = NUMBER.test(value) ? Number(value) : value;
    }
    return keys;
  }

  /**
   * The log's drivetrain motions, start to end, like slt_read.py's motions():
   * each `end` closes the most recent open `start` of the same kind; a motion
   * with no end was cut short and stops at the first phase change after it
   * started (or the end of the log).
   */
  function motions(log) {
    const done = [];
    const stack = [];
    const cut = new Map();
    const closeCut = (motion) => {
      motion.stop = cut.has(motion) ? cut.get(motion) : log.end;
      done.push(motion);
    };
    const events = log.events.slice().sort((a, b) => a.t - b.t);
    for (const { t, tag, msg } of events) {
      if (tag === 'phase') {
        for (const motion of stack) if (!cut.has(motion)) cut.set(motion, t);
        continue;
      }
      if (tag !== 'motion') continue;
      const parts = msg.split(',');
      if (parts.length < 2) continue;
      const [what, kind] = parts;
      const keys = parseKeys(parts.slice(2));
      if (what === 'start') {
        stack.push({ kind, start: t, params: keys, end: null, reason: null, error: null, ms: null,
          stop: null, depth: stack.length });
      } else if (what === 'end') {
        let match = -1;
        for (let i = stack.length - 1; i >= 0; --i) {
          if (stack[i].kind === kind) {
            match = i;
            break;
          }
        }
        if (match < 0) continue;
        while (stack.length > match + 1) closeCut(stack.pop());
        const motion = stack.pop();
        motion.end = motion.stop = t;
        motion.reason = keys.reason !== undefined ? String(keys.reason) : null;
        motion.error = typeof keys.error === 'number' ? keys.error : null;
        motion.ms = typeof keys.ms === 'number' ? keys.ms : null;
        done.push(motion);
      }
    }
    while (stack.length) closeCut(stack.pop());
    return done.sort((a, b) => a.start - b.start);
  }

  /** Autonomous routines: { name, start, end|null }. */
  function autons(log) {
    const out = [];
    const open = [];
    for (const { t, tag, msg } of log.events) {
      if (tag !== 'auton') continue;
      const comma = msg.indexOf(',');
      const what = comma < 0 ? msg : msg.slice(0, comma);
      const name = comma < 0 ? '' : msg.slice(comma + 1);
      if (what === 'start') {
        const routine = { name, start: t, end: null };
        open.push(routine);
        out.push(routine);
      } else if (what === 'end') {
        const i = open.map((r) => r.name).lastIndexOf(name);
        if (i >= 0) open.splice(i, 1)[0].end = t;
      }
    }
    return out;
  }

  /** Competition phase changes: { t, mode, comp, field }. */
  function phases(log) {
    const out = [];
    for (const { t, tag, msg } of log.events) {
      if (tag !== 'phase') continue;
      const parts = msg.split(',');
      const keys = parseKeys(parts.slice(1));
      out.push({ t, mode: parts[0], comp: keys.comp === 1, field: keys.field === 1 });
    }
    return out;
  }

  /** Contiguous stretches of one mode: { mode, comp, field, start, end }. */
  function periods(log) {
    const list = phases(log);
    const out = [];
    for (let i = 0; i < list.length; ++i) {
      const end = i + 1 < list.length ? list[i + 1].t : log.end;
      const p = list[i];
      const last = out[out.length - 1];
      if (last && last.mode === p.mode && last.comp === p.comp && last.field === p.field) {
        last.end = end;
      } else {
        out.push({ mode: p.mode, comp: p.comp, field: p.field, start: p.t, end });
      }
    }
    return out;
  }

  /**
   * What a person would call the parts of a run: matches (an autonomous
   * period under competition control followed by driver control), lone
   * autonomous or driver runs under competition control (skills, a switch),
   * and practice (enabled with no competition control, which is also where
   * Auto-Tune runs). Each is { kind, label, start, end, auton: [s, e]|null,
   * driver: [s, e]|null }. Disabled time between is left out.
   */
  function sessions(log) {
    const list = periods(log).filter((p) => p.end > p.start);
    const out = [];
    let practice = 0;
    let matches = 0;
    let runs = 0;
    for (let i = 0; i < list.length; ++i) {
      const p = list[i];
      if (p.mode === 'disabled') continue;
      if (!p.comp) {
        practice++;
        out.push({ kind: 'practice', label: `Practice ${practice}`, start: p.start, end: p.end,
          auton: p.mode === 'autonomous' ? [p.start, p.end] : null,
          driver: p.mode === 'opcontrol' ? [p.start, p.end] : null });
        continue;
      }
      if (p.mode === 'autonomous') {
        // A match: autonomous, then (after at most a short disable) driver.
        let j = i + 1;
        while (j < list.length && list[j].mode === 'disabled' && list[j].end - list[j].start < 30) j++;
        const next = list[j];
        if (next && next.mode === 'opcontrol' && next.comp && next.start - p.end < 30) {
          matches++;
          out.push({ kind: 'match', label: `Match ${matches}`, start: p.start, end: next.end,
            auton: [p.start, p.end], driver: [next.start, next.end] });
          i = j;
          continue;
        }
        runs++;
        out.push({ kind: 'auton', label: `Autonomous run ${runs}`, start: p.start, end: p.end,
          auton: [p.start, p.end], driver: null });
        continue;
      }
      runs++;
      out.push({ kind: 'driver', label: `Driver run ${runs}`, start: p.start, end: p.end,
        auton: null, driver: [p.start, p.end] });
    }
    return out;
  }

  /**
   * Auto-Tune activity from `tune` events: { starts: [{ t, axis }], models:
   * [{ t, axis, kS, kV, kA, kG, r2, delayS }] }.
   */
  function tuneEvents(log) {
    const starts = [];
    const models = [];
    for (const { t, tag, msg } of log.events) {
      if (tag !== 'tune') continue;
      const parts = msg.split(',');
      if (parts[0] === 'start' && parts[1]) starts.push({ t, axis: parts[1] });
      if (parts[0] === 'model' && parts[1]) {
        const k = parseKeys(parts.slice(2));
        models.push({ t, axis: parts[1], kS: k.kS, kV: k.kV, kA: k.kA, kG: k.kG ?? 0, r2: k.r2,
          delayS: typeof k.delay_ms === 'number' ? k.delay_ms / 1000 : 0 });
      }
    }
    return { starts, models };
  }

  /**
   * A char.* channel's rows split into the runner's segments, as
   * tuning::CharacterizationRun arrays ({ timeMs, volts, position }, timeMs
   * from the segment's first row). Rows within a segment are one sample
   * period apart; the runner waits ≥100ms between segments, which `gapS`
   * detects. `dropExtra` drops the one extra row a limit-cut segment ends
   * with (0V or held, after a row that wasn't), so the data matches exactly
   * what the robot fitted.
   */
  function characterizationSegments(channel, gapS = 0.05, dropExtra = true) {
    const segments = [];
    const volts = channel.cols.volts;
    const pos = channel.cols.pos;
    if (!volts || !pos) return segments;
    let current = null;
    let start = 0;
    let last = null;
    for (let i = 0; i < channel.length; ++i) {
      const t = channel.t[i];
      if (last === null || t - last > gapS) {
        current = { t: t, run: [] };
        segments.push(current);
        start = t;
      }
      current.run.push({ timeMs: Math.round((t - start) * 1000), volts: volts[i], position: pos[i] });
      last = t;
    }
    if (dropExtra) {
      for (const segment of segments) {
        const run = segment.run;
        if (run.length < 2) continue;
        const end = run[run.length - 1];
        const before = run[run.length - 2];
        const idle = (v) => v === 0 || Number.isNaN(v);
        if (idle(end.volts) && !idle(before.volts)) run.pop();
      }
    }
    return segments;
  }

  /**
   * Auto-Tune runs of one axis from its char.* channel: segments grouped by
   * the `tune,start,<axis>` events that begin each run (or by fours, the
   * runner's ramp/ramp/step/step, if there are none), as tuning's
   * CharacterizationData ({ ramps, steps }) plus when the run started and the
   * model the robot reported for it, if any.
   */
  function characterizationRuns(log, channelName, axisName) {
    const channel = log.get(channelName);
    if (!channel) return [];
    const segments = characterizationSegments(channel);
    if (segments.length === 0) return [];
    const { starts, models } = tuneEvents(log);
    const marks = starts.filter((s) => s.axis === axisName).map((s) => s.t);
    const groups = [];
    if (marks.length > 0) {
      for (const segment of segments) {
        const i = lastAtOrBefore(marks, segment.t + 1e-6);
        const key = Math.max(i, 0);
        if (!groups[key]) groups[key] = { start: marks[key] ?? segment.t, segments: [] };
        groups[key].segments.push(segment);
      }
    } else {
      for (let i = 0; i < segments.length; i += 4) {
        groups.push({ start: segments[i].t, segments: segments.slice(i, i + 4) });
      }
    }
    return groups.filter(Boolean).map((group, index) => {
      const data = { ramps: [], steps: [] };
      group.segments.forEach((segment, i) => {
        const kind = group.segments.length === 4 ? (i < 2 ? 'ramps' : 'steps') : classify(segment.run);
        data[kind].push(segment.run);
      });
      const end = group.segments[group.segments.length - 1].t + 3;
      const reported = models.find((m) => m.axis === axisName && m.t >= group.start && m.t <= end + 5);
      return { index, start: group.start, data, reported: reported || null,
        segments: group.segments.length };
    });
  }

  /** A segment is a step if its first command is (nearly) its biggest. */
  function classify(run) {
    let first = null;
    let biggest = 0;
    for (const s of run) {
      if (!Number.isFinite(s.volts) || s.volts === 0) continue;
      if (first === null) first = Math.abs(s.volts);
      biggest = Math.max(biggest, Math.abs(s.volts));
    }
    return first !== null && first >= 0.9 * biggest ? 'steps' : 'ramps';
  }

  return {
    FIRST_STEP,
    PID_COLUMNS,
    Channel,
    Log,
    parse,
    mergeRuns,
    fileIndex,
    lastAtOrBefore,
    firstAtOrAfter,
    motions,
    autons,
    phases,
    periods,
    sessions,
    tuneEvents,
    characterizationSegments,
    characterizationRuns,
    parseKeys,
  };
});
