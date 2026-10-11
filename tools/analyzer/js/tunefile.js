/*
 * SapphireLib telemetry analyzer: tunefile.js
 *
 * TUNE.CFG, the tuning profile the robot loads from its SD card at startup
 * (src/robot/tune.cpp), read and written here so the tools can hand their
 * results straight to the robot: the simulator's MCL tuner writes mcl.*
 * lines, the analyzer's PID refinement writes pid.* and model.* lines, and
 * each keeps the other's.
 *
 * The rules are the robot's (src/sapphirelib/tuning/tune_profile.cpp), down
 * to the error messages: a file this module writes is one the robot accepts,
 * and a file the robot would reject is rejected here with the same line and
 * reason. test/tunefile.test.js holds both to tests/tuning/
 * tune_profile_golden.cfg, and LOCALIZER_SETTINGS to the C++ table it
 * mirrors.
 *
 * On top of the robot's reader: a changelog. Comments after the
 * "# --- changelog" line are the file's history, one line per revision
 * ("# r8 2026-10-02 sim tuner: mcl.filter.motionNoise.perInch 0.05 -> 0.07"),
 * newest first, kept by every tool that rewrites the file. The robot skips
 * them like any comment.
 *
 * Plain script: window.SA.tunefile in a browser, require('./tunefile.js') in
 * Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.tunefile = factory();
  }
})(function () {
  'use strict';

  const FORMAT_VERSION = 1;
  const MAX_BYTES = 32 * 1024;
  const MAX_NOTE_CHARS = 160;
  const CHANGELOG_MARK = '# --- changelog, newest first ---';
  const MAX_CHANGELOG = 40;
  const NUMBER = /^[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?$/;

  /**
   * Every LocalizerConfig field, by path: kFields in
   * src/sapphirelib/localization/localizer_config.cpp (key, range, whole),
   * plus the field's default in LocalizerConfig and ParticleFilterConfig.
   */
  const LOCALIZER_SETTINGS = [
    { key: 'filter.particleCount', min: 10, max: 5000, whole: true, default: 300 },
    { key: 'filter.motionNoise.baseIn', min: 0, max: 2, whole: false, default: 0.02 },
    { key: 'filter.motionNoise.perInch', min: 0, max: 1, whole: false, default: 0.05 },
    { key: 'filter.motionNoise.perDegreeIn', min: 0, max: 0.5, whole: false, default: 0.005 },
    { key: 'filter.beam.maxRangeIn', min: 1, max: 200, whole: false, default: 78 },
    { key: 'filter.beam.minSigmaIn', min: 0.01, max: 20, whole: false, default: 0.6 },
    { key: 'filter.beam.sigmaFraction', min: 0, max: 1, whole: false, default: 0.05 },
    { key: 'filter.beam.outlierProbability', min: 0.001, max: 0.99, whole: false, default: 0.1 },
    { key: 'filter.resampleThreshold', min: 0, max: 1, whole: false, default: 0.5 },
    { key: 'filter.recovery.enabled', min: 0, max: 1, whole: true, default: 1 },
    { key: 'filter.recovery.slowRate', min: 0, max: 1, whole: false, default: 0.01 },
    { key: 'filter.recovery.fastRate', min: 0, max: 1, whole: false, default: 0.2 },
    { key: 'filter.recovery.triggerRatio', min: 0, max: 1, whole: false, default: 0.8 },
    { key: 'filter.recovery.radiusIn', min: 0, max: 200, whole: false, default: 18 },
    { key: 'filter.recovery.maxFraction', min: 0, max: 1, whole: false, default: 0.1 },
    { key: 'filter.seed', min: 0, max: 4294967295, whole: true, default: 1 },
    { key: 'startSpreadIn', min: 0, max: 72, whole: false, default: 2 },
    { key: 'sensorLatencyMs', min: 0, max: 500, whole: false, default: 30 },
    { key: 'minConfidence', min: 0, max: 63, whole: true, default: 20 },
    { key: 'maxTurnRateDegPerS', min: 0, max: 5000, whole: false, default: 200 },
    { key: 'correctOdometry', min: 0, max: 1, whole: true, default: 1 },
    { key: 'waitForSetPose', min: 0, max: 1, whole: true, default: 1 },
    { key: 'maxCorrectionSpreadIn', min: 0, max: 100, whole: false, default: 3 },
    { key: 'minAgreeingSensors', min: 0, max: 16, whole: true, default: 2 },
    { key: 'agreementSigmas', min: 0, max: 100, whole: false, default: 3 },
    { key: 'maxCorrectionRateInPerS', min: 0, max: 1000, whole: false, default: 4 },
  ];

  /** The localizer's update period: not a LocalizerConfig field, so the robot names it itself. */
  const DEFAULT_PERIOD_MS = 50;

  /** What 96671H's robot accepts besides mcl.*: schema() in src/robot/tune.cpp. */
  const ROBOT_SCHEMA = {
    pids: ['drive', 'turn', 'hold', 'lift'],
    models: ['fwd', 'strafe', 'turn'],
    values: [
      { key: 'lift.gravityVolts', min: -12, max: 12, whole: false },
      { key: 'mcl.periodMs', min: 10, max: 1000, whole: true },
    ],
  };

  function findSetting(path) {
    return LOCALIZER_SETTINGS.find((s) => s.key === path) || null;
  }

  /** setLocalizerSetting()'s checks: null if fine, else the C++'s settingErrorText(). */
  function settingError(path, value) {
    const setting = findSetting(path);
    if (!setting) return 'unknown setting';
    if (!Number.isFinite(value)) return 'not a number';
    if (value < setting.min || value > setting.max) return 'out of range';
    if (setting.whole && value !== Math.floor(value)) return 'must be a whole number';
    return null;
  }

  /** An empty profile: what a robot with no TUNE.CFG runs. */
  function emptyProfile() {
    return { revision: 0, note: '', pids: {}, models: {}, values: {}, mcl: {}, changelog: [] };
  }

  function parseNumber(text) {
    const t = text.trim();
    return NUMBER.test(t) ? Number(t) : NaN;
  }

  function parseNumbers(text, count) {
    const parts = text.split(',');
    if (parts.length !== count) return null;
    const out = parts.map(parseNumber);
    return out.every(Number.isFinite) ? out : null;
  }

  /**
   * parseTuneProfile(): { ok, profile, errorLine, error }. The profile also
   * carries the file's changelog lines, which the robot ignores.
   */
  function parse(text, schema = ROBOT_SCHEMA) {
    const fail = (errorLine, error) => ({ ok: false, profile: null, errorLine, error });
    if (text.length > MAX_BYTES) return fail(0, 'file is too big');
    const profile = emptyProfile();
    const seen = new Set();
    let sawFormat = false;
    let inChangelog = false;
    const lines = text.split('\n');
    // a final newline leaves an empty last piece, which is no line at all
    if (lines.length && lines[lines.length - 1] === '') lines.pop();

    for (let i = 0; i < lines.length; ++i) {
      const lineNumber = i + 1;
      let line = lines[i];
      const hash = line.indexOf('#');
      if (hash >= 0) {
        const comment = line.slice(hash).replace(/[\r\t ]+$/, '');
        if (comment === CHANGELOG_MARK) inChangelog = true;
        else if (inChangelog && /^# r\d+\b/.test(comment)) profile.changelog.push(comment.slice(2));
        line = line.slice(0, hash);
      }
      line = line.replace(/^[ \t\r]+|[ \t\r]+$/g, '');
      if (!line) continue;

      const eq = line.indexOf('=');
      if (eq < 0) return fail(lineNumber, 'expected key=value');
      const key = line.slice(0, eq).replace(/^[ \t\r]+|[ \t\r]+$/g, '');
      const value = line.slice(eq + 1).replace(/^[ \t\r]+|[ \t\r]+$/g, '');
      if (!key) return fail(lineNumber, 'missing key');
      if (seen.has(key)) return fail(lineNumber, `${key}: set twice`);
      seen.add(key);

      if (!sawFormat) {
        if (key !== 'format') return fail(lineNumber, 'the first setting must be format=1');
        const version = parseNumber(value);
        if (!Number.isFinite(version) || version !== Math.floor(version) || version < 1) {
          return fail(lineNumber, 'format: needs a version number');
        }
        if (version !== FORMAT_VERSION) {
          return fail(lineNumber, `format ${version} is newer than this program reads`);
        }
        sawFormat = true;
        continue;
      }

      if (key === 'rev') {
        const rev = parseNumber(value);
        if (!Number.isFinite(rev) || rev !== Math.floor(rev) || rev < 0 || rev > 1e9) {
          return fail(lineNumber, 'rev: needs a whole number');
        }
        profile.revision = rev;
      } else if (key === 'note') {
        if (value.length > MAX_NOTE_CHARS) return fail(lineNumber, 'note: too long');
        profile.note = value;
      } else if (schema.values.some((v) => v.key === key)) {
        const known = schema.values.find((v) => v.key === key);
        const number = parseNumber(value);
        if (!Number.isFinite(number)) return fail(lineNumber, `${key}: needs a number`);
        if (number < known.min || number > known.max) return fail(lineNumber, `${key}: out of range`);
        if (known.whole && number !== Math.floor(number)) {
          return fail(lineNumber, `${key}: must be a whole number`);
        }
        profile.values[key] = number;
      } else if (key.startsWith('pid.')) {
        const name = key.slice(4);
        if (!schema.pids.includes(name)) return fail(lineNumber, `${key}: no such controller`);
        const g = parseNumbers(value, 3);
        if (!g) return fail(lineNumber, `${key}: needs kP,kI,kD`);
        if (g.some((x) => x < 0)) return fail(lineNumber, `${key}: gains can't be negative`);
        profile.pids[name] = { kP: g[0], kI: g[1], kD: g[2] };
      } else if (key.startsWith('model.')) {
        const name = key.slice(6);
        if (!schema.models.includes(name)) return fail(lineNumber, `${key}: no such axis`);
        const m = parseNumbers(value, 3);
        if (!m) return fail(lineNumber, `${key}: needs kS,kV,kA`);
        if (m[0] < 0 || !(m[1] > 0 && m[2] > 0)) {
          return fail(lineNumber, `${key}: kS can't be negative, kV and kA must be positive`);
        }
        profile.models[name] = { kS: m[0], kV: m[1], kA: m[2] };
      } else if (key.startsWith('mcl.')) {
        const path = key.slice(4);
        const number = parseNumber(value);
        if (!Number.isFinite(number)) return fail(lineNumber, `${key}: needs a number`);
        const error = settingError(path, number);
        if (error) return fail(lineNumber, `${key}: ${error}`);
        profile.mcl[path] = number;
      } else {
        return fail(lineNumber, `${key}: unknown setting`);
      }
    }
    if (!sawFormat) return fail(0, 'no format=1 line');
    return { ok: true, profile, errorLine: 0, error: '' };
  }

  // --- Writing -----------------------------------------------------------------------

  /** Six significant digits, no trailing zeros: what the tools write. */
  function num(value) {
    if (Number.isInteger(value)) return String(value);
    return String(Number(value.toPrecision(6)));
  }

  function gainsText(g) {
    return `${num(g.kP)},${num(g.kI)},${num(g.kD)}`;
  }

  function modelText(m) {
    return `${num(m.kS)},${num(m.kV)},${num(m.kA)}`;
  }

  /**
   * Every setting as full key → the text it's written as, in the order
   * format() writes them.
   */
  function entries(profile, schema = ROBOT_SCHEMA) {
    const out = new Map();
    for (const name of schema.pids) if (profile.pids[name]) out.set(`pid.${name}`, gainsText(profile.pids[name]));
    for (const name of schema.models) {
      if (profile.models[name]) out.set(`model.${name}`, modelText(profile.models[name]));
    }
    for (const v of schema.values) {
      if (v.key in profile.values) out.set(v.key, num(profile.values[v.key]));
    }
    for (const s of LOCALIZER_SETTINGS) if (s.key in profile.mcl) out.set(`mcl.${s.key}`, num(profile.mcl[s.key]));
    return out;
  }

  /** The file's text: the robot reads every setting, the tools the changelog too. */
  function format(profile, schema = ROBOT_SCHEMA) {
    const lines = [
      '# SapphireLib tuning profile: the robot loads it from the SD card at startup',
      '# (/usd/sl/TUNE.CFG). Written by the SapphireLib tools; safe to edit by hand.',
      '# One key=value per line; # starts a comment. One bad line and the robot uses',
      '# none of it, and says so on its Home page. See docs/TUNING.md.',
      `format=${FORMAT_VERSION}`,
      `rev=${profile.revision}`,
    ];
    if (profile.note) lines.push(`note=${profile.note.replace(/[#\r\n]/g, ' ').slice(0, MAX_NOTE_CHARS)}`);
    const all = entries(profile, schema);
    const section = (title, prefix) => {
      const keys = [...all.keys()].filter(prefix);
      if (!keys.length) return;
      lines.push('', title);
      for (const k of keys) lines.push(`${k}=${all.get(k)}`);
    };
    section('# Controllers: kP,kI,kD', (k) => k.startsWith('pid.'));
    section('# Axis models: kS,kV,kA', (k) => k.startsWith('model.'));
    section('# Mechanisms', (k) => !k.startsWith('pid.') && !k.startsWith('model.') && !k.startsWith('mcl.'));
    section('# Localizer: LocalizerConfig fields, and its update period', (k) => k.startsWith('mcl.'));
    if (profile.changelog.length) {
      lines.push('', CHANGELOG_MARK);
      for (const entry of profile.changelog.slice(0, MAX_CHANGELOG)) lines.push(`# ${entry}`);
    }
    return `${lines.join('\n')}\n`;
  }

  // --- Changing a profile ------------------------------------------------------------

  function copy(profile) {
    return {
      revision: profile.revision,
      note: profile.note,
      pids: Object.fromEntries(Object.entries(profile.pids).map(([k, v]) => [k, { ...v }])),
      models: Object.fromEntries(Object.entries(profile.models).map(([k, v]) => [k, { ...v }])),
      values: { ...profile.values },
      mcl: { ...profile.mcl },
      changelog: profile.changelog.slice(),
    };
  }

  /**
   * Sets one setting by full key: 'pid.turn' takes { kP, kI, kD },
   * 'model.fwd' { kS, kV, kA }, anything else a number. null removes it (the
   * robot falls back to the code's value). Returns an error, or null.
   */
  function set(profile, key, value, schema = ROBOT_SCHEMA) {
    if (key.startsWith('pid.')) {
      const name = key.slice(4);
      if (!schema.pids.includes(name)) return `${key}: no such controller`;
      if (value === null) delete profile.pids[name];
      else profile.pids[name] = { kP: value.kP, kI: value.kI, kD: value.kD };
      return null;
    }
    if (key.startsWith('model.')) {
      const name = key.slice(6);
      if (!schema.models.includes(name)) return `${key}: no such axis`;
      if (value === null) delete profile.models[name];
      else profile.models[name] = { kS: value.kS, kV: value.kV, kA: value.kA };
      return null;
    }
    const known = schema.values.find((v) => v.key === key);
    if (known) {
      if (value === null) delete profile.values[key];
      else profile.values[key] = value;
      return null;
    }
    if (key.startsWith('mcl.')) {
      const path = key.slice(4);
      if (value === null) {
        delete profile.mcl[path];
        return null;
      }
      const error = settingError(path, value);
      if (error) return `${key}: ${error}`;
      profile.mcl[path] = value;
      return null;
    }
    return `${key}: unknown setting`;
  }

  /** Where a key goes in the file, for listing keys in the order format() writes them. */
  function keyOrder(key, schema = ROBOT_SCHEMA) {
    const at = (list, item, base) => (list.indexOf(item) >= 0 ? base + list.indexOf(item) : Infinity);
    if (key.startsWith('pid.')) return at(schema.pids, key.slice(4), 0);
    if (key.startsWith('model.')) return at(schema.models, key.slice(6), 100);
    const value = schema.values.findIndex((v) => v.key === key);
    if (value >= 0) return 200 + value;
    return at(LOCALIZER_SETTINGS.map((s) => s.key), key.slice(4), 300);
  }

  /** What changed from `a` to `b`: [{ key, from, to }] with the written text ('' if unset). */
  function diff(a, b, schema = ROBOT_SCHEMA) {
    const before = entries(a, schema);
    const after = entries(b, schema);
    const keys = [...new Set([...before.keys(), ...after.keys()])]
      .sort((x, y) => keyOrder(x, schema) - keyOrder(y, schema));
    const out = [];
    for (const key of keys) {
      const from = before.get(key) || '';
      const to = after.get(key) || '';
      if (from !== to) out.push({ key, from, to });
    }
    return out;
  }

  /**
   * The next revision of `profile` with `changes` ([{ key, value }]) applied,
   * and a changelog line saying who changed what: "r8 2026-10-02 sim tuner:
   * mcl.sensorLatencyMs 30 -> 45; ...". Returns { profile, changes: diff,
   * error }. A change that doesn't change anything is left out; with none at
   * all, the revision stays.
   */
  function revise(profile, changes, { source = 'SapphireLib', date = new Date(), note = null } = {},
    schema = ROBOT_SCHEMA) {
    const next = copy(profile);
    for (const { key, value } of changes) {
      const error = set(next, key, value, schema);
      if (error) return { profile, changes: [], error };
    }
    const changed = diff(profile, next, schema);
    if (!changed.length) return { profile, changes: [], error: null };
    next.revision = profile.revision + 1;
    if (note !== null) next.note = note;
    const day = date.toISOString().slice(0, 10);
    const what = changed.map((c) => `${c.key} ${c.from || 'code'} -> ${c.to || 'code'}`).join('; ');
    next.changelog.unshift(`r${next.revision} ${day} ${source}: ${what}`);
    next.changelog = next.changelog.slice(0, MAX_CHANGELOG);
    // the robot reads at most MAX_BYTES; the oldest history goes first
    while (format(next, schema).length > MAX_BYTES - 1024 && next.changelog.length > 1) next.changelog.pop();
    return { profile: next, changes: changed, error: null };
  }

  /** The profile's mcl.* lines over the defaults, as { path: value }, plus periodMs. */
  function localizerSettings(profile) {
    const out = {};
    for (const s of LOCALIZER_SETTINGS) out[s.key] = s.key in profile.mcl ? profile.mcl[s.key] : s.default;
    out.periodMs = 'mcl.periodMs' in profile.values ? profile.values['mcl.periodMs'] : DEFAULT_PERIOD_MS;
    return out;
  }

  return {
    FORMAT_VERSION,
    MAX_BYTES,
    CHANGELOG_MARK,
    LOCALIZER_SETTINGS,
    DEFAULT_PERIOD_MS,
    ROBOT_SCHEMA,
    findSetting,
    settingError,
    emptyProfile,
    parse,
    format,
    entries,
    copy,
    set,
    diff,
    revise,
    localizerSettings,
    num,
  };
});
