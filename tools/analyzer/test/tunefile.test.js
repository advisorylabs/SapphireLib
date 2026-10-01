// Tests for TUNE.CFG reading and writing (js/tunefile.js). Run:
// node --test tools/analyzer/test/*.test.js
//
// The robot's reader (src/sapphirelib/tuning/tune_profile.cpp) and this one
// must agree on every file, so both parse tests/tuning/tune_profile_golden.cfg
// and check the same values, and LOCALIZER_SETTINGS is checked against the
// C++ table it mirrors.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('fs');
const path = require('path');
const TF = require('../js/tunefile.js');

const repo = path.join(__dirname, '..', '..', '..');
const GOLDEN = fs.readFileSync(path.join(repo, 'tests', 'tuning', 'tune_profile_golden.cfg'), 'utf8');

test('golden file: the same values tune_profile_test.cpp checks', () => {
  const r = TF.parse(GOLDEN);
  assert.ok(r.ok, r.error);
  const p = r.profile;
  assert.equal(p.revision, 7);
  assert.equal(p.note, 'turn refit from SL000041-SL000048; MCL from the sim tuner');
  assert.deepEqual(p.pids.drive, { kP: 18.2, kI: 0, kD: 0.71 });
  assert.deepEqual(p.pids.turn, { kP: 0.42, kI: 0, kD: 0.031 });
  assert.equal(p.pids.hold, undefined);
  assert.deepEqual(p.models.fwd, { kS: 1.02, kV: 0.198, kA: 0.047 });
  assert.equal(p.models.turn.kA, 0.0066);
  assert.equal(p.values['lift.gravityVolts'], 1.35);
  assert.equal(p.values['mcl.periodMs'], 50);
  assert.deepEqual(p.mcl, {
    'filter.particleCount': 400, 'filter.motionNoise.perInch': 0.07,
    'filter.beam.outlierProbability': 0.15, 'filter.recovery.enabled': 1, sensorLatencyMs: 42.5,
  });
  const settings = TF.localizerSettings(p);
  assert.equal(settings['filter.particleCount'], 400);
  assert.equal(settings.maxCorrectionRateInPerS, 4);
  assert.equal(settings.periodMs, 50);
});

test('LOCALIZER_SETTINGS mirrors the C++ table, key for key and range for range', () => {
  const cpp = fs.readFileSync(path.join(repo, 'src', 'sapphirelib', 'localization', 'localizer_config.cpp'), 'utf8');
  const rows = [...cpp.matchAll(/\{\{"([\w.]+)", ([-\d.e]+), ([-\d.e]+), (true|false)\}/g)]
    .map((m) => ({ key: m[1], min: Number(m[2]), max: Number(m[3]), whole: m[4] === 'true' }));
  assert.equal(rows.length, 26);
  assert.deepEqual(TF.LOCALIZER_SETTINGS.map(({ key, min, max, whole }) => ({ key, min, max, whole })), rows);
  for (const s of TF.LOCALIZER_SETTINGS) {
    assert.ok(s.default >= s.min && s.default <= s.max, `${s.key} default in range`);
  }
});

test('the same errors as the robot, on the same lines', () => {
  const schema = { pids: ['turn'], models: [], values: [] };
  const cases = [
    ['rev=1\nformat=1\n', 1, 'the first setting must be format=1'],
    ['format=2\n', 1, 'format 2 is newer than this program reads'],
    ['format=one\n', 1, 'format: needs a version number'],
    ['# nothing but comments\n\n', 0, 'no format=1 line'],
    ['', 0, 'no format=1 line'],
    ['format=1\npid.turn=1,0,0\nbogus\n', 3, 'expected key=value'],
    ['format=1\n=3\n', 2, 'missing key'],
    ['format=1\npid.turn=1,0,0\npid.turn=2,0,0\n', 3, 'pid.turn: set twice'],
    ['format=1\nformat=1\n', 2, 'format: set twice'],
    ['format=1\npid.drive=1,0,0\n', 2, 'pid.drive: no such controller'],
    ['format=1\npid.turn=1,0\n', 2, 'pid.turn: needs kP,kI,kD'],
    ['format=1\npid.turn=1,0,0,4\n', 2, 'pid.turn: needs kP,kI,kD'],
    ['format=1\npid.turn=1,,0\n', 2, 'pid.turn: needs kP,kI,kD'],
    ['format=1\npid.turn=1,nan,0\n', 2, 'pid.turn: needs kP,kI,kD'],
    ['format=1\npid.turn=1,0,-0.1\n', 2, 'pid.turn: gains can\'t be negative'],
    ['format=1\nmodel.fwd=1,0.2,0.04\n', 2, 'model.fwd: no such axis'],
    ['format=1\nrev=1.5\n', 2, 'rev: needs a whole number'],
    ['format=1\nspeed=11\n', 2, 'speed: unknown setting'],
    ['format=1\nmcl.filter.particleCount=2\n', 2, 'mcl.filter.particleCount: out of range'],
    ['format=1\nmcl.filter.particleCount=300.5\n', 2, 'mcl.filter.particleCount: must be a whole number'],
    ['format=1\nmcl.filter.warp=1\n', 2, 'mcl.filter.warp: unknown setting'],
    ['format=1\nmcl.sensorLatencyMs=fast\n', 2, 'mcl.sensorLatencyMs: needs a number'],
    ['format=1\nmcl.correctOdometry=2\n', 2, 'mcl.correctOdometry: out of range'],
  ];
  for (const [text, line, error] of cases) {
    const r = TF.parse(text, schema);
    assert.equal(r.ok, false, text);
    assert.equal(r.errorLine, line, text);
    assert.equal(r.error, error, text);
  }
  const zero = TF.parse('format=1\nmodel.fwd=1,0,0.04\n', { pids: [], models: ['fwd'], values: [] });
  assert.equal(zero.error, 'model.fwd: kS can\'t be negative, kV and kA must be positive');
  const big = TF.parse(`format=1\n${'#'.repeat(TF.MAX_BYTES)}`);
  assert.equal(big.error, 'file is too big');
  assert.equal(TF.parse('format=1\nmcl.periodMs=50.5\n').error, 'mcl.periodMs: must be a whole number');
  // Windows line endings and inline comments, like the C++ test
  const crlf = TF.parse('\r\n# hand edited\r\nformat=1\r\n  pid.turn = 1 , 0 , 0.5   # comment\r\n');
  assert.ok(crlf.ok);
  assert.equal(crlf.profile.pids.turn.kD, 0.5);
});

test('what format() writes parses back to the same profile, changelog included', () => {
  const p = TF.parse(GOLDEN).profile;
  p.changelog = ['r7 2026-09-30 analyzer: pid.turn 0.35,0,0.0002 -> 0.42,0,0.031'];
  const text = TF.format(p);
  const back = TF.parse(text);
  assert.ok(back.ok, back.error);
  assert.deepEqual(back.profile, p);
  // settings in a fixed order, sections commented
  assert.ok(text.indexOf('pid.drive=18.2,0,0.71') < text.indexOf('model.fwd='));
  assert.ok(text.indexOf('mcl.periodMs=50') < text.indexOf('mcl.filter.particleCount=400'));
  assert.match(text, /^# --- changelog, newest first ---$/m);
});

test('revise() bumps the revision, logs what changed, and keeps everything else', () => {
  const start = TF.parse(GOLDEN).profile;
  const date = new Date('2026-10-02T12:00:00Z');
  const r = TF.revise(start, [
    { key: 'mcl.sensorLatencyMs', value: 45 },
    { key: 'pid.hold', value: { kP: 0.2, kI: 0, kD: 0.012 } },
    { key: 'mcl.filter.particleCount', value: 400 }, // unchanged: left out of the log
  ], { source: 'sim tuner', date });
  assert.equal(r.error, null);
  assert.equal(r.profile.revision, 8);
  assert.deepEqual(r.changes, [
    { key: 'pid.hold', from: '', to: '0.2,0,0.012' },
    { key: 'mcl.sensorLatencyMs', from: '42.5', to: '45' },
  ]);
  assert.equal(r.profile.changelog[0],
    'r8 2026-10-02 sim tuner: pid.hold code -> 0.2,0,0.012; mcl.sensorLatencyMs 42.5 -> 45');
  assert.deepEqual(r.profile.pids.drive, start.pids.drive);
  assert.equal(start.revision, 7, 'the original is untouched');
  // the robot reads what was written
  assert.ok(TF.parse(TF.format(r.profile)).ok);

  const same = TF.revise(start, [{ key: 'mcl.sensorLatencyMs', value: 42.5 }]);
  assert.equal(same.profile.revision, 7);
  assert.equal(same.changes.length, 0);

  const bad = TF.revise(start, [{ key: 'mcl.filter.particleCount', value: 3 }]);
  assert.equal(bad.error, 'mcl.filter.particleCount: out of range');

  const removed = TF.revise(start, [{ key: 'model.turn', value: null }], { date });
  assert.equal(removed.profile.models.turn, undefined);
  assert.match(removed.profile.changelog[0], /model\.turn 0\.61,0\.0452,0\.0066 -> code$/);
});

test('a long history is trimmed to fit what the robot reads', () => {
  let p = TF.emptyProfile();
  for (let i = 0; i < 200; ++i) {
    p = TF.revise(p, [{ key: 'mcl.sensorLatencyMs', value: 20 + (i % 50) + 0.123456 }],
      { source: 'a tuner with a long name that writes a lot', date: new Date('2026-10-02T00:00:00Z') }).profile;
  }
  const text = TF.format(p);
  assert.ok(text.length < TF.MAX_BYTES);
  assert.ok(TF.parse(text).ok);
  assert.ok(p.changelog.length <= 40);
  assert.match(p.changelog[0], /^r200 /);
});
