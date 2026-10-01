// Tests for js/slt.js, the analyzer's SLT v1 reader. Run: node --test
// tools/analyzer/test/*.test.js
//
// The golden file is read out of tools/telemetry/slt_read.py (its
// SELFTEST_FILE, itself byte for byte the encoder's golden file in
// tests/telemetry/csv_format_test.cpp), and the checks mirror slt_read.py's
// own self-test, so the reference reader and this one read the same bytes
// the same way.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const slt = require('../js/slt.js');

function goldenFile() {
  // A Windows checkout (core.autocrlf) has CRLF line endings; the block is found by its LFs.
  const source = fs.readFileSync(
    path.join(__dirname, '..', '..', 'telemetry', 'slt_read.py'), 'utf8').replace(/\r\n/g, '\n');
  const block = source.slice(source.indexOf('SELFTEST_FILE = ('));
  const body = block.slice(0, block.indexOf('\n)\n'));
  const literals = body.match(/"((?:[^"\\]|\\.)*)"/g);
  return literals.map((l) => JSON.parse(l)).join('');
}

const GOLDEN = goldenFile();

test('golden file: header, channels, rows', () => {
  // A torn final line (power cut mid-write) must be discarded, not parsed.
  const log = slt.parse(GOLDEN + 'S,5,61000100,1,2');
  assert.equal(log.skipped, 0);
  assert.equal(log.meta.file, 'SL000042.CSV');
  assert.equal(log.meta.robot, '96671H');
  assert.equal(log.meta.build, 'Sep 27 2026 14:02:11');
  assert.deepEqual(log.channels.map((c) => c.name),
    ['sys', 'events', 'drive', 'turn', 'odom', 'batt', 'lift', 'lift.act']);
  assert.equal(log.lastUs, 61000044);

  const turn = log.get('turn');
  assert.equal(turn.kind, 'pid');
  assert.equal(turn.id, 3);
  assert.equal(turn.length, 5);
  assert.deepEqual(Array.from(slt.PID_COLUMNS, (c) => turn.cols[c][1]),
    [89.59, 0, 89.59, 31.3565, 0, -0.0082, 31.3483, 12, 0.01, 5]);
  assert.equal(turn.t[1], 15.013604);
  assert.deepEqual(turn.gains, [{ t: 15.003512, kP: 0.35, kI: 0, kD: 0.0002 }]);
  assert.deepEqual(turn.config, { integral_limit: 0, output_limit: 12, slew_rate: 0,
    derivative_on_measurement: 0, nominal_dt_s: 0.01 });
  assert.deepEqual(turn.resets, [16.871021]);
  assert.equal(turn.gainsAt(15.003511), null);
  assert.equal(turn.gainsAt(16.871027).kP, 0.35);

  assert.equal(log.get('lift').config.derivative_on_measurement, 1);
  assert.deepEqual(Array.from(log.get('lift.act').cols.volts), [12, 12]);
  assert.deepEqual(Array.from(log.get('odom').cols.heading), [0, 0.12, 91.87]);
  assert.deepEqual(log.get('events').dropped, [0, 1]);
  assert.equal(log.get('nope'), null);
});

test('golden file: events and health', () => {
  const log = slt.parse(GOLDEN);
  assert.equal(log.events.length, 9);
  assert.deepEqual(log.events[3], { t: 15.00339, tag: 'auton', msg: 'start,Turn Testing' });
  assert.deepEqual(log.health, [{ t: 3.104771, values: { rows: 4, bytes: 1034, writes: 1,
    wmax_us: 21873, wavg_us: 21873, drops: 0, unlogged: 2, resyncs: 0, breaks: 0, faults: 0,
    samp_us: 1840, fmt_us: 2615 } }]);
});

test('golden file: motions pair and cut short at the phase change', () => {
  const log = slt.parse(GOLDEN);
  const found = slt.motions(log);
  assert.equal(found.length, 2);
  const [first, second] = found;
  assert.equal(first.kind, 'turnToHeading');
  assert.deepEqual(first.params, { target_deg: 90, threshold: 2, settle_ms: 200, timeout_ms: 3000 });
  assert.deepEqual([first.reason, first.error, first.ms], ['settled', 0.412, 609]);
  assert.equal(second.end, null);
  assert.equal(second.stop, 30.001022);
});

test('motions: pairing across a phase change, with nesting and orphans', () => {
  const extra = slt.parse(
    '#SLT,1\n' +
    'E,5,motion,end,driveDistance,reason=timeout,error=3.000,ms=2000\n' +
    'E,10,motion,start,followPath,waypoints=3,lookahead_in=12.000,cruise_v=8.000,timeout_ms=0\n' +
    'E,20,phase,disabled,comp=0,field=0\n' +
    'E,30,motion,start,moveToPoint,x=1.000,y=2.000\n' +
    'E,40,motion,end,moveToPoint,reason=settled,error=0.500,ms=10\n' +
    'E,50,motion,end,followPath,reason=settled,error=0.500,ms=40\n' +
    'E,60,motion,start,turnToHeading\n' +
    'E,70,motion,end,turnToHeading,reason=aborted,error=0.000,ms=0\n');
  const paired = slt.motions(extra);
  assert.deepEqual(paired.map((m) => [m.kind, m.start * 1e6, m.end * 1e6, m.depth]),
    [['followPath', 10, 50, 0], ['moveToPoint', 30, 40, 1], ['turnToHeading', 60, 70, 0]]);
  assert.deepEqual(paired[2].params, {});
  assert.equal(paired[2].reason, 'aborted');
});

test('bad lines are counted and skipped; other versions refused', () => {
  const messy = slt.parse('#SLT,1\n#chan,2,x,samples,2,a,b\nS,2,1,1\nS,9,2,1,2\nG,2,3\nS,2,4,1,2\n' +
    'Q,future,row\n#future,directive\n');
  assert.equal(messy.skipped, 3);
  assert.deepEqual(Array.from(messy.get('x').cols.a), [1]);
  for (const bad of ['', '#SLT,2\n', 'time,value\n1,2\n']) {
    assert.throws(() => slt.parse(bad), /not an SLT v1/);
  }
  const specials = slt.parse('#SLT,1\n#chan,2,x,samples,2,a\nS,2,1,nan\nS,2,2,-inf\nS,2,3,1.5e+12\n');
  assert.ok(Number.isNaN(specials.get('x').cols.a[0]));
  assert.equal(specials.get('x').cols.a[1], -Infinity);
  assert.equal(specials.get('x').cols.a[2], 1.5e12);
});

test('time lookups', () => {
  const log = slt.parse('#SLT,1\n#chan,2,x,samples,2,a\nS,2,1000000,1\nS,2,2000000,2\nS,2,3000000,3\n');
  const x = log.get('x');
  assert.equal(x.valueAt('a', 0.5), NaN);
  assert.equal(x.valueAt('a', 2.5), 2);
  assert.equal(x.valueAt('a', 9), 3);
  assert.ok(Number.isNaN(x.valueAt('a', 9, 1)), 'too old');
  assert.deepEqual(x.range(1.5, 3), [1, 3]);
});

test('sessions: practice, a match, a skills run', () => {
  const log = slt.parse('#SLT,1\n' +
    'E,1000000,phase,opcontrol,comp=0,field=0\n' + // bench practice
    'E,60000000,phase,disabled,comp=1,field=1\n' +
    'E,70000000,phase,autonomous,comp=1,field=1\n' +
    'E,85000000,phase,disabled,comp=1,field=1\n' +
    'E,87000000,phase,opcontrol,comp=1,field=1\n' +
    'E,192000000,phase,disabled,comp=1,field=1\n' +
    'E,300000000,phase,autonomous,comp=1,field=0\n' +
    'E,360000000,phase,disabled,comp=1,field=0\n');
  const s = slt.sessions(log);
  assert.deepEqual(s.map((x) => [x.kind, x.start, x.end]),
    [['practice', 1, 60], ['match', 70, 192], ['auton', 300, 360]]);
  assert.deepEqual(s[1].auton, [70, 85]);
  assert.deepEqual(s[1].driver, [87, 192]);
});

test('characterization runs split at the runner\'s waits and drop the extra row', () => {
  let text = '#SLT,1\n#chan,9,char.lift,samples,4,volts,pos\n' +
    'E,500000,tune,start,Lift\n';
  let t = 1000000;
  const segment = (rows) => {
    for (const [v, p] of rows) {
      text += `S,9,${t},${v},${p}\n`;
      t += 10000;
    }
    t += 300000;
  };
  // Held pre-roll, then a ramp, then the NaN row at the out-of-range position.
  segment([['nan', 0], ['nan', 0], [0.5, 0], [1, 2], ['nan', 5]]);
  segment([['nan', 5], [-0.5, 5], [-1, 3], ['nan', 0]]);
  segment([['nan', 0], [6, 0], [6, 3], [6, 6]]); // cut by duration: no extra row
  segment([['nan', 6], [-4, 6], [-4, 2], ['nan', -1]]);
  text += 'E,9000000,tune,model,Lift,kS=0.6000,kV=0.01700,kA=0.00200,kG=2.0000,r2=0.990,' +
    'delay_ms=20\n';
  const log = slt.parse(text);
  const runs = slt.characterizationRuns(log, 'char.lift', 'Lift');
  assert.equal(runs.length, 1);
  const { data, reported } = runs[0];
  assert.deepEqual(data.ramps.map((r) => r.length), [4, 3]);
  assert.deepEqual(data.steps.map((r) => r.length), [4, 3]);
  assert.equal(data.ramps[0][1].timeMs, 10);
  assert.ok(Number.isNaN(data.ramps[0][0].volts));
  assert.equal(reported.kG, 2);
  assert.equal(reported.delayS, 0.02);
});

test('files split by an SD fault merge back into one run', () => {
  const first = slt.parse('#SLT,1\n#meta,file,SL000007.CSV\n#chan,2,x,samples,2,a\n' +
    'E,1000,file,open,SL000007.CSV\nS,2,2000,1\nS,2,3000,2\n', 'SL000007.CSV');
  const second = slt.parse('#SLT,1\n#meta,file,SL000008.CSV\n#chan,2,x,samples,2,a\n' +
    'E,9000,file,open,SL000008.CSV\nE,9000,sd,reopened,prev=SL000007.CSV,faults=1\n' +
    'S,2,9500,3\n', 'SL000008.CSV');
  const other = slt.parse('#SLT,1\n#meta,file,SL000009.CSV\n#chan,2,x,samples,2,a\nS,2,10,9\n',
    'SL000009.CSV');
  const runs = slt.mergeRuns([second, other, first]);
  assert.equal(runs.length, 2);
  assert.deepEqual(runs[0].files, ['SL000007.CSV', 'SL000008.CSV']);
  assert.deepEqual(Array.from(runs[0].get('x').cols.a), [1, 2, 3]);
  assert.equal(runs[1].files[0], 'SL000009.CSV');
});
