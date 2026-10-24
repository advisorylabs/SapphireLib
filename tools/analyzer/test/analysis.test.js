// Tests for js/analysis.js (troubleshooting findings) and js/demo.js (the
// simulated logs). Run: node --test tools/analyzer/test/*.test.js
//
// Small hand-written logs pin each rule down; the demo match is the
// integration test: a match with known failures planted in it, which the
// analysis has to find, at the right moments, and without false alarms.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const slt = require('../js/slt.js');
const A = require('../js/analysis.js');
const demo = require('../js/demo.js');

/** A log from rows, with a comp=1 match: auton 10-25s, driver 27-132s. */
function matchLog(body) {
  return slt.parse('#SLT,1\n' +
    'E,1000000,phase,disabled,comp=1,field=1\n' +
    'E,10000000,phase,autonomous,comp=1,field=1\n' +
    'E,25000000,phase,disabled,comp=1,field=1\n' +
    'E,27000000,phase,opcontrol,comp=1,field=1\n' +
    'E,132000000,phase,disabled,comp=1,field=1\n' + body);
}

function motorRows(id, from, to, row) {
  let text = '';
  for (let t = from; t < to; t += 0.1) {
    text += `S,${id},${Math.round(t * 1e6)},${row(t).join(',')}\n`;
  }
  return text;
}

const MOTOR = '#chan,8,motor.lift,samples,2,volts,amps,temp,rpm,eff,faults\n';

test('time labels read like a match', () => {
  const log = matchLog('');
  const sessions = slt.sessions(log);
  assert.equal(A.timeLabel(sessions, 17.5), 'Match 1, auton 0:07.5');
  assert.equal(A.timeLabel(sessions, 27 + 75.21), 'Match 1, driver 1:15.2');
  assert.equal(A.timeLabel(sessions, 500), 't=500.0s');
});

test('an overheating motor is critical at the moment it hit 55°C', () => {
  const log = matchLog(MOTOR + motorRows(8, 27, 132, (t) => {
    const temp = t < 100 ? 45 : t < 120 ? 55 : 60;
    return [8, 1.2, temp, 60, 40, temp >= 55 ? 1 : 0];
  }));
  const { findings, motors } = A.analyze(log);
  const hot = findings.find((f) => /overheated/.test(f.title));
  assert.ok(hot);
  assert.equal(hot.severity, 'critical');
  assert.ok(Math.abs(hot.t - 100) < 0.11);
  assert.match(hot.detail, /25% at its peak of 60°C/);
  assert.equal(motors[0].peakTemp, 60);
  assert.equal(A.reportedDerating(55), 0.5);
});

test('a motor that drops out mid-match is critical; one never plugged in is only info', () => {
  const log = matchLog(MOTOR + motorRows(8, 27, 132, (t) =>
    t > 60 && t < 62.5 ? ['nan', 'nan', 'nan', 'nan', 'nan', 'nan'] : [6, 0.5, 35, 120, 50, 0]) +
    '#chan,9,motor.spare,samples,2,volts,amps,temp,rpm,eff,faults\n' +
    motorRows(9, 27, 132, () => ['nan', 'nan', 'nan', 'nan', 'nan', 'nan']));
  const { findings } = A.analyze(log);
  const drop = findings.find((f) => f.title.startsWith('lift disconnected'));
  assert.ok(drop && drop.severity === 'critical');
  assert.ok(Math.abs(drop.t - 60.1) < 0.11 && Math.abs(drop.end - drop.t - 2.4) < 0.25);
  const never = findings.find((f) => f.title.startsWith('spare: never answered'));
  assert.ok(never && never.severity === 'info');
});

test('stalls: long, high-current, not turning', () => {
  const log = matchLog(MOTOR + motorRows(8, 27, 132, (t) =>
    t > 40 && t < 70 ? [12, 2.5, 40, 0, 0, 4] : [3, 0.2, 40, 60, 60, 0]));
  const stall = A.analyze(log).findings.find((f) => /stalled/.test(f.title));
  assert.ok(stall && stall.severity === 'warning');
  assert.match(stall.detail, /2\.5A/);
});

test('a controller that drops out while enabled is critical', () => {
  let rows = '#chan,5,driver,samples,3,lx,ly,rx,ry,buttons,connected\n';
  for (let t = 27; t < 132; t += 0.02) {
    const on = t < 90 || t > 91.2 ? 1 : 0;
    rows += `S,5,${Math.round(t * 1e6)},0,${on ? 0.5 : 0},0,0,0,${on}\n`;
  }
  const f = A.analyze(matchLog(rows)).findings.find((x) => /Controller disconnected/.test(x.title));
  assert.ok(f && f.severity === 'critical' && Math.abs(f.end - f.t - 1.2) < 0.05);
});

test('a log that ends while enabled in a match means the robot died', () => {
  const log = slt.parse('#SLT,1\nE,1000000,phase,autonomous,comp=1,field=1\n' +
    'E,9000000,phase,opcontrol,comp=1,field=1\nE,40000000,mark,x\n');
  const f = A.analyze(log).findings.find((x) => /Log ends mid-match/.test(x.title));
  assert.ok(f && f.severity === 'critical');
  // Practice (no competition control) just ends when someone turns it off.
  const bench = slt.parse('#SLT,1\nE,1000000,phase,opcontrol,comp=0,field=0\nE,9000000,mark,x\n');
  assert.ok(!A.analyze(bench).findings.some((x) => /Log ends/.test(x.title)));
});

test('a steady loop that dithers around zero is not "oscillating"', () => {
  let rows = '#chan,3,hold,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags\n';
  for (let i = 0; i < 3000; ++i) {
    const err = 0.05 * Math.sin(i);
    rows += `S,3,${27000000 + i * 20000},${err},0,${err},0,0,0,0,0,0.02,${i === 0 ? 8 : 0}\n`;
  }
  assert.ok(!A.analyze(matchLog(rows)).findings.some((f) => /oscillated/.test(f.title)));
});

// --- followTrajectory(): tracking against the plan ---------------------------------

/**
 * A match log with one followTrajectory() at 12s: 48in straight up the field (+y) at 24in/s
 * for 2s, then `holdS` settling on the end. `offset(t)` is where the robot is from the
 * reference [dx, dy], `paused(t)` whether the plan was waiting, `extraMs` added to the
 * motion's time, and `saturated(t)` sets trackFwd's flag.
 */
function trajectoryLog({ offset = () => [0, 0], paused = () => false, extraMs = 0, saturated = () => false,
  holdS = 0.2 } = {}) {
  const start = 12;
  let rows = '#chan,4,odom,samples,4,x,y,heading\n' +
    '#chan,6,traj.plan,samples,3,t_plan,x,y,hdg,v,sec,stop\n' +
    '#chan,7,traj,samples,3,ref_t,ref_x,ref_y,ref_hdg,ref_vf,ref_vs,ref_w,ff_f,ff_s,ff_t,lag,paused,sec\n' +
    '#chan,8,trackFwd,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags\n';
  const us = (t) => Math.round(t * 1e6);
  rows += `E,${us(start)},motion,start,followTrajectory,sections=1,samples=97,length_in=48.000,duration_ms=2000,` +
    'overtime_ms=1500\n';
  for (let k = 0; k <= 20; ++k) {
    rows += `S,6,${us(start) + 10},${k * 0.1},0,${k * 2.4},0,24,0,${k === 0 || k === 20 ? 1 : 0}\n`;
  }
  let refT = 0;
  const end = start + 2 + holdS + extraMs / 1000;
  for (let t = start; t < end - 1e-9; t += 0.01) {
    const waiting = paused(t);
    const holding = refT >= 2;
    const y = 24 * Math.min(refT, 2);
    const [dx, dy] = offset(t);
    rows += `S,7,${us(t) + 20},${refT},0,${y},0,${holding ? 0 : 24},0,0,0,0,0,${-dy},${waiting ? 1 : 0},0\n`;
    rows += `S,4,${us(t)},${dx},${y + dy},0\n`;
    rows += `S,8,${us(t) + 30},${-dy},0,${-dy},0,0,0,0,0,0.01,${saturated(t) ? 1 : 0}\n`;
    if (!waiting) refT += 0.01;
  }
  rows += `E,${us(end)},motion,end,followTrajectory,reason=settled,error=0.100,ms=${Math.round((end - start) * 1000)}\n`;
  return matchLog(rows);
}

test('followTrajectory: tracking stats against the plan, and no alarms when it tracked', () => {
  const report = A.analyze(trajectoryLog({ offset: () => [0.3, -0.4] }));
  const m = report.motions.find((x) => x.kind === 'followTrajectory');
  const k = m.tracking;
  assert.ok(Math.abs(k.worstIn - 0.5) < 1e-3 && Math.abs(k.rmsIn - 0.5) < 1e-3);
  assert.ok(Math.abs(k.plannedS - 2) < 1e-9 && Math.abs(k.actualS - 2.2) < 0.02);
  assert.equal(k.waitedS, 0);
  // heading north, so the robot 0.3in east is right of the path, and 0.4in behind
  const mid = Math.floor(k.series.t.length / 2);
  assert.ok(Math.abs(k.series.cross[mid] - 0.3) < 1e-3);
  assert.ok(Math.abs(k.series.along[mid] - 0.4) < 1e-3);
  assert.ok(!report.findings.some((f) => /followTrajectory/.test(f.title)),
    report.findings.map((f) => f.title).join('\n'));
  assert.ok(m.trajectory.plan && m.trajectory.plan.x.length === 21);
});

test('followTrajectory: far off its plan is a warning, saying whether the loops were maxed out', () => {
  const off = (t) => (t > 13 && t < 13.2 ? [4.5, 0] : [0, 0]);
  let f = A.analyze(trajectoryLog({ offset: off })).findings.find((x) => /fell .* off its plan/.test(x.title));
  assert.ok(f && f.severity === 'warning' && Math.abs(f.t - 13) < 0.05);
  assert.match(f.title, /fell 4\.5 in off/);
  assert.match(f.detail, /weren't saturated/);
  f = A.analyze(trajectoryLog({ offset: off, saturated: (t) => t > 12.9 && t < 13.2 })).findings
    .find((x) => /off its plan/.test(x.title));
  assert.match(f.detail, /at full power/);
  // settling on the end doesn't count: the plan is over
  const settling = A.analyze(trajectoryLog({ offset: (t) => (t > 14.05 ? [3.5, 0] : [0, 0]) }));
  assert.ok(!settling.findings.some((x) => /off its plan/.test(x.title)));
});

test('followTrajectory: waiting for the robot, and taking longer than planned, are info', () => {
  const waited = A.analyze(trajectoryLog({ paused: (t) => t > 12.5 && t < 13.2, offset: (t) =>
    (t > 12.5 && t < 13.2 ? [0, -2] : [0, 0]) }));
  const wait = waited.findings.find((x) => /waited/.test(x.title));
  assert.ok(wait && wait.severity === 'info');
  assert.match(wait.title, /waited 0\.7 s/);
  // the waiting explains the extra time: not also "slow"
  assert.ok(!waited.findings.some((x) => /longer than planned/.test(x.title)));

  const slow = A.analyze(trajectoryLog({ extraMs: 900 })).findings.find((x) => /longer than planned/.test(x.title));
  assert.ok(slow && slow.severity === 'info');
  assert.match(slow.title, /took 55% longer/);
  assert.ok(!A.analyze(trajectoryLog({ extraMs: 300 })).findings.some((x) => /longer than planned/.test(x.title)));
});

test('demo pit session: the Trajectory Test tracks its plan', () => {
  const pit = slt.parse(demo.pitSession(), 'SL000041.CSV');
  const [tr] = slt.trajectories(pit);
  assert.ok(tr && tr.plan && tr.rows && tr.motion.reason === 'settled');
  const k = A.trajectoryTracking(pit, tr);
  assert.ok(k.worstIn < 1.5 && k.rmsIn < 1, `worst ${k.worstIn}, RMS ${k.rmsIn}`);
});

// --- The demo match: planted failures, found ---------------------------------------

const MATCH = slt.parse(demo.match(), 'SL000042.CSV');
const REPORT = A.analyze(MATCH);
const titles = REPORT.findings.map((f) => `${f.severity} ${f.title}`);
const find = (re) => REPORT.findings.find((f) => re.test(f.title));

test('demo match: one match, parsed cleanly', () => {
  assert.equal(MATCH.skipped, 0);
  const sessions = slt.sessions(MATCH);
  assert.deepEqual(sessions.map((s) => [s.kind, s.auton, s.driver]),
    [['match', [20, 35], [37.5, 142.5]]]);
  assert.ok(MATCH.get('motor.liftA') && MATCH.get('driver') && MATCH.get('mech'));
});

test('demo match: the lift motors overheat, then the lift gets stuck at full power', () => {
  const liftA = find(/^liftA overheated/);
  const liftB = find(/^liftB overheated/);
  assert.ok(liftA && liftB, titles.join('\n'));
  assert.ok(liftA.t < liftB.t, 'motor A (the tight bearing) goes first');
  const stuck = find(/^lift stuck/);
  assert.ok(stuck && stuck.severity === 'critical', titles.join('\n'));
  assert.ok(stuck.t > liftA.t, 'stuck only after it derated');
  assert.ok(find(/^liftA lost \d+% of its speed per volt/));
});

test('demo match: the dropped cable, controller, and battery', () => {
  const br = find(/^br disconnected for 2\.\ds/);
  assert.ok(br && br.severity === 'critical', titles.join('\n'));
  assert.match(br.detail, /Back-right drive motor/, 'the device event folded in');
  assert.ok(!find(/Back-right drive motor unplugged/), 'reported once, not twice');
  assert.ok(Math.abs(br.t - (37.5 + 48)) < 0.2);
  assert.ok(find(/^Controller disconnected for 0\.8s/));
  assert.ok(find(/^Battery sagged/));
});

test('demo match: the claw stalls holding pieces, and the lift sags without feedforward', () => {
  assert.ok(find(/^claw stalled/));
  assert.ok(find(/^claw running hot/));
  const sag = find(/^lift settles [\d.]+ below its targets/);
  assert.ok(sag, titles.join('\n'));
  assert.ok(REPORT.mechanisms[0].sag > 15, 'about kG/kP with a heavy piece');
});

test('demo match: autonomous timeouts, with the blocked turn at full power', () => {
  const timeouts = REPORT.findings.filter((f) => /timed out/.test(f.title));
  assert.equal(timeouts.length, 3, titles.join('\n'));
  const blocked = timeouts.find((f) => /63\.\d° short/.test(f.title));
  assert.match(blocked.detail, /still at full power/);
  const small = timeouts.find((f) => /1\.6° short/.test(f.title));
  assert.match(small.detail, /wasn't saturated/);
  assert.ok(find(/^followPath cut short/));
});

test('demo match: no false alarms on the healthy parts', () => {
  for (const name of ['fl', 'fr', 'bl', 'ml', 'mr', 'intake']) {
    assert.ok(!REPORT.findings.some((f) => f.system === name), `${name}: ${titles.join('\n')}`);
  }
  assert.ok(!find(/oscillated/), titles.join('\n'));
  assert.ok(!find(/Log ends/));
  assert.ok(!find(/Odometry jumped/));
});

test('demo pit session: Auto-Tune runs on every axis', () => {
  const pit = slt.parse(demo.pitSession(), 'SL000041.CSV');
  assert.equal(pit.skipped, 0);
  const { starts, models } = slt.tuneEvents(pit);
  assert.deepEqual(starts.map((s) => s.axis), ['Fwd', 'Strafe', 'Turn', 'Lift']);
  assert.deepEqual(models.map((m) => m.axis), ['Fwd', 'Strafe', 'Turn', 'Lift']);
  for (const axis of ['fwd', 'strafe', 'turn', 'lift']) {
    const name = axis === 'fwd' ? 'Fwd' : axis[0].toUpperCase() + axis.slice(1);
    const runs = slt.characterizationRuns(pit, `char.${axis}`, name);
    assert.equal(runs.length, 1, axis);
    assert.equal(runs[0].data.ramps.length, 2, axis);
    assert.equal(runs[0].data.steps.length, 2, axis);
  }
  const report = A.analyze(pit);
  assert.ok(report.findings.some((f) => /Auto-Tune drove the lift/.test(f.title)));
  assert.ok(!report.findings.some((f) => f.severity === 'critical'),
    report.findings.map((f) => f.title).join('\n'));
});
