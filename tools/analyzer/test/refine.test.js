// Tests for refining PIDs from their logged step responses (js/refine.js).
// Run: node --test tools/analyzer/test/*.test.js
//
// The logs here are built from a robot whose true dynamics are known: its
// turn axis really has more inertia and more delay than the Auto-Tune model
// its gains were designed from. Refining has to find that, and propose gains
// that do better on the same steps.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const M = require('../js/model.js');
const D = require('../js/demo.js');
const slt = require('../js/slt.js');
const RF = require('../js/refine.js');
const TF = require('../js/tunefile.js');

const MEASURED = { kS: 0.6, kV: 0.045, kA: 0.0065 };
const MEASURED_DELAY = 0.03;
const TURN = RF.CONTROLLERS.find((c) => c.pid === 'turn');

/**
 * A log of turnToHeading() steps on a turn axis with `trueModel` and
 * `trueDelay`, with gains designed (as the robot would) from the Auto-Tune
 * measurement MEASURED, unless `gains` is given. Logged the way the robot
 * logs it: C/G/S/R rows on "turn", motion events, a tune,model event.
 */
function turnLog({ trueModel, trueDelay, gains = null, fileIndex = 1, rev = null, steps = [30, 0, -45, 15, 90, 0, -20],
  autoTune = true }) {
  const file = `SL${String(fileIndex).padStart(6, '0')}.CSV`;
  const w = new D.Writer({ file, robot: 'test', openUs: 1e6, build: 'test' });
  if (rev !== null) w.meta.push(['tune', 'loaded'], ['tune.rev', String(rev)]);
  const id = w.chan('turn', 'pid', 4, slt.PID_COLUMNS);
  const designed = M.designPositionGains(M.feedforward(MEASURED.kS, MEASURED.kV, MEASURED.kA), TURN.spec, MEASURED_DELAY);
  const pid = new D.LoggedPID(w, id, M.pidConfig({ gains: gains || designed.gains, outputLimit: 12, nominalDtS: 0.01 }));
  w.event(1e6, 'phase', 'autonomous,comp=0,field=0');
  if (autoTune) {
    w.event(1.1e6, 'tune', `model,Turn,kS=${MEASURED.kS},kV=${MEASURED.kV},kA=${MEASURED.kA},r2=0.990,delay_ms=${MEASURED_DELAY * 1000}`);
  }
  const plant = new M.PlantSim(M.mechanismModel(M.feedforward(trueModel.kS, trueModel.kV, trueModel.kA)), trueDelay);
  let t = 2;
  for (const target of steps) {
    pid.reset(t);
    w.event(t * 1e6, 'motion', `start,turnToHeading,target_deg=${target}.000,threshold=2.000,settle_ms=200,timeout_ms=3000`);
    let inside = 0;
    let elapsed = 0;
    let reason = 'timeout';
    for (; elapsed < 3; elapsed += 0.01) {
      const error = target - plant.x;
      const out = pid.update(t, error, 0);
      plant.command(out);
      plant.advance(0.01);
      t += 0.01;
      inside = Math.abs(error) <= 2 ? inside + 0.01 : 0;
      if (inside >= 0.2) {
        reason = 'settled';
        break;
      }
    }
    w.event(t * 1e6, 'motion', `end,turnToHeading,reason=${reason},error=${Math.abs(target - plant.x).toFixed(3)},ms=${Math.round(elapsed * 1000)}`);
    plant.hold();
    for (let k = 0; k < 50; ++k) plant.advance(0.01);
    t += 0.5;
  }
  w.health(t * 1e6);
  return slt.parse(w.toString(D.mulberry32(fileIndex)), file);
}

test('a heavier, laggier turn than Auto-Tune measured is found, and the redesign does better', () => {
  const trueModel = { kS: 0.6, kV: 0.045, kA: 0.0065 * 1.5 };
  const logs = [turnLog({ trueModel, trueDelay: 0.075, fileIndex: 1 }), turnLog({ trueModel, trueDelay: 0.075, fileIndex: 2 })];
  const r = RF.refine(logs, TURN);
  assert.ok(r.ok, r.why);
  assert.match(r.base.source, /Auto-Tune on the robot/);
  assert.ok(r.responses.length >= 6, `${r.responses.length} responses`);
  assert.ok(Math.abs(r.identified.delayS - 0.075) < 0.025, `delay ${r.identified.delayS}`);
  assert.ok(Math.abs(r.identified.kAScale - 1.5) < 0.35, `kA ×${r.identified.kAScale}`);
  assert.ok(r.identified.rms < 0.5 * r.identified.baseRms, `mismatch ${r.identified.baseRms} → ${r.identified.rms}`);
  // the recorded steps overshoot; the design had none
  assert.ok(r.measured.overshootFraction > 0.08, `measured overshoot ${r.measured.overshootFraction}`);
  // the model the gains came from promised less
  assert.ok(r.predicted.promised.overshootFraction < r.measured.overshootFraction - 0.05);
  assert.equal(r.change, 'retune');
  assert.ok(r.predicted.proposed.overshootFraction < r.predicted.now.overshootFraction - 0.03);
  assert.ok(r.proposed.kP < r.current.kP, 'softer for the extra delay');
  assert.equal(r.reasons.length, 3);
  assert.match(r.reasons[1], /ms of delay, not 30ms/);

  // and the analyzer's prediction holds on the true robot: run the proposed gains there
  const after = RF.refine([turnLog({ trueModel, trueDelay: 0.075, gains: r.proposed, fileIndex: 3 })], TURN);
  assert.ok(after.measured.overshootFraction < r.measured.overshootFraction,
    `overshoot ${r.measured.overshootFraction} → ${after.measured.overshootFraction} on the robot`);
});

test('a turn that matches its model, and meets its spec, is left alone', () => {
  const logs = [turnLog({ trueModel: MEASURED, trueDelay: MEASURED_DELAY })];
  const r = RF.refine(logs, TURN);
  assert.ok(r.ok, r.why);
  assert.ok(r.identified.rms < 0.05, `mismatch ${r.identified.rms}`);
  assert.equal(r.change, 'none');
  assert.match(r.reasons[r.reasons.length - 1], /doing what they were designed to: no change/);
  assert.equal(r.saves, null);
  assert.deepEqual(RF.tuneChanges([r]), []);
});

test('big turns that saturate are motor-limited: the model is refined, the gains left alone', () => {
  const trueModel = { kS: 0.6, kV: 0.045, kA: 0.0065 * 1.5 };
  const r = RF.refine([turnLog({ trueModel, trueDelay: 0.075, steps: [120, 0, -150, 30, 180, 0] })], TURN);
  assert.ok(r.ok, r.why);
  assert.ok(r.identified.rms < 0.5 * r.identified.baseRms);
  assert.equal(r.change, 'none');
  assert.match(r.reasons[2], /limited by the motors/);
  // the model is worth saving on its own; the gains aren't touched
  assert.equal(r.saves, 'model');
  assert.deepEqual(RF.tuneChanges([r]).map((c) => c.key), ['model.turn']);
});

test('too few steps: no proposal, and why', () => {
  const r = RF.refine([turnLog({ trueModel: MEASURED, trueDelay: 0.03, steps: [90, 0] })], TURN);
  assert.equal(r.ok, false);
  assert.match(r.why, /2 usable turn steps/);
  assert.equal(r.responses.length, 2);
});

test('history follows the gains from run to run, and says what changed them', () => {
  const trueModel = { kS: 0.6, kV: 0.045, kA: 0.0065 * 1.5 };
  const first = turnLog({ trueModel, trueDelay: 0.075, fileIndex: 11, rev: 3 });
  const proposal = RF.refine([first], TURN);
  const second = turnLog({ trueModel, trueDelay: 0.075, fileIndex: 12, rev: 4, gains: proposal.proposed, autoTune: false });
  const third = turnLog({ trueModel, trueDelay: 0.075, fileIndex: 13, rev: 4, gains: { kP: 0.2, kI: 0, kD: 0.01 },
    autoTune: false });
  const rows = RF.history([third, first, second]);
  assert.deepEqual(rows.map((r) => r.file), ['SL000011.CSV', 'SL000012.CSV', 'SL000013.CSV']);
  assert.deepEqual(rows.map((r) => r.tune.rev), [3, 4, 4]);
  const turn = (row) => row.controllers.find((c) => c.pid === 'turn');
  assert.equal(turn(rows[0]).responses, 7);
  assert.ok(turn(rows[1]).overshoot < turn(rows[0]).overshoot, 'the next run shows the change working');
  assert.equal(rows[1].changes.length, 1);
  assert.equal(rows[1].changes[0].source, 'TUNE.CFG r4');
  assert.equal(rows[2].changes[0].source, 'a rebuild (the code\'s gains changed)');
  assert.equal(rows[0].autoTuned, true);
  // Auto-Tune set gains in the first run; a restart without TUNE.CFG loses them
  const restarted = RF.history([first, turnLog({ trueModel, trueDelay: 0.075, fileIndex: 14,
    gains: { kP: 0.3, kI: 0, kD: 0.01 }, autoTune: false })]);
  assert.match(restarted[1].changes[0].source, /^a restart/);
});

test('accepted refinements become TUNE.CFG lines the robot accepts', () => {
  const trueModel = { kS: 0.6, kV: 0.045, kA: 0.0065 * 1.5 };
  const r = RF.refine([turnLog({ trueModel, trueDelay: 0.075 })], TURN);
  const changes = RF.tuneChanges([r]);
  assert.deepEqual(changes.map((c) => c.key), ['pid.turn', 'model.turn']);
  const revised = TF.revise(TF.emptyProfile(), changes, { source: 'analyzer', date: new Date('2026-10-02') });
  assert.equal(revised.error, null);
  const back = TF.parse(TF.format(revised.profile));
  assert.ok(back.ok, back.error);
  assert.ok(Math.abs(back.profile.pids.turn.kP - r.proposed.kP) / r.proposed.kP < 1e-5);
  assert.match(revised.profile.changelog[0], /^r1 2026-10-02 analyzer: pid\.turn code -> /);
});

test('a distance error that dips and comes back up counts as overshoot', () => {
  const drive = RF.CONTROLLERS.find((c) => c.pid === 'drive');
  const t = Float64Array.from({ length: 6 }, (_, i) => i * 0.1);
  // 24in away, through the point to 3in past it, then back
  const m = RF.measuredMetrics(t, Float64Array.from([24, 12, 2, 0.5, 3, 0.4]), 24, drive);
  assert.equal(m.overshootFraction, 3 / 24);
  const clean = RF.measuredMetrics(t, Float64Array.from([24, 12, 4, 1.5, 0.6, 0.2]), 24, drive);
  assert.equal(clean.overshootFraction, 0);
  assert.ok(clean.settled);
});
