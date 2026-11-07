// Tests for the whole simulation: the robot program's odometry, localizer
// and motions running against the simulated world. Run:
// node --test tools/sim/test/*.test.js
//
// These are the claims the page makes when you show it to someone: MCL keeps
// the pose honest where odometry alone drifts, gets the robot back after a
// bump, ignores what it can't explain, and stays out of the way until
// setPose() puts odometry in the field frame.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const SIM = require('../js/sim.js');
const W = require('../js/world.js');
const R = require('../js/robot.js');
const T = require('../js/trajectory.js');
const { ROUTINES, TOUR_CHAIN, planBranch } = require('../js/routines.js');

function runRoutine(routine, tweak = () => {}, during = () => {}) {
  const options = SIM.defaultOptions();
  tweak(options);
  const sim = new SIM.Simulation(options);
  sim.runRoutine(routine);
  let worst = 0;
  let sum = 0;
  let ticks = 0;
  while (sim.program && sim.world.timeMs < 90000) {
    during(sim);
    sim.tick();
    const error = sim.errors().corrected;
    worst = Math.max(worst, error);
    sum += error;
    ticks++;
  }
  return { sim, end: sim.errors(), worst, mean: sum / ticks, finished: !sim.program };
}

test('every routine finishes within an inch, and MCL is closer on average than odometry alone', () => {
  // the ones that turn the localizer off themselves (Triple Pin) have their own tests below
  for (const routine of ROUTINES.filter((r) => r.correctsWithMcl !== false)) {
    const alone = runRoutine(routine, (o) => { o.correctOdometry = false; });
    const mcl = runRoutine(routine);
    assert.ok(alone.finished && mcl.finished, `${routine.id} finished`);
    assert.ok(mcl.end.corrected < 1.0, `${routine.id}: MCL ends ${mcl.end.corrected.toFixed(2)}in off`);
    // the average, not the worst or the end: a closed lap's odometry errors can cancel out by the
    // finish, and the worst moment can come before there's been anything to correct
    assert.ok(mcl.mean < 0.75 * alone.mean, `${routine.id}: mean ${mcl.mean} vs ${alone.mean}`);
  }
});

test('worn tracking wheels: odometry drifts, MCL holds', () => {
  const worn = (o) => {
    o.world.wheelDiameterErrorPct = 3;
    o.world.wheelSlipPct = 2;
    o.world.imuDriftDegPerMin = 3;
  };
  const alone = runRoutine(ROUTINES[0], (o) => { worn(o); o.correctOdometry = false; });
  const mcl = runRoutine(ROUTINES[0], worn);
  assert.ok(alone.worst > 3, `odometry alone worst ${alone.worst}`);
  assert.ok(mcl.worst < 2.5, `MCL worst ${mcl.worst}`);
  assert.ok(mcl.end.corrected < 1.0, `MCL end ${mcl.end.corrected}`);
});

test('a bump odometry never saw is corrected', () => {
  const bump = (sim) => { if (sim.world.timeMs === 6000) sim.world.bump(6, -4); };
  const alone = runRoutine(ROUTINES[0], (o) => { o.correctOdometry = false; }, bump);
  const mcl = runRoutine(ROUTINES[0], () => {}, bump);
  assert.ok(alone.end.corrected > 5, `odometry alone ends ${alone.end.corrected}`);
  assert.ok(mcl.end.corrected < 1.0, `MCL ends ${mcl.end.corrected}`);
});

test('field elements missing from the map are shrugged off as outliers', () => {
  const mcl = runRoutine(ROUTINES[0], (o) => { o.world.elements = W.exampleElements(); });
  assert.ok(mcl.worst < 2.0, `worst ${mcl.worst}`);
});

test('no corrections before setPose(), unless waitForSetPose is off', () => {
  for (const wait of [true, false]) {
    const options = SIM.defaultOptions();
    options.localizer = { waitForSetPose: wait };
    const sim = new SIM.Simulation(options);
    // like the robot at initialize(): really at (-48, -60), odometry says (0, 0), no setPose()
    sim.reset({ xIn: -48, yIn: -60, headingDeg: 0 });
    sim.robot.odometry.poseGeneration = 0;
    sim.robot.odometry.state.raw = { xIn: 0, yIn: 0, headingDeg: 0 };
    sim.robot.localizer.started = false;
    let corrected = false;
    for (let i = 0; i < 2000; ++i) {
      sim.setDriverInput(i % 800 < 400 ? 0.5 : -0.5, i % 600 < 300 ? 0.4 : -0.4, 0);
      sim.tick();
      corrected = corrected || sim.robot.localizer.status.correcting;
    }
    assert.equal(corrected, !wait, `waitForSetPose ${wait}`);
  }
});

test('reachedFinalApproach matches pure_pursuit_math_test.cpp', () => {
  assert.equal(R.reachedFinalApproach(0, 6, 0, 4), false);
  assert.equal(R.reachedFinalApproach(0, 6, 1, 4), false);
  assert.equal(R.reachedFinalApproach(5.9, 6, 2, 4), true);
  assert.equal(R.reachedFinalApproach(6.1, 6, 2, 4), false);
  assert.equal(R.reachedFinalApproach(3, 6, 3, 4), true);
  assert.equal(R.reachedFinalApproach(3, 6, 0, 2), true);
  assert.equal(R.reachedFinalApproach(3, 6, 0, 1), true);
});

test('findLookaheadPoint() stays on a lap\'s first side when its last passes beside it (pure_pursuit_math_test.cpp)', () => {
  const lap = [{ xIn: -30, yIn: -30 }, { xIn: -30, yIn: 30 }, { xIn: 30, yIn: 30 }, { xIn: 30, yIn: -30 },
    { xIn: -30, yIn: -30 }];
  const result = R.findLookaheadPoint(-28.86, -20.05, lap, 10, 0);
  assert.equal(result.segmentIndex, 0);
  assert.ok(Math.abs(result.point.xIn + 30) < 1e-9 && result.point.yIn > -11 && result.point.yIn < -9);
});

test('followPath() drives a closed lap instead of finishing where it starts', () => {
  const laps = ROUTINES.find((r) => r.id === 'laps');
  const options = SIM.defaultOptions();
  const sim = new SIM.Simulation(options);
  sim.runRoutine(laps);
  const corners = [[-30, 30], [30, 30], [30, -30]];
  const closest = corners.map(() => Infinity);
  while (sim.program && sim.world.timeMs < 90000) {
    sim.tick();
    corners.forEach(([x, y], i) => {
      closest[i] = Math.min(closest[i], Math.hypot(sim.world.truth.xIn - x, sim.world.truth.yIn - y));
    });
  }
  assert.ok(!sim.program, 'finished');
  // three 240in laps at cruise speed: well over ten seconds, not the instant finish it used to be
  assert.ok(sim.world.timeMs > 12000, `took ${sim.world.timeMs}ms`);
  for (const d of closest) assert.ok(d < 12, `came within ${d.toFixed(1)}in of every corner`);
  assert.ok(sim.errors().corrected < 1.5, `ended ${sim.errors().corrected.toFixed(2)}in off`);
});

test('tracking wheels follow OdometryConfig\'s sign: spinning in place leaves odometry still', () => {
  for (const flipped of [false, true]) {
    const options = SIM.defaultOptions();
    options.correctOdometry = false;
    Object.assign(options.world, { wheelDiameterErrorPct: 0, wheelSlipPct: 0, imuDriftDegPerMin: 0,
      imuScaleErrorPct: 0, imuNoiseDeg: 0, verticalWheelFlipped: flipped });
    const sim = new SIM.Simulation(options);
    sim.reset({ xIn: 0, yIn: 0, headingDeg: 0 });
    let worst = 0;
    for (let i = 0; i < 300; ++i) {
      sim.setDriverInput(0, 0, 0.5);
      sim.tick();
      const raw = sim.robot.odometry.snapshot().rawPose;
      worst = Math.max(worst, Math.hypot(raw.xIn, raw.yIn));
    }
    if (flipped) {
      // the wheel is on the other side from what the config says: a lap of phantom circles
      assert.ok(worst > 5, `flipped: odometry wandered ${worst.toFixed(2)}in`);
    } else {
      assert.ok(worst < 0.05, `odometry wandered ${worst.toFixed(3)}in on a spin in place`);
    }
  }
});

test('Auto-Tune designs the drive and turn gains from the axis models', () => {
  const designs = R.autoTuneGains(W.defaultSettings().models, 0.03);
  assert.ok(designs.drive.ok && designs.turn.ok);
  assert.ok(designs.drive.gains.kP > R.HAND_GAINS.drive.kP);
  assert.equal(designs.drive.gains.kI, 0);
  assert.ok(designs.drive.phaseMarginDeg >= 49.9);
  // each tracking controller from its own axis: strafe isn't the forward axis's gains
  for (const axis of ['forward', 'strafe', 'turn']) assert.ok(designs.tracking[axis].ok, axis);
  assert.notEqual(designs.tracking.strafe.gains.kP, designs.tracking.forward.gains.kP);
});

// --- followTrajectory() ---------------------------------------------------------------------

/**
 * Runs `motion(robot)` (a generator, like a routine's body) from `start`, calling during(sim)
 * before each tick. Returns the sim, the motion's result, and every tick's progress.
 */
function runMotion(start, motion, during = () => {}, tweak = () => {}) {
  const options = SIM.defaultOptions();
  tweak(options);
  const sim = new SIM.Simulation(options);
  let result = null;
  sim.runRoutine({ name: 'test', start, *run(robot) {
    robot.odometry.setPose(start);
    result = yield* motion(robot);
  } });
  const progress = [];
  while (sim.program && sim.world.timeMs < 60000) {
    during(sim);
    sim.tick();
    progress.push(Object.assign({ atMs: sim.world.timeMs }, sim.robot.drivetrain.trajectoryProgress()));
  }
  return { sim, result, progress };
}

function planFor(robot, sections) {
  const plan = T.generateTrajectory(sections, robot.drivetrain.axisModels);
  assert.ok(plan.ok, plan.error);
  return plan.trajectory;
}

test('the Trajectory tour lands within an inch, close to the planned time', () => {
  const tour = ROUTINES.find((r) => r.id === 'trajectory');
  let planned = 0;
  const run = runMotion(tour.start, function* (robot) {
    const trajectory = planFor(robot, TOUR_CHAIN);
    planned = trajectory.durationS() * 1000;
    return yield* robot.drivetrain.followTrajectory(trajectory);
  });
  assert.equal(run.result.reason, 'settled');
  assert.ok(run.sim.errors().corrected < 1.0, `ended ${run.sim.errors().corrected.toFixed(2)}in off`);
  // the plan's time plus a settle, not the plan's time plus seconds of hunting
  assert.ok(run.result.elapsedMs < 1.1 * planned + 300, `${run.result.elapsedMs}ms for a ${planned.toFixed(0)}ms plan`);
  assert.ok(!run.progress.some((p) => p.paused), 'never fell far enough behind to pause');
});

test('a diagonal drives straight on a trajectory, where moveToPoint() bends', () => {
  const start = { xIn: -40, yIn: -40, headingDeg: 0 };
  const end = { xIn: 20, yIn: 20 };
  const offLine = (motion) => {
    let worst = 0;
    runMotion(start, motion, (sim) => {
      const t = sim.world.truth;
      const dx = end.xIn - start.xIn;
      const dy = end.yIn - start.yIn;
      worst = Math.max(worst, Math.abs((t.xIn - start.xIn) * dy - (t.yIn - start.yIn) * dx) / Math.hypot(dx, dy));
    });
    return worst;
  };
  const trajectory = offLine(function* (robot) {
    yield* robot.drivetrain.followTrajectory(planFor(robot, [{ waypoints: [start, end] }]));
  });
  const point = offLine(function* (robot) {
    yield* robot.drivetrain.moveToPoint(end.xIn, end.yIn);
  });
  assert.ok(trajectory < 0.5, `trajectory strayed ${trajectory.toFixed(2)}in off the line`);
  // strafing is slower per volt than driving forward, so splitting one PID's volts by direction
  // pulls the robot toward forward; the trajectory gives each axis its own model
  assert.ok(point > 2 * trajectory, `moveToPoint() strayed ${point.toFixed(2)}in, the trajectory ${trajectory.toFixed(2)}in`);
});

test('progress reports each section as the plan enters it, and a chain without stops never stops', () => {
  const chain = [
    { waypoints: [{ xIn: -48, yIn: -48, headingDeg: 0 }, { xIn: -48, yIn: -10 }] },
    { waypoints: [{ xIn: -20, yIn: 10 }], constraints: { voltageBudget: 6 } },
    { waypoints: [{ xIn: 20, yIn: 10, headingDeg: 90 }] },
  ];
  let trajectory = null;
  let slowest = Infinity;
  const run = runMotion({ xIn: -48, yIn: -48, headingDeg: 0 }, function* (robot) {
    trajectory = planFor(robot, chain);
    return yield* robot.drivetrain.followTrajectory(trajectory);
  }, (sim) => {
    const p = sim.robot.drivetrain.trajectoryProgress();
    if (p.running && p.distanceIn > 5 && p.distanceIn < trajectory.lengthIn() - 5) {
      const v = sim.world.plants;
      slowest = Math.min(slowest, Math.hypot(v.forward.v, v.strafe.v));
    }
  });
  assert.equal(run.result.reason, 'settled');
  for (const section of [1, 2]) {
    const first = run.progress.find((p) => p.section >= section);
    assert.ok(first, `reached section ${section}`);
    const start = trajectory.sectionStart(section);
    assert.ok(Math.abs(first.timeS - start.tS) < 0.02, `section ${section} at ${first.timeS}s, planned ${start.tS}s`);
    assert.ok(first.distanceIn >= start.sIn - 1e-9);
  }
  assert.ok(slowest > 10, `slowest mid-chain ${slowest.toFixed(1)}in/s`);
  assert.ok(!run.progress[run.progress.length - 1].running);
});

test('a recorded run logs traj.plan, traj and the motion events the way the robot does', () => {
  const REC = require('../js/recorder.js');
  const slt = require('../../analyzer/js/slt.js');
  const tour = ROUTINES.find((r) => r.id === 'trajectory');
  const sim = new SIM.Simulation(SIM.defaultOptions());
  sim.runRoutine(tour);
  sim.recorder = new REC.LogRecorder(sim, { fileIndex: 900123, name: tour.name });
  while (sim.program && sim.world.timeMs < 60000) sim.tick();
  const log = slt.parse(sim.recorder.text(), 'SL900123.CSV');

  const plan = log.get('traj.plan');
  const traj = log.get('traj');
  assert.deepEqual(plan.columns, REC.PLAN_COLUMNS);
  assert.deepEqual(traj.columns, REC.TRAJ_COLUMNS);
  assert.ok(plan.length > 50 && plan.length <= 200, `${plan.length} plan rows`);
  // the stop between the second and third sections is in the plan
  assert.ok(Array.from(plan.cols.stop).filter((s) => s === 1).length >= 3, 'start, the stop, the end');
  // a traj row every tick of the motion
  const motion = slt.motions(log).find((m) => m.kind === 'followTrajectory');
  assert.ok(motion && motion.reason === 'settled', 'start and end events');
  assert.equal(motion.params.sections, 3);
  const ticks = motion.ms / 10;
  assert.ok(Math.abs(traj.length - ticks) <= 2, `${traj.length} traj rows for ${ticks} ticks`);
  assert.equal(traj.cols.sec[traj.length - 1], 2);
});

test('a branch at speed: the front sensor picks the branch at the junction, without stopping', () => {
  const routine = ROUTINES.find((r) => r.id === 'branch');
  const REC = require('../js/recorder.js');
  const slt = require('../../analyzer/js/slt.js');
  const drive = (defender) => {
    const options = SIM.defaultOptions();
    if (defender) Object.assign(options.world.defender, { enabled: true, xIn: -48, yIn: 30, sizeIn: 18 });
    const sim = new SIM.Simulation(options);
    sim.runRoutine(routine);
    sim.recorder = new REC.LogRecorder(sim, { fileIndex: 900124, name: routine.name });
    const plan = planBranch(sim.robot.drivetrain.axisModels);
    assert.ok(plan.ok, plan.error);
    let slowest = Infinity;
    let furthestEast = -Infinity;
    let closestToDefender = Infinity;
    let last = null;
    while (sim.program && sim.world.timeMs < 30000) {
      sim.tick();
      const p = sim.robot.drivetrain.trajectoryProgress();
      const truth = sim.world.truth;
      const now = { xIn: truth.xIn, yIn: truth.yIn, ms: sim.world.timeMs };
      // through the junction: a third of a second either side of the trunk's end
      if (last && p.running && p.piece <= 1 && Math.abs(p.timeS - plan.trunk.durationS()) < 0.33) {
        const speed = Math.hypot(now.xIn - last.xIn, now.yIn - last.yIn) / ((now.ms - last.ms) / 1000);
        slowest = Math.min(slowest, speed);
      }
      last = now;
      furthestEast = Math.max(furthestEast, truth.xIn);
      const d = options.world.defender;
      const dx = Math.max(Math.abs(truth.xIn - d.xIn) - d.sizeIn / 2, 0);
      const dy = Math.max(Math.abs(truth.yIn - d.yIn) - d.sizeIn / 2, 0);
      closestToDefender = Math.min(closestToDefender, Math.hypot(dx, dy));
    }
    const log = slt.parse(sim.recorder.text(), 'SL900124.CSV');
    return { sim, plan, slowest, furthestEast, closestToDefender, log };
  };

  for (const defender of [false, true]) {
    const run = drive(defender);
    assert.ok(!run.sim.program, 'the routine finished');
    assert.ok(run.sim.errors().corrected < 1.0, `ended ${run.sim.errors().corrected.toFixed(2)}in from home`);
    assert.ok(run.slowest > 15, `slowest through the junction ${run.slowest.toFixed(1)}in/s`);
    // clear: straight up the left side; blocked: off to the right, round the defender
    if (defender) {
      assert.ok(run.furthestEast > -20, `swerved to x=${run.furthestEast.toFixed(1)}`);
      assert.ok(run.closestToDefender > 9, `${run.closestToDefender.toFixed(1)}in from the defender`);
    } else {
      assert.ok(run.furthestEast < -45, `straight on, east as far as x=${run.furthestEast.toFixed(1)}`);
    }
    // the log reads like the robot's: one followTrajectory for the chain, its plan the trunk and
    // the branch together, then the plain trajectory home
    const trajectories = slt.trajectories(run.log);
    assert.equal(trajectories.length, 2);
    const chain = trajectories[0];
    assert.equal(chain.pieces.length, 2);
    assert.equal(chain.motion.reason, 'settled');
    const branch = defender ? run.plan.swerve : run.plan.straight;
    const plannedS = run.plan.trunk.durationS() + branch.durationS();
    assert.ok(Math.abs(chain.plan.durationS - plannedS) < 1e-3, `chain plan ${chain.plan.durationS}s`);
    assert.ok(Math.abs(chain.pieces[1].startS - run.plan.trunk.durationS()) < 2e-3);
    assert.equal(chain.plan.sec[chain.plan.sec.length - 1], branch.sectionCount());
    assert.equal(trajectories[1].pieces.length, 1);
  }
});

test('blocked by the defender, the plan waits, and the motion still finishes', () => {
  const start = { xIn: -40, yIn: -40, headingDeg: 0 };
  const run = runMotion(start, function* (robot) {
    return yield* robot.drivetrain.followTrajectory(planFor(robot, [{ waypoints: [start, { xIn: -40, yIn: 40 }] }]));
  }, (sim) => {
    // a defender parked across the path for a second, then gone
    const d = sim.options.world.defender;
    const blocking = sim.world.timeMs >= 600 && sim.world.timeMs < 1600;
    if (blocking !== d.enabled) {
      Object.assign(d, { enabled: blocking, xIn: -40, yIn: 0, sizeIn: 18 });
      sim.refreshMaps();
    }
  });
  const paused = run.progress.filter((p) => p.paused).length;
  assert.ok(paused > 20, `paused ${paused} ticks`);
  assert.equal(run.result.reason, 'settled');
  assert.ok(run.sim.errors().corrected < 1.0);
});

// --- Triple Pin (paths): autons.cpp's triplePinPaths() ---

const F = require('../js/field.js');
const A = require('../js/auton.js');
const RT = require('../js/routines.js');

const { TRIPLE_PIN, TRIPLE_PIN_LEGS, ROBOT_SENSOR_MOUNTS, ROBOT_WALL_RESET } = RT;
const triplePin = ROUTINES.find((r) => r.id === 'triplePin');
// devices.cpp's Auto-Tune axis models: the robot's own speed
const kRobotModels = {
  forward: { kS: 0.30, kV: 0.1698, kA: 0.0383 },
  strafe: { kS: 2.21, kV: 0.3234, kA: 0.0838 },
  turn: { kS: 1.10, kV: 0.0318, kA: 0.0019 },
};

// The routine with its wall resets doing nothing, to see what they buy
function withoutResets(body) {
  const create = A.create;
  A.create = (robot, settings) => {
    const a = create(robot, settings);
    for (const k of ['resetFront', 'resetBack', 'resetLeft', 'resetRight']) a[k] = () => false;
    return a;
  };
  try {
    return body();
  } finally {
    A.create = create;
  }
}

test('Triple Pin (paths) stacks its three Pins at L1, L2 and L3, inside autonomous\'s 15 s', () => {
  for (const models of [null, kRobotModels]) {
    const run = runRoutine(triplePin, (o) => { if (models) o.world.models = models; });
    const label = models ? 'the robot\'s models' : 'the simulator\'s';
    assert.ok(run.finished, label);
    assert.ok(run.sim.world.timeMs < 15000, `${label}: done at ${run.sim.world.timeMs} ms`);
    const scored = run.sim.robot.macros.scored();
    assert.deepEqual(scored.map((p) => p.goal), ['red goal (left)', 'red goal (left)', 'red goal (left)'], label);
    // each at the next level of the stack, with the lift at that level
    assert.deepEqual(scored.map((p) => [p.level, p.liftLevel]), [[1, 1], [2, 2], [3, 3]], label);
    // macros.js lands a Pin on a Goal within 1.25in of its middle: each well inside that
    for (const p of scored) assert.ok(p.offIn < 1.0, `${label}: L${p.level} ${p.offIn.toFixed(2)}in off the goal's middle`);
    // four readings of y east of the third Pin, then four rounds of x and y on it, every one taken
    const resets = run.sim.robot.events.filter((e) => /^reset/.test(e.text));
    const round = ['resetFront', 'resetLeft'];
    assert.deepEqual(resets.map((e) => e.text.split(':')[0]),
      ['resetLeft', 'resetLeft', 'resetLeft', 'resetLeft', ...round, ...round, ...round, ...round], label);
    // a reading the sensor drops is left out of the average, but nearly all of them take
    const took = resets.filter((e) => !/skipped/.test(e.text)).length;
    assert.ok(took >= 10, `${label}: ${took} of 12 took: ${resets.map((e) => e.text).join('; ')}`);
  }
});

test('Triple Pin (paths) resets only where its sensors\' cones see clean wall, on the real field', () => {
  const field = F.override();
  const T = field.tilePitchIn;
  const without = (gone) => field.objects.filter((o) => !(o.kind === 'pin' && gone.some(([x, y]) => Math.hypot(o.xIn - x, o.yIn - y) < 0.5)));
  const robot = (deg) => ROBOT_SENSOR_MOUNTS.find((m) => m.facingDeg === deg);
  const leg = TRIPLE_PIN.toThirdPin;
  const at = (point) => ({ xIn: point.xIn, yIn: point.yIn, headingDeg: 270 });
  // the stop east of the third Pin (point 3) with the second Pin gone, and on the third Pin with it
  // in the claw: an inch and 3 degrees either way of the plan, with config.hpp's sensors
  const checks = [
    { where: 'east of the third Pin', pose: at(leg[1].waypoints[1]), sensor: robot(270), axis: 'y', gone: [[-T, -T]] },
    { where: 'on the third Pin', pose: at(leg[2].waypoints[0]), sensor: robot(0), axis: 'x', gone: [[-T, -T], [-2 * T, -2 * T]] },
    { where: 'on the third Pin', pose: at(leg[2].waypoints[0]), sensor: robot(270), axis: 'y', gone: [[-T, -T], [-2 * T, -2 * T]] },
  ];
  for (const c of checks) {
    for (const dx of [-1, 0, 1]) {
      for (const dy of [-1, 0, 1]) {
        for (const dh of [-3, 0, 3]) {
          const pose = { xIn: c.pose.xIn + dx, yIn: c.pose.yIn + dy, headingDeg: c.pose.headingDeg + dh };
          const r = F.wallResetCheck(field, pose, c.sensor,
            { cornerMarginIn: ROBOT_WALL_RESET.cornerMarginIn, objects: without(c.gone), toleranceIn: dh ? 0.5 : 0.25 });
          assert.ok(r.ok && r.axis === c.axis, `${c.where}, off by (${dx}, ${dy}, ${dh}): ${r.why}`);
        }
      }
    }
  }
  // and where the routine's comment says it doesn't reset, it couldn't
  const skipped = [
    [{ xIn: -60.6, yIn: -5, headingDeg: 90 }, robot(180), /Toggle/], // back on the Toggle after the rams
    [{ xIn: -60.6, yIn: -5, headingDeg: 90 }, robot(90), /corner/],
    [TRIPLE_PIN.toGoal[1].waypoints[0], robot(90), /cup/], // on the goal from the north
    [{ xIn: -41.79, yIn: -23.55, headingDeg: 270 }, robot(0), /red goal/], // on it from the east
    [TRIPLE_PIN.toSecondPin[2].waypoints[0], robot(0), /red goal/], // the second Pin
    [TRIPLE_PIN.toSecondPin[2].waypoints[0], robot(90), /red goal/],
  ];
  for (const [point, mount, why] of skipped) {
    const pose = { xIn: point.xIn, yIn: point.yIn, headingDeg: point.headingDeg === undefined ? 180 : point.headingDeg };
    const r = F.wallResetCheck(field, pose, mount, { cornerMarginIn: ROBOT_WALL_RESET.cornerMarginIn });
    assert.ok(!r.ok && why.test(r.why), `(${pose.xIn}, ${pose.yIn}): ${r.why || 'ok'}`);
  }
});

test('Triple Pin (paths): a bump on the way to the third Pin costs it the Pin, unless the resets put it back', () => {
  const bump = (sim) => { if (sim.world.timeMs === 9500) sim.world.bump(0, 1.0); };
  const errorAt = new Map();
  const run = runRoutine(triplePin, () => {}, (sim) => {
    bump(sim);
    errorAt.set(sim.world.timeMs, sim.errors().corrected);
  });
  const events = run.sim.robot.events;
  const firstReset = events.find((e) => /^resetLeft/.test(e.text));
  const grab = events.find((e) => /autoClaw grabbed/.test(e.text) && e.tMs > firstReset.tMs);
  // (each tick's error is taken before the tick, so the one before the reset's is the tick before)
  assert.ok(errorAt.get(firstReset.tMs - 10) > 1.0, `before the resets ${errorAt.get(firstReset.tMs - 10)}`);
  assert.ok(errorAt.get(grab.tMs) < 0.5, `at the grab ${errorAt.get(grab.tMs)}`);
  assert.equal(run.sim.robot.macros.scored().length, 3);
  // without them, the claw runs past the Pin's side and never shuts on it
  const blind = withoutResets(() => runRoutine(triplePin, () => {}, bump));
  assert.equal(blind.sim.robot.macros.scored().length, 2);
});

test('averaged resets: four readings land nearer than one', () => {
  const averaged = { x: [], y: [] };
  const single = { x: [], y: [] };
  for (let seed = 1; seed <= 24; ++seed) {
    for (const [out, readings] of [[averaged, 4], [single, 1]]) {
      const options = SIM.defaultOptions();
      options.world.seed = seed;
      // still on the third Pin's spot, with the robot's sensors, odometry an inch off each way
      const spot = { xIn: TRIPLE_PIN.toThirdPin[2].waypoints[0].xIn, yIn: -47.09, headingDeg: 270 };
      const sim = new SIM.Simulation(options);
      sim.runRoutine({ name: 'test', start: spot, world: triplePin.world, *run(robot) {
        const a = A.create(robot, { wallReset: ROBOT_WALL_RESET });
        a.setPose(spot.xIn + 1, spot.yIn - 1, spot.headingDeg);
        yield* a.delay(100);
        yield* RT.averagedResets(a, [[a.resetFront, 'x'], [a.resetLeft, 'y']], readings);
      } });
      while (sim.program) sim.tick();
      const pose = sim.robot.odometry.getPose();
      out.x.push(pose.xIn - sim.world.truth.xIn);
      out.y.push(pose.yIn - sim.world.truth.yIn);
    }
  }
  const rms = (v) => Math.sqrt(v.reduce((sum, e) => sum + e * e, 0) / v.length);
  for (const axis of ['x', 'y']) {
    assert.ok(rms(averaged[axis]) < 0.7 * rms(single[axis]), `${axis}: ${rms(averaged[axis])} vs one reading's ${rms(single[axis])}`);
    assert.ok(rms(averaged[axis]) < 0.5, `${axis}: ${rms(averaged[axis])}`);
  }
});

test('Triple Pin (paths): its legs plan, and only the goal it scores on is in the footprint\'s way', () => {
  const field = F.override();
  const models = W.defaultSettings().models;
  // where it takes the goal into the front of the robot, and backs off it
  const onGoal = [[-47.09, -18.25], [-41.79, -23.55]];
  for (const size of [15, 16, 17]) {
    for (const leg of TRIPLE_PIN_LEGS) {
      const plan = T.generateTrajectory(TRIPLE_PIN[leg], models);
      assert.ok(plan.ok, `${leg}: ${plan.error}`);
      for (const s of plan.trajectory.samples) {
        for (const hit of F.footprintHits(F.footprint(s, size, size), field)) {
          const scoring = hit === 'red goal (left)' && onGoal.some(([x, y]) => Math.hypot(s.xIn - x, s.yIn - y) <= 6.6);
          assert.ok(scoring, `${size}in, ${leg} at ${s.tS.toFixed(2)}s: ${hit}`);
        }
      }
    }
  }
});
