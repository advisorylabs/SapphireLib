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

// --- Triple Pin (points): autons.cpp's triplePinPoints() ---

const A = require('../js/auton.js');
const F = require('../js/field.js');
const RT = require('../js/routines.js');
const M = require('../js/macros.js');

const { TRIPLE_PIN } = RT;
const triplePin = ROUTINES.find((r) => r.id === 'triplePin');
// devices.cpp's Auto-Tune axis models: the robot's own speed
const kRobotModels = {
  forward: { kS: 0.30, kV: 0.1698, kA: 0.0383 },
  strafe: { kS: 2.21, kV: 0.3234, kA: 0.0838 },
  turn: { kS: 1.10, kV: 0.0318, kA: 0.0019 },
};
// devices.cpp's PID gains, which the simulator's Auto-Tune doesn't land on exactly from the models
// above (its Drive kP is 3.6, the robot's 2.1)
const kDeviceGains = {
  drive: { kP: 2.086, kI: 0.0, kD: 0.396 },
  turn: { kP: 0.2587, kI: 0.0, kD: 0.0125 },
  tracking: { forward: { kP: 2.086, kI: 0.0, kD: 0.396 }, strafe: { kP: 6.0, kI: 0.0, kD: 1.1 }, turn: { kP: 0.405, kI: 0.0, kD: 0.024 } },
};
const robotModels = (o) => { o.world.models = kRobotModels; };
// the robot's gains too, set once the routine has started (runRoutine() applies Auto-Tune's first)
const deviceGains = (during = () => {}) => (sim) => {
  if (sim.world.timeMs === 0) sim.robot.drivetrain.setGains(kDeviceGains);
  during(sim);
};
// the simulator's chassis, the robot's models, and the robot's models with its own gains
const kTriplePinCases = [
  ['the simulator\'s chassis', () => {}, (during = () => {}) => during],
  ['the robot\'s models', robotModels, (during = () => {}) => during],
  ['the robot\'s models and gains', robotModels, deviceGains],
];
const grabsOf = (run) => run.sim.robot.events.filter((e) => /claw closed on/.test(e.text)).map((e) => e.text);

test('Triple Pin (points) stacks its three Pins at L1, L2 and L3 inside 14.5 s, with no wall resets', () => {
  for (const [label, tweak, during] of kTriplePinCases) {
    const run = runRoutine(triplePin, tweak, during());
    assert.ok(run.finished, label);
    // 13.1 to 14.3 s: the 15 s autonomous period with room for a slower robot
    assert.ok(run.sim.world.timeMs < 14500, `${label}: done at ${run.sim.world.timeMs} ms`);
    const scored = run.sim.robot.macros.scored();
    assert.deepEqual(scored.map((p) => p.goal), ['red goal (left)', 'red goal (left)', 'red goal (left)'], label);
    // each at the next level of the stack, with the lift at that level
    assert.deepEqual(scored.map((p) => [p.level, p.liftLevel]), [[1, 1], [2, 2], [3, 3]], label);
    // macros.js lands a Pin on a Goal within 1.25in of its middle: each well inside that
    for (const p of scored) assert.ok(p.offIn < 0.75, `${label}: L${p.level} ${p.offIn.toFixed(2)}in off the goal's middle`);
    // odometry alone: not one wall reset, and the localizer's corrections off
    assert.deepEqual(run.sim.robot.events.filter((e) => /^reset|goal_reset/.test(e.text)).map((e) => e.text), [], label);
  }
});

test('Triple Pin (points) gets the second Pin where the field has it, even on a robot slower than its models', () => {
  // SL000060: the robot lagged its plans by up to 4in, and the routine ended with the claw open. A
  // robot 15% and 30% slower than the models it drives by does the same without ontoPin()'s creep
  for (const slower of [1.0, 1.15, 1.3]) {
    const run = runRoutine(triplePin, (o) => {
      o.world.models = JSON.parse(JSON.stringify(kRobotModels));
      for (const axis of ['forward', 'strafe', 'turn']) {
        o.world.models[axis].kV *= slower;
        o.world.models[axis].kA *= slower;
      }
    }, deviceGains((sim) => { if (sim.world.timeMs === 0) sim.robot.drivetrain.setAxisModels(kRobotModels); }));
    const grabs = grabsOf(run);
    assert.match(grabs[0] || '', /\(-23\.5, -23\.5\)/, `${slower}x slower: ${grabs.join('; ')}`);
    assert.equal(run.sim.robot.macros.scored().length, 3, `${slower}x slower`);
  }
});

test('Triple Pin (points): each run onto a Pin is at autoClaw speed for its last third, and no turn after it is held to 5V', () => {
  // each moveToPoint()'s target and where the robot was as it started, each approach's start (its
  // volts cap, whether a camera steers it), each turn's volts cap, the robot's true speed each
  // tick, and when autoClaw turns on
  const moves = [];
  const approaches = [];
  const turns = [];
  const ticks = [];
  const autoClawAt = [];
  let wasOn = false;
  let prev = null;
  const run = runRoutine(triplePin, robotModels, deviceGains((sim) => {
    const dt = sim.robot.drivetrain;
    if (!dt.wrappedForTest) {
      const moveToPoint = dt.moveToPoint.bind(dt);
      dt.moveToPoint = (xIn, yIn, exit) => {
        moves.push({ tMs: sim.world.timeMs, target: { xIn, yIn }, from: sim.robot.odometry.getPose() });
        return moveToPoint(xIn, yIn, exit);
      };
      const approach = dt.approach.bind(dt);
      dt.approach = (target, config, refine) => {
        approaches.push({ tMs: sim.world.timeMs, target, maxVolts: config.maxVolts, refine: !!refine });
        return approach(target, config, refine);
      };
      const turnToHeading = dt.turnToHeading.bind(dt);
      dt.turnToHeading = (headingDeg, exit, direction) => {
        turns.push({ tMs: sim.world.timeMs, maxVolts: exit.maxVolts });
        return turnToHeading(headingDeg, exit, direction);
      };
      dt.wrappedForTest = true;
    }
    const truth = sim.world.truth;
    if (prev) ticks.push({ tMs: sim.world.timeMs, truth: Object.assign({}, truth), speed: Math.hypot(truth.xIn - prev.xIn, truth.yIn - prev.yIn) / 0.01 });
    prev = Object.assign({}, truth);
    const on = !!sim.robot.macros && sim.robot.macros.autoClaw();
    if (on && !wasOn) autoClawAt.push({ tMs: sim.world.timeMs, pose: sim.robot.odometry.getPose() });
    wasOn = on;
  }));
  assert.ok(run.finished);
  assert.equal(autoClawAt.length, 2);
  // what autoClaw's speed limit lets the robot's forward axis hold
  const fiveVoltInPerS = (M.kClawSpeedVolts - kRobotModels.forward.kS) / kRobotModels.forward.kV;
  [TRIPLE_PIN.secondPin, TRIPLE_PIN.thirdPin].forEach((pin, i) => {
    const label = `Pin ${i + 2}`;
    const toSeat = (p) => Math.hypot(pin.xIn - p.xIn, pin.yIn - p.yIn) - RT.kPinSeatIn;
    // the run: from where the moveToPoint() before autoClaw started, facing the Pin, to the seat
    const move = moves.filter((m) => m.tMs <= autoClawAt[i].tMs).pop();
    const runIn = toSeat(move.from);
    near(toSeat(move.target), runIn / 3, 0.05, `${label}: the moveToPoint() ends a third of the run out`);
    // autoClaw on at the hand-off: a third of the run to go, or up to the chained move's 3in more
    const leftIn = toSeat(autoClawAt[i].pose);
    assert.ok(leftIn >= runIn / 3 - 0.5 && leftIn <= runIn / 3 + 3.5, `${label}: autoClaw on with ${leftIn.toFixed(1)}in to go of ${runIn.toFixed(1)}`);
    // the approach starting with it, held to autoClaw's 5V, no camera steering it, aimed at the Pin
    const a = approaches.find((x) => x.tMs >= autoClawAt[i].tMs);
    assert.ok(a.tMs - autoClawAt[i].tMs <= 10, `${label}: approach at ${a.tMs} ms, autoClaw at ${autoClawAt[i].tMs}`);
    assert.equal(a.maxVolts, M.kClawSpeedVolts, `${label}: the approach's volts`);
    assert.equal(a.refine, false, `${label}: no camera`);
    assert.deepEqual(a.target, { xIn: pin.xIn, yIn: pin.yIn });
    // the robot no faster than autoClaw's 5V lets it go, the whole last third
    const lastThird = ticks.filter((t) => t.tMs >= autoClawAt[i].tMs && t.tMs <= autoClawAt[i].tMs + 1500 && toSeat(t.truth) >= 0);
    const fastest = Math.max(...lastThird.map((t) => t.speed));
    assert.ok(fastest < fiveVoltInPerS, `${label}: ${fastest.toFixed(1)} in/s in the last third, past autoClaw's ${fiveVoltInPerS.toFixed(1)}`);
  });
  // autoClaw's lead assumes the speed holds as the Pin comes in: here it does
  const speeds = run.sim.robot.events.filter((e) => /autoClaw grabbed/.test(e.text)).map((e) => +e.text.match(/([\d.]+) in\/s/)[1]);
  assert.equal(speeds.length, 2);
  for (const s of speeds) near(s, RT.kPinApproach.cruiseInPerS, 1.5, 'grab speed');
  // the claw shuts on each Pin where the field has it
  const grabs = grabsOf(run);
  assert.match(grabs[0] || '', /\(-23\.5, -23\.5\)/, grabs.join('; '));
  assert.match(grabs[1] || '', /\(-47\.1, -47\.1\)/, grabs.join('; '));
  // autoClaw off once the claw has the Pin: every turn at full volts
  assert.deepEqual(turns.filter((t) => t.maxVolts < 12).map((t) => t.tMs), []);
});

test('Triple Pin (points): the robot touches the wall only in the rams, and a Goal only as the one it scores on comes into the aligner', () => {
  const field = F.override();
  const W = require('../js/world.js');
  // the robot's footprint, 13.75 x 17in, its middle (kFrontIn - back) / 2 ahead of the tracking
  // center: its front edge is kFrontIn ahead, its back the rest of the 17in behind
  const back = RT.kRobotLengthIn - RT.kFrontIn;
  const middleAheadIn = (RT.kFrontIn - back) / 2;
  const footprint = (p) => {
    const a = p.headingDeg * Math.PI / 180;
    return F.footprint({ xIn: p.xIn + middleAheadIn * Math.sin(a), yIn: p.yIn + middleAheadIn * Math.cos(a), headingDeg: p.headingDeg },
      RT.kRobotWidthIn, RT.kRobotLengthIn);
  };
  // The footprint is a plain rectangle, without the goal aligner's 3.5in recess in its front: a
  // Goal at the front edge is the Goal coming into the aligner, square to it, which is how every
  // score starts. So the goal it scores on may touch it there: its middle ahead, from where it
  // first reaches the front edge back to an inch past where it seats, and no further across than
  // what the aligner takes (world.js's goalStopHalfWidthIn). Anywhere else on the robot, or any
  // other Goal, is a collision
  const across = W.defaultSettings().goalStopHalfWidthIn;
  const inTheAligner = (p) => {
    const g = TRIPLE_PIN.goal;
    const a = p.headingDeg * Math.PI / 180;
    const dx = g.xIn - p.xIn;
    const dy = g.yIn - p.yIn;
    const ahead = dx * Math.sin(a) + dy * Math.cos(a);
    const right = dx * Math.cos(a) - dy * Math.sin(a);
    return ahead >= RT.kGoalSeatIn - 1.0 && ahead <= RT.kFrontIn + 5.61 / Math.sqrt(3) + 0.5 && Math.abs(right) <= across;
  };
  for (const [label, tweak, during] of kTriplePinCases) {
    const hits = [];
    runRoutine(triplePin, tweak, during((sim) => {
      const truth = sim.world.truth;
      // the rams: pinned against the left wall, facing away from it
      const ramming = truth.xIn < -60 && Math.abs(truth.headingDeg - 90) < 3;
      for (const hit of F.footprintHits(footprint(truth), field)) {
        if (hit === 'wall' && ramming) continue;
        if (hit === 'red goal (left)' && inTheAligner(truth)) continue;
        hits.push(`${hit} at ${sim.world.timeMs} ms, (${truth.xIn.toFixed(1)}, ${truth.yIn.toFixed(1)}) facing ${truth.headingDeg.toFixed(0)}`);
      }
    }));
    assert.deepEqual(hits.slice(0, 5), [], label);
  }
});

test('Triple Pin (points) stops on its stops: within 0.75in of each, with the robot\'s models and gains', () => {
  // each stop is on the line the robot gets to it along (backTo() and offGoalTo() back straight
  // along its heading), so it stops on the point, not beside it; and the push floors (devices.cpp,
  // robot.js's kPushFloorVolts) carry each drive into its exit. Odometry's pose: where it thinks
  // it is, which is what the routine's numbers say
  const stops = ['aboveGoal', 'afterL1', 'afterSecondPin', 'afterL2', 'belowGoal', 'afterL3'];
  const closest = Object.fromEntries(stops.map((k) => [k, Infinity]));
  runRoutine(triplePin, robotModels, deviceGains((sim) => {
    const p = sim.robot.odometry.getPose();
    for (const k of stops) closest[k] = Math.min(closest[k], Math.hypot(p.xIn - TRIPLE_PIN[k].xIn, p.yIn - TRIPLE_PIN[k].yIn));
  }));
  for (const k of stops) assert.ok(closest[k] < 0.75, `${k}: ${closest[k].toFixed(2)}in at the closest`);
});

test('followTrajectory() with until hands off still moving, and approach() carries straight on', () => {
  const start = { xIn: 0, yIn: -48, headingDeg: 0 };
  let handoff = null;
  const run = runMotion(start, function* (robot) {
    const a = A.create(robot);
    const leg = [{ waypoints: [{ xIn: 0, yIn: -48, headingDeg: 0 }, { xIn: 0, yIn: 0 }] }];
    const followed = yield* a.followTrajectory(leg, { until: () => robot.odometry.getPose().yIn >= -30 });
    handoff = { reason: followed.reason, speedInPerS: robot.odometry.snapshot().rawVelocity.yInPerS,
      volts: robot.drivetrain.applied.forward, yIn: robot.odometry.getPose().yIn };
    return yield* a.approach(0, 0, { cruiseInPerS: 18, endShortIn: 6 });
  });
  assert.equal(handoff.reason, 'settled');
  near(handoff.yIn, -30, 1.0, 'handed off where until said');
  assert.ok(handoff.speedInPerS > 15, `${handoff.speedInPerS.toFixed(1)} in/s at the handoff`);
  assert.ok(handoff.volts > 1, `${handoff.volts.toFixed(2)}V left on, not braked`);
  assert.equal(run.result.reason, 'settled');
  near(run.sim.robot.odometry.getPose().yIn, -6, 1.0, 'ended its end short of the target');
});

function near(actual, expected, tolerance, label) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} vs ${expected}`);
}
