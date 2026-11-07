/*
 * SapphireLib simulator: routines.js
 *
 * Autonomous routines for the simulator, written the way src/robot/autons.cpp
 * writes them, with the auton functions (auton.js, the port of
 * sapphirelib/auton/auton.hpp): setPose() to the real start, then motions.
 * Each routine is a generator, and `yield*` an auton call is the C++'s plain
 * call. Every routine ends where it started, so "how far off is it now" is a
 * fair question to ask of odometry alone and of odometry with MCL.
 *
 * To try your own: copy one, change the points, and add it to ROUTINES. Field
 * coordinates have the origin in the middle of the field (walls at +-70.25in),
 * +y downfield, headings clockwise from +y. Then copy it into autons.cpp,
 * dropping the `yield*`s and the `a.`s.
 *
 * Plain script: window.SIM.routines in a browser, require('./routines.js') in
 * Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./trajectory.js'), require('./auton.js'), require('./macros.js'), require('./mcl.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.routines = factory(root.SIM.trajectory, root.SIM.auton, root.SIM.macros, root.SIM.mcl);
  }
})(function (T, A, M, L) {
  'use strict';

  // A trajectory chain, the way autons.cpp would hold one (the Paths tab writes these): full speed
  // up the left side turning to face +x, a slower section that stops facing the far wall, then
  // back home from the stop at a sharp angle.
  const TOUR_CHAIN = [
    { waypoints: [{ xIn: -48, yIn: -60, headingDeg: 0 }, { xIn: -50, yIn: -24 }, { xIn: -30, yIn: 0, headingDeg: 90 }] },
    { waypoints: [{ xIn: 0, yIn: 12 }, { xIn: 30, yIn: 24, headingDeg: 180 }],
      constraints: { voltageBudget: 7.0 }, stopAtEnd: true },
    { waypoints: [{ xIn: 24, yIn: -20 }, { xIn: -20, yIn: -44 }, { xIn: -48, yIn: -60, headingDeg: 0 }] },
  ];

  // A branch at speed, the way examples/trajectory.cpp's branchAtSpeed() is written: a trunk up the
  // left side that ends at 30in/s, and two branches carrying on from its end. The front distance
  // sensor picks one at the junction: clear ahead, straight on; something within 36in (the
  // defender), a swerve to the right. Then home from wherever that branch stopped, a plain
  // trajectory planned from its end (branching with a stop).
  const BRANCH = {
    trunk: [{ waypoints: [{ xIn: -48, yIn: -60, headingDeg: 0 }, { xIn: -48, yIn: -12 }] }],
    junctionSpeedInPerS: 30,
    clearAheadIn: 36,
    straight: [{ waypoints: [{ xIn: -48, yIn: 40 }] }],
    swerve: [{ waypoints: [{ xIn: -32, yIn: 8 }, { xIn: -12, yIn: 30, headingDeg: 90 }] }],
    // each branch's way home, from where it stops
    homeFromStraight: [{ waypoints: [{ xIn: -48, yIn: -60 }] }],
    homeFromSwerve: [{ waypoints: [{ xIn: -40, yIn: -20 }, { xIn: -48, yIn: -60, headingDeg: 0 }] }],
  };

  // --- Triple Pin on trajectories: autons.cpp's triplePinPaths() -----------------------------------
  //
  // Three Pins stacked on the alliance Goal by the left wall, at (-47.09, -23.55), "the goal": the
  // preload at L1, the Pin standing at (-23.55, -23.55) at L2, and the one at (-47.09, -47.09) at
  // L3. Each drive is a trajectory of its own, as the Paths tab writes one: Paths → Paste C++ reads
  // all of autons.cpp's at once, a path each. They're split where the robot has to stop: on the
  // goal while the lift dips and the claw opens, and at each Pin, where the claw shuts.
  //
  // The claw holds a Pin 5.3in ahead of the tracking center (macros.js's kClawReachIn, from where
  // the old routine's moveToPose() grabbed its second Pin), so the robot stops 5.3in short of the
  // goal's middle, facing it, settling to within half an inch (kScoreFollow) rather than
  // followTrajectory()'s inch. At a Pin it aims an inch past the claw's middle, 4.3in short, the
  // back of the claw stopping the Pin: autoClaw shuts the claw when its sensor reads 125mm, the Pin
  // 5.7in ahead, and a robot aiming at 5.3in that settles short of it, or was bumped an inch back,
  // never gets there. Every approach ends with a short straight run at no more than 24 in/s
  // forward (kSlow): a Pin goes into the claw, and the goal into the front of the robot, square,
  // and autoClaw's sensor has time to see the Pin before the robot is on it.
  //
  // The wall resets: at every stop where a sensor sees a clear stretch of wall and something that
  // needs the pose follows. field.js's wallResetCheck() says where, with config.hpp's sensors
  // (ROBOT_SENSOR_MOUNTS, which the routine runs with) on Override's field, and test/sim.test.js
  // checks each one an inch and 3 degrees either way. A reset can't tell a Cup, a Pin or a Goal in
  // its beam's cone from the wall, and the localizer's own checks (devices.cpp: 18in clear of the
  // corners, at most 8in) let a reading a couple of inches short through. Stop by stop:
  //   start, after the rams       none: the back sensor is on the Toggle, the side ones within
  //                               the corners' 18in. x is the wall's it's pinned against
  //   above the goal, and on it   none: the right sensor's cone takes in the Cups and the Pin
  //   from the north              along the left wall; the front one sees the goal
  //   at and above the second Pin none: the two Goals block the near and left walls
  //   on the goal from the east   resetLeft() would read the near wall, 42in off, but a reset on
  //                               the goal can't fix where the robot already stopped, and the
  //                               back-off after it would steer the correction out sideways with
  //                               the claw still around the stack
  //   east of the third Pin       y off the near wall, 18in, while the robot runs west onto the
  //                               Pin, which needs y right to land in the claw. Not x: the front
  //                               sensor's beam is along the run
  //   on the third Pin            x off the left wall, 28in, and y again, still. For the drive
  //                               back to the goal
  // So nothing corrects the pose between the start and the second score: that stretch rides on
  // the start's y and odometry. The localizer stays off (setMcl(false)), as triplePin() has it.
  // If the back sensor's 3in at the start turns out to be its mount rather than the Toggle, the
  // Toggles are fine to reset off, and the start and the stop above the goal get one each.
  //
  // Each reset is one reading, and one reading is noisy: in the simulator's default noise (half
  // the V5 spec) a reset at 28in is +-0.7in, enough by itself to miss the goal. So the routine
  // averages kResetReadings of each (averagedResets()), which halves it, for kResetGapMs a reading.
  // The sensors' real noise, from a log in the Tune tab, says how many it needs.
  const kSlow = { voltageBudget: 10, forward: { maxVelocity: 24 } };
  const kScoreFollow = { positionErrorThresholdIn: 0.5 };
  const kResetReadings = 4;
  const kResetGapMs = 40; // the sensors read about every 33ms
  const TRIPLE_PIN = {
    start: { xIn: -60.6, yIn: -5, headingDeg: 90 },
    // start → the goal from the north, facing it (180): the preload, at L1
    toGoal: [
      { waypoints: [{ xIn: -60.6, yIn: -5, headingDeg: 90 }, { xIn: -52, yIn: -7 }, { xIn: -47.09, yIn: -12.25, headingDeg: 180 }],
        stopAtEnd: true },
      { waypoints: [{ xIn: -47.09, yIn: -18.25 }], constraints: kSlow },
    ],
    // straight back off the goal (point 1, where the lift goes down), then along the north side
    // to the second Pin, and down onto it from the north
    toSecondPin: [
      { waypoints: [{ xIn: -47.09, yIn: -18.25, headingDeg: 180 }, { xIn: -47.09, yIn: -12.25 }], stopAtEnd: true },
      { waypoints: [{ xIn: -35, yIn: -11 }, { xIn: -23.55, yIn: -12.25 }], stopAtEnd: true },
      { waypoints: [{ xIn: -23.55, yIn: -19.25 }], constraints: kSlow },
    ],
    // the second Pin → the goal from the east (270), turning on the way: at L2
    secondToGoal: [
      { waypoints: [{ xIn: -23.55, yIn: -19.25, headingDeg: 180 }, { xIn: -30, yIn: -21 }, { xIn: -35.3, yIn: -23.55, headingDeg: 270 }],
        stopAtEnd: true },
      { waypoints: [{ xIn: -41.79, yIn: -23.55 }], constraints: kSlow },
    ],
    // back off east (point 1, the lift down), down the gap between the two Goals to a stop east of
    // the third Pin (point 3, the y reset), and west onto it
    toThirdPin: [
      { waypoints: [{ xIn: -41.79, yIn: -23.55, headingDeg: 270 }, { xIn: -35.3, yIn: -23.55 }], stopAtEnd: true },
      { waypoints: [{ xIn: -35.3, yIn: -35 }, { xIn: -37.5, yIn: -47.09 }], stopAtEnd: true },
      { waypoints: [{ xIn: -42.79, yIn: -47.09 }], constraints: kSlow },
    ],
    // the third Pin → the goal from the east again: at L3
    thirdToGoal: [
      { waypoints: [{ xIn: -42.79, yIn: -47.09, headingDeg: 270 }, { xIn: -36, yIn: -40 }, { xIn: -35.3, yIn: -23.55 }],
        stopAtEnd: true },
      { waypoints: [{ xIn: -41.79, yIn: -23.55 }], constraints: kSlow },
    ],
    // off the goal, so driver control starts clear of it, the lift going down 3in out
    offGoal: [
      { waypoints: [{ xIn: -41.79, yIn: -23.55, headingDeg: 270 }, { xIn: -35.79, yIn: -23.55 }] },
    ],
  };
  const TRIPLE_PIN_LEGS = ['toGoal', 'toSecondPin', 'secondToGoal', 'toThirdPin', 'thirdToGoal', 'offGoal'];

  // config.hpp's distance sensors (kFrontSensorForwardIn and the rest): the ones the Triple Pin
  // routine runs with (its `world`), since its resets were chosen for them. The simulator's other
  // routines have world.js's, a few inches from these.
  const ROBOT_SENSOR_MOUNTS = [
    { forwardIn: 0.59, rightIn: 3.99, facingDeg: 0 },
    { forwardIn: -0.22, rightIn: 4.82, facingDeg: 90 },
    { forwardIn: -1.86, rightIn: -4.49, facingDeg: 180 },
    { forwardIn: -0.85, rightIn: -4.88, facingDeg: 270 },
  ];
  // auton::use()'s in devices.cpp
  const ROBOT_WALL_RESET = { cornerMarginIn: 18.0, maxJumpIn: 8.0 };

  /**
   * Wall resets averaged over several readings, with the auton functions as they are, as
   * autons.cpp's averagedResets(): resetFront() and the rest each set x or y from one reading
   * alone, so `readings` of them kResetGapMs apart, averaged, average the readings. Each is carried
   * forward to now by what odometry has moved since, so the robot may move across the beams
   * meanwhile, not along them. `walls` is [[reset, 'x' | 'y'], ...], all read every round; a
   * reading a reset skips is left out. Returns how many took.
   */
  function* averagedResets(a, walls, readings = kResetReadings) {
    const at = (axis) => a.pose()[axis === 'x' ? 'xIn' : 'yIn'];
    const sums = walls.map(() => ({ total: 0, count: 0, last: 0 }));
    let took = 0;
    for (let i = 0; i < readings; ++i) {
      if (i > 0) yield* a.delay(kResetGapMs);
      walls.forEach(([reset, axis], k) => {
        const s = sums[k];
        s.total += s.count * (at(axis) - s.last);
        if (reset()) {
          s.total += at(axis);
          s.count++;
          took++;
        }
        s.last = at(axis);
      });
    }
    const p = Object.assign({}, a.pose());
    walls.forEach(([, axis], k) => {
      const s = sums[k];
      if (s.count) p[axis === 'x' ? 'xIn' : 'yIn'] = (s.total + s.count * (at(axis) - s.last)) / s.count;
    });
    a.setPose(p.xIn, p.yIn, p.headingDeg);
    return took;
  }

  /**
   * The routine, in the simulator. Where the C++ has setPose(-60.6, ...) after the rams, the
   * simulated robot's x against the wall is its own: world.js collides it as a circle as wide as
   * its longer side.
   */
  function* triplePinPaths(robot) {
    const a = A.create(robot, { wallReset: ROBOT_WALL_RESET });
    const m = M.create(robot, a);
    const goal = m.Goal.alliance;
    const s = TRIPLE_PIN.start;
    a.setPose(s.xIn, s.yIn, s.headingDeg);
    a.setMcl(false);
    // planned now, a few ms each on the robot, rather than once moving
    for (const leg of TRIPLE_PIN_LEGS) if (!a.planTrajectory(TRIPLE_PIN[leg])) return;
    m.setLiftLevel(goal, 1);
    // the Toggle, twice, on the drive encoders: out, and back harder than it went, until the wall
    // stops it
    for (let i = 0; i < 2; ++i) {
      yield* a.drive(10, { chain: true, exitError: 1 });
      yield* a.drive(-14, { timeoutMs: 750 });
    }
    const w = robot.world.settings;
    const here = a.pose();
    a.setPose(-w.fieldSizeIn / 2 + Math.max(w.robotWidthIn, w.robotLengthIn) / 2, here.yIn, here.headingDeg);

    // 1: the preload, at L1
    if ((yield* a.followTrajectory(TRIPLE_PIN.toGoal, kScoreFollow)).reason !== 'settled') return;
    yield* score(a, m);

    // 2: the second Pin, picked up by autoClaw on the way in, and onto the goal at L2
    a.runAsync(function* () { yield* a.followTrajectory(TRIPLE_PIN.toSecondPin); });
    if (!(yield* a.waitUntilWaypoint(1))) return;
    m.setLiftLevel(goal, 0); // backed off the goal: down, as R2 waits to
    m.setAutoClaw(true);
    yield* a.waitForMotion();
    if (!m.clawClosed()) return; // missed it: stop here
    m.setLiftLevel(goal, 2);
    yield* a.followTrajectory(TRIPLE_PIN.secondToGoal, kScoreFollow);
    yield* score(a, m);

    // 3: the third Pin, and onto the goal at L3
    a.runAsync(function* () { yield* a.followTrajectory(TRIPLE_PIN.toThirdPin); });
    if (!(yield* a.waitUntilWaypoint(1))) return;
    m.setLiftLevel(goal, 0);
    m.setAutoClaw(true);
    if (!(yield* a.waitUntilWaypoint(3))) return;
    yield* averagedResets(a, [[a.resetLeft, 'y']]); // east of the Pin: y, running west onto it
    yield* a.waitForMotion();
    if (!m.clawClosed()) return;
    yield* averagedResets(a, [[a.resetFront, 'x'], [a.resetLeft, 'y']]); // on it, still: for the drive back
    m.setLiftLevel(goal, 3);
    yield* a.followTrajectory(TRIPLE_PIN.thirdToGoal, kScoreFollow);
    yield* score(a, m);

    a.runAsync(function* () { yield* a.followTrajectory(TRIPLE_PIN.offGoal); });
    yield* a.waitUntilTraveled(3);
    m.setLiftLevel(goal, 0);
    yield* a.waitForMotion();
  }

  // R2's score from autonomous: up at its level, the dip, and let go
  function* score(a, m) {
    yield* m.waitForLift(1500);
    m.setLiftDeg(m.liftScoreDeg());
    yield* m.waitForLift(1000);
    m.setClawClosed(false);
    yield* a.delay(100); // the claw open before backing off
  }

  /** Plans the branch routine's pieces: { ok, error, trunk, straight, swerve, homeFromStraight, homeFromSwerve }. */
  function planBranch(models) {
    const out = { ok: false, error: '' };
    const step = (name, sections, options) => {
      if (out.error) return;
      const r = T.generateTrajectory(sections, models, options);
      if (!r.ok) out.error = `${name}: ${r.error}`;
      out[name] = r.trajectory;
    };
    step('trunk', BRANCH.trunk, { endSpeedInPerS: BRANCH.junctionSpeedInPerS });
    step('straight', BRANCH.straight, { from: out.trunk });
    step('swerve', BRANCH.swerve, { from: out.trunk });
    step('homeFromStraight', BRANCH.homeFromStraight, { from: out.straight });
    step('homeFromSwerve', BRANCH.homeFromSwerve, { from: out.swerve });
    out.ok = !out.error;
    return out;
  }

  const ROUTINES = [
    {
      id: 'tour',
      name: 'Field tour',
      description: 'moveToPose() around the middle, then a followPath() lap near the walls, then home.',
      start: { xIn: -48, yIn: -60, headingDeg: 0 },
      *run(robot) {
        const a = A.create(robot);
        a.setPose(-48, -60, 0);
        yield* a.moveToPoint(-48, -24);
        yield* a.moveToPose(-30, 26, 90);
        yield* a.moveToPose(28, 26, 180);
        yield* a.moveToPose(28, -26, 270);
        yield* a.moveToPose(-28, -28, 0);
        yield* a.followPath([
          [-28, -28], [-58, -10], [-58, 30], [-20, 60], [20, 60], [58, 30], [58, -20], [20, -58],
        ], { lookaheadIn: 12, maxVolts: 8 });
        yield* a.moveToPose(-48, -60, 0);
      },
    },
    {
      id: 'laps',
      name: 'Square laps',
      description: 'Three followPath() laps of a 60in square, each lap one closed path. Try it with Worn tracking wheels.',
      start: { xIn: -30, yIn: -30, headingDeg: 45 },
      *run(robot) {
        const a = A.create(robot);
        a.setPose(-30, -30, 45);
        const lap = [[-30, -30], [-30, 30], [30, 30], [30, -30], [-30, -30]];
        for (let i = 0; i < 3; ++i) {
          yield* a.followPath(lap, { lookaheadIn: 10, maxVolts: 9, timeoutMs: 20000 });
        }
      },
    },
    {
      id: 'sprints',
      name: 'Sprints',
      description: 'Full-speed moveToPoint() back and forth along the near wall, turning each time.',
      start: { xIn: -44, yIn: -56, headingDeg: 90 },
      *run(robot) {
        const a = A.create(robot);
        a.setPose(-44, -56, 90);
        for (let i = 0; i < 3; ++i) {
          yield* a.moveToPoint(44, -56);
          yield* a.turnTo(270);
          yield* a.moveToPoint(-44, -56);
          yield* a.turnTo(90);
        }
      },
    },
    {
      id: 'trajectory',
      name: 'Trajectory tour',
      description: 'followTrajectory() through a three-section chain in the background: fast, slower to a stop, ' +
        'then back home at a sharp angle. The routine waits for the stop (waitUntilWaypoint()) as a mechanism would.',
      start: { xIn: -48, yIn: -60, headingDeg: 0 },
      *run(robot) {
        const a = A.create(robot);
        a.setPose(-48, -60, 0);
        // planned at the start of the routine, from the drivetrain's axis models, as the robot does
        if (!a.planTrajectory(TOUR_CHAIN)) return;
        a.runAsync(function* () { yield* a.followTrajectory(TOUR_CHAIN); });
        // the stop at the end of the second section is point 4: where a claw would open
        yield* a.waitUntilWaypoint(4);
        yield* a.waitForMotion();
      },
    },
    {
      id: 'branch',
      name: 'Branch at speed',
      description: 'A trajectory that branches on the front distance sensor without stopping: clear ahead, ' +
        'straight on; turn on the Defender and it swerves right. Then home from wherever it stopped.',
      start: { xIn: -48, yIn: -60, headingDeg: 0 },
      *run(robot) {
        const a = A.create(robot);
        const dt = robot.drivetrain;
        a.setPose(-48, -60, 0);
        // every piece planned up front, so the junction only has to pick one. Branching drives the
        // drivetrain's own followTrajectory(), as examples/trajectory.cpp does
        const plan = planBranch(dt.axisModels);
        if (!plan.ok) return;
        let took = null;
        yield* dt.followTrajectory(plan.trunk, {
          // at the junction, on the motion's task: one sensor read, one pick
          next: () => {
            const front = robot.world.distanceSensor(0).mm / 25.4;
            took = front < BRANCH.clearAheadIn ? plan.swerve : plan.straight;
            return took;
          },
        });
        if (!took) return; // never reached the junction (blocked until it timed out)
        yield* dt.followTrajectory(took === plan.swerve ? plan.homeFromSwerve : plan.homeFromStraight);
      },
    },
    {
      id: 'triplePin',
      name: 'Triple Pin (paths)',
      description: 'autons.cpp\'s triplePinPaths(): three Pins stacked on the alliance Goal by the left wall, every ' +
        'drive a trajectory, with the lift, the claw and autoClaw, and averaged wall resets where a sensor sees clean ' +
        'wall, with the robot\'s own sensors (config.hpp). MCL off, as on the robot. Hover the result for each reset.',
      start: TRIPLE_PIN.start,
      // the robot's own distance sensors (config.hpp), which its resets were chosen for
      world: { sensors: ROBOT_SENSOR_MOUNTS.map((m) => L.mount(m.forwardIn, m.rightIn, m.facingDeg)) },
      // it turns the localizer off itself, and doesn't end where it started
      correctsWithMcl: false,
      run: triplePinPaths,
    },
  ];

  return {
    ROUTINES, TOUR_CHAIN, BRANCH, planBranch,
    TRIPLE_PIN, TRIPLE_PIN_LEGS, ROBOT_SENSOR_MOUNTS, ROBOT_WALL_RESET, averagedResets,
  };
});
