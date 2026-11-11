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
    module.exports = factory(require('./trajectory.js'), require('./auton.js'), require('./macros.js'), require('./mcl.js'),
      require('./field.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.routines = factory(root.SIM.trajectory, root.SIM.auton, root.SIM.macros, root.SIM.mcl, root.SIM.field);
  }
})(function (T, A, M, L, F) {
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

  // --- Triple Pin on points: autons.cpp's triplePinPoints() ----------------------------------------
  //
  // Three Pins stacked on the alliance Goal by the left wall, at (-47.09, -23.55), "the goal": the
  // preload at L1 from the north, the Pin standing at (-23.55, -23.55) at L2 from the east, and the
  // one at (-47.09, -47.09) at L3 from the south. Every drive is a turn to face where it goes and a
  // straight drive there (turnToPoint(), moveToPoint()), as autons.cpp's triplePin() drives, so each
  // stop is a point to change in TRIPLE_PIN (autons.cpp's triplePinPoints() has the same numbers).
  // The goal and the two Pins are where field.js has them (the manual's Appendix A): ontoGoal() and
  // ontoPin() take one and work out where the robot stops for it, facing it.
  //
  // The goal aligner in the front of the robot is what a Goal or a Pin stops against: its back is
  // kAlignerDepthIn (3.5in) behind the front edge, kFrontIn (8.35in) ahead of the tracking center,
  // so a seated Goal's middle is kGoalSeatIn (7.65in) ahead and a seated Pin's kPinSeatIn (5.65in),
  // half of each one's width past the aligner's back. The claw holds a Pin there (macros.js's
  // kClawReachIn) and lets go over a seated Goal (kClawReachUpIn).
  //
  // No wall resets, and the localizer's corrections off (setMcl(false)): odometry alone carries the
  // routine from the rams at the start.
  //
  // a seated Goal's middle and a seated Pin's, ahead of the tracking center: the front edge is
  // kFrontIn ahead, the goal aligner's back kAlignerDepthIn behind that, and a Goal (5.61in across
  // the flats) or a Pin (1.6in across) stops against it with its middle half its width further on
  // (autons.cpp's kGoalSeatIn and kPinSeatIn)
  const kFrontIn = 8.35;
  const kAlignerDepthIn = 3.5;
  // the robot: 13.75in wide and 17in long (the user, 2026-11-10), its front edge kFrontIn ahead of
  // the tracking center, so its back 8.65in behind it
  const kRobotWidthIn = 13.75;
  const kRobotLengthIn = 17.0;
  const kGoalSeatIn = kFrontIn - kAlignerDepthIn + 5.61 / 2; // 7.65
  const kPinSeatIn = kFrontIn - kAlignerDepthIn + 1.6 / 2; // 5.65
  // Every turn chains (exits within 5 degrees, still turning, no settle): settling each one to 2
  // degrees cost 1.4s over the routine and put the last score past 14.5s. What a turn leaves, the
  // drive after it takes out: ontoGoal()'s and ontoPin()'s approaches keep turning to face their
  // target as they go (autons.cpp's kTurn)
  const kTurn = { chain: true };
  // ontoGoal(): an approach facing the goal's middle, at kGoalApproach.cruiseInPerS, ending
  // kGoalShortIn short of where the goal seats; then on at no more than 6V until the goal stops the
  // robot and the timeout ends the press. A moveToPoint() there holds the heading the turn left it
  // on, a few degrees off, and the goal landed 1 to 3in off the aligner's middle. Ending further
  // than kGoalPressReachIn from its end means something stopped the robot well short of the goal:
  // no press and no score
  const kGoalShortIn = 1.0;
  const kGoalApproach = { cruiseInPerS: 30.0, accelInPerS2: 120.0, decelInPerS2: 120.0,
    endShortIn: kGoalSeatIn + kGoalShortIn, steerMinIn: kGoalSeatIn + 3.0, maxLagIn: 2.0, timeoutMs: 1500 };
  const kGoalPressReachIn = 8.0;
  const kGoalPressIn = 3.0;
  // a score's claw opens once the lift reaches the dip or this long into it, as R2's
  // (macros.cpp's kScoreReleaseMs)
  const kScoreReleaseMs = 100;
  // how far off the goal the robot backs before the lift comes down: the claw clear of the stack
  const kLiftClearIn = 3.5;
  // the drives to the stops, where the robot turns for its next run: chained, ending the first tick
  // within 1in, still rolling, not settling 200ms (1.1s over the routine, the same scores). With the
  // robot's models and gains it stops within 0.5in of every stop (autons.cpp's kStop)
  const kStop = { chain: true, exitError: 1.0 };
  // The last third of each run onto a Pin at autoClaw speed (autons.cpp's kAutoClawFromRun and the
  // rest): a moveToPoint() for two thirds of the way (chained: it hands off within 3in of there,
  // slowing as it comes), then autoClaw on, whose speed limit holds the approach that drives the
  // rest: facing the Pin at a steady kPinApproach.cruiseInPerS, through to kPinPastIn past where it
  // seats, so autoClaw's lead (which assumes the speed holds) times the grab right. autoClaw off
  // again once the claw has the Pin: its speed limit would hold the turn after it to 5V
  const kAutoClawFromRun = 2 / 3;
  const kPinPastIn = 3.0;
  const kPinApproach = { cruiseInPerS: 18.0, accelInPerS2: 60.0, decelInPerS2: 80.0, endShortIn: kPinSeatIn - kPinPastIn,
    steerMinIn: kPinSeatIn + 2.0, maxLagIn: 2.0, timeoutMs: 2500 };
  // a Pin the approach stopped short of: the rest of the way straight ahead, at autoClaw's 5V, until
  // the claw shuts, stopping kPinCreepAfterGrabMs after it does (autons.cpp's kPinCreepIn and the
  // rest)
  const kPinCreepIn = 2.5;
  const kPinCreepMs = 600;
  const kPinCreepAfterGrabMs = 125;

  // autons.cpp's triplePinPoints(), point for point
  const TRIPLE_PIN = {
    start: { xIn: -60.6, yIn: -5, headingDeg: 90 },
    // the field's (field.js, the manual's Appendix A)
    goal: { xIn: -47.09, yIn: -23.55 },
    secondPin: { xIn: -23.55, yIn: -23.55 },
    thirdPin: { xIn: -47.09, yIn: -47.09 },
    // the stops between, where it turns, each on the line it gets there along, so it stops on the
    // point and not beside it. aboveGoal: straight out from the wall (no turn against it), near the
    // line the old trajectories ran onto the goal along (170). afterL1: level with y = 0, straight
    // back off the goal: on the line from the goal's middle out through aboveGoal. afterSecondPin:
    // 8.3in back off the second Pin, up the line it ran in along from afterL1. afterL2: straight back
    // off the goal, on the line from its middle out through afterSecondPin, far enough out that the
    // third Pin's run passes the goal 3in clear. belowGoal: the old trajectories' stop 16in below
    // the goal, facing north
    aboveGoal: { xIn: -50, yIn: -5 },
    afterL1: { xIn: -50.8, yIn: 0 },
    afterSecondPin: { xIn: -29.8, yIn: -18.1 },
    afterL2: { xIn: -27.1, yIn: -17.2 },
    belowGoal: { xIn: -47.09, yIn: -39.55 },
    // and where it ended, backed off the goal for driver control
    afterL3: { xIn: -47.09, yIn: -35.5 },
  };

  // config.hpp's distance sensors (kFrontSensorForwardIn and the rest): the robot's own, for the
  // routine's `world` (MCL is off, so only the page's localizer readout uses them). The simulator's
  // other routines have world.js's, a few inches from these.
  const ROBOT_SENSOR_MOUNTS = [
    { forwardIn: 0.59, rightIn: 3.99, facingDeg: 0 },
    { forwardIn: -0.22, rightIn: 4.82, facingDeg: 90 },
    { forwardIn: -1.86, rightIn: -4.49, facingDeg: 180 },
    { forwardIn: -0.85, rightIn: -4.88, facingDeg: 270 },
  ];

  // `inches` short of `target` on the line from `from` to it
  function shortOf(from, target, inches) {
    const d = Math.hypot(target.xIn - from.xIn, target.yIn - from.yIn);
    const k = d > 0 ? Math.max(d - inches, 0) / d : 0;
    return { xIn: from.xIn + (target.xIn - from.xIn) * k, yIn: from.yIn + (target.yIn - from.yIn) * k };
  }

  /**
   * The routine, in the simulator. Where the C++ has setPose(-60.6, ...) after the rams, the
   * simulated robot's x against the wall is its own: world.js collides it as a circle as wide as
   * its longer side.
   */
  function* triplePinPoints(robot) {
    const a = A.create(robot);
    const m = M.create(robot, a, { field: F.override(robot.world.settings.fieldSizeIn) });
    const goal = m.Goal.alliance;
    const P = TRIPLE_PIN;
    a.setPose(P.start.xIn, P.start.yIn, P.start.headingDeg);
    // odometry alone: the localizer's corrections off for the whole routine
    a.setMcl(false);
    m.setLiftLevel(goal, 1);
    // the Toggle, twice, on the drive encoders, as autons.cpp's triplePin() rams it: out until
    // within 5in of the 10, and back harder than it went until the 600ms runs out against the wall
    for (let i = 0; i < 2; ++i) {
      yield* a.drive(10, { chain: true, exitError: 5 });
      yield* a.drive(-14, { timeoutMs: 600 });
    }
    const w = robot.world.settings;
    const here = a.pose();
    a.setPose(-w.fieldSizeIn / 2 + Math.max(w.robotWidthIn, w.robotLengthIn) / 2, here.yIn, here.headingDeg);

    // 1: straight out from the wall to above the goal (no turn pinned against the wall: a back
    // corner would swing into it), and the preload onto it from the north (170) at L1
    yield* a.moveToPoint(P.aboveGoal.xIn, P.aboveGoal.yIn);
    if (!(yield* ontoGoal(a, P.goal))) return;
    yield* score(a, m);

    // 2: back off the goal to its stop, the second Pin (125), back off the Pin to its stop, then
    // onto the goal from the east (260) at L2
    yield* offGoalTo(a, m, P.afterL1);
    if (!(yield* ontoPin(a, m, P.secondPin))) return; // missed it: stop here
    m.setLiftLevel(goal, 2);
    yield* a.drive(-behindIn(a, P.afterSecondPin), kStop);
    if (!(yield* ontoGoal(a, P.goal))) return;
    yield* score(a, m);

    // 3: back off the goal to its stop, the third Pin (215), then to below the goal and onto it from
    // the south at L3
    yield* offGoalTo(a, m, P.afterL2);
    if (!(yield* ontoPin(a, m, P.thirdPin))) return;
    m.setLiftLevel(goal, 3);
    yield* driveTo(a, P.belowGoal);
    if (!(yield* ontoGoal(a, P.goal))) return;
    yield* score(a, m);

    // back off the goal, so driver control starts clear of it
    yield* offGoalTo(a, m, P.afterL3, {}); // settled, not chained: the routine's last motion
  }

  // autons.cpp's ontoGoal(): turn to face the goal's middle, the approach on to just short of where
  // it seats in the aligner, then on at no more than 6V until the goal stops the robot and the
  // timeout ends the press. False, without the press, if something stopped it well short
  function* ontoGoal(a, goal) {
    yield* a.turnToPoint(goal.xIn, goal.yIn, kTurn);
    const result = yield* a.approach(goal.xIn, goal.yIn, kGoalApproach);
    if (result.finalError > kGoalPressReachIn) return false;
    yield* a.drive(kGoalPressIn, { timeoutMs: 300, maxVolts: 6 });
    return true;
  }

  // autons.cpp's behindIn(): how far behind the robot `point` is, along its heading (backTo() is
  // a drive back that far)
  function behindIn(a, point) {
    const here = a.pose();
    const r = here.headingDeg * Math.PI / 180;
    return -((point.xIn - here.xIn) * Math.sin(r) + (point.yIn - here.yIn) * Math.cos(r));
  }

  // autons.cpp's driveTo(): turn to face `point`, then straight there
  function* driveTo(a, point) {
    yield* a.turnToPoint(point.xIn, point.yIn, kTurn);
    yield* a.moveToPoint(point.xIn, point.yIn, kStop);
  }

  // autons.cpp's offGoalTo(): straight back off the goal, holding the heading, until level with
  // `point`, the lift down once the claw has cleared the stack. Along the heading, not to the
  // point: each stop is on the line straight back from the goal through where it came onto it
  function* offGoalTo(a, m, point, options = kStop) {
    const backIn = behindIn(a, point);
    a.runAsync(function* () { yield* a.drive(-backIn, options); });
    yield* a.waitUntilTraveled(kLiftClearIn);
    m.setLiftLevel(m.Goal.alliance, 0);
    yield* a.waitForMotion();
  }

  // autons.cpp's ontoPin(): turn to face the Pin, a moveToPoint() for two thirds of the way to
  // where it seats, then autoClaw on and the approach at its speed for the last third
  // (kPinApproach). The robot stops kPinCreepAfterGrabMs after the claw shuts; an approach that ends
  // with the claw still open creeps on until it shuts. autoClaw off at the end. False if the claw
  // never shuts
  function* ontoPin(a, m, pin) {
    yield* a.turnToPoint(pin.xIn, pin.yIn, kTurn);
    const here = a.pose();
    const runIn = Math.max(Math.hypot(pin.xIn - here.xIn, pin.yIn - here.yIn) - kPinSeatIn, 0);
    const handoff = shortOf(here, pin, kPinSeatIn + runIn * (1 - kAutoClawFromRun));
    yield* a.moveToPoint(handoff.xIn, handoff.yIn, { chain: true });
    m.setAutoClaw(true); // before the approach starts, so its speed limit holds it
    a.runAsync(function* () { yield* a.approach(pin.xIn, pin.yIn, kPinApproach); });
    yield* untilGrabbed(a, m);
    if (!m.clawClosed()) {
      a.runAsync(function* () { yield* a.drive(kPinCreepIn, { timeoutMs: kPinCreepMs }); });
      yield* untilGrabbed(a, m);
    }
    m.setAutoClaw(false); // full speed for what comes next
    return m.clawClosed();
  }

  // the running motion until the claw shuts (and kPinCreepAfterGrabMs on, then stopped) or it ends
  function* untilGrabbed(a, m) {
    yield* a.waitUntil(() => m.clawClosed() || !a.isMotionRunning(), 0);
    if (m.clawClosed()) {
      yield* a.delay(kPinCreepAfterGrabMs); // the jaws shut on it before the robot stops
      a.cancelMotion();
    }
    yield* a.waitForMotion();
  }

  /**
   * What the robot's distance sensors see, as the world's elements, every one in the localizer's
   * map as devices.cpp's fieldMap() has it: the Loaders; the center goal, tall enough to be seen
   * itself; the Pin in each medium goal and each Midfield Cup (the goals and Cups below the
   * sensors, but the Pins in them not); and the stack on the goal the routine scores on. Solid only
   * the Loaders: the robot never reaches the rest, and goalStopIn stops it on the goal.
   */
  function robotSeen() {
    const field = F.override();
    const T = field.tilePitchIn;
    const square = (name, xIn, yIn, acrossIn) => ({ name, minXIn: xIn - acrossIn / 2, minYIn: yIn - acrossIn / 2,
      maxXIn: xIn + acrossIn / 2, maxYIn: yIn + acrossIn / 2, inWorld: true, inMap: true, solid: false, field: true });
    const seen = F.obstacleBoxes(field).filter((e) => /loader/.test(e.name))
      .map((e) => Object.assign(e, { inWorld: true, inMap: true, field: true }));
    seen.push(square('center goal', 0, 0, 5.61));
    for (const [x, y] of [[-1, 2], [-2, 1], [2, -1], [1, -2]]) seen.push(square('medium goal pin', x * T, y * T, field.pinIn));
    for (const [x, y] of [[-1, 0], [0, -1], [0, 1], [1, 0]]) seen.push(square('midfield cup and pin', x * T, y * T, field.cupIn));
    seen.push(square('the stack', -2 * T, -T, field.pinIn));
    return seen;
  }

  /**
   * The Toggles, on top of the middle of every wall: they read the wall 3 to 4in long (SL000040 head
   * on, SL000070's back sensor 4in long at a slant), so 3.3in further back here, and they're
   * devices.cpp's blind stretches.
   */
  function toggleRecesses() {
    const field = F.override();
    const h = field.halfIn;
    return field.toggles.map((t) => {
      const along = t.lengthIn / 2;
      return t.angleDeg === 90
        ? { x1In: t.xIn, y1In: t.yIn - along, x2In: t.xIn, y2In: t.yIn + along, depthIn: 3.3, inMap: true }
        : { x1In: t.xIn - along, y1In: t.yIn, x2In: t.xIn + along, y2In: t.yIn, depthIn: 3.3, inMap: true };
    }).map((r) => Object.assign(r, { x1In: Math.max(-h, Math.min(h, r.x1In)), x2In: Math.max(-h, Math.min(h, r.x2In)) }));
  }

  // R2's score from autonomous: up at its level, the dip, and let go as R2 does (kScoreReleaseMs)
  function* score(a, m) {
    yield* m.waitForLift(1500);
    m.setLiftDeg(m.liftScoreDeg());
    yield* m.waitForLift(kScoreReleaseMs);
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
      name: 'Triple Pin (points)',
      description: 'autons.cpp\'s triplePinPoints(): three Pins stacked on the alliance Goal by the left wall, every ' +
        'drive a turn to face a point and a straight drive to it (turnToPoint(), moveToPoint()), with the lift, the ' +
        'claw and autoClaw, the last third onto each Pin at autoClaw speed. Odometry alone: no wall resets, MCL off, ' +
        'as on the robot.',
      start: TRIPLE_PIN.start,
      // the robot's own distance sensors (config.hpp), and the Goals, which stop the robot as it
      // presses one into the goal aligner (kGoalSeatIn). What the sensors see is what the robot's
      // do (robotSeen()), and the localizer's map is devices.cpp's fieldMap(), Toggles and all
      world: {
        // the robot's own size (kRobotWidthIn x kRobotLengthIn)
        robotWidthIn: kRobotWidthIn,
        robotLengthIn: kRobotLengthIn,
        sensors: ROBOT_SENSOR_MOUNTS.map((m) => L.mount(m.forwardIn, m.rightIn, m.facingDeg)),
        goals: F.override().goals.map((g) => ({ xIn: g.xIn, yIn: g.yIn })),
        goalStopIn: kGoalSeatIn,
        elements: robotSeen(),
        recesses: toggleRecesses(),
      },
      // devices.cpp's localizer: tune.cpp's correction rate, and its 33ms period
      localizer: { config: { maxCorrectionRateInPerS: 18.0 }, periodMs: 33 },
      // it doesn't end where it started
      correctsWithMcl: false,
      run: triplePinPoints,
    },
  ];

  return {
    ROUTINES, TOUR_CHAIN, BRANCH, planBranch,
    TRIPLE_PIN, kGoalSeatIn, kPinSeatIn, kFrontIn, kRobotWidthIn, kRobotLengthIn, ROBOT_SENSOR_MOUNTS,
    kAutoClawFromRun, kPinApproach, kPinPastIn, kGoalApproach,
  };
});
