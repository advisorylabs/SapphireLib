/*
 * SapphireLib simulator: routines.js
 *
 * Autonomous routines for the simulator, written the way src/robot/autons.cpp
 * would write them: setPose() to the real start, then motions. Each one is a
 * generator, and `yield*` a motion is the C++'s blocking call. Every routine
 * ends where it started, so "how far off is it now" is a fair question to
 * ask of odometry alone and of odometry with MCL.
 *
 * To try your own: copy one, change the points, and add it to ROUTINES. Field
 * coordinates have the origin in the middle of the field (walls at +-70.25in),
 * +y downfield, headings clockwise from +y.
 *
 * Plain script: window.SIM.routines in a browser, require('./routines.js') in
 * Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./trajectory.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.routines = factory(root.SIM.trajectory);
  }
})(function (T) {
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

  // A branch at speed, the way autons.cpp's Branch At Speed routine is written: a trunk up the left
  // side that ends at 30in/s, and two branches carrying on from its end. The front distance sensor
  // picks one at the junction: clear ahead, straight on; something within 36in (the defender), a
  // swerve to the right. Then home from wherever that branch stopped, a plain trajectory planned
  // from its end (branching with a stop).
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
        const dt = robot.drivetrain;
        robot.odometry.setPose(this.start);
        yield* dt.moveToPoint(-48, -24);
        yield* dt.moveToPose(-30, 26, 90);
        yield* dt.moveToPose(28, 26, 180);
        yield* dt.moveToPose(28, -26, 270);
        yield* dt.moveToPose(-28, -28, 0);
        yield* dt.followPath([
          { xIn: -28, yIn: -28 }, { xIn: -58, yIn: -10 }, { xIn: -58, yIn: 30 }, { xIn: -20, yIn: 60 },
          { xIn: 20, yIn: 60 }, { xIn: 58, yIn: 30 }, { xIn: 58, yIn: -20 }, { xIn: 20, yIn: -58 },
        ], { lookaheadIn: 12, cruiseVoltage: 8 });
        yield* dt.moveToPose(-48, -60, 0);
      },
    },
    {
      id: 'laps',
      name: 'Square laps',
      description: 'Three followPath() laps of a 60in square, each lap one closed path. Try it with Worn tracking wheels.',
      start: { xIn: -30, yIn: -30, headingDeg: 45 },
      *run(robot) {
        const dt = robot.drivetrain;
        robot.odometry.setPose(this.start);
        const lap = [
          { xIn: -30, yIn: -30 }, { xIn: -30, yIn: 30 }, { xIn: 30, yIn: 30 }, { xIn: 30, yIn: -30 },
          { xIn: -30, yIn: -30 },
        ];
        for (let i = 0; i < 3; ++i) {
          yield* dt.followPath(lap, { lookaheadIn: 10, cruiseVoltage: 9, timeoutMs: 20000 });
        }
      },
    },
    {
      id: 'sprints',
      name: 'Sprints',
      description: 'Full-speed moveToPoint() back and forth along the near wall, turning each time.',
      start: { xIn: -44, yIn: -56, headingDeg: 90 },
      *run(robot) {
        const dt = robot.drivetrain;
        robot.odometry.setPose(this.start);
        for (let i = 0; i < 3; ++i) {
          yield* dt.moveToPoint(44, -56);
          yield* dt.turnToHeading(270);
          yield* dt.moveToPoint(-44, -56);
          yield* dt.turnToHeading(90);
        }
      },
    },
    {
      id: 'trajectory',
      name: 'Trajectory tour',
      description: 'followTrajectory() through a three-section chain: fast, slower to a stop, then back home at a sharp angle.',
      start: { xIn: -48, yIn: -60, headingDeg: 0 },
      *run(robot) {
        const dt = robot.drivetrain;
        robot.odometry.setPose(this.start);
        // planned at the start of the routine, from the drivetrain's axis models, as the robot does
        const tour = T.generateTrajectory(TOUR_CHAIN, dt.axisModels);
        if (!tour.ok) return;
        yield* dt.followTrajectory(tour.trajectory);
      },
    },
    {
      id: 'branch',
      name: 'Branch at speed',
      description: 'A trajectory that branches on the front distance sensor without stopping: clear ahead, ' +
        'straight on; turn on the Defender and it swerves right. Then home from wherever it stopped.',
      start: { xIn: -48, yIn: -60, headingDeg: 0 },
      *run(robot) {
        const dt = robot.drivetrain;
        robot.odometry.setPose(this.start);
        // every piece planned up front, so the junction only has to pick one
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
  ];

  return { ROUTINES, TOUR_CHAIN, BRANCH, planBranch };
});
