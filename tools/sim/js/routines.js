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
    module.exports = factory();
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.routines = factory();
  }
})(function () {
  'use strict';

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
  ];

  return { ROUTINES };
});
