/*
 * SapphireLib simulator: auton.js
 *
 * The auton functions (include/sapphirelib/auton/auton.hpp, src/sapphirelib/auton/auton.cpp)
 * for a simulated robot, so a routine in routines.js reads like one in src/robot/autons.cpp.
 * create(robot) gives one robot's set. Anything that takes time is a generator: `yield*` it
 * where the C++ just calls it.
 *
 *   C++ (autons.cpp)                          simulator (routines.js)
 *   setPose(-48, -60, 0);                     a.setPose(-48, -60, 0);
 *   moveToPoint(-48, -24, {.maxVolts = 8});   yield* a.moveToPoint(-48, -24, { maxVolts: 8 });
 *   turnTo(180, clockwise);                   yield* a.turnTo(180, a.clockwise);
 *   runAsync([] { moveToPoint(0, 48); });     a.runAsync(function* () { yield* a.moveToPoint(0, 48); });
 *   waitUntilTraveled(12);                    yield* a.waitUntilTraveled(12);
 *   pros::delay(250);                         yield* a.delay(250);
 *
 * Background motions (runAsync()) take turns with the routine a tick at a time, inside the auton
 * calls the routine waits in: wait with these (a.delay(), a.waitUntil()), never a bare `yield`,
 * or the background motion stands still meanwhile.
 *
 * Plain script: window.SIM.auton in a browser, require('./auton.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./mcl.js'), require('./trajectory.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.auton = factory(root.SIM.mcl, root.SIM.trajectory);
  }
})(function (L, T) {
  'use strict';

  const kLoopMs = 10;
  // how close counts as there, by default and when chaining (Options::exitError)
  const kExitIn = 1.0;
  const kChainExitIn = 3.0;
  const kExitDeg = 2.0;
  const kChainExitDeg = 5.0;
  // a sensor counts as facing a side within this many degrees of it
  const kFacingToleranceDeg = 45.0;

  /** auton.hpp's Options: { timeoutMs, maxVolts, chain, exitError }, all optional. */
  function exitFor(options, error, chainError) {
    const exit = {
      errorThreshold: options.exitError > 0 ? options.exitError : (options.chain ? chainError : error),
      maxVolts: options.maxVolts === undefined ? 12.0 : options.maxVolts,
      chain: options.chain === true,
    };
    if (options.timeoutMs > 0) exit.timeoutMs = options.timeoutMs;
    return exit;
  }

  /** followPath()'s points as auton.hpp takes them, {{x, y}, ...}: [[x, y], ...] or [{ xIn, yIn }, ...]. */
  function waypoints(points) {
    return points.map((p) => (Array.isArray(p) ? { xIn: p[0], yIn: p[1] } : { xIn: p.xIn, yIn: p.yIn }));
  }

  /**
   * One robot's auton functions. `settings` is auton::Settings: { wallReset } (WallResetConfig's
   * fields, e.g. { cornerMarginIn: 18 }).
   */
  function create(robot, settings = {}) {
    const dt = robot.drivetrain;
    const odometry = robot.odometry;
    const localizer = robot.localizer;
    const wallReset = L.wallResetConfig(settings.wallReset || {});

    // runAsync()'s blocks, in order: the running one, and the ones waiting their turn
    let current = null;
    const waiting = [];
    let inBackground = false;
    // every motion takes the next number as it starts; the running block's first motion gets
    // blockFirst, so the waits never look at the motion before it
    let motionCount = 0;
    let blockFirst = 1;
    let start = { travelledIn: 0, turnedDeg: 0 };
    // the latest motion's trajectory, if it's one, and its number on trajectoryProgress()
    let plan = null;
    let planMotion = 0;
    // followTrajectory()'s plans, by the sections they're for
    const plans = new Map();

    const isMotionRunning = () => current !== null || waiting.length > 0;

    // one tick of the background motions, if there are any
    function stepBackground() {
      if (!current && waiting.length) {
        current = waiting.shift();
        blockFirst = motionCount + 1;
      }
      if (!current) return;
      inBackground = true;
      try {
        if (current.next().done) current = null;
      } finally {
        inBackground = false;
      }
    }

    // a motion getting ready: waits out the background motions if it isn't one of them
    function* begin() {
      if (!inBackground) {
        while (isMotionRunning()) {
          stepBackground();
          if (isMotionRunning()) yield;
        }
      }
      const snapshot = odometry.snapshot();
      start = { travelledIn: snapshot.travelledIn, turnedDeg: snapshot.turnedDeg };
      plan = null;
      motionCount++;
    }

    // waits, while background motions run, for done() once the running block's first motion has started
    function* waitWhileRunning(done) {
      if (inBackground || !isMotionRunning()) return false;
      for (;;) {
        stepBackground();
        if (motionCount >= blockFirst && done()) return true;
        if (!isMotionRunning()) return false;
        yield;
      }
    }

    function planFor(sections) {
      const models = dt.axisModels;
      const known = plans.get(sections);
      if (known && known.models === models) return known.result;
      const result = T.generateTrajectory(sections, models);
      plans.set(sections, { models, result });
      return result;
    }

    function resetFacing(facingDeg) {
      const mounts = localizer.filter.sensors;
      let sensor = -1;
      let closest = kFacingToleranceDeg;
      mounts.forEach((m, i) => {
        const off = Math.abs(L.wrapDegrees180(m.facingDeg - facingDeg));
        if (off <= closest) {
          closest = off;
          sensor = i;
        }
      });
      return sensor >= 0 && localizer.resetFromWall(sensor, wallReset).ok;
    }

    const a = {
      clockwise: 'clockwise',
      counterclockwise: 'counterclockwise',

      setPose(xIn, yIn, headingDeg) {
        odometry.setPose({ xIn, yIn, headingDeg });
      },

      pose() {
        return odometry.getPose();
      },

      *drive(inches, options = {}) {
        yield* begin();
        return yield* dt.driveDistance(inches, exitFor(options, kExitIn, kChainExitIn));
      },

      /** turnTo(heading, options), or turnTo(heading, a.clockwise / a.counterclockwise, options). */
      *turnTo(headingDeg, direction = 'shortest', options = {}) {
        if (typeof direction === 'object') return yield* a.turnTo(headingDeg, 'shortest', direction);
        yield* begin();
        return yield* dt.turnToHeading(headingDeg, exitFor(options, kExitDeg, kChainExitDeg), direction);
      },

      *turnToPoint(xIn, yIn, options = {}) {
        yield* begin();
        const now = odometry.getPose();
        const bearingDeg = L.wrapDegrees360(Math.atan2(xIn - now.xIn, yIn - now.yIn) * 180.0 / Math.PI);
        return yield* dt.turnToHeading(bearingDeg, exitFor(options, kExitDeg, kChainExitDeg));
      },

      *moveToPoint(xIn, yIn, options = {}) {
        yield* begin();
        return yield* dt.moveToPoint(xIn, yIn, exitFor(options, kExitIn, kChainExitIn));
      },

      *moveToPose(xIn, yIn, headingDeg, options = {}) {
        yield* begin();
        const exit = {
          positionErrorThresholdIn: options.exitError > 0 ? options.exitError
            : (options.chain ? kChainExitIn : kExitIn),
          headingErrorThresholdDeg: options.chain ? kChainExitDeg : kExitDeg,
          maxVolts: options.maxVolts === undefined ? 12.0 : options.maxVolts,
          chain: options.chain === true,
        };
        if (options.timeoutMs > 0) exit.timeoutMs = options.timeoutMs;
        return yield* dt.moveToPose(xIn, yIn, headingDeg, exit);
      },

      *strafe(inches, options = {}) {
        yield* begin();
        const now = odometry.getPose();
        const headingRad = now.headingDeg * Math.PI / 180.0;
        return yield* dt.moveToPoint(now.xIn + inches * Math.cos(headingRad), now.yIn - inches * Math.sin(headingRad),
          exitFor(options, kExitIn, kChainExitIn));
      },

      /** followPath(points, { timeoutMs, maxVolts, lookaheadIn, chain }): PathOptions. */
      *followPath(points, options = {}) {
        yield* begin();
        const maxVolts = options.maxVolts === undefined ? 8.0 : options.maxVolts;
        return yield* dt.followPath(waypoints(points), {
          lookaheadIn: options.lookaheadIn === undefined ? 10.0 : options.lookaheadIn,
          cruiseVoltage: maxVolts,
          timeoutMs: options.timeoutMs === undefined ? 10000 : options.timeoutMs,
          finalExit: { errorThreshold: options.chain ? kChainExitIn : kExitIn, maxVolts, chain: options.chain === true },
        });
      },

      *followTrajectory(sections, config = {}) {
        yield* begin();
        const result = planFor(sections);
        if (!result.ok) return { reason: 'aborted', finalError: 0, elapsedMs: 0 };
        planMotion = dt.trajectoryProgress().motion + 1;
        plan = result;
        return yield* dt.followTrajectory(result.trajectory, config);
      },

      planTrajectory(sections) {
        return planFor(sections).ok;
      },

      /** runAsync(function* () { yield* a.moveToPoint(0, 48); }): its blocks run one after another. */
      runAsync(motions) {
        if (inBackground) throw new Error('runAsync() inside runAsync(): yield* the motions instead');
        waiting.push(motions());
      },

      isMotionRunning,

      *waitForMotion() {
        if (inBackground) return;
        while (isMotionRunning()) {
          stepBackground();
          if (isMotionRunning()) yield;
        }
      },

      cancelMotion() {
        waiting.length = 0;
        if (current) {
          current.return();
          current = null;
          dt.stop();
        }
      },

      *waitUntilTraveled(inches) {
        return yield* waitWhileRunning(() => odometry.snapshot().travelledIn - start.travelledIn >= inches);
      },

      *waitUntilTurned(degrees) {
        return yield* waitWhileRunning(() => odometry.snapshot().turnedDeg - start.turnedDeg >= degrees);
      },

      *waitUntilNear(xIn, yIn, radiusIn) {
        return yield* waitWhileRunning(() => {
          const now = odometry.getPose();
          return Math.hypot(xIn - now.xIn, yIn - now.yIn) <= radiusIn;
        });
      },

      *waitUntilWaypoint(waypoint) {
        return yield* waitWhileRunning(() => {
          if (!plan || waypoint >= plan.waypointIn.length) return false;
          const progress = dt.trajectoryProgress();
          return progress.motion === planMotion && progress.distanceIn >= plan.waypointIn[waypoint] - 1e-6;
        });
      },

      resetFront: () => resetFacing(0.0),
      resetBack: () => resetFacing(180.0),
      resetLeft: () => resetFacing(270.0),
      resetRight: () => resetFacing(90.0),

      setMcl(on) {
        localizer.setCorrectionEnabled(on);
      },

      /** pros::delay(), with the background motions running meanwhile. */
      *delay(ms) {
        for (let t = 0; t < ms; t += kLoopMs) {
          stepBackground();
          yield;
        }
      },

      /** sapphirelib::waitUntil(done, timeoutMs), with the background motions running meanwhile. */
      *waitUntil(done, timeoutMs) {
        for (let t = 0; ; t += kLoopMs) {
          if (done()) return true;
          if (timeoutMs > 0 && t >= timeoutMs) return false;
          stepBackground();
          yield;
        }
      },
    };
    return a;
  }

  return { create, exitFor, waypoints };
});
