/*
 * SapphireLib simulator: sim.js
 *
 * Puts a World and a Robot together and runs them: one 10ms tick at a time,
 * the world's physics first, then the robot's tasks (odometry every tick, the
 * localizer every 50ms), then the robot program itself, either driver control
 * or the running autonomous routine. It also keeps the numbers the page
 * charts: how far raw odometry, the corrected pose, and the particles'
 * estimate each are from where the robot really is.
 *
 * The page (app.js) and the headless tests (test/sim.test.js) both drive
 * this, so what the tests check is what the page shows.
 *
 * Plain script: window.SIM.sim in a browser, require('./sim.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./mcl.js'), require('./world.js'), require('./robot.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.sim = factory(root.SIM.mcl, root.SIM.world, root.SIM.robot);
  }
})(function (L, W, R) {
  'use strict';

  const kTickMs = R.kLoopMs;
  const kHistorySamples = 1200; // 60s at one sample per 50ms

  /** Everything the page can change, with the robot program's defaults. */
  function defaultOptions() {
    return {
      world: W.defaultSettings(),
      localizer: {},
      localizerPeriodMs: 50,
      correctOdometry: true,
      useAutoTune: true,
      // which of the world's elements the localizer's map includes
      elementsInMap: false,
      start: { xIn: -48, yIn: -60, headingDeg: 0 },
    };
  }

  class Simulation {
    constructor(options = defaultOptions()) {
      this.options = options;
      this.reset(options.start);
    }

    /** A fresh world and robot program, the robot sitting at `start` and told so. */
    reset(start = this.options.start) {
      const o = this.options;
      this.world = new W.World(o.world, start);
      this.gains = this.currentGains();
      this.robot = new R.Robot(this.world, {
        odometryConfig: { verticalOffsetIn: o.world.verticalOffsetIn, horizontalOffsetIn: o.world.horizontalOffsetIn },
        localizer: o.localizer,
        gains: this.gains,
        startPose: start,
        mapFor: () => this.localizerMap(),
      });
      this.robot.localizerPeriodMs = o.localizerPeriodMs;
      this.robot.localizer.setCorrectionEnabled(o.correctOdometry);
      this.program = null;
      this.programName = null;
      this.programResult = null;
      this.pendingStep = null;
      this.driverInput = { throttle: 0, strafe: 0, turn: 0 };
      this.history = [];
      this.trails = { truth: [], raw: [], corrected: [], estimate: [] };
      this.robot.localizer.update();
    }

    /** The localizer's map: the walls, plus the elements it's told about. */
    localizerMap() {
      const o = this.options;
      const map = L.FieldMap.centered(o.world.fieldSizeIn);
      if (o.elementsInMap) {
        for (const e of o.world.elements) map.addBox(e.minXIn, e.minYIn, e.maxXIn, e.maxYIn);
      }
      return map;
    }

    /** Rebuild the localizer's map in place, keeping its particles. */
    refreshMaps() {
      this.world.rebuildMap();
      this.robot.localizer.filter.map = this.localizerMap();
    }

    currentGains() {
      if (!this.options.useAutoTune) return R.HAND_GAINS;
      const designs = R.autoTuneGains(this.options.world.models, this.options.world.delayS);
      this.designs = designs;
      return {
        drive: designs.drive.ok ? designs.drive.gains : R.HAND_GAINS.drive,
        turn: designs.turn.ok ? designs.turn.gains : R.HAND_GAINS.turn,
      };
    }

    applyGains() {
      this.gains = this.currentGains();
      this.robot.drivetrain.setGains(this.gains);
    }

    /** Start an autonomous routine (from routines.js) where it expects to start. */
    runRoutine(routine) {
      this.reset(routine.start);
      this.programName = routine.name;
      this.program = routine.run(this.robot);
    }

    stopProgram() {
      this.program = null;
      this.programName = null;
      this.robot.drivetrain.stop();
    }

    /** Driver control, robot-centric, like holonomic() in voltage mode. -1 to 1 each. */
    setDriverInput(throttle, strafe, turn) {
      this.driverInput = { throttle, strafe, turn };
    }

    /** One 10ms tick: physics, then odometry and the localizer, then the program. */
    tick() {
      if (this.pendingStep) this.finishLocalizerStep();
      this.world.step(kTickMs);
      this.robot.tick();
      this.endTick();
    }

    /**
     * Step mode, part 1: run ticks up to the localizer's next update, and stop
     * inside it, just after predict(). The page shows each part, then calls
     * weighLocalizerStep() and finishLocalizerStep().
     */
    beginLocalizerStep() {
      if (this.pendingStep) this.finishLocalizerStep();
      for (;;) {
        this.world.step(kTickMs);
        this.robot.odometry.update();
        if (this.world.timeMs % this.robot.localizerPeriodMs === 0) {
          this.robot.localizer.beginUpdate();
          this.pendingStep = 'predicted';
          return;
        }
        this.endTick();
      }
    }

    /** Step mode, part 2: read the sensors and weigh the particles. */
    weighLocalizerStep() {
      if (this.pendingStep !== 'predicted') return;
      this.robot.localizer.weighPhase();
      this.pendingStep = 'weighed';
    }

    /** Step mode, part 3: resample, correct odometry, and finish the tick. */
    finishLocalizerStep() {
      if (!this.pendingStep) return;
      if (this.pendingStep === 'predicted') this.robot.localizer.weighPhase();
      this.robot.localizer.finishUpdate();
      this.pendingStep = null;
      this.endTick();
    }

    endTick() {
      if (this.program) {
        const step = this.program.next();
        if (step.done) {
          this.programResult = { name: this.programName, atMs: this.world.timeMs, errors: this.errors() };
          this.program = null;
          this.programName = null;
          this.robot.drivetrain.stop();
        }
      } else {
        const d = this.driverInput;
        if (d.throttle || d.strafe || d.turn) this.robot.drivetrain.holonomic(d.throttle, d.strafe, d.turn);
        else this.robot.drivetrain.stop();
      }
      if (this.world.timeMs % 50 === 0) this.sample();
    }

    /** Errors against the truth, right now. */
    errors() {
      const t = this.world.truth;
      const snap = this.robot.odometry.snapshot();
      const est = this.robot.localizer.status.estimate;
      return {
        raw: Math.hypot(snap.rawPose.xIn - t.xIn, snap.rawPose.yIn - t.yIn),
        corrected: Math.hypot(snap.pose.xIn - t.xIn, snap.pose.yIn - t.yIn),
        estimate: Math.hypot(est.xIn - t.xIn, est.yIn - t.yIn),
        headingDeg: L.wrapDegrees180(snap.pose.headingDeg - t.headingDeg),
      };
    }

    sample() {
      const e = this.errors();
      const st = this.robot.localizer.status;
      this.history.push({
        t: this.world.timeMs / 1000,
        raw: e.raw,
        corrected: e.corrected,
        estimate: e.estimate,
        spread: st.spreadIn,
        neff: st.effectiveParticles / this.robot.localizer.filter.particles.length,
        correcting: st.correcting,
      });
      if (this.history.length > kHistorySamples) this.history.shift();
      const snap = this.robot.odometry.snapshot();
      const t = this.world.truth;
      pushTrail(this.trails.truth, t.xIn, t.yIn);
      pushTrail(this.trails.raw, snap.rawPose.xIn, snap.rawPose.yIn);
      pushTrail(this.trails.corrected, snap.pose.xIn, snap.pose.yIn);
      pushTrail(this.trails.estimate, st.estimate.xIn, st.estimate.yIn);
    }
  }

  function pushTrail(trail, x, y) {
    trail.push({ x, y });
    if (trail.length > kHistorySamples) trail.shift();
  }

  return { kTickMs, defaultOptions, Simulation };
});
