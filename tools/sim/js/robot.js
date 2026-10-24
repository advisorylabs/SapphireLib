/*
 * SapphireLib simulator: robot.js
 *
 * The robot program's side of the simulation, ported from the library so the
 * simulator runs what the robot runs:
 *
 *   sensors::Imu's frames          src/sapphirelib/sensors/imu.cpp
 *   odom::Odometry                 src/sapphirelib/odom/odometry.cpp
 *   MonteCarloLocalizer            src/sapphirelib/localization/monte_carlo_localizer.cpp
 *   the holonomic motions          src/sapphirelib/chassis/holonomic_drivetrain.cpp
 *   ExitTracker, pure pursuit      src/sapphirelib/motion/
 *   TrajectoryFollower,
 *   followTrajectory()             src/sapphirelib/motion/holonomic_tracking.cpp and the
 *                                  drivetrain (the planner and tracking math: trajectory.js)
 *
 * The PID controller and Auto-Tune's gain design come from the analyzer's
 * ports (tools/analyzer/js/model.js), so a gain set means the same thing here
 * as on the robot.
 *
 * Tasks become one loop: every 10ms the simulator updates odometry, the
 * localizer when its period comes around, and steps the running motion. Each
 * motion is a generator that yields where the C++ calls pros::delay(10), so a
 * routine reads like an autonomous:
 *
 *   function* myAuton(robot) {
 *     robot.odometry.setPose({ xIn: -36, yIn: -60, headingDeg: 0 });
 *     yield* robot.drivetrain.moveToPoint(-36, -24);
 *   }
 *
 * Plain script: window.SIM.robot in a browser, require('./robot.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./mcl.js'), require('../../analyzer/js/model.js'), require('./trajectory.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.robot = factory(root.SIM.mcl, root.SA.model, root.SIM.trajectory);
  }
})(function (L, M, T) {
  'use strict';

  const kDegToRad = L.kDegToRad;
  const kLoopMs = 10;
  const kFullScaleVolts = 12.0;
  const kMaxCorrectionDtS = 0.1;

  // --- sensors::Imu --------------------------------------------------------------

  class Imu {
    constructor(world) {
      this.world = world;
      this.offsetDeg = 0;
    }

    getCumulativeHeadingDeg() {
      return this.world.imuCumulativeDeg();
    }

    headingOffsetDeg() {
      return this.offsetDeg;
    }

    setHeadingDeg(headingDeg) {
      // headingOffsetFor()
      this.offsetDeg = L.wrapDegrees180(headingDeg - this.getCumulativeHeadingDeg());
    }

    getHeadingDeg() {
      return L.wrapDegrees360(this.getCumulativeHeadingDeg() + this.offsetDeg);
    }
  }

  // --- odom::Odometry ------------------------------------------------------------

  class Odometry {
    /**
     * `wheels` is { vertical: () => inches, horizontal: () => inches }; `clock`
     * returns milliseconds.
     */
    constructor(imu, wheels, config, startPose, clock) {
      this.imu = imu;
      this.wheels = wheels;
      this.config = Object.assign({}, config);
      this.clock = clock;
      this.poseGeneration = 0;
      this.seenGeneration = 0;
      this.lastUpdateMs = 0;
      // lastUpdateUs_: the velocity's own timestamp (null before the first update)
      this.lastVelocityMs = null;
      this.imu.setHeadingDeg(startPose.headingDeg);
      this.state = {
        raw: { xIn: startPose.xIn, yIn: startPose.yIn, headingDeg: L.wrapDegrees360(startPose.headingDeg) },
        rawVelocity: { xInPerS: 0, yInPerS: 0, headingDegPerS: 0 },
        correctionXIn: 0, correctionYIn: 0, targetXIn: 0, targetYIn: 0, maxCorrectionRateInPerS: 0,
      };
      this.lastRotationDeg = this.imu.getCumulativeHeadingDeg();
      this.lastVerticalIn = this.wheels.vertical();
      this.lastHorizontalIn = this.wheels.horizontal();
      this.reset = { rotationDeg: 0, verticalIn: 0, horizontalIn: 0 };
    }

    update() {
      const nowMs = this.clock();
      const dtS = this.lastUpdateMs === 0 ? 0.0 : Math.min((nowMs - this.lastUpdateMs) / 1000.0, kMaxCorrectionDtS);
      this.lastUpdateMs = nowMs;
      const elapsedS = this.lastVelocityMs === null ? 0.0 : (nowMs - this.lastVelocityMs) / 1000.0;
      this.lastVelocityMs = nowMs;

      const generation = this.poseGeneration;
      if (generation !== this.seenGeneration) {
        this.lastRotationDeg = this.reset.rotationDeg;
        this.lastVerticalIn = this.reset.verticalIn;
        this.lastHorizontalIn = this.reset.horizontalIn;
        this.seenGeneration = generation;
      }
      const config = this.config;
      const rotationDeg = this.imu.getCumulativeHeadingDeg();
      const offsetDeg = this.imu.headingOffsetDeg();
      const headingDeg = L.wrapDegrees360(rotationDeg + offsetDeg);
      const lastHeadingDeg = this.lastRotationDeg + offsetDeg;
      const verticalIn = this.wheels.vertical();
      const horizontalIn = this.wheels.horizontal();

      const delta = L.computeOdometryDelta(lastHeadingDeg, headingDeg, verticalIn - this.lastVerticalIn,
        horizontalIn - this.lastHorizontalIn, config.verticalOffsetIn, config.horizontalOffsetIn);
      const velocity = L.velocityFromDelta(delta, rotationDeg - this.lastRotationDeg, elapsedS);

      this.lastRotationDeg = rotationDeg;
      this.lastVerticalIn = verticalIn;
      this.lastHorizontalIn = horizontalIn;

      const state = this.state;
      state.raw.xIn += delta.dxIn;
      state.raw.yIn += delta.dyIn;
      state.raw.headingDeg = headingDeg;
      state.rawVelocity = velocity;
      const maxStepIn = state.maxCorrectionRateInPerS > 0.0 ? state.maxCorrectionRateInPerS * dtS : Infinity;
      const step = L.correctionStep(state.targetXIn - state.correctionXIn, state.targetYIn - state.correctionYIn,
        maxStepIn);
      state.correctionXIn += step.dxIn;
      state.correctionYIn += step.dyIn;
    }

    snapshot() {
      const s = this.state;
      return {
        pose: { xIn: s.raw.xIn + s.correctionXIn, yIn: s.raw.yIn + s.correctionYIn, headingDeg: s.raw.headingDeg },
        rawPose: Object.assign({}, s.raw),
        resetCount: this.poseGeneration,
        rawVelocity: Object.assign({}, s.rawVelocity),
      };
    }

    getPose() {
      return this.snapshot().pose;
    }

    setPositionCorrection(xIn, yIn, maxRateInPerS, resetCount) {
      if (this.poseGeneration !== resetCount) return false;
      this.state.targetXIn = xIn;
      this.state.targetYIn = yIn;
      this.state.maxCorrectionRateInPerS = maxRateInPerS;
      return true;
    }

    setPose(pose) {
      this.reset = {
        rotationDeg: this.imu.getCumulativeHeadingDeg(),
        verticalIn: this.wheels.vertical(),
        horizontalIn: this.wheels.horizontal(),
      };
      this.imu.setHeadingDeg(pose.headingDeg);
      this.poseGeneration++;
      this.state = {
        raw: { xIn: pose.xIn, yIn: pose.yIn, headingDeg: L.wrapDegrees360(pose.headingDeg) },
        rawVelocity: { xInPerS: 0, yInPerS: 0, headingDegPerS: 0 },
        correctionXIn: 0, correctionYIn: 0, targetXIn: 0, targetYIn: 0, maxCorrectionRateInPerS: 0,
      };
    }
  }

  // --- localization::MonteCarloLocalizer -------------------------------------------

  /** LocalizerConfig defaults; `filter` merges like filterConfig(). */
  function localizerConfig(overrides = {}) {
    const config = Object.assign({
      startSpreadIn: 2.0,
      sensorLatencyMs: 30.0,
      minConfidence: 20,
      maxTurnRateDegPerS: 200.0,
      correctOdometry: true,
      waitForSetPose: true,
      maxCorrectionSpreadIn: 3.0,
      minAgreeingSensors: 2,
      agreementSigmas: 3.0,
      maxCorrectionRateInPerS: 4.0,
    }, overrides);
    config.filter = L.filterConfig(overrides.filter || {});
    return config;
  }

  /** LocalizationStatus's blockedBy flags: why an update didn't correct odometry. */
  const BLOCKED = {
    correctionOff: 1 << 0,
    noSetPose: 1 << 1,
    noReadings: 1 << 2,
    tooSpread: 1 << 3,
    tooFewAgree: 1 << 4,
    refused: 1 << 5,
    spinning: 1 << 6,
  };

  class MonteCarloLocalizer {
    /**
     * `sensors` is one function per sensor returning { mm, confidence }, like
     * pros::Distance; `mounts` their mounts, in the same order.
     */
    constructor(odometry, sensors, mounts, map, config, clock) {
      this.odometry = odometry;
      this.sensors = sensors;
      this.config = localizerConfig(config);
      this.filter = new L.ParticleFilter(map, mounts, this.config.filter);
      this.clock = clock;
      this.correctionEnabled = this.config.correctOdometry;
      this.relocalizeRequested = false;
      this.started = false;
      this.lastResetCount = 0;
      this.lastRawPose = { xIn: 0, yIn: 0, headingDeg: 0 };
      this.lastUpdateMs = 0;
      this.status = {
        estimate: { xIn: 0, yIn: 0, headingDeg: 0 }, spreadIn: 0, effectiveParticles: 0, sensorsUsed: 0,
        sensorsAgreeing: 0, correcting: false, blockedBy: 0, correctionXIn: 0, correctionYIn: 0, updates: 0,
      };
      // the last update's working, for the visualizer and the recorder
      this.work = null;
    }

    update() {
      this.beginUpdate();
      this.weighPhase();
      this.finishUpdate();
    }

    /** Step 1: the snapshot, starting over if needed, and predict(). */
    beginUpdate() {
      const nowMs = this.clock();
      const snapshot = this.odometry.snapshot();
      const raw = snapshot.rawPose;
      const relocalize = this.relocalizeRequested;
      this.relocalizeRequested = false;
      let restarted = false;
      if (!this.started || snapshot.resetCount !== this.lastResetCount || relocalize) {
        if (relocalize) this.filter.resetUniform();
        else this.filter.reset(snapshot.pose.xIn, snapshot.pose.yIn, this.config.startSpreadIn);
        this.started = true;
        this.lastResetCount = snapshot.resetCount;
        this.lastRawPose = Object.assign({}, raw);
        this.lastUpdateMs = nowMs;
        this.status.correctionXIn = 0;
        this.status.correctionYIn = 0;
        restarted = true;
      }
      const dxIn = raw.xIn - this.lastRawPose.xIn;
      const dyIn = raw.yIn - this.lastRawPose.yIn;
      const turnedDeg = L.wrapDegrees180(raw.headingDeg - this.lastRawPose.headingDeg);
      const dtS = (nowMs - this.lastUpdateMs) / 1000.0;
      this.lastRawPose = Object.assign({}, raw);
      this.lastUpdateMs = nowMs;
      this.filter.predict(dxIn, dyIn, turnedDeg);
      this.work = { snapshot, raw, dxIn, dyIn, turnedDeg, dtS, restarted };
    }

    /** Step 2: read the sensors, weigh(), and check the estimate against the walls. */
    weighPhase() {
      const w = this.work;
      const config = this.config;
      const latencyS = config.sensorLatencyMs / 1000.0;
      const turnRateDegPerS = w.dtS > 0.0 ? w.turnedDeg / w.dtS : 0.0;
      const velocityX = w.dtS > 0.0 ? w.dxIn / w.dtS : 0.0;
      const velocityY = w.dtS > 0.0 ? w.dyIn / w.dtS : 0.0;
      const beamHeadingDeg = w.raw.headingDeg - turnRateDegPerS * latencyS;
      const spinning = Math.abs(turnRateDegPerS) > config.maxTurnRateDegPerS;

      const readings = [];
      const closing = [];
      let used = 0;
      for (let s = 0; s < this.sensors.length; ++s) {
        const raw = this.sensors[s]();
        let r = L.distanceReadingFromMm(raw.mm, raw.confidence, config.minConfidence, config.filter.beam);
        if (spinning) r.valid = false;
        const ray = L.sensorRay(0.0, 0.0, beamHeadingDeg, this.filter.sensors[s]);
        const closingSpeed = velocityX * ray.dirX + velocityY * ray.dirY;
        r = L.compensateLatency(r, closingSpeed, latencyS);
        readings.push(r);
        closing.push(closingSpeed);
        if (r.valid) ++used;
      }
      w.closing = closing;

      w.weighed = this.filter.weigh(beamHeadingDeg, readings);
      w.estimate = this.filter.estimate();
      const check = this.filter.checkSensors(w.estimate.xIn, w.estimate.yIn, beamHeadingDeg, readings,
        config.agreementSigmas);
      w.readings = readings;
      w.used = used;
      w.agreeing = check.agreeing;
      w.checks = check.checks;
      w.beamHeadingDeg = beamHeadingDeg;
      w.spinning = spinning;
    }

    /** Step 3: resample, and correct odometry if the estimate passes every check. */
    finishUpdate() {
      const w = this.work;
      const config = this.config;
      w.resampled = this.filter.resampleIfNeeded();
      const inFieldFrame = !config.waitForSetPose || w.snapshot.resetCount > 0;
      let blockedBy = 0;
      if (!this.correctionEnabled) blockedBy |= BLOCKED.correctionOff;
      if (!inFieldFrame) blockedBy |= BLOCKED.noSetPose;
      if (!w.weighed) blockedBy |= BLOCKED.noReadings;
      if (w.spinning) blockedBy |= BLOCKED.spinning;
      if (!(w.estimate.spreadIn <= config.maxCorrectionSpreadIn)) blockedBy |= BLOCKED.tooSpread;
      if (w.agreeing < config.minAgreeingSensors) blockedBy |= BLOCKED.tooFewAgree;
      const correctionXIn = w.estimate.xIn - w.raw.xIn;
      const correctionYIn = w.estimate.yIn - w.raw.yIn;
      if (blockedBy === 0 && !this.odometry.setPositionCorrection(correctionXIn, correctionYIn,
        config.maxCorrectionRateInPerS, w.snapshot.resetCount)) {
        blockedBy |= BLOCKED.refused;
      }
      const correcting = blockedBy === 0;
      w.blockedBy = blockedBy;
      w.recovered = this.filter.lastRecovered;
      w.fit = this.filter.recoveryFit();
      const st = this.status;
      st.estimate = { xIn: w.estimate.xIn, yIn: w.estimate.yIn, headingDeg: w.raw.headingDeg };
      st.spreadIn = w.estimate.spreadIn;
      st.effectiveParticles = w.estimate.effectiveParticles;
      st.sensorsUsed = w.used;
      st.sensorsAgreeing = w.agreeing;
      st.correcting = correcting;
      st.blockedBy = blockedBy;
      if (correcting) {
        st.correctionXIn = correctionXIn;
        st.correctionYIn = correctionYIn;
      }
      st.updates++;
    }

    /**
     * The last update's BeamSamples, as the C++ hands its update callback:
     * each sensor's reading (NaN if unused), what the map says it should read
     * from the estimate (worked out for every sensor), and its closing speed.
     * What the robot logs in "mcl.beams".
     */
    beamSamples() {
      const w = this.work;
      if (!w || !w.readings || !w.estimate) return [];
      return this.filter.sensors.map((mount, s) => {
        const r = w.readings[s];
        let expectedIn;
        if (w.checks[s] && w.checks[s].used) {
          expectedIn = w.checks[s].expectedIn;
        } else {
          const ray = L.sensorRay(w.estimate.xIn, w.estimate.yIn, w.beamHeadingDeg, mount);
          expectedIn = this.filter.map.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
        }
        return { used: r.valid, measuredIn: r.valid ? r.distanceIn : NaN, expectedIn,
          closingSpeedInPerS: w.closing[s] };
      });
    }

    setCorrectionEnabled(enabled) {
      this.correctionEnabled = enabled;
    }

    relocalizeGlobally() {
      this.relocalizeRequested = true;
    }
  }

  // --- motion helpers (exit_tracker.cpp, pure_pursuit_math.cpp) --------------------

  class ExitTracker {
    constructor(settleTimeMs, timeoutMs, startMs) {
      this.settleTimeMs = settleTimeMs;
      this.timeoutMs = timeoutMs;
      this.startMs = startMs;
      this.lastTickMs = startMs;
      this.settledForMs = 0;
      this.elapsedMs = 0;
    }

    update(withinThreshold, nowMs) {
      let reason = 'running';
      if (withinThreshold) {
        this.settledForMs += Math.max(0, nowMs - this.lastTickMs);
        if (this.settledForMs >= this.settleTimeMs) reason = 'settled';
      } else {
        this.settledForMs = 0;
      }
      if (reason === 'running' && this.timeoutMs > 0 && nowMs - this.startMs >= this.timeoutMs) {
        reason = 'timeout';
      }
      this.lastTickMs = nowMs;
      this.elapsedMs = nowMs - this.startMs;
      return reason;
    }

    extendTimeout(extraMs) {
      if (this.timeoutMs > 0) this.timeoutMs += extraMs;
    }
  }

  function toLocalFrame(dxIn, dyIn, headingDeg) {
    const headingRad = headingDeg * kDegToRad;
    return {
      forwardIn: dxIn * Math.sin(headingRad) + dyIn * Math.cos(headingRad),
      lateralIn: dxIn * Math.cos(headingRad) - dyIn * Math.sin(headingRad),
    };
  }

  function findLookaheadPoint(xIn, yIn, points, lookaheadIn, fromIndex) {
    let found = false;
    let bestPoint = null;
    let bestIndex = fromIndex;
    const startIndex = fromIndex < points.length ? fromIndex : points.length - 1;
    for (let i = startIndex; i + 1 < points.length; ++i) {
      const p1 = points[i];
      const p2 = points[i + 1];
      const d = { x: p2.xIn - p1.xIn, y: p2.yIn - p1.yIn };
      const f = { x: p1.xIn - xIn, y: p1.yIn - yIn };
      const a = d.x * d.x + d.y * d.y;
      if (a < 1e-9) continue;
      const b = 2.0 * (f.x * d.x + f.y * d.y);
      const c = f.x * f.x + f.y * f.y - lookaheadIn * lookaheadIn;
      const discriminant = b * b - 4.0 * a * c;
      const t2 = discriminant < 0.0 ? -1.0 : (-b + Math.sqrt(discriminant)) / (2.0 * a);
      if (t2 < 0.0 || t2 > 1.0) {
        // once the path has left the circle, a later segment back within reach is for later
        if (found) break;
        continue;
      }
      found = true;
      bestPoint = { xIn: p1.xIn + d.x * t2, yIn: p1.yIn + d.y * t2 };
      bestIndex = i;
    }
    if (found) return { point: bestPoint, segmentIndex: bestIndex };
    // the rest of the path ends inside the circle: head for the end
    const last = points[points.length - 1];
    if (Math.hypot(last.xIn - xIn, last.yIn - yIn) <= lookaheadIn || startIndex + 1 >= points.length) {
      return { point: last, segmentIndex: points.length - 1 };
    }
    // it never comes within reach (it starts elsewhere, or the robot was knocked off it): back to
    // the nearest point of what's left
    let nearest = points[startIndex];
    let nearestIndex = startIndex;
    let nearestSq = Infinity;
    for (let i = startIndex; i + 1 < points.length; ++i) {
      const p1 = points[i];
      const d = { x: points[i + 1].xIn - p1.xIn, y: points[i + 1].yIn - p1.yIn };
      const lengthSq = d.x * d.x + d.y * d.y;
      const t = lengthSq < 1e-9 ? 0.0
        : Math.min(1.0, Math.max(0.0, ((xIn - p1.xIn) * d.x + (yIn - p1.yIn) * d.y) / lengthSq));
      const point = { xIn: p1.xIn + d.x * t, yIn: p1.yIn + d.y * t };
      const distanceSq = (point.xIn - xIn) ** 2 + (point.yIn - yIn) ** 2;
      if (distanceSq < nearestSq) {
        nearestSq = distanceSq;
        nearest = point;
        nearestIndex = i;
      }
    }
    return { point: nearest, segmentIndex: nearestIndex };
  }

  function reachedFinalApproach(distanceToFinalIn, finalApproachIn, segmentIndex, waypointCount) {
    const onLastSegment = segmentIndex + 2 >= waypointCount;
    return onLastSegment && distanceToFinalIn <= finalApproachIn;
  }

  // --- chassis::HolonomicDrivetrain ------------------------------------------------

  /**
   * The robot program's own gains (src/robot/devices.cpp), before Auto-Tune.
   * The tracking PIDs start from Drive and Turn's, as the drivetrain's do.
   */
  const HAND_GAINS = {
    drive: { kP: 1.2, kI: 0.0, kD: 0.001 },
    turn: { kP: 0.35, kI: 0.0, kD: 0.0002 },
    tracking: {
      forward: { kP: 1.2, kI: 0.0, kD: 0.001 },
      strafe: { kP: 1.2, kI: 0.0, kD: 0.001 },
      turn: { kP: 0.35, kI: 0.0, kD: 0.0002 },
    },
  };

  /** The specs src/robot/tuning.cpp designs Drive, Turn and the three Track controllers to. */
  const RESPONSE_SPECS = {
    drive: M.responseSpec(0.6, 1.0, 50.0),
    turn: M.responseSpec(0.5, 1.0, 50.0),
    tracking: M.responseSpec(0.4, 1.0, 50.0),
  };

  /**
   * Auto-Tune's design step: kP and kD placed from each axis's model, backed
   * off for the measured delay, exactly as PidTunerPage does on the robot
   * (Drive from the Fwd axis, Turn from the Turn axis). The page adds half of
   * any loop slower than the 10ms sampling to the delay; the drivetrain's
   * loops run at 10ms, so that's nothing here. The model is the sim's true
   * one, as if characterization had measured it perfectly.
   */
  function autoTuneGains(models, delayS) {
    const model = (m) => M.feedforward(m.kS, m.kV, m.kA);
    return {
      drive: M.designPositionGains(model(models.forward), RESPONSE_SPECS.drive, delayS),
      turn: M.designPositionGains(model(models.turn), RESPONSE_SPECS.turn, delayS),
      // Track F, Track S, Track T: each from its own axis
      tracking: {
        forward: M.designPositionGains(model(models.forward), RESPONSE_SPECS.tracking, delayS),
        strafe: M.designPositionGains(model(models.strafe), RESPONSE_SPECS.tracking, delayS),
        turn: M.designPositionGains(model(models.turn), RESPONSE_SPECS.tracking, delayS),
      },
    };
  }

  // --- motion::TrajectoryFollower (holonomic_tracking.cpp) ----------------------------

  class TrajectoryFollower {
    constructor(forward, strafe, turn) {
      this.pids = { forward, strafe, turn };
    }

    begin(trajectory, models, config, nowMs) {
      this.trajectory = trajectory;
      this.models = models;
      this.config = T.followConfig(config);
      const durationMs = Math.ceil(trajectory.durationS() * 1000.0);
      this.tracker = new ExitTracker(this.config.settleTimeMs, durationMs + this.config.overtimeMs, nowMs);
      this.reason = 'running';
      this.referenceTimeS = 0.0;
      this.lastAlongTrackIn = 0.0;
      this.lastMs = nowMs;
      this.started = false;
      this.finalErrorIn = 0.0;
      this.progress = { running: true, section: 0, distanceIn: 0, timeS: 0, paused: false, piece: 0 };
      // a chain: which trajectory, and where it starts in the chain's time, sections and distance
      this.piece = 0;
      this.pieceStartS = 0.0;
      this.pieceFirstSection = 0;
      this.pieceStartIn = 0.0;
      this.chainEnded = false;
      this.pids.forward.reset();
      this.pids.strafe.reset();
      this.pids.turn.reset();
    }

    update(pose, velocity, nowMs) {
      const dtS = this.started ? (nowMs - this.lastMs) / 1000.0 : 0.0;
      this.lastMs = nowMs;
      const clock = this.started
        ? T.advanceTrajectoryClock(this.referenceTimeS, dtS, this.lastAlongTrackIn, this.config.maxLagIn)
        : { tS: 0.0, paused: false };
      this.started = true;
      this.referenceTimeS = clock.tS;

      // past the end of a trajectory that ends moving: carry on into the next one
      let chain = 'none';
      while (!this.chainEnded && this.referenceTimeS >= this.trajectory.durationS() && T.endsMoving(this.trajectory)) {
        const next = this.config.next ? this.config.next(this.trajectory) : null;
        if (!next || next.empty() || !T.continuesFrom(next, this.trajectory)) {
          chain = !next || next.empty() ? 'noNext' : 'mismatched';
          this.chainEnded = true;
          break;
        }
        this.referenceTimeS -= this.trajectory.durationS();
        this.pieceStartS += this.trajectory.durationS();
        this.pieceFirstSection += this.trajectory.sectionCount();
        this.pieceStartIn += this.trajectory.lengthIn();
        this.tracker.extendTimeout(Math.ceil(next.durationS() * 1000.0));
        this.trajectory = next;
        ++this.piece;
        chain = 'continued';
      }

      const trajectory = this.trajectory;
      const holding = this.referenceTimeS >= trajectory.durationS();

      const reference = holding ? T.atRest(trajectory.sample(this.referenceTimeS))
        : trajectory.sample(this.referenceTimeS);
      const errors = T.trackingErrors(reference, pose, velocity);
      const feedforward = holding ? { forward: 0, strafe: 0, turn: 0 }
        : T.trajectoryFeedforward(this.models, reference, L.wrapDegrees180(pose.headingDeg - reference.headingDeg));
      // PID.update()'s explicit dt; 0 falls back to the nominal 10ms, as on the robot
      const pidDtS = dtS > 0.0 ? dtS : 0.0;
      const volts = {
        forward: feedforward.forward + this.pids.forward.updateWithRate(errors.forwardIn, 0.0,
          errors.forwardRateInPerS, pidDtS),
        strafe: feedforward.strafe + this.pids.strafe.updateWithRate(errors.strafeIn, 0.0,
          errors.strafeRateInPerS, pidDtS),
        turn: feedforward.turn + this.pids.turn.updateWithRate(errors.headingDeg, 0.0,
          errors.headingRateDegPerS, pidDtS),
      };
      const section = this.pieceFirstSection + trajectory.sectionAt(reference.sIn);
      this.lastAlongTrackIn = errors.alongTrackIn;
      const referenceTimeS = this.pieceStartS + this.referenceTimeS;

      const last = trajectory.samples[trajectory.samples.length - 1];
      this.finalErrorIn = Math.hypot(last.xIn - pose.xIn, last.yIn - pose.yIn);
      const headingErrorDeg = Math.abs(L.wrapDegrees180(last.headingDeg - pose.headingDeg));
      const within = holding && this.finalErrorIn <= this.config.positionErrorThresholdIn &&
        headingErrorDeg <= this.config.headingErrorThresholdDeg;
      this.reason = this.tracker.update(within, nowMs);
      this.progress = { running: this.reason === 'running', section, distanceIn: this.pieceStartIn + reference.sIn,
        timeS: referenceTimeS, paused: clock.paused, piece: this.piece };
      return { referenceTimeS, reference, feedforward, volts, errors, paused: clock.paused, holding, section,
        piece: this.piece, chain };
    }

    result() {
      return { reason: this.reason, finalError: this.finalErrorIn, elapsedMs: this.tracker.elapsedMs };
    }
  }

  class Drivetrain {
    constructor(world, imu, odometry, gains, clock) {
      this.world = world;
      this.imu = imu;
      this.odometry = odometry;
      this.clock = clock;
      this.drivePID = new M.PID({ gains: Object.assign({}, gains.drive), outputLimit: 12.0 });
      this.turnPID = new M.PID({ gains: Object.assign({}, gains.turn), outputLimit: 12.0 });
      const tracking = gains.tracking || HAND_GAINS.tracking;
      this.trackingPIDs = {
        forward: new M.PID({ gains: Object.assign({}, tracking.forward), outputLimit: 12.0 }),
        strafe: new M.PID({ gains: Object.assign({}, tracking.strafe), outputLimit: 12.0 }),
        turn: new M.PID({ gains: Object.assign({}, tracking.turn), outputLimit: 12.0 }),
      };
      // setAxisModels(): the sim's are the world's own, as if Auto-Tune measured them perfectly
      this.axisModels = null;
      this.applied = { forward: 0, strafe: 0, turn: 0 };
      // trajectoryProgress(), and the number of the followTrajectory() it's from
      this.progress = { running: false, section: 0, distanceIn: 0, timeS: 0, paused: false, piece: 0 };
      this.progressMotion = 0;
      // setTrajectoryObserver(): { onTrajectoryStart(trajectory), onTrackingTick(tick) }
      this.trajectoryObserver = null;
      // for the visualizer: what the running motion is aiming at
      this.target = null;
    }

    setGains(gains) {
      this.drivePID.setGains(gains.drive);
      this.turnPID.setGains(gains.turn);
      if (gains.tracking) {
        this.trackingPIDs.forward.setGains(gains.tracking.forward);
        this.trackingPIDs.strafe.setGains(gains.tracking.strafe);
        this.trackingPIDs.turn.setGains(gains.tracking.turn);
      }
    }

    setAxisModels(models) {
      this.axisModels = models;
    }

    trajectoryProgress() {
      return Object.assign({}, this.progress, { motion: this.progressMotion });
    }

    setTrajectoryObserver(observer) {
      this.trajectoryObserver = observer;
    }

    holonomicVolts(forwardVolts, strafeVolts, turnVolts) {
      const fl = forwardVolts + strafeVolts + turnVolts;
      const fr = forwardVolts - strafeVolts - turnVolts;
      const bl = forwardVolts - strafeVolts + turnVolts;
      const br = forwardVolts + strafeVolts - turnVolts;
      const largest = Math.max(Math.abs(fl), Math.abs(fr), Math.abs(bl), Math.abs(br), kFullScaleVolts) /
        kFullScaleVolts;
      this.setWheelVoltages(fl / largest, fr / largest, bl / largest, br / largest);
    }

    setWheelVoltages(fl, fr, bl, br) {
      // recover the axes from the (scaled) corner mix, as setWheelVoltages() does; the axis
      // models take those
      const throttle = (fl + fr + bl + br) / 4.0;
      const strafe = (fl - fr - bl + br) / 4.0;
      const turn = (fl - fr + bl - br) / 4.0;
      this.applied = { forward: throttle, strafe, turn };
      this.world.command(throttle, strafe, turn);
    }

    /** Driver control in DriverInputMode::voltage. */
    holonomic(throttle, strafe, turn) {
      this.holonomicVolts(throttle * kFullScaleVolts, strafe * kFullScaleVolts, turn * kFullScaleVolts);
    }

    stop() {
      this.applied = { forward: 0, strafe: 0, turn: 0 };
      this.world.brake();
    }

    *moveToPoint(xIn, yIn, exit = {}) {
      const holdHeadingDeg = this.odometry.getPose().headingDeg;
      return yield* this.moveToPointHolding(xIn, yIn, holdHeadingDeg, exitConditions(exit, 1.0));
    }

    *moveToPointHolding(xIn, yIn, holdHeadingDeg, exit) {
      this.drivePID.reset();
      this.turnPID.reset();
      this.target = { kind: 'point', xIn, yIn };
      const tracker = new ExitTracker(exit.settleTimeMs, exit.timeoutMs, this.clock());
      let reason = 'running';
      let finalErrorIn = 0;
      for (;;) {
        const pose = this.odometry.getPose();
        const dxIn = xIn - pose.xIn;
        const dyIn = yIn - pose.yIn;
        const distanceIn = Math.hypot(dxIn, dyIn);
        const outputVolts = this.drivePID.update(distanceIn, 0.0);
        const local = toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;
        const turnOutput = this.turnPID.update(L.wrapDegrees180(holdHeadingDeg - pose.headingDeg), 0.0);
        this.holonomicVolts(local.forwardIn * scale, local.lateralIn * scale, turnOutput);
        finalErrorIn = distanceIn;
        reason = tracker.update(distanceIn <= exit.errorThreshold, this.clock());
        if (reason !== 'running') break;
        yield;
      }
      this.stop();
      this.target = null;
      return { reason, finalError: finalErrorIn, elapsedMs: tracker.elapsedMs };
    }

    *moveToPose(xIn, yIn, headingDeg, exit = {}) {
      exit = Object.assign({ positionErrorThresholdIn: 1.0, headingErrorThresholdDeg: 2.0, settleTimeMs: 200,
        timeoutMs: 4000 }, exit);
      this.drivePID.reset();
      this.turnPID.reset();
      this.target = { kind: 'pose', xIn, yIn, headingDeg };
      const tracker = new ExitTracker(exit.settleTimeMs, exit.timeoutMs, this.clock());
      let reason = 'running';
      let finalErrorIn = 0;
      for (;;) {
        const pose = this.odometry.getPose();
        const dxIn = xIn - pose.xIn;
        const dyIn = yIn - pose.yIn;
        const distanceIn = Math.hypot(dxIn, dyIn);
        const outputVolts = this.drivePID.update(distanceIn, 0.0);
        const local = toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;
        const headingError = L.wrapDegrees180(headingDeg - pose.headingDeg);
        const turnOutput = this.turnPID.update(headingError, 0.0);
        this.holonomicVolts(local.forwardIn * scale, local.lateralIn * scale, turnOutput);
        const within = distanceIn <= exit.positionErrorThresholdIn &&
          Math.abs(headingError) <= exit.headingErrorThresholdDeg;
        finalErrorIn = distanceIn;
        reason = tracker.update(within, this.clock());
        if (reason !== 'running') break;
        yield;
      }
      this.stop();
      this.target = null;
      return { reason, finalError: finalErrorIn, elapsedMs: tracker.elapsedMs };
    }

    *turnToHeading(headingDeg, exit = {}) {
      exit = exitConditions(exit, 2.0);
      this.turnPID.reset();
      this.target = { kind: 'heading', headingDeg };
      const tracker = new ExitTracker(exit.settleTimeMs, exit.timeoutMs, this.clock());
      let reason = 'running';
      let finalErrorDeg = 0;
      for (;;) {
        const error = L.wrapDegrees180(headingDeg - this.imu.getHeadingDeg());
        const output = this.turnPID.update(error, 0.0);
        this.holonomicVolts(0.0, 0.0, output);
        finalErrorDeg = Math.abs(error);
        reason = tracker.update(Math.abs(error) <= exit.errorThreshold, this.clock());
        if (reason !== 'running') break;
        yield;
      }
      this.stop();
      this.target = null;
      return { reason, finalError: finalErrorDeg, elapsedMs: tracker.elapsedMs };
    }

    *followPath(points, config) {
      config = Object.assign({ cruiseVoltage: 8.0, finalApproachIn: 6.0, timeoutMs: 10000 }, config);
      config.finalExit = exitConditions(config.finalExit || {}, 1.0);
      if (points.length === 0) return { reason: 'aborted', finalError: 0, elapsedMs: 0 };
      let segmentIndex = 0;
      const finalPoint = points[points.length - 1];
      const holdHeadingDeg = this.odometry.getPose().headingDeg;
      this.turnPID.reset();
      const startMs = this.clock();
      for (;;) {
        const pose = this.odometry.getPose();
        const distToFinalIn = Math.hypot(finalPoint.xIn - pose.xIn, finalPoint.yIn - pose.yIn);
        if (reachedFinalApproach(distToFinalIn, config.finalApproachIn, segmentIndex, points.length)) break;
        const pursuitMs = this.clock() - startMs;
        if (config.timeoutMs > 0 && pursuitMs >= config.timeoutMs) {
          this.stop();
          this.target = null;
          return { reason: 'timeout', finalError: distToFinalIn, elapsedMs: pursuitMs };
        }
        const lookahead = findLookaheadPoint(pose.xIn, pose.yIn, points, config.lookaheadIn, segmentIndex);
        segmentIndex = lookahead.segmentIndex;
        this.target = { kind: 'path', points, lookahead: lookahead.point };
        const dxIn = lookahead.point.xIn - pose.xIn;
        const dyIn = lookahead.point.yIn - pose.yIn;
        const distanceIn = Math.hypot(dxIn, dyIn);
        const local = toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const scale = distanceIn > 1e-6 ? config.cruiseVoltage / distanceIn : 0.0;
        const turnOutput = this.turnPID.update(L.wrapDegrees180(holdHeadingDeg - pose.headingDeg), 0.0);
        this.holonomicVolts(local.forwardIn * scale, local.lateralIn * scale, turnOutput);
        yield;
      }
      const result = yield* this.moveToPointHolding(finalPoint.xIn, finalPoint.yIn, holdHeadingDeg, config.finalExit);
      result.elapsedMs = this.clock() - startMs;
      return result;
    }

    /** followTrajectory(trajectory, config): a Trajectory from trajectory.js's generateTrajectory(). */
    *followTrajectory(trajectory, config = {}) {
      if (!trajectory || trajectory.empty()) return { reason: 'aborted', finalError: 0, elapsedMs: 0 };
      const models = this.axisModels;
      const valid = (m) => m && m.kV > 0 && m.kA > 0;
      if (!models || !valid(models.forward) || !valid(models.strafe) || !valid(models.turn)) {
        return { reason: 'aborted', finalError: 0, elapsedMs: 0 };
      }
      const follower = new TrajectoryFollower(this.trackingPIDs.forward, this.trackingPIDs.strafe,
        this.trackingPIDs.turn);
      follower.begin(trajectory, models, config, this.clock());
      this.progress = follower.progress;
      this.progressMotion++;
      // the config too, and an end: the robot's drivetrain logs its motion events itself, the
      // simulator's recorder does it from these
      if (this.trajectoryObserver) this.trajectoryObserver.onTrajectoryStart(trajectory, follower.config);
      this.target = { kind: 'trajectory', trajectory, reference: null };
      for (;;) {
        const snapshot = this.odometry.snapshot();
        const tick = follower.update(snapshot.pose, snapshot.rawVelocity, this.clock());
        this.holonomicVolts(tick.volts.forward, tick.volts.strafe, tick.volts.turn);
        this.progress = follower.progress;
        if (tick.chain === 'continued') {
          // the visualizer draws the trajectory being followed
          this.target.trajectory = follower.trajectory;
          const o = this.trajectoryObserver;
          if (o && o.onTrajectoryContinue) {
            o.onTrajectoryContinue(follower.trajectory, follower.pieceStartS, follower.pieceFirstSection, tick.piece);
          }
        } else if (tick.chain !== 'none') {
          const o = this.trajectoryObserver;
          if (o && o.onTrajectoryNoContinue) o.onTrajectoryNoContinue(tick.chain, tick.piece);
        }
        this.target.reference = tick.reference;
        if (this.trajectoryObserver) this.trajectoryObserver.onTrackingTick(tick);
        if (follower.reason !== 'running') break;
        yield;
      }
      this.stop();
      this.target = null;
      const result = follower.result();
      if (this.trajectoryObserver && this.trajectoryObserver.onTrajectoryEnd) this.trajectoryObserver.onTrajectoryEnd(result);
      return result;
    }
  }

  function exitConditions(exit, defaultThreshold) {
    return Object.assign({ errorThreshold: defaultThreshold, settleTimeMs: 200, timeoutMs: 3000 }, exit);
  }

  // --- the whole robot ---------------------------------------------------------------

  /**
   * The robot program: IMU, odometry, localizer, and drivetrain, wired the way
   * src/robot/devices.cpp wires them, against a World.
   */
  class Robot {
    /**
     * `mounts` is where the robot program believes its distance sensors are
     * (LocalizerConfig's DistanceSensorConfig mounts); by default exactly
     * where the world has them. Different ones are a mount measured wrong.
     */
    constructor(world, { odometryConfig, localizer, gains, startPose, mapFor, mounts = null }) {
      this.world = world;
      const clock = () => world.timeMs;
      this.clock = clock;
      this.imu = new Imu(world);
      this.odometry = new Odometry(this.imu, {
        vertical: () => world.verticalWheelIn(),
        horizontal: () => world.horizontalWheelIn(),
      }, odometryConfig, startPose, clock);
      const sensors = world.settings.sensors.map((_, i) => () => world.distanceSensor(i));
      this.localizer = new MonteCarloLocalizer(this.odometry, sensors, mounts || world.settings.sensors, mapFor(),
        localizer, clock);
      this.drivetrain = new Drivetrain(world, this.imu, this.odometry, gains, clock);
      this.localizerPeriodMs = 50;
    }

    /** One 10ms tick of every task, after the world has advanced. */
    tick() {
      this.odometry.update();
      if (this.world.timeMs % this.localizerPeriodMs === 0) this.localizer.update();
    }
  }

  return {
    kLoopMs,
    Imu,
    Odometry,
    localizerConfig,
    BLOCKED,
    MonteCarloLocalizer,
    ExitTracker,
    toLocalFrame,
    findLookaheadPoint,
    reachedFinalApproach,
    HAND_GAINS,
    RESPONSE_SPECS,
    autoTuneGains,
    TrajectoryFollower,
    Drivetrain,
    Robot,
  };
});
