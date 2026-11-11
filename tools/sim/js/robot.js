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
  // devices.cpp's DrivetrainConfig::headingCorrectionKP, for driveDistance()
  const kHeadingCorrectionKP = 0.4;
  // devices.cpp's DrivetrainConfig::minForwardVolts, minStrafeVolts and minTurnVolts: the least a
  // point motion pushes with on each axis until it first gets within its exit threshold (see
  // atLeast()). Drivetrain.floors; all 0 leaves the motions bare PIDs
  const kPushFloorVolts = { forward: 1.0, strafe: 4.0, turn: 2.5 };

  // holonomic_drivetrain.cpp's floorOff(): whether a motion's push floor is off, from when the
  // error first gets within the threshold until it's back past twice the threshold
  function floorOff(off, error, threshold) {
    const size = Math.abs(error);
    if (size <= threshold) return true;
    if (size > 2.0 * threshold) return false;
    return off;
  }

  // holonomic_drivetrain.cpp's atLeast(): volts raised to at least minVolts toward the error while
  // the floor is on; never against the error (the PID braking)
  function atLeast(volts, error, minVolts, reached) {
    if (reached || !(minVolts > 0.0) || volts * error < 0.0 || Math.abs(volts) >= minVolts) return volts;
    return Math.sign(error) * minVolts;
  }

  // holonomic_drivetrain.cpp's translationFloor(): the forward and sideways floors, blended by how
  // much of the way (forwardIn, lateralIn) is each
  function translationFloor(forwardIn, lateralIn, floors) {
    const squared = forwardIn * forwardIn + lateralIn * lateralIn;
    if (!(squared > 0.0)) return 0.0;
    return (forwardIn * forwardIn * floors.forward + lateralIn * lateralIn * floors.strafe) / squared;
  }

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
      // the odometers (Snapshot::travelledIn, turnedDeg): never reset, not even by setPose()
      this.travelledIn = 0;
      this.turnedDeg = 0;
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
      this.travelledIn += Math.hypot(delta.dxIn, delta.dyIn);
      this.turnedDeg += Math.abs(rotationDeg - this.lastRotationDeg);

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
        travelledIn: this.travelledIn,
        turnedDeg: this.turnedDeg,
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
      this.restartRequested = false;
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
        this.restartRequested = false;
        restarted = true;
      } else if (this.restartRequested) {
        // restartHere(): around odometry's pose, its correction kept
        this.restartRequested = false;
        this.filter.reset(snapshot.pose.xIn, snapshot.pose.yIn, this.config.startSpreadIn);
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
        // monte_carlo_localizer.cpp: a beam landing on a blind stretch (a Toggle), from where
        // odometry has the robot, is left out
        const map = this.filter.map;
        if (r.valid && map.blindStretches && map.blindStretches.length) {
          const p = w.snapshot.pose;
          const beam = L.sensorRay(p.xIn, p.yIn, beamHeadingDeg, this.filter.sensors[s]);
          if (map.landsOnBlindStretch(beam.xIn, beam.yIn, beam.dirX, beam.dirY, config.filter.beam.coneHalfAngleDeg)) {
            r = Object.assign({}, r, { valid: false });
          }
        }
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
      // monte_carlo_localizer.cpp: only along an axis some agreeing reading measures; the other
      // keeps the correction it had
      let measuresX = false;
      let measuresY = false;
      for (const c of w.checks) {
        if (!c.used || !c.agrees) continue;
        measuresX = measuresX || c.measuresX;
        measuresY = measuresY || c.measuresY;
      }
      const correctionXIn = measuresX ? w.estimate.xIn - w.raw.xIn : this.status.correctionXIn;
      const correctionYIn = measuresY ? w.estimate.yIn - w.raw.yIn : this.status.correctionYIn;
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

    /** resetFromWall(sensor, config): one sensor's reading resets x or y through setPose(). */
    resetFromWall(sensor, config = L.wallResetConfig()) {
      const snapshot = this.odometry.snapshot();
      let fix;
      if (sensor >= this.sensors.length) {
        fix = { ok: false, why: 'no such sensor' };
      } else if (this.config.waitForSetPose && snapshot.resetCount === 0) {
        fix = { ok: false, why: 'no setPose() yet' };
      } else {
        const m = this.filter.sensors[sensor];
        const raw = this.sensors[sensor]();
        let r = L.distanceReadingFromMm(raw.mm, raw.confidence, this.config.minConfidence, this.config.filter.beam);
        const ray = L.sensorRay(0.0, 0.0, snapshot.pose.headingDeg, m);
        const closingSpeed = snapshot.rawVelocity.xInPerS * ray.dirX + snapshot.rawVelocity.yInPerS * ray.dirY;
        r = L.compensateLatency(r, closingSpeed, this.config.sensorLatencyMs / 1000.0);
        fix = r.valid
          ? L.wallFix(this.filter.map, snapshot.pose.xIn, snapshot.pose.yIn, snapshot.pose.headingDeg, m,
            r.distanceIn, config)
          : { ok: false, why: 'no reading' };
      }
      if (fix.ok) {
        const pose = Object.assign({}, snapshot.pose);
        pose[fix.axis === 'x' ? 'xIn' : 'yIn'] = fix.valueIn;
        this.odometry.setPose(pose);
      }
      return fix;
    }

    setCorrectionEnabled(enabled) {
      this.correctionEnabled = enabled;
    }

    relocalizeGlobally() {
      this.relocalizeRequested = true;
    }

    /** monte_carlo_localizer.cpp's restartHere(): the particles over around odometry's pose next update. */
    restartHere() {
      this.restartRequested = true;
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

  // --- motion::TargetApproach (target_approach.cpp) ----------------------------------

  /** MotorFeedforward::volts(): kS takes the velocity's sign, or the acceleration's from rest. */
  function motorVolts(model, velocity, acceleration = 0.0) {
    const sign = (x) => (x > 0 ? 1 : (x < 0 ? -1 : 0));
    const direction = velocity !== 0 ? sign(velocity) : sign(acceleration);
    return model.kS * direction + model.kV * velocity + model.kA * acceleration;
  }

  /** approachSpeedInPerS(): the fastest the robot may go this far from the end and still stop there. */
  function approachSpeedInPerS(remainingIn, cruiseInPerS, decelInPerS2) {
    if (!(remainingIn > 0)) return 0.0;
    const cruise = Math.max(cruiseInPerS, 0.0);
    if (!(decelInPerS2 > 0)) return cruise;
    return Math.min(cruise, Math.sqrt(2.0 * decelInPerS2 * remainingIn));
  }

  // the heading error at which TargetApproach adds the turn model's whole kS
  const kTurnFrictionRampDeg = 1.5;

  /** ApproachConfig's defaults. */
  function approachConfig(c = {}) {
    return Object.assign({ cruiseInPerS: 15.0, accelInPerS2: 60.0, decelInPerS2: 60.0, endShortIn: 0.0,
      endThresholdIn: 0.5, steerMinIn: 6.0, maxLagIn: 2.0, timeoutMs: 3000, maxVolts: 12.0 }, c);
  }

  class TargetApproach {
    constructor(forward, turn) {
      this.forward = forward;
      this.turn = turn;
    }

    begin(config, startSpeedInPerS, nowMs) {
      this.config = approachConfig(config);
      this.tracker = new ExitTracker(0, this.config.timeoutMs, nowMs);
      this.reason = 'running';
      this.speedRefInPerS = Math.max(startSpeedInPerS, 0.0);
      this.lagIn = 0.0;
      this.aimDeg = 0.0;
      this.aimed = false;
      this.started = false;
      this.lastMs = nowMs;
      this.finalErrorIn = 0.0;
      this.forward.reset();
      this.turn.reset();
    }

    update(pose, velocity, target, forwardModel, turnModel, nowMs) {
      const c = this.config;
      const dtS = this.started ? (nowMs - this.lastMs) / 1000.0 : 0.0;
      this.started = true;
      this.lastMs = nowMs;
      const h = pose.headingDeg * kDegToRad;
      const speedInPerS = velocity.xInPerS * Math.sin(h) + velocity.yInPerS * Math.cos(h);
      const dx = target.xIn - pose.xIn;
      const dy = target.yIn - pose.yIn;
      const tick = { distanceIn: Math.hypot(dx, dy), steering: false };
      tick.remainingIn = tick.distanceIn - c.endShortIn;
      if (!this.aimed || tick.distanceIn > c.steerMinIn) {
        let aim = Math.atan2(dx, dy) / kDegToRad;
        if (aim < 0) aim += 360.0;
        this.aimDeg = aim;
        this.aimed = true;
        tick.steering = true;
      }
      tick.aimDeg = this.aimDeg;
      tick.headingErrorDeg = L.wrapDegrees180(this.aimDeg - pose.headingDeg);

      const goal = approachSpeedInPerS(tick.remainingIn, c.cruiseInPerS, c.decelInPerS2);
      const before = this.speedRefInPerS;
      if (dtS > 0) {
        this.speedRefInPerS = goal > this.speedRefInPerS
          ? Math.min(goal, this.speedRefInPerS + c.accelInPerS2 * dtS)
          : Math.max(goal, this.speedRefInPerS - c.decelInPerS2 * dtS);
        this.lagIn = Math.min(Math.max(this.lagIn + (this.speedRefInPerS - speedInPerS) * dtS, -c.maxLagIn), c.maxLagIn);
      }
      tick.speedRefInPerS = this.speedRefInPerS;
      tick.accelRefInPerS2 = dtS > 0 ? (this.speedRefInPerS - before) / dtS : 0.0;
      tick.lagIn = this.lagIn;
      // PID.updateWithRate()'s explicit dt; 0 falls back to the nominal 10ms, as on the robot
      tick.forwardVolts = motorVolts(forwardModel, this.speedRefInPerS, tick.accelRefInPerS2) +
        this.forward.updateWithRate(this.lagIn, 0.0, this.speedRefInPerS - speedInPerS, dtS);
      // the turn's static friction toward the target, eased in over kTurnFrictionRampDeg
      const friction = turnModel.kS * Math.min(Math.max(tick.headingErrorDeg / kTurnFrictionRampDeg, -1), 1);
      tick.turnVolts = friction + this.turn.updateWithRate(tick.headingErrorDeg, 0.0, -velocity.headingDegPerS, dtS);

      this.finalErrorIn = Math.abs(tick.remainingIn);
      this.reason = this.tracker.update(tick.remainingIn <= c.endThresholdIn, nowMs);
      return tick;
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
      // the point motions' push floors (kPushFloorVolts), the robot program's
      this.floors = Object.assign({}, kPushFloorVolts);
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
      this.driveAxes(forwardVolts, strafeVolts, turnVolts, kFullScaleVolts);
    }

    /** holonomicVolts() with no wheel past ceilingVolts instead of 12: ExitConditions::maxVolts. */
    driveAxes(forwardVolts, strafeVolts, turnVolts, ceilingVolts) {
      const fl = forwardVolts + strafeVolts + turnVolts;
      const fr = forwardVolts - strafeVolts - turnVolts;
      const bl = forwardVolts - strafeVolts + turnVolts;
      const br = forwardVolts + strafeVolts - turnVolts;
      const ceiling = Math.min(Math.max(ceilingVolts, 1e-6), kFullScaleVolts);
      const largest = Math.max(Math.abs(fl), Math.abs(fr), Math.abs(bl), Math.abs(br), ceiling) / ceiling;
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
      const tracker = new ExitTracker(settleMs(exit), exit.timeoutMs, this.clock());
      let reason = 'running';
      let finalErrorIn = 0;
      // whether the push floor is off, see floorOff()
      let reached = false;
      for (;;) {
        const pose = this.odometry.getPose();
        const dxIn = xIn - pose.xIn;
        const dyIn = yIn - pose.yIn;
        const distanceIn = Math.hypot(dxIn, dyIn);
        reached = floorOff(reached, distanceIn, exit.errorThreshold);
        const local = toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const outputVolts = atLeast(this.drivePID.update(distanceIn, 0.0), distanceIn,
          translationFloor(local.forwardIn, local.lateralIn, this.floors), reached);
        const scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;
        const turnOutput = this.turnPID.update(L.wrapDegrees180(holdHeadingDeg - pose.headingDeg), 0.0);
        this.driveAxes(local.forwardIn * scale, local.lateralIn * scale, turnOutput, exit.maxVolts);
        finalErrorIn = distanceIn;
        reason = tracker.update(distanceIn <= exit.errorThreshold, this.clock());
        if (reason !== 'running') break;
        yield;
      }
      if (!chainsOn(exit, reason)) this.stop();
      this.target = null;
      return { reason, finalError: finalErrorIn, elapsedMs: tracker.elapsedMs };
    }

    *moveToPose(xIn, yIn, headingDeg, exit = {}) {
      exit = Object.assign({ positionErrorThresholdIn: 1.0, headingErrorThresholdDeg: 2.0, settleTimeMs: 200,
        timeoutMs: 4000, maxVolts: 12.0, chain: false }, exit);
      this.drivePID.reset();
      this.turnPID.reset();
      this.target = { kind: 'pose', xIn, yIn, headingDeg };
      const tracker = new ExitTracker(settleMs(exit), exit.timeoutMs, this.clock());
      let reason = 'running';
      let finalErrorIn = 0;
      // whether position's and heading's push floors are off, see floorOff()
      let positionReached = false;
      let headingReached = false;
      for (;;) {
        const pose = this.odometry.getPose();
        const dxIn = xIn - pose.xIn;
        const dyIn = yIn - pose.yIn;
        const distanceIn = Math.hypot(dxIn, dyIn);
        positionReached = floorOff(positionReached, distanceIn, exit.positionErrorThresholdIn);
        const local = toLocalFrame(dxIn, dyIn, pose.headingDeg);
        const outputVolts = atLeast(this.drivePID.update(distanceIn, 0.0), distanceIn,
          translationFloor(local.forwardIn, local.lateralIn, this.floors), positionReached);
        const scale = distanceIn > 1e-6 ? outputVolts / distanceIn : 0.0;
        const headingError = L.wrapDegrees180(headingDeg - pose.headingDeg);
        headingReached = floorOff(headingReached, headingError, exit.headingErrorThresholdDeg);
        const turnOutput = atLeast(this.turnPID.update(headingError, 0.0), headingError, this.floors.turn, headingReached);
        this.driveAxes(local.forwardIn * scale, local.lateralIn * scale, turnOutput, exit.maxVolts);
        const within = distanceIn <= exit.positionErrorThresholdIn &&
          Math.abs(headingError) <= exit.headingErrorThresholdDeg;
        finalErrorIn = distanceIn;
        reason = tracker.update(within, this.clock());
        if (reason !== 'running') break;
        yield;
      }
      if (!chainsOn(exit, reason)) this.stop();
      this.target = null;
      return { reason, finalError: finalErrorIn, elapsedMs: tracker.elapsedMs };
    }

    /** turnToHeading(heading, exit, direction): 'shortest' (the default), 'clockwise' or 'counterclockwise'. */
    *turnToHeading(headingDeg, exit = {}, direction = 'shortest') {
      exit = exitConditions(exit, 2.0);
      this.turnPID.reset();
      const turn = new T.DirectedTurn(headingDeg, this.imu.getHeadingDeg(), direction);
      this.target = { kind: 'heading', headingDeg };
      const tracker = new ExitTracker(settleMs(exit), exit.timeoutMs, this.clock());
      let reason = 'running';
      let finalErrorDeg = 0;
      // whether the push floor is off, see floorOff()
      let reached = false;
      for (;;) {
        const error = turn.errorDeg(this.imu.getHeadingDeg());
        reached = floorOff(reached, error, exit.errorThreshold);
        const output = atLeast(this.turnPID.update(error, 0.0), error, this.floors.turn, reached);
        this.driveAxes(0.0, 0.0, output, exit.maxVolts);
        finalErrorDeg = Math.abs(error);
        reason = tracker.update(Math.abs(error) <= exit.errorThreshold, this.clock());
        if (reason !== 'running') break;
        yield;
      }
      if (!chainsOn(exit, reason)) this.stop();
      this.target = null;
      return { reason, finalError: finalErrorDeg, elapsedMs: tracker.elapsedMs };
    }

    /**
     * driveDistance(inches, exit): straight by the drive encoders (the world's forward travel),
     * holding the heading it started on with DrivetrainConfig::headingCorrectionKP's 0.4.
     */
    *driveDistance(inches, exit = {}) {
      exit = exitConditions(exit, 1.0);
      const startHeadingDeg = this.imu.getCumulativeHeadingDeg();
      const startIn = this.world.forwardEncoderIn;
      this.drivePID.reset();
      this.target = null;
      const tracker = new ExitTracker(settleMs(exit), exit.timeoutMs, this.clock());
      let reason = 'running';
      let finalErrorIn = 0;
      // whether the push floor is off, see floorOff()
      let reached = false;
      for (;;) {
        const traveledIn = this.world.forwardEncoderIn - startIn;
        const error = inches - traveledIn;
        reached = floorOff(reached, error, exit.errorThreshold);
        const output = atLeast(this.drivePID.update(inches, traveledIn), error, this.floors.forward, reached);
        const correction = kHeadingCorrectionKP *
          L.wrapDegrees180(startHeadingDeg - this.imu.getCumulativeHeadingDeg());
        this.driveAxes(output, 0.0, correction, exit.maxVolts);
        finalErrorIn = Math.abs(error);
        reason = tracker.update(Math.abs(error) <= exit.errorThreshold, this.clock());
        if (reason !== 'running') break;
        yield;
      }
      if (!chainsOn(exit, reason)) this.stop();
      return { reason, finalError: finalErrorIn, elapsedMs: tracker.elapsedMs };
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
      let handedOff = false;
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
        // TrajectoryFollowConfig::until: handed off, still moving, the volts above left on
        if (config.until && config.until()) {
          handedOff = true;
          break;
        }
        yield;
      }
      if (!handedOff) this.stop();
      this.target = null;
      const result = follower.result();
      if (handedOff) {
        result.reason = 'settled';
        this.progress = Object.assign({}, this.progress, { running: false });
      }
      if (this.trajectoryObserver && this.trajectoryObserver.onTrajectoryEnd) this.trajectoryObserver.onTrajectoryEnd(result);
      return result;
    }

    /**
     * approach(target, config, refine): HolonomicDrivetrain::approach(). Onto `target` ({ xIn, yIn })
     * facing it, from the speed the robot has; refine(pose, target), if given, may move the target
     * every tick (vision.js's PinSight). this.approachTicks, if it's an array, gets every tick.
     */
    *approach(target, config = {}, refine = null) {
      const models = this.axisModels;
      if (!models || !(models.forward.kV > 0 && models.forward.kA > 0)) {
        return { reason: 'aborted', finalError: 0, elapsedMs: 0 };
      }
      config = approachConfig(config);
      config.maxVolts = Math.min(Math.max(config.maxVolts, 0.0), kFullScaleVolts);
      const first = this.odometry.snapshot();
      const h = first.pose.headingDeg * kDegToRad;
      const startInPerS = first.rawVelocity.xInPerS * Math.sin(h) + first.rawVelocity.yInPerS * Math.cos(h);
      const controller = new TargetApproach(this.trackingPIDs.forward, this.trackingPIDs.turn);
      controller.begin(config, startInPerS, this.clock());
      target = { xIn: target.xIn, yIn: target.yIn };
      // for the visualizer: the target, as the camera moves it
      this.target = { kind: 'approach', xIn: target.xIn, yIn: target.yIn };
      for (;;) {
        const snapshot = this.odometry.snapshot();
        if (refine) refine(snapshot.pose, target);
        const tick = controller.update(snapshot.pose, snapshot.rawVelocity, target, models.forward, models.turn,
          this.clock());
        this.driveAxes(tick.forwardVolts, 0.0, tick.turnVolts, config.maxVolts);
        this.target.xIn = target.xIn;
        this.target.yIn = target.yIn;
        if (Array.isArray(this.approachTicks)) {
          const a = snapshot.pose.headingDeg * kDegToRad;
          const speedInPerS = snapshot.rawVelocity.xInPerS * Math.sin(a) + snapshot.rawVelocity.yInPerS * Math.cos(a);
          this.approachTicks.push(Object.assign({ tMs: this.clock(), pose: snapshot.pose, speedInPerS,
            targetXIn: target.xIn, targetYIn: target.yIn }, tick));
        }
        if (controller.reason !== 'running') break;
        yield;
      }
      this.stop();
      this.target = null;
      return controller.result();
    }
  }

  function exitConditions(exit, defaultThreshold) {
    return Object.assign({ errorThreshold: defaultThreshold, settleTimeMs: 200, timeoutMs: 3000, maxVolts: 12.0,
      chain: false }, exit);
  }

  // a chained motion ends the first tick it's there, and leaves the motors running (ExitConditions::chain)
  function settleMs(exit) {
    return exit.chain ? 0 : exit.settleTimeMs;
  }

  function chainsOn(exit, reason) {
    return exit.chain && reason === 'settled';
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
      // the robot's other tasks, each called with the time every tick after odometry and the
      // localizer: the lift's and autoClaw's (macros.js) once a routine makes them
      this.tasks = [];
      // the macros (macros.js), once a routine makes them
      this.macros = null;
      // what the program printed on the terminal: { tMs, source, text }
      this.events = [];
    }

    /** One 10ms tick of every task, after the world has advanced. */
    tick() {
      this.odometry.update();
      if (this.world.timeMs % this.localizerPeriodMs === 0) this.localizer.update();
      for (const task of this.tasks) task(this.world.timeMs);
    }

    /** A line on the terminal, as SAPPHIRELIB_LOG_INFO() prints one. */
    log(source, text) {
      this.events.push({ tMs: this.world.timeMs, source, text });
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
    kPushFloorVolts,
    RESPONSE_SPECS,
    autoTuneGains,
    TrajectoryFollower,
    motorVolts,
    approachSpeedInPerS,
    approachConfig,
    TargetApproach,
    Drivetrain,
    Robot,
  };
});
