/*
 * SapphireLib simulator: world.js
 *
 * The physical side of the simulation: where the robot really is, and what
 * its sensors report about it.
 *
 * The chassis is the three axis models Auto-Tune measures (forward, strafe,
 * turn: V = kS·sign(v) + kV·v + kA·a, plus a command delay), driven by the
 * axis volts the robot program commands, each one a PlantSim from the
 * analyzer's model.js. The six motors and the Asterisk center wheels are
 * folded into those models, as they are on the robot when Auto-Tune measures
 * the whole chassis at once; per-motor effects (thermal derating, drift
 * correction) aren't simulated.
 *
 * The sensors are where the realism goes, since they're what localization is
 * about:
 *   - two tracking wheels, mounted where OdometryConfig's offsets say, rolling
 *     with the chassis, with a diameter error and random slip;
 *   - an IMU with drift, a scale error, and noise;
 *   - four distance sensors measuring at about 30Hz, each reading a few tens
 *     of milliseconds stale by the time it's read, with the V5 sensor's noise,
 *     its 2000mm range, and dropouts; they see the walls, any field elements,
 *     and the defender robot;
 *   - and things odometry never sees: bumps, and being picked up and put down.
 *
 * Plain script: window.SIM.world in a browser, require('./world.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./mcl.js'), require('../../analyzer/js/model.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.world = factory(root.SIM.mcl, root.SA.model);
  }
})(function (L, M) {
  'use strict';

  const kDegToRad = L.kDegToRad;
  const kPhysicsStepS = 0.001;
  const kHistoryMs = 250;
  const kNoObjectMm = 9999;
  const kMaxRangeMm = 2000;

  /** Everything about the simulated robot and field that isn't the robot program. */
  function defaultSettings() {
    return {
      fieldSizeIn: L.kVrcFieldSizeIn,
      robotSizeIn: 15,
      // the chassis, as Auto-Tune would measure it
      models: {
        forward: { kS: 1.0, kV: 0.2, kA: 0.045 },
        strafe: { kS: 1.3, kV: 0.24, kA: 0.055 },
        turn: { kS: 0.6, kV: 0.045, kA: 0.0065 },
      },
      delayS: 0.03,
      // tracking wheels, where OdometryConfig's offsets say they are
      verticalOffsetIn: 3.59,
      horizontalOffsetIn: 4.18,
      // what the robot program doesn't know
      wheelDiameterErrorPct: 1.0, // the real wheels roll this much farther than configured
      offsetErrorIn: 0.0, // the real offsets are off by this much
      verticalWheelFlipped: false, // the vertical wheel is really on the other side: the sign mistake
      wheelSlipPct: 0.5, // random slip, as a percent of travel (1 sigma)
      imuDriftDegPerMin: 0.5,
      imuScaleErrorPct: 0.3,
      imuNoiseDeg: 0.01,
      // distance sensors
      sensors: [
        L.mount(7, 0, 0),
        L.mount(0, 7, 90),
        L.mount(-7, 0, 180),
        L.mount(0, -7, 270),
      ],
      sensorNoiseScale: 1.0, // 1 is half the V5 spec as the standard deviation
      sensorPeriodMs: 33,
      sensorDelayMs: 15, // measurement to the value being readable
      sensorDropoutPct: 1.0,
      // field elements (boxes), and whether each is really there
      elements: [],
      defender: { enabled: false, xIn: 30, yIn: 20, sizeIn: 18, patrol: false },
      seed: 7,
    };
  }

  /** A few generic field elements for trying things out; not any season's game. */
  function exampleElements() {
    return [
      { name: 'center structure', minXIn: -10, minYIn: -10, maxXIn: 10, maxYIn: 10 },
      { name: 'goal NE', minXIn: 41, minYIn: 41, maxXIn: 49, maxYIn: 49 },
      { name: 'goal NW', minXIn: -49, minYIn: 41, maxXIn: -41, maxYIn: 49 },
      { name: 'goal SE', minXIn: 41, minYIn: -49, maxXIn: 49, maxYIn: -41 },
      { name: 'goal SW', minXIn: -49, minYIn: -49, maxXIn: -41, maxYIn: -41 },
    ];
  }

  function gaussianAt(rng, sigma) {
    return sigma > 0 ? sigma * rng.gaussian() : 0;
  }

  class World {
    constructor(settings = defaultSettings(), start = { xIn: 0, yIn: 0, headingDeg: 0 }) {
      this.settings = settings;
      this.rng = new L.Rng(settings.seed);
      const s = settings;
      const plant = (m) => new M.PlantSim(M.mechanismModel(M.feedforward(m.kS, m.kV, m.kA)), s.delayS);
      this.plants = { forward: plant(s.models.forward), strafe: plant(s.models.strafe), turn: plant(s.models.turn) };

      this.timeMs = 0;
      this.truth = { xIn: start.xIn, yIn: start.yIn, headingDeg: start.headingDeg };
      // total rotation since the start, what a perfect IMU would report
      this.rotationDeg = 0;
      this.imuDeg = 0;
      this.verticalIn = 0;
      this.horizontalIn = 0;
      this.forwardEncoderIn = 0;
      this.history = [];
      this.readingsMm = s.sensors.map(() => kNoObjectMm);
      // stagger the sensors so they don't all update on the same millisecond
      this.sensorPhaseMs = s.sensors.map((_, i) => Math.round((i * s.sensorPeriodMs) / s.sensors.length));
      this.lastBeams = s.sensors.map(() => null);
      this.bumps = 0;
      this.patrolT = 0;
      this.rebuildMap();
      this.recordHistory();
    }

    /** The walls, the elements that are really there, and the defender. */
    rebuildMap() {
      const s = this.settings;
      const map = L.FieldMap.centered(s.fieldSizeIn);
      for (const e of s.elements) {
        if (e.inWorld !== false) map.addBox(e.minXIn, e.minYIn, e.maxXIn, e.maxYIn);
      }
      const d = s.defender;
      if (d.enabled) {
        const h = d.sizeIn / 2;
        map.addBox(d.xIn - h, d.yIn - h, d.xIn + h, d.yIn + h);
      }
      this.map = map;
    }

    /** Axis volts, after the drivetrain's wheel mixing and 12V scaling. */
    command(forwardVolts, strafeVolts, turnVolts) {
      this.plants.forward.command(forwardVolts);
      this.plants.strafe.command(strafeVolts);
      this.plants.turn.command(turnVolts);
    }

    /** The drivetrain's stop(): every axis on its brake. */
    brake() {
      this.plants.forward.hold();
      this.plants.strafe.hold();
      this.plants.turn.hold();
    }

    /** Advance the world by whole milliseconds. */
    step(ms) {
      for (let i = 0; i < ms; ++i) this.stepOne();
    }

    stepOne() {
      const s = this.settings;
      const p = this.plants;
      for (const plant of [p.forward, p.strafe, p.turn]) plant.advance(kPhysicsStepS);

      const x0 = this.truth.xIn;
      const y0 = this.truth.yIn;
      const h0 = this.truth.headingDeg;

      // integrate at the middle of the step's rotation
      const turnDeg = p.turn.v * kPhysicsStepS;
      const midRad = (h0 + turnDeg / 2) * kDegToRad;
      const sin = Math.sin(midRad);
      const cos = Math.cos(midRad);
      this.truth.xIn += (p.forward.v * sin + p.strafe.v * cos) * kPhysicsStepS;
      this.truth.yIn += (p.forward.v * cos - p.strafe.v * sin) * kPhysicsStepS;
      this.truth.headingDeg += turnDeg;
      this.collide();

      // what the robot really did this step, in its own frame
      const dx = this.truth.xIn - x0;
      const dy = this.truth.yIn - y0;
      const dThetaRad = (this.truth.headingDeg - h0) * kDegToRad;
      const forwardIn = dx * sin + dy * cos;
      const rightIn = dx * cos - dy * sin;
      this.forwardEncoderIn += forwardIn;

      // Tracking wheels roll with the chassis. OdometryConfig's arc correction
      // is `wheel - offset * dTheta`, so a wheel it calls offset +o rolls
      // +o * dTheta on a clockwise turn: for the vertical wheel that's a wheel
      // physically o inches LEFT of center (a right-side wheel rolls backward
      // on a right turn); for the horizontal wheel, o inches ahead of center.
      // tests/odom/odometry_math_test.cpp pins that convention.
      const verticalOffset = (s.verticalWheelFlipped ? -1 : 1) * s.verticalOffsetIn + s.offsetErrorIn;
      const horizontalOffset = s.horizontalOffsetIn + s.offsetErrorIn;
      const scale = 1 + s.wheelDiameterErrorPct / 100;
      const slip = s.wheelSlipPct / 100;
      const verticalTravel = forwardIn + verticalOffset * dThetaRad;
      const horizontalTravel = rightIn + horizontalOffset * dThetaRad;
      this.verticalIn += scale * verticalTravel + gaussianAt(this.rng, slip * Math.abs(verticalTravel));
      this.horizontalIn += scale * horizontalTravel + gaussianAt(this.rng, slip * Math.abs(horizontalTravel));

      // the IMU
      const turned = this.truth.headingDeg - h0;
      this.rotationDeg += turned;
      this.imuDeg += turned * (1 + s.imuScaleErrorPct / 100) + (s.imuDriftDegPerMin / 60000);

      this.timeMs += 1;
      this.moveDefender();
      this.recordHistory();
      this.sampleSensors();
    }

    /** Keep the robot (a circle) out of the walls and elements, sliding along them. */
    collide() {
      const s = this.settings;
      const r = s.robotSizeIn / 2;
      const half = s.fieldSizeIn / 2;
      const t = this.truth;
      if (t.xIn < -half + r) this.pushOut(-half + r - t.xIn, 0);
      if (t.xIn > half - r) this.pushOut(half - r - t.xIn, 0);
      if (t.yIn < -half + r) this.pushOut(0, -half + r - t.yIn);
      if (t.yIn > half - r) this.pushOut(0, half - r - t.yIn);
      const boxes = s.elements.filter((e) => e.inWorld !== false);
      if (s.defender.enabled) {
        const d = s.defender;
        const h = d.sizeIn / 2;
        boxes.push({ minXIn: d.xIn - h, minYIn: d.yIn - h, maxXIn: d.xIn + h, maxYIn: d.yIn + h });
      }
      for (const b of boxes) {
        const cx = Math.min(Math.max(t.xIn, b.minXIn), b.maxXIn);
        const cy = Math.min(Math.max(t.yIn, b.minYIn), b.maxYIn);
        const dx = t.xIn - cx;
        const dy = t.yIn - cy;
        const dist = Math.hypot(dx, dy);
        if (dist >= r) continue;
        if (dist > 1e-9) {
          this.pushOut((dx / dist) * (r - dist), (dy / dist) * (r - dist));
        } else {
          // the center is inside the box: out the nearest side
          const exits = [
            [b.minXIn - r - t.xIn, 0], [b.maxXIn + r - t.xIn, 0],
            [0, b.minYIn - r - t.yIn], [0, b.maxYIn + r - t.yIn],
          ];
          exits.sort((a, c) => Math.hypot(a[0], a[1]) - Math.hypot(c[0], c[1]));
          this.pushOut(exits[0][0], exits[0][1]);
        }
      }
    }

    /** Move the robot by (dx, dy) and drop the velocity pushing back the other way. */
    pushOut(dx, dy) {
      this.truth.xIn += dx;
      this.truth.yIn += dy;
      const length = Math.hypot(dx, dy);
      if (length < 1e-12) return;
      const nx = dx / length;
      const ny = dy / length;
      const rad = this.truth.headingDeg * kDegToRad;
      const sin = Math.sin(rad);
      const cos = Math.cos(rad);
      const p = this.plants;
      let vx = p.forward.v * sin + p.strafe.v * cos;
      let vy = p.forward.v * cos - p.strafe.v * sin;
      const into = vx * nx + vy * ny;
      if (into >= 0) return;
      vx -= into * nx;
      vy -= into * ny;
      p.forward.v = vx * sin + vy * cos;
      p.strafe.v = vx * cos - vy * sin;
    }

    moveDefender() {
      const d = this.settings.defender;
      if (!d.enabled || !d.patrol) return;
      // back and forth across the field's middle, 12 seconds a lap
      this.patrolT += 0.001;
      d.xIn = 40 * Math.sin((this.patrolT * 2 * Math.PI) / 12);
      if (this.timeMs % 20 === 0) this.rebuildMap();
    }

    recordHistory() {
      this.history.push({ t: this.timeMs, xIn: this.truth.xIn, yIn: this.truth.yIn,
        headingDeg: this.truth.headingDeg });
      while (this.history.length > kHistoryMs) this.history.shift();
    }

    /** Where the robot was `ageMs` ago. */
    pastTruth(ageMs) {
      const index = Math.max(0, this.history.length - 1 - Math.round(ageMs));
      return this.history[index];
    }

    sampleSensors() {
      const s = this.settings;
      for (let i = 0; i < s.sensors.length; ++i) {
        if ((this.timeMs + this.sensorPhaseMs[i]) % s.sensorPeriodMs !== 0) continue;
        const pose = this.pastTruth(s.sensorDelayMs);
        const ray = L.sensorRay(pose.xIn, pose.yIn, pose.headingDeg, s.sensors[i]);
        const trueIn = this.map.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
        let mm = trueIn * 25.4;
        mm += gaussianAt(this.rng, s.sensorNoiseScale * 0.5 * Math.max(15, 0.05 * mm));
        const dropped = this.rng.uniform() * 100 < s.sensorDropoutPct;
        this.readingsMm[i] = !Number.isFinite(mm) || mm > kMaxRangeMm || dropped ? kNoObjectMm
          : Math.max(0, Math.round(mm));
        this.lastBeams[i] = { ray, trueIn };
      }
    }

    // --- what the robot program reads -------------------------------------------

    /** pros::Distance::get_distance() and get_confidence(). */
    distanceSensor(i) {
      return { mm: this.readingsMm[i], confidence: 63 };
    }

    /** RotationTrackingWheel::getDistanceIn(). */
    verticalWheelIn() {
      return this.verticalIn;
    }

    horizontalWheelIn() {
      return this.horizontalIn;
    }

    /** The IMU's cumulative rotation, with drift, scale error, and noise. */
    imuCumulativeDeg() {
      return this.imuDeg + gaussianAt(this.rng, this.settings.imuNoiseDeg);
    }

    // --- things odometry never sees ----------------------------------------------

    /** A hit: the robot slides (dx, dy) without its tracking wheels turning. */
    bump(dxIn, dyIn) {
      this.truth.xIn += dxIn;
      this.truth.yIn += dyIn;
      this.collide();
      this.bumps++;
    }

    /** Pick the robot up and put it down somewhere else, facing the same way. */
    kidnap(xIn, yIn) {
      this.truth.xIn = xIn;
      this.truth.yIn = yIn;
      this.collide();
    }
  }

  return { defaultSettings, exampleElements, World };
});
