/*
 * SapphireLib simulator: trajectory.js
 *
 * The trajectory planner and follower math, ported line for line from the
 * library so the simulator (and its Paths tab) plans exactly what the robot
 * plans:
 *
 *   samplePath()               src/sapphirelib/motion/trajectory_path.cpp
 *   generateTrajectory(),
 *   voltageAccelRange()        src/sapphirelib/motion/trajectory_generator.cpp
 *   Trajectory                 src/sapphirelib/motion/trajectory.cpp
 *   frictionVolts(), holonomicFeedforward(), rotateAxisMotion(),
 *   trajectoryFeedforward()    src/sapphirelib/motion/holonomic_feedforward.cpp
 *
 * Shapes follow the C++ structs, with plain objects: a waypoint is
 * { xIn, yIn, headingDeg } (headingDeg left out or null for none), a section
 * { waypoints, constraints, stopAtEnd }, constraints { voltageBudget,
 * forward, strafe, turn } with each axis { maxVelocity, maxAcceleration },
 * and models { forward, strafe, turn } with each { kS, kV, kA }.
 *
 * tests/motion/trajectory_*_test.cpp and test/trajectory.test.js share golden
 * values. Change the C++, change the port.
 *
 * Plain script: window.SIM.trajectory in a browser, require('./trajectory.js')
 * in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./mcl.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.trajectory = factory(root.SIM.mcl);
  }
})(function (L) {
  'use strict';

  const kPi = 3.14159265358979323846;
  const kDegToRad = kPi / 180.0;
  const kRadToDeg = 180.0 / kPi;

  // trajectory_path.cpp
  const kTrajectorySampleSpacingIn = 0.5;
  const kMinChordIn = 1e-6;
  const kArcStepIn = 0.05;

  // trajectory_generator.cpp
  const kTiny = 1e-12;
  const kBisectionSteps = 40;
  const kMaxVolts = 12.0;

  // holonomic_feedforward.cpp
  const kMovingFraction = 1e-3;

  // --- MotorFeedforward (feedforward.cpp) -----------------------------------------------------

  function modelValid(m) {
    return m.kV > 0.0 && m.kA > 0.0;
  }

  function modelMaxVelocity(m, availableVolts = 12.0) {
    if (!(m.kV > 0.0)) return 0.0;
    return Math.max(0.0, (Math.abs(availableVolts) - m.kS) / m.kV);
  }

  // --- Defaults (trajectory_config.hpp) -------------------------------------------------------

  function axisLimits(a = {}) {
    return { maxVelocity: a.maxVelocity || 0.0, maxAcceleration: a.maxAcceleration || 0.0 };
  }

  function trajectoryConstraints(c = {}) {
    return {
      voltageBudget: c.voltageBudget === undefined ? 10.0 : c.voltageBudget,
      forward: axisLimits(c.forward),
      strafe: axisLimits(c.strafe),
      turn: axisLimits(c.turn),
    };
  }

  function trajectorySection(s) {
    return {
      waypoints: s.waypoints || [],
      constraints: trajectoryConstraints(s.constraints),
      stopAtEnd: !!s.stopAtEnd,
    };
  }

  function followConfig(c = {}) {
    return Object.assign({ positionErrorThresholdIn: 1.0, headingErrorThresholdDeg: 2.0, settleTimeMs: 200,
      overtimeMs: 1500, maxLagIn: 4.0, next: null }, c);
  }

  function hasHeading(w) {
    return w.headingDeg !== undefined && w.headingDeg !== null;
  }

  // --- samplePath() (trajectory_path.cpp) -----------------------------------------------------

  /** startSlope: null for a natural start, or the first derivative to clamp it to. */
  function secondDerivatives(t, values, startSlope = null) {
    const n = t.length - 1;
    const m = new Array(n + 1).fill(0.0);
    const clamped = startSlope !== null && startSlope !== undefined;
    const first = clamped ? 0 : 1;
    if (n < first + 1) return m;
    const diag = new Array(n + 1).fill(0.0);
    const rhs = new Array(n + 1).fill(0.0);
    if (clamped) {
      const h1 = t[1] - t[0];
      diag[0] = 2.0 * h1;
      rhs[0] = 6.0 * ((values[1] - values[0]) / h1 - startSlope);
    }
    for (let i = 1; i < n; ++i) {
      const h0 = t[i] - t[i - 1];
      const h1 = t[i + 1] - t[i];
      diag[i] = 2.0 * (h0 + h1);
      rhs[i] = 6.0 * ((values[i + 1] - values[i]) / h1 - (values[i] - values[i - 1]) / h0);
    }
    for (let i = first + 1; i < n; ++i) {
      const h0 = t[i] - t[i - 1];
      const factor = h0 / diag[i - 1];
      diag[i] -= factor * h0;
      rhs[i] -= factor * rhs[i - 1];
    }
    for (let i = n - 1; i >= first; --i) {
      const h1 = t[i + 1] - t[i];
      m[i] = (rhs[i] - h1 * m[i + 1]) / diag[i];
    }
    return m;
  }

  function evaluate(piece, i, t) {
    const h = piece.t[i + 1] - piece.t[i];
    const u = t - piece.t[i];
    const w = piece.t[i + 1] - t;
    const axis = (v, m) => {
      const a = v[i] / h - m[i] * h / 6.0;
      const b = v[i + 1] / h - m[i + 1] * h / 6.0;
      return {
        f: m[i] * w * w * w / (6.0 * h) + m[i + 1] * u * u * u / (6.0 * h) + a * w + b * u,
        df: -m[i] * w * w / (2.0 * h) + m[i + 1] * u * u / (2.0 * h) - a + b,
        ddf: m[i] * w / h + m[i + 1] * u / h,
      };
    };
    const x = axis(piece.x, piece.mx);
    const y = axis(piece.y, piece.my);
    return { x: x.f, y: y.f, dx: x.df, dy: y.df, ddx: x.ddf, ddy: y.ddf };
  }

  function headingAt(keys, sIn, sample) {
    sample.headingRateDegPerIn = 0.0;
    sample.headingAccelDegPerIn2 = 0.0;
    const last = keys[keys.length - 1];
    if (sIn >= last.sIn) {
      sample.headingDeg = last.headingDeg;
      return;
    }
    let k = 0;
    while (k + 1 < keys.length && keys[k + 1].sIn <= sIn) ++k;
    const a = keys[k];
    const b = keys[k + 1];
    const span = b.sIn - a.sIn;
    const delta = b.headingDeg - a.headingDeg;
    const u = (sIn - a.sIn) / span;
    const u2 = u * u;
    const u3 = u2 * u;
    sample.headingDeg = a.headingDeg + delta * (6.0 * u2 * u3 - 15.0 * u2 * u2 + 10.0 * u3);
    sample.headingRateDegPerIn = delta * (30.0 * u2 * u2 - 60.0 * u3 + 30.0 * u2) / span;
    sample.headingAccelDegPerIn2 = delta * (120.0 * u3 - 180.0 * u2 + 60.0 * u) / (span * span);
  }

  function pathFailure(error) {
    return { ok: false, error, samples: [], lengthIn: 0.0, sectionStartIn: [] };
  }

  /**
   * ends (PathEnds): { startTangentDeg (null to start at rest), movingAtEnd }.
   */
  function samplePath(sections, ends = {}) {
    const startTangentDeg = ends.startTangentDeg === undefined ? null : ends.startTangentDeg;
    const movingAtEnd = !!ends.movingAtEnd;
    const waypoints = [];
    const segmentSection = [];
    const stopAt = [];
    const sectionStartWaypoint = [];
    for (let k = 0; k < sections.length; ++k) {
      const section = sections[k];
      if (!section.waypoints || section.waypoints.length === 0) return pathFailure(`section ${k} has no waypoints`);
      sectionStartWaypoint.push(waypoints.length === 0 ? 0 : waypoints.length - 1);
      for (const waypoint of section.waypoints) {
        if (waypoints.length > 0) segmentSection.push(k);
        waypoints.push(waypoint);
        stopAt.push(false);
      }
      if (section.stopAtEnd) stopAt[stopAt.length - 1] = true;
    }
    if (waypoints.length < 2) return pathFailure('a path needs at least 2 waypoints');
    if (!hasHeading(waypoints[0])) return pathFailure('the first waypoint needs a heading (where the robot starts)');
    for (let j = 0; j + 1 < waypoints.length; ++j) {
      const chord = Math.hypot(waypoints[j + 1].xIn - waypoints[j].xIn, waypoints[j + 1].yIn - waypoints[j].yIn);
      if (chord < kMinChordIn) return pathFailure(`waypoints ${j} and ${j + 1} are at the same spot`);
    }
    stopAt[0] = true;
    stopAt[stopAt.length - 1] = true;

    const pieces = [];
    let start = 0;
    for (let j = 1; j < waypoints.length; ++j) {
      if (!stopAt[j]) continue;
      const piece = { t: [], x: [], y: [], mx: null, my: null, firstWaypoint: start };
      let t = 0.0;
      for (let i = start; i <= j; ++i) {
        if (i > start) t += Math.hypot(waypoints[i].xIn - waypoints[i - 1].xIn, waypoints[i].yIn - waypoints[i - 1].yIn);
        piece.t.push(t);
        piece.x.push(waypoints[i].xIn);
        piece.y.push(waypoints[i].yIn);
      }
      const clamp = start === 0 && startTangentDeg !== null;
      piece.mx = secondDerivatives(piece.t, piece.x, clamp ? Math.sin(startTangentDeg / kRadToDeg) : null);
      piece.my = secondDerivatives(piece.t, piece.y, clamp ? Math.cos(startTangentDeg / kRadToDeg) : null);
      pieces.push(piece);
      start = j;
    }

    const tables = [];
    const waypointS = new Array(waypoints.length).fill(0.0);
    let pathS = 0.0;
    for (const piece of pieces) {
      const table = { t: [0.0], s: [pathS], segment: [0] };
      let last = evaluate(piece, 0, 0.0);
      for (let i = 0; i + 1 < piece.t.length; ++i) {
        const h = piece.t[i + 1] - piece.t[i];
        const steps = Math.max(8, Math.ceil(h / kArcStepIn));
        for (let k = 1; k <= steps; ++k) {
          const t = k === steps ? piece.t[i + 1] : piece.t[i] + h * k / steps;
          const point = evaluate(piece, i, t);
          pathS += Math.hypot(point.x - last.x, point.y - last.y);
          last = point;
          table.t.push(t);
          table.s.push(pathS);
          table.segment.push(i);
        }
        waypointS[piece.firstWaypoint + i + 1] = pathS;
      }
      tables.push(table);
    }

    const keys = [];
    for (let j = 0; j < waypoints.length; ++j) {
      if (!hasHeading(waypoints[j])) continue;
      const heading = waypoints[j].headingDeg;
      keys.push({
        sIn: waypointS[j],
        headingDeg: keys.length === 0 ? heading
          : keys[keys.length - 1].headingDeg + L.wrapDegrees180(heading - keys[keys.length - 1].headingDeg),
      });
    }

    const result = { ok: true, error: '', samples: [], lengthIn: pathS, sectionStartIn: [] };
    for (const waypoint of sectionStartWaypoint) result.sectionStartIn.push(waypointS[waypoint]);
    for (let p = 0; p < pieces.length; ++p) {
      const piece = pieces[p];
      const table = tables[p];
      const s0 = table.s[0];
      const length = table.s[table.s.length - 1] - s0;
      const count = Math.max(1, Math.ceil(length / kTrajectorySampleSpacingIn - 1e-9));
      let fine = 1;
      for (let k = 0; k <= count; ++k) {
        const sIn = k === count ? table.s[table.s.length - 1] : s0 + length * k / count;
        while (fine + 1 < table.s.length && table.s[fine] < sIn) ++fine;
        const ds = table.s[fine] - table.s[fine - 1];
        const f = ds > 0.0 ? Math.min(1.0, Math.max(0.0, (sIn - table.s[fine - 1]) / ds)) : 0.0;
        const t = table.t[fine - 1] + f * (table.t[fine] - table.t[fine - 1]);
        const segment = table.segment[fine];
        const point = evaluate(piece, segment, t);
        const speed = Math.hypot(point.dx, point.dy);
        const sample = {
          sIn,
          xIn: point.x,
          yIn: point.y,
          tangentDeg: Math.atan2(point.dx, point.dy) * kRadToDeg,
          curvatureRadPerIn: speed > 1e-9 ? (point.dy * point.ddx - point.dx * point.ddy) / (speed * speed * speed) : 0.0,
          headingDeg: 0.0,
          headingRateDegPerIn: 0.0,
          headingAccelDegPerIn2: 0.0,
          section: segmentSection[piece.firstWaypoint + segment],
          stop: (k === 0 && !(p === 0 && startTangentDeg !== null)) ||
            (k === count && !(p + 1 === pieces.length && movingAtEnd)),
        };
        headingAt(keys, sIn, sample);
        result.samples.push(sample);
      }
    }
    return result;
  }

  // --- holonomic_feedforward.cpp --------------------------------------------------------------

  function fraction(model, value) {
    const top = modelValid(model) ? modelMaxVelocity(model, 12.0) : 0.0;
    return top > 0.0 ? value / top : 0.0;
  }

  function frictionVolts(models, direction) {
    const f = fraction(models.forward, direction.forward);
    const s = fraction(models.strafe, direction.strafe);
    const t = fraction(models.turn, direction.turn);
    const length = Math.sqrt(f * f + s * s + t * t);
    if (!(length > 0.0)) return { forward: 0, strafe: 0, turn: 0 };
    return {
      forward: models.forward.kS * f / length,
      strafe: models.strafe.kS * s / length,
      turn: models.turn.kS * t / length,
    };
  }

  function holonomicFeedforward(models, velocity, acceleration) {
    const moving = Math.sqrt(Math.pow(fraction(models.forward, velocity.forward), 2) +
      Math.pow(fraction(models.strafe, velocity.strafe), 2) + Math.pow(fraction(models.turn, velocity.turn), 2));
    const friction = frictionVolts(models, moving > kMovingFraction ? velocity : acceleration);
    return {
      forward: friction.forward + models.forward.kV * velocity.forward + models.forward.kA * acceleration.forward,
      strafe: friction.strafe + models.strafe.kV * velocity.strafe + models.strafe.kA * acceleration.strafe,
      turn: friction.turn + models.turn.kV * velocity.turn + models.turn.kA * acceleration.turn,
    };
  }

  function rotateAxisMotion(motion, headingOffsetDeg) {
    const c = Math.cos(headingOffsetDeg * kDegToRad);
    const s = Math.sin(headingOffsetDeg * kDegToRad);
    return {
      forward: motion.forward * c + motion.strafe * s,
      strafe: motion.strafe * c - motion.forward * s,
      turn: motion.turn,
    };
  }

  function trajectoryFeedforward(models, sample, headingOffsetDeg = 0.0) {
    const direction = rotateAxisMotion({ forward: sample.forwardPerIn, strafe: sample.strafePerIn,
      turn: sample.turnDegPerIn }, headingOffsetDeg);
    const velocity = rotateAxisMotion({ forward: sample.forwardInPerS, strafe: sample.strafeInPerS,
      turn: sample.omegaDegPerS }, headingOffsetDeg);
    const acceleration = rotateAxisMotion({ forward: sample.forwardAccelInPerS2, strafe: sample.strafeAccelInPerS2,
      turn: sample.turnAccelDegPerS2 }, headingOffsetDeg);
    const friction = frictionVolts(models, direction);
    return {
      forward: friction.forward + models.forward.kV * velocity.forward + models.forward.kA * acceleration.forward,
      strafe: friction.strafe + models.strafe.kV * velocity.strafe + models.strafe.kA * acceleration.strafe,
      turn: friction.turn + models.turn.kV * velocity.turn + models.turn.kA * acceleration.turn,
    };
  }

  // --- Trajectory (trajectory.cpp) ------------------------------------------------------------

  const SAMPLE_FIELDS = ['sIn', 'xIn', 'yIn', 'headingDeg', 'speedInPerS', 'accelInPerS2', 'vxInPerS',
    'vyInPerS', 'omegaDegPerS', 'forwardPerIn', 'strafePerIn', 'turnDegPerIn', 'forwardInPerS', 'strafeInPerS',
    'forwardAccelInPerS2', 'strafeAccelInPerS2', 'turnAccelDegPerS2'];

  function emptySample() {
    const s = { tS: 0.0, section: 0, stop: false };
    for (const f of SAMPLE_FIELDS) s[f] = 0.0;
    return s;
  }

  class Trajectory {
    constructor(samples = [], sectionStarts = []) {
      this.samples = samples;
      this.sectionStarts = sectionStarts.length ? sectionStarts : [{ tS: 0.0, sIn: 0.0 }];
    }

    empty() {
      return this.samples.length === 0;
    }

    durationS() {
      return this.samples.length ? this.samples[this.samples.length - 1].tS : 0.0;
    }

    lengthIn() {
      return this.samples.length ? this.samples[this.samples.length - 1].sIn : 0.0;
    }

    sectionCount() {
      return this.samples.length ? this.sectionStarts.length : 0;
    }

    sectionStart(section) {
      if (section < this.sectionStarts.length) return this.sectionStarts[section];
      return { tS: this.durationS(), sIn: this.lengthIn() };
    }

    sectionAt(sIn) {
      let section = 0;
      while (section + 1 < this.sectionStarts.length && this.sectionStarts[section + 1].sIn <= sIn) ++section;
      return section;
    }

    sample(tS) {
      const s = this.samples;
      if (s.length === 0) return emptySample();
      if (!(tS > s[0].tS)) return Object.assign({}, s[0]);
      if (tS >= s[s.length - 1].tS) return Object.assign({}, s[s.length - 1]);
      // upper_bound: the first sample after tS
      let lo = 0;
      let hi = s.length;
      while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (tS < s[mid].tS) hi = mid;
        else lo = mid + 1;
      }
      const b = s[lo];
      const a = s[lo - 1];
      const span = b.tS - a.tS;
      const f = span > 0.0 ? (tS - a.tS) / span : 1.0;
      const out = { tS, section: a.section, stop: false };
      for (const field of SAMPLE_FIELDS) out[field] = a[field] + (b[field] - a[field]) * f;
      return out;
    }
  }

  // --- generateTrajectory() (trajectory_generator.cpp) ----------------------------------------

  function sumOfSizes(p, q, a) {
    return Math.abs(p[0] + q[0] * a) + Math.abs(p[1] + q[1] * a) + Math.abs(p[2] + q[2] * a);
  }

  /** Returns { ok, lo, hi }. */
  function voltageAccelRange(p, q, budgetVolts) {
    const breaks = [];
    let slope = 0.0;
    for (let i = 0; i < 3; ++i) {
      if (Math.abs(q[i]) <= kTiny) continue;
      breaks.push(-p[i] / q[i]);
      slope += Math.abs(q[i]);
    }
    if (breaks.length === 0) return { ok: sumOfSizes(p, q, 0.0) <= budgetVolts, lo: -Infinity, hi: Infinity };
    breaks.sort((x, y) => x - y);
    let best = 0;
    for (let k = 1; k < breaks.length; ++k) {
      if (sumOfSizes(p, q, breaks[k]) < sumOfSizes(p, q, breaks[best])) best = k;
    }
    const lowest = sumOfSizes(p, q, breaks[best]);
    if (lowest > budgetVolts) return { ok: false, lo: 0, hi: 0 };

    let a = breaks[best];
    let f = lowest;
    let hi = Infinity;
    for (let k = best + 1; k < breaks.length && hi === Infinity; ++k) {
      const fk = sumOfSizes(p, q, breaks[k]);
      if (fk > budgetVolts) hi = a + (budgetVolts - f) * (breaks[k] - a) / (fk - f);
      else {
        a = breaks[k];
        f = fk;
      }
    }
    if (hi === Infinity) hi = a + (budgetVolts - f) / slope;

    a = breaks[best];
    f = lowest;
    let lo = -Infinity;
    for (let k = best - 1; k >= 0 && lo === -Infinity; --k) {
      const fk = sumOfSizes(p, q, breaks[k]);
      if (fk > budgetVolts) lo = a - (budgetVolts - f) * (a - breaks[k]) / (fk - f);
      else {
        a = breaks[k];
        f = fk;
      }
    }
    if (lo === -Infinity) lo = a - (budgetVolts - f) / slope;
    return { ok: true, lo, hi };
  }

  const AXES = ['forward', 'strafe', 'turn'];

  /** Returns { ok, lo, hi }. */
  function accelRange(point, models, v) {
    const p = [0, 0, 0];
    const q = [0, 0, 0];
    for (let i = 0; i < 3; ++i) {
      const model = models[AXES[i]];
      p[i] = point.friction[i] + model.kV * v * point.g[i] + model.kA * v * v * point.h[i];
      q[i] = model.kA * point.g[i];
    }
    const range = voltageAccelRange(p, q, point.budgetVolts);
    if (!range.ok) return range;
    let { lo, hi } = range;
    for (const constraints of [point.constraints, point.nextConstraints]) {
      if (!constraints) continue;
      for (let i = 0; i < 3; ++i) {
        const cap = constraints[AXES[i]].maxAcceleration;
        if (!(cap > 0.0)) continue;
        const turning = v * v * point.h[i];
        if (Math.abs(point.g[i]) > kTiny) {
          const a1 = (-cap - turning) / point.g[i];
          const a2 = (cap - turning) / point.g[i];
          lo = Math.max(lo, Math.min(a1, a2));
          hi = Math.min(hi, Math.max(a1, a2));
        } else if (Math.abs(turning) > cap) {
          return { ok: false, lo, hi };
        }
      }
    }
    return { ok: lo <= hi, lo, hi };
  }

  function atDistance(what, sIn) {
    return `${what} at ${sIn.toFixed(1)}in along the path`;
  }

  function failure(error) {
    return { ok: false, error, trajectory: new Trajectory() };
  }

  // a trajectory's direction of travel at a sample, on the field, in degrees clockwise from +y
  function directionOfTravelDeg(sample) {
    const heading = sample.headingDeg * kDegToRad;
    const ux = sample.forwardPerIn * Math.sin(heading) + sample.strafePerIn * Math.cos(heading);
    const uy = sample.forwardPerIn * Math.cos(heading) - sample.strafePerIn * Math.sin(heading);
    if (Math.hypot(ux, uy) > kTiny) return Math.atan2(ux, uy) / kDegToRad;
    return Math.atan2(sample.vxInPerS, sample.vyInPerS) / kDegToRad;
  }

  function aboveLimit(speed, limit) {
    return speed > limit + 1e-6 * Math.max(1.0, limit);
  }

  /**
   * generateTrajectory(sections, models, options), or
   * generateTrajectory(waypoints, constraints, models) for one section.
   * options (TrajectoryOptions): { from: a Trajectory to carry on from,
   * endSpeedInPerS }.
   */
  function generateTrajectory(a, b, c) {
    let sections;
    let models;
    let options = {};
    const isSections = a.length === 0 || (a[0] && Array.isArray(a[0].waypoints));
    if (!isSections) {
      sections = [trajectorySection({ waypoints: a, constraints: b })];
      models = c;
    } else {
      sections = a.map(trajectorySection);
      models = b;
      options = c || {};
    }
    const names = { forward: 'Fwd', strafe: 'Strafe', turn: 'Turn' };
    for (const axis of AXES) {
      if (!modelValid(models[axis])) {
        return failure(`the ${names[axis]} model isn't measured (kV and kA must be above 0): run Auto-Tune`);
      }
    }

    const ends = { startTangentDeg: null, movingAtEnd: options.endSpeedInPerS > 0.0 };
    const endSpeedInPerS = options.endSpeedInPerS || 0.0;
    let startSpeed = 0.0;
    if (options.from) {
      if (options.from.empty()) return failure('the trajectory to continue from is empty');
      const end = options.from.samples[options.from.samples.length - 1];
      if (sections.length === 0) sections.push(trajectorySection({}));
      sections[0] = Object.assign({}, sections[0], {
        waypoints: [{ xIn: end.xIn, yIn: end.yIn, headingDeg: end.headingDeg }, ...sections[0].waypoints],
      });
      startSpeed = Math.max(0.0, end.speedInPerS);
      if (startSpeed > 0.0) ends.startTangentDeg = directionOfTravelDeg(end);
    }

    const path = samplePath(sections, ends);
    if (!path.ok) return failure(path.error);
    const samples = path.samples;
    const n = samples.length;

    const points = new Array(n);
    for (let k = 0; k < n; ++k) {
      const sample = samples[k];
      const phi = (sample.tangentDeg - sample.headingDeg) * kDegToRad;
      const dphi = sample.curvatureRadPerIn - sample.headingRateDegPerIn * kDegToRad;
      const g = [Math.cos(phi), Math.sin(phi), sample.headingRateDegPerIn];
      const h = [-Math.sin(phi) * dphi, Math.cos(phi) * dphi, sample.headingAccelDegPerIn2];
      const friction = frictionVolts(models, { forward: g[0], strafe: g[1], turn: g[2] });
      const point = {
        g, h,
        friction: [friction.forward, friction.strafe, friction.turn],
        constraints: sections[sample.section].constraints,
        nextConstraints: null,
        budgetVolts: 0,
      };
      point.budgetVolts = Math.min(point.constraints.voltageBudget, kMaxVolts);
      if (k + 1 < n && samples[k + 1].section !== sample.section) {
        const next = samples[k + 1].section;
        if (Math.abs(sample.sIn - path.sectionStartIn[next]) < 1e-9) {
          point.nextConstraints = sections[next].constraints;
          point.budgetVolts = Math.min(point.budgetVolts, Math.min(point.nextConstraints.voltageBudget, kMaxVolts));
        }
      }
      points[k] = point;
    }

    const cap = new Array(n).fill(0.0);
    for (let k = 0; k < n; ++k) {
      const point = points[k];
      if (!accelRange(point, models, 0.0).ok) {
        return failure(atDistance('the voltage budget is below what friction needs', samples[k].sIn));
      }
      if (samples[k].stop) continue;
      let upper = Infinity;
      for (const constraints of [point.constraints, point.nextConstraints]) {
        if (!constraints) continue;
        for (let i = 0; i < 3; ++i) {
          if (Math.abs(point.g[i]) <= kTiny) continue;
          const limit = constraints[AXES[i]].maxVelocity;
          const top = limit > 0.0 ? limit : modelMaxVelocity(models[AXES[i]], point.budgetVolts);
          upper = Math.min(upper, top / Math.abs(point.g[i]));
        }
      }
      if (accelRange(point, models, upper).ok) {
        cap[k] = upper;
        continue;
      }
      let feasible = 0.0;
      let infeasible = upper;
      for (let step = 0; step < kBisectionSteps; ++step) {
        const middle = 0.5 * (feasible + infeasible);
        if (accelRange(point, models, middle).ok) feasible = middle;
        else infeasible = middle;
      }
      cap[k] = feasible;
    }
    if (!samples[n - 1].stop) cap[n - 1] = Math.min(cap[n - 1], endSpeedInPerS);
    if (aboveLimit(startSpeed, cap[0])) {
      return failure(`can't carry on at ${startSpeed.toFixed(1)}in/s: at most ${cap[0].toFixed(1)}in/s fits where ` +
        'this path starts. Plan the trajectory it continues from with a lower endSpeedInPerS');
    }

    const speed = new Array(n).fill(0.0);
    speed[0] = startSpeed;
    for (let k = 0; k + 1 < n; ++k) {
      const ds = samples[k + 1].sIn - samples[k].sIn;
      if (ds <= 0.0) continue;
      const startRange = accelRange(points[k], models, speed[k]);
      const hiStart = startRange.ok ? startRange.hi : 0.0;
      if (speed[k] <= 0.0 && hiStart <= 0.0) {
        return failure(atDistance('the voltage budget can\'t start the robot moving', samples[k].sIn));
      }
      const fits = (end) => {
        const endRange = accelRange(points[k + 1], models, end);
        if (!endRange.ok) return false;
        const required = (end * end - speed[k] * speed[k]) / (2.0 * ds);
        return required <= Math.min(hiStart, 0.5 * (hiStart + endRange.hi));
      };
      const reach = Math.sqrt(Math.max(0.0, speed[k] * speed[k] + 2.0 * hiStart * ds));
      const upper = Math.min(cap[k + 1], reach);
      if (fits(upper)) {
        speed[k + 1] = upper;
        continue;
      }
      let good = 0.0;
      let bad = upper;
      for (let step = 0; step < kBisectionSteps; ++step) {
        const middle = 0.5 * (good + bad);
        if (fits(middle)) good = middle;
        else bad = middle;
      }
      speed[k + 1] = good;
    }
    for (let k = n - 1; k >= 1; --k) {
      const ds = samples[k].sIn - samples[k - 1].sIn;
      if (ds <= 0.0) continue;
      const range = accelRange(points[k], models, speed[k]);
      const lo = range.ok ? range.lo : 0.0;
      const squared = speed[k] * speed[k] - 2.0 * lo * ds;
      speed[k - 1] = Math.min(speed[k - 1], Math.sqrt(Math.max(0.0, squared)));
    }
    if (aboveLimit(startSpeed, speed[0])) {
      return failure(`can't brake from ${startSpeed.toFixed(1)}in/s in time for this path (at most ` +
        `${speed[0].toFixed(1)}in/s): plan the trajectory it continues from with a lower endSpeedInPerS`);
    }
    speed[0] = startSpeed;

    const stepAccel = new Array(n).fill(0.0);
    const time = new Array(n).fill(0.0);
    for (let k = 0; k + 1 < n; ++k) {
      const ds = samples[k + 1].sIn - samples[k].sIn;
      let dt = 0.0;
      if (ds > 0.0) {
        const sum = speed[k] + speed[k + 1];
        if (!(sum > 0.0)) return failure(atDistance('the plan stalls', samples[k].sIn));
        dt = 2.0 * ds / sum;
        stepAccel[k] = (speed[k + 1] * speed[k + 1] - speed[k] * speed[k]) / (2.0 * ds);
      }
      time[k + 1] = time[k] + dt;
    }

    const starts = [];
    for (const startIn of path.sectionStartIn) {
      let k = 0;
      while (k + 1 < n && samples[k + 1].sIn < startIn) ++k;
      let tS = time[k];
      if (k + 1 < n && samples[k + 1].sIn > samples[k].sIn) {
        const f = (startIn - samples[k].sIn) / (samples[k + 1].sIn - samples[k].sIn);
        tS = time[k] + Math.min(1.0, Math.max(0.0, f)) * (time[k + 1] - time[k]);
      }
      starts.push({ tS, sIn: startIn });
    }

    const out = new Array(n);
    for (let k = 0; k < n; ++k) {
      const sample = samples[k];
      const point = points[k];
      const stepAfter = k + 1 < n && samples[k + 1].sIn > sample.sIn;
      const v = speed[k];
      const a = stepAfter || k === 0 ? stepAccel[k] : stepAccel[k - 1];
      const tangent = sample.tangentDeg * kDegToRad;
      let section = 0;
      while (section + 1 < starts.length && starts[section + 1].sIn <= sample.sIn) ++section;
      out[k] = {
        tS: time[k],
        sIn: sample.sIn,
        section,
        stop: sample.stop,
        xIn: sample.xIn,
        yIn: sample.yIn,
        headingDeg: sample.headingDeg,
        speedInPerS: v,
        accelInPerS2: a,
        vxInPerS: v * Math.sin(tangent),
        vyInPerS: v * Math.cos(tangent),
        omegaDegPerS: v * point.g[2],
        forwardPerIn: point.g[0],
        strafePerIn: point.g[1],
        turnDegPerIn: point.g[2],
        forwardInPerS: v * point.g[0],
        strafeInPerS: v * point.g[1],
        forwardAccelInPerS2: a * point.g[0] + v * v * point.h[0],
        strafeAccelInPerS2: a * point.g[1] + v * v * point.h[1],
        turnAccelDegPerS2: a * point.g[2] + v * v * point.h[2],
      };
    }
    return { ok: true, error: '', trajectory: new Trajectory(out, starts) };
  }

  // --- holonomic_tracking.cpp: the follower's math -------------------------------------------
  // (TrajectoryFollower itself is in robot.js, beside the drivetrain and ExitTracker)

  function toLocalFrame(dxIn, dyIn, headingDeg) {
    const headingRad = headingDeg * kDegToRad;
    return {
      forwardIn: dxIn * Math.sin(headingRad) + dyIn * Math.cos(headingRad),
      lateralIn: dxIn * Math.cos(headingRad) - dyIn * Math.sin(headingRad),
    };
  }

  function trackingErrors(reference, pose, velocity) {
    const dxIn = reference.xIn - pose.xIn;
    const dyIn = reference.yIn - pose.yIn;
    const position = toLocalFrame(dxIn, dyIn, pose.headingDeg);
    const rate = toLocalFrame(reference.vxInPerS - velocity.xInPerS, reference.vyInPerS - velocity.yInPerS,
      pose.headingDeg);
    const heading = reference.headingDeg * kDegToRad;
    const ux = reference.forwardPerIn * Math.sin(heading) + reference.strafePerIn * Math.cos(heading);
    const uy = reference.forwardPerIn * Math.cos(heading) - reference.strafePerIn * Math.sin(heading);
    return {
      forwardIn: position.forwardIn,
      strafeIn: position.lateralIn,
      headingDeg: L.wrapDegrees180(reference.headingDeg - pose.headingDeg),
      forwardRateInPerS: rate.forwardIn,
      strafeRateInPerS: rate.lateralIn,
      headingRateDegPerS: reference.omegaDegPerS - velocity.headingDegPerS,
      alongTrackIn: dxIn * ux + dyIn * uy,
      crossTrackIn: -(dxIn * uy - dyIn * ux),
    };
  }

  function advanceTrajectoryClock(tS, dtS, alongTrackIn, maxLagIn) {
    if (maxLagIn > 0.0 && alongTrackIn > maxLagIn) return { tS, paused: true };
    return { tS: tS + (dtS > 0.0 ? dtS : 0.0), paused: false };
  }

  /** The reference with its motion taken out: where the plan ends, at rest. */
  function atRest(sample) {
    return Object.assign({}, sample, {
      speedInPerS: 0, accelInPerS2: 0, vxInPerS: 0, vyInPerS: 0, omegaDegPerS: 0, forwardInPerS: 0,
      strafeInPerS: 0, forwardAccelInPerS2: 0, strafeAccelInPerS2: 0, turnAccelDegPerS2: 0, stop: true,
    });
  }

  // a chain's junctions (holonomic_tracking.cpp): how close the start of a trajectory that
  // carries on from another must be to that one's end
  const kJoinPositionIn = 0.25;
  const kJoinHeadingDeg = 1.0;
  const kJoinSpeedInPerS = 0.5;

  function endsMoving(trajectory) {
    const end = trajectory.samples[trajectory.samples.length - 1];
    return !end.stop && end.speedInPerS > 0.0;
  }

  function continuesFrom(next, previous) {
    const a = previous.samples[previous.samples.length - 1];
    const b = next.samples[0];
    return Math.hypot(b.xIn - a.xIn, b.yIn - a.yIn) <= kJoinPositionIn &&
      Math.abs(L.wrapDegrees180(b.headingDeg - a.headingDeg)) <= kJoinHeadingDeg &&
      Math.abs(b.speedInPerS - a.speedInPerS) <= kJoinSpeedInPerS;
  }

  return {
    endsMoving,
    continuesFrom,
    kTrajectorySampleSpacingIn,
    trajectoryConstraints,
    trajectorySection,
    followConfig,
    trackingErrors,
    advanceTrajectoryClock,
    atRest,
    samplePath,
    frictionVolts,
    holonomicFeedforward,
    rotateAxisMotion,
    trajectoryFeedforward,
    Trajectory,
    voltageAccelRange,
    generateTrajectory,
  };
});
