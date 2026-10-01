/*
 * SapphireLib simulator: mcl.js
 *
 * The robot's localization math, ported line for line so the simulator runs
 * exactly what the robot runs:
 *
 *   the random numbers     include/sapphirelib/util/random.hpp
 *   lookup tables          include/sapphirelib/util/lookup_table.hpp
 *   the field map          src/sapphirelib/localization/field_map.cpp
 *   the sensor model       src/sapphirelib/localization/sensor_model.cpp
 *   the particle filter    src/sapphirelib/localization/particle_filter.cpp
 *   odometry's math        src/sapphirelib/odom/odometry_math.cpp
 *   angle wrapping         src/sapphirelib/util/angle.cpp, sensors/imu_scale_math.cpp
 *
 * Each port keeps the C++'s order of operations, so the same seed and inputs
 * give the same particles (only libm's exp, log, sin and cos may differ in
 * their last bit). That includes the C++'s speedups: predict() draws its noise
 * from the ziggurat (Rng.fastGaussian()), and weigh() casts with a
 * ParallelRayCaster and scores with a ReadingScorer's lookup table.
 * test/mcl.test.js holds it to the golden values in
 * tests/util/random_test.cpp and tests/localization/*_test.cpp. Change one
 * side, change the other.
 *
 * On top of the ports, for the visualizer only: every particle carries the
 * score its last weigh() gave it, and whether recovery just scattered it.
 * Neither changes a single number the filter produces.
 *
 * Plain script: window.SIM.mcl in a browser, require('./mcl.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.mcl = factory();
  }
})(function () {
  'use strict';

  const kPi = 3.14159265358979323846;
  const kDegToRad = kPi / 180.0;
  const kInvSqrtTwoPi = 0.39894228040143267794;
  const kMmPerIn = 25.4;
  const kVrcFieldSizeIn = 140.5;

  // --- Angles (angle.cpp, imu_scale_math.cpp) ---------------------------------

  /** C's fmod: the result takes the sign of the dividend, like JavaScript's %. */
  function wrapDegrees180(degrees) {
    let wrapped = degrees % 360.0;
    if (wrapped <= -180.0) wrapped += 360.0;
    if (wrapped > 180.0) wrapped -= 360.0;
    return wrapped;
  }

  function wrapDegrees360(degrees) {
    let wrapped = degrees % 360.0;
    if (wrapped < 0.0) wrapped += 360.0;
    return wrapped;
  }

  // --- Random numbers (util/random.hpp) ---------------------------------------

  // the ziggurat behind fastGaussian(): ZigguratTables
  const kZigTailStart = 3.442619855899;
  const kZigLayerArea = 9.91256303526217e-3;
  const kZigScale = 16777216.0;
  let ziggurat = null;

  /** ZigguratTables: built once, on the first call. */
  function zigguratTables() {
    if (ziggurat) return ziggurat;
    const k = new Uint32Array(128);
    const w = new Float64Array(128);
    const f = new Float64Array(128);
    let outer = kZigTailStart;
    let previous = outer;
    const q = kZigLayerArea / Math.exp(-0.5 * outer * outer);
    // static_cast<std::uint32_t> truncates
    k[0] = Math.trunc((outer / q) * kZigScale);
    k[1] = 0;
    w[0] = q / kZigScale;
    w[127] = outer / kZigScale;
    f[0] = 1.0;
    f[127] = Math.exp(-0.5 * outer * outer);
    for (let i = 126; i >= 1; --i) {
      outer = Math.sqrt(-2.0 * Math.log(kZigLayerArea / outer + Math.exp(-0.5 * outer * outer)));
      k[i + 1] = Math.trunc((outer / previous) * kZigScale);
      previous = outer;
      f[i] = Math.exp(-0.5 * outer * outer);
      w[i] = outer / kZigScale;
    }
    ziggurat = { k, w, f };
    return ziggurat;
  }

  /** xoshiro128**, seeded through splitmix32. All arithmetic is uint32. */
  class Rng {
    constructor(seed = 1) {
      this.state = new Uint32Array(4);
      this.reseed(seed);
    }

    reseed(seed) {
      let z = seed >>> 0;
      for (let i = 0; i < 4; ++i) {
        z = (z + 0x9e3779b9) >>> 0;
        let mixed = z;
        mixed = Math.imul(mixed ^ (mixed >>> 16), 0x85ebca6b) >>> 0;
        mixed = Math.imul(mixed ^ (mixed >>> 13), 0xc2b2ae35) >>> 0;
        this.state[i] = (mixed ^ (mixed >>> 16)) >>> 0;
      }
    }

    next() {
      const s = this.state;
      const result = Math.imul(rotl(Math.imul(s[1], 5) >>> 0, 7), 9) >>> 0;
      const t = (s[1] << 9) >>> 0;
      s[2] ^= s[0];
      s[3] ^= s[1];
      s[1] ^= s[2];
      s[0] ^= s[3];
      s[2] ^= t;
      s[3] = rotl(s[3], 11);
      return result;
    }

    uniform() {
      return this.next() * (1.0 / 4294967296.0);
    }

    gaussian() {
      let u = 0;
      let v = 0;
      let s = 0;
      do {
        u = 2.0 * this.uniform() - 1.0;
        v = 2.0 * this.uniform() - 1.0;
        s = u * u + v * v;
      } while (s >= 1.0 || s === 0.0);
      return u * Math.sqrt((-2.0 * Math.log(s)) / s);
    }

    /** The ziggurat: gaussian()'s distribution, a different sequence, no log() or sqrt() 99% of the time. */
    fastGaussian() {
      const z = zigguratTables();
      for (;;) {
        // independent bits for the layer (7), the sign (1) and the position across it (24)
        const bits = this.next();
        const layer = bits & 127;
        const negative = (bits & 128) !== 0;
        const across = bits >>> 8;
        let x = across * z.w[layer];
        if (across < z.k[layer]) return negative ? -x : x;
        if (layer === 0) {
          let tailX = 0.0;
          let tailY = 0.0;
          do {
            tailX = -Math.log(1.0 - this.uniform()) / kZigTailStart;
            tailY = -Math.log(1.0 - this.uniform());
          } while (tailY + tailY < tailX * tailX);
          x = kZigTailStart + tailX;
          return negative ? -x : x;
        }
        if (z.f[layer] + this.uniform() * (z.f[layer - 1] - z.f[layer]) < Math.exp(-0.5 * x * x)) {
          return negative ? -x : x;
        }
      }
    }
  }

  // --- Lookup tables (util/lookup_table.hpp) -----------------------------------

  /** A function sampled at even steps, read back along a straight line between samples. */
  class LookupTable {
    constructor(minX, maxX, stepsPerUnit, fn) {
      this.minX = minX;
      this.maxX = maxX;
      this.stepsPerUnit = stepsPerUnit;
      this.steps = Math.trunc((maxX - minX) * stepsPerUnit);
      this.values = new Float64Array(this.steps + 1);
      for (let i = 0; i <= this.steps; ++i) this.values[i] = fn(minX + i / stepsPerUnit);
    }

    at(x) {
      const position = (x - this.minX) * this.stepsPerUnit;
      if (!(position > 0.0)) return this.values[0];
      if (position >= this.steps) return this.values[this.steps];
      const i = Math.trunc(position);
      const fraction = position - i;
      return this.values[i] + fraction * (this.values[i + 1] - this.values[i]);
    }
  }

  function rotl(x, k) {
    return ((x << k) | (x >>> (32 - k))) >>> 0;
  }

  // --- The field map (field_map.cpp) ------------------------------------------

  const kParallelEpsilon = 1e-12;

  class FieldMap {
    constructor(minXIn, minYIn, maxXIn, maxYIn) {
      this.minXIn = minXIn;
      this.minYIn = minYIn;
      this.maxXIn = maxXIn;
      this.maxYIn = maxYIn;
      this.segments = [
        { x1In: minXIn, y1In: minYIn, x2In: maxXIn, y2In: minYIn },
        { x1In: maxXIn, y1In: minYIn, x2In: maxXIn, y2In: maxYIn },
        { x1In: maxXIn, y1In: maxYIn, x2In: minXIn, y2In: maxYIn },
        { x1In: minXIn, y1In: maxYIn, x2In: minXIn, y2In: minYIn },
      ];
    }

    static centered(sizeIn = kVrcFieldSizeIn) {
      const half = sizeIn / 2.0;
      return new FieldMap(-half, -half, half, half);
    }

    clone() {
      const copy = new FieldMap(this.minXIn, this.minYIn, this.maxXIn, this.maxYIn);
      copy.segments = this.segments.map((s) => Object.assign({}, s));
      return copy;
    }

    addSegment(segment) {
      this.segments.push(Object.assign({}, segment));
    }

    addBox(minXIn, minYIn, maxXIn, maxYIn) {
      this.segments.push({ x1In: minXIn, y1In: minYIn, x2In: maxXIn, y2In: minYIn });
      this.segments.push({ x1In: maxXIn, y1In: minYIn, x2In: maxXIn, y2In: maxYIn });
      this.segments.push({ x1In: maxXIn, y1In: maxYIn, x2In: minXIn, y2In: maxYIn });
      this.segments.push({ x1In: minXIn, y1In: maxYIn, x2In: minXIn, y2In: minYIn });
    }

    castRayIn(xIn, yIn, dirX, dirY) {
      let nearest = Infinity;
      const segments = this.segments;
      for (let i = 0; i < segments.length; ++i) {
        const s = segments[i];
        const ex = s.x2In - s.x1In;
        const ey = s.y2In - s.y1In;
        const denom = dirX * ey - dirY * ex;
        if (Math.abs(denom) < kParallelEpsilon) continue;
        const wx = s.x1In - xIn;
        const wy = s.y1In - yIn;
        const t = (wx * ey - wy * ex) / denom;
        const u = (wx * dirY - wy * dirX) / denom;
        if (t >= 0.0 && u >= 0.0 && u <= 1.0 && t < nearest) nearest = t;
      }
      return nearest;
    }

    castRayAtHeadingIn(xIn, yIn, headingDeg) {
      const headingRad = headingDeg * kDegToRad;
      return this.castRayIn(xIn, yIn, Math.sin(headingRad), Math.cos(headingRad));
    }

    contains(xIn, yIn) {
      return xIn >= this.minXIn && xIn <= this.maxXIn && yIn >= this.minYIn && yIn <= this.maxYIn;
    }
  }

  /** castRayIn() for many rays pointing one way: no division per ray, two multiplies for the walls. */
  class ParallelRayCaster {
    constructor() {
      this.edges = [];
      this.perimeterEdges = 0;
      this.dirX = 0;
      this.dirY = 0;
    }

    aim(map, dirX, dirY) {
      this.dirX = dirX;
      this.dirY = dirY;
      this.minXIn = map.minXIn;
      this.minYIn = map.minYIn;
      this.maxXIn = map.maxXIn;
      this.maxYIn = map.maxYIn;
      const reachesX = Math.abs(dirX * (this.maxYIn - this.minYIn)) >= kParallelEpsilon;
      const reachesY = Math.abs(dirY * (this.maxXIn - this.minXIn)) >= kParallelEpsilon;
      this.wallXIn = dirX > 0.0 ? this.maxXIn : this.minXIn;
      this.wallYIn = dirY > 0.0 ? this.maxYIn : this.minYIn;
      this.inverseDirX = reachesX ? 1.0 / dirX : 0.0;
      this.inverseDirY = reachesY ? 1.0 / dirY : 0.0;
      this.edges.length = 0;
      this.perimeterEdges = 0;
      const segments = map.segments;
      for (let i = 0; i < segments.length; ++i) {
        const s = segments[i];
        const ex = s.x2In - s.x1In;
        const ey = s.y2In - s.y1In;
        const denom = dirX * ey - dirY * ex;
        if (Math.abs(denom) < kParallelEpsilon) continue;
        this.edges.push({ ex, ey, inverseDenominator: 1.0 / denom, crossT: s.x1In * ey - s.y1In * ex,
          crossU: s.x1In * dirY - s.y1In * dirX });
        if (i < 4) ++this.perimeterEdges;
      }
    }

    castIn(xIn, yIn) {
      let nearest = Infinity;
      let first = 0;
      if (xIn >= this.minXIn && xIn <= this.maxXIn && yIn >= this.minYIn && yIn <= this.maxYIn) {
        if (this.inverseDirX !== 0.0) nearest = (this.wallXIn - xIn) * this.inverseDirX;
        if (this.inverseDirY !== 0.0) nearest = Math.min(nearest, (this.wallYIn - yIn) * this.inverseDirY);
        first = this.perimeterEdges;
      }
      const startCrossDir = xIn * this.dirY - yIn * this.dirX;
      const edges = this.edges;
      for (let i = first; i < edges.length; ++i) {
        const e = edges[i];
        const t = (e.crossT - (xIn * e.ey - yIn * e.ex)) * e.inverseDenominator;
        const u = (e.crossU - startCrossDir) * e.inverseDenominator;
        if (t >= 0.0 && u >= 0.0 && u <= 1.0 && t < nearest) nearest = t;
      }
      return nearest;
    }
  }

  // --- The sensor model (sensor_model.cpp) ------------------------------------

  const kNoObjectMm = 9999;
  const kMinRangeMm = 20;
  const kConfidenceMinMm = 200;

  /** BeamModel defaults. */
  function beamModel(overrides = {}) {
    return Object.assign(
      { maxRangeIn: 78.0, minSigmaIn: 0.6, sigmaFraction: 0.05, outlierProbability: 0.1 },
      overrides,
    );
  }

  function mount(forwardIn = 0, rightIn = 0, facingDeg = 0) {
    return { forwardIn, rightIn, facingDeg };
  }

  function reading(distanceIn = 0, valid = false, extraSigmaIn = 0) {
    return { distanceIn, valid, extraSigmaIn };
  }

  function sensorRay(robotXIn, robotYIn, headingDeg, m) {
    const headingRad = headingDeg * kDegToRad;
    const sinHeading = Math.sin(headingRad);
    const cosHeading = Math.cos(headingRad);
    const facingRad = (headingDeg + m.facingDeg) * kDegToRad;
    return {
      xIn: robotXIn + m.forwardIn * sinHeading + m.rightIn * cosHeading,
      yIn: robotYIn + m.forwardIn * cosHeading - m.rightIn * sinHeading,
      dirX: Math.sin(facingRad),
      dirY: Math.cos(facingRad),
    };
  }

  function readingSigmaIn(beam, r) {
    const sensorSigma = Math.max(beam.minSigmaIn, beam.sigmaFraction * r.distanceIn);
    return Math.hypot(sensorSigma, r.extraSigmaIn);
  }

  function readingLogLikelihood(measuredIn, expectedIn, sigmaIn, beam) {
    const z = (measuredIn - expectedIn) / sigmaIn;
    const hit = (Math.exp(-0.5 * z * z) * kInvSqrtTwoPi) / sigmaIn;
    const density = (1.0 - beam.outlierProbability) * hit + beam.outlierProbability / beam.maxRangeIn;
    return Math.log(density);
  }

  // softplus(x) = log(1 + e^x) from -16 to 16, 64 samples to the unit: built by the first ReadingScorer
  let softplus = null;

  function softplusTable() {
    if (!softplus) softplus = new LookupTable(-16.0, 16.0, 64, (x) => Math.log1p(Math.exp(x)));
    return softplus;
  }

  /** readingLogLikelihood() for one reading against many expected distances, by lookup table; within 1e-5. */
  class ReadingScorer {
    constructor(measuredIn = 0, sigmaIn = Infinity, beam = null) {
      this.measuredIn = measuredIn;
      this.invSigma = beam ? 1.0 / sigmaIn : 0.0;
      this.logFloor = 0.0;
      this.logPeak = 0.0;
      this.noOutliers = true;
      this.softplus = null;
      if (!beam) return; // like the C++ default: everything scores 0
      this.softplus = softplusTable();
      const peak = ((1.0 - beam.outlierProbability) * kInvSqrtTwoPi) / sigmaIn;
      const floor = beam.outlierProbability / beam.maxRangeIn;
      this.noOutliers = !(floor > 0.0);
      this.logFloor = this.noOutliers ? 0.0 : Math.log(floor);
      this.logPeak = this.noOutliers ? Math.log(peak) : Math.log(peak) - this.logFloor;
    }

    logLikelihood(expectedIn) {
      const z = (this.measuredIn - expectedIn) * this.invSigma;
      const x = this.logPeak - 0.5 * z * z;
      if (this.noOutliers) return x;
      if (!(x > this.softplus.minX)) return this.logFloor;
      if (x >= this.softplus.maxX) return this.logFloor + x;
      return this.logFloor + this.softplus.at(x);
    }
  }

  function distanceReadingFromMm(millimeters, confidence, minConfidence, beam) {
    const r = reading();
    if (millimeters < kMinRangeMm || millimeters >= kNoObjectMm) return r;
    if (millimeters > kConfidenceMinMm && confidence < minConfidence) return r;
    r.distanceIn = millimeters / kMmPerIn;
    r.valid = r.distanceIn <= beam.maxRangeIn;
    return r;
  }

  function compensateLatency(r, closingSpeedInPerS, latencyS) {
    if (!r.valid) return r;
    const out = Object.assign({}, r);
    const shiftIn = closingSpeedInPerS * latencyS;
    out.distanceIn -= shiftIn;
    out.extraSigmaIn = Math.hypot(out.extraSigmaIn, 0.5 * shiftIn);
    if (out.distanceIn <= 0.0) out.valid = false;
    return out;
  }

  // --- The particle filter (particle_filter.cpp) ------------------------------

  const kOutsideLogPenalty = -30.0;

  // e^x for recovery's fit, which only takes x <= 0: below -32 a particle counts as 0
  let fitExp = null;

  function fitExpTable() {
    if (!fitExp) fitExp = new LookupTable(-32.0, 0.0, 64, (x) => Math.exp(x));
    return fitExp;
  }

  /** ParticleFilterConfig defaults; nested objects merge. */
  function filterConfig(overrides = {}) {
    const config = {
      particleCount: 300,
      motionNoise: { baseIn: 0.02, perInch: 0.05, perDegreeIn: 0.005 },
      beam: beamModel(),
      resampleThreshold: 0.5,
      recovery: { enabled: true, slowRate: 0.01, fastRate: 0.2, triggerRatio: 0.8, radiusIn: 18.0, maxFraction: 0.1 },
      seed: 1,
    };
    for (const key of Object.keys(overrides)) {
      const value = overrides[key];
      if (value && typeof value === 'object' && !Array.isArray(value)) {
        config[key] = Object.assign({}, config[key], value);
      } else {
        config[key] = value;
      }
    }
    return config;
  }

  class ParticleFilter {
    constructor(map, sensors, config = {}) {
      this.map = map;
      this.sensors = sensors.map((s) => Object.assign({}, s));
      this.config = filterConfig(config);
      this.config.particleCount = Math.max(1, this.config.particleCount | 0);
      this.rng = new Rng(this.config.seed);
      const count = this.config.particleCount;
      this.particles = [];
      for (let i = 0; i < count; ++i) this.particles.push(this.makeParticle(0, 0, 1.0 / count));
      this.resampled = [];
      for (let i = 0; i < count; ++i) this.resampled.push(this.makeParticle(0, 0, 0));
      // each weight as a log, kept between updates; unused while uniformWeights
      this.logWeights = new Float64Array(count);
      this.uniformWeights = true;
      this.logLikelihoods = new Float64Array(count);
      this.rays = this.sensors.map(() => ({ xIn: 0, yIn: 0, dirX: 0, dirY: 0 }));
      this.casters = this.sensors.map(() => new ParallelRayCaster());
      this.scorers = this.sensors.map(() => new ReadingScorer());
      this.usedSensors = [];
      this.slowAverage = 0;
      this.fastAverage = 0;
      this.averagesPrimed = false;
      this.lastRecovered = 0;
    }

    makeParticle(xIn, yIn, weight) {
      // score: how well the last weigh() explained this particle's readings,
      // relative to the best (1). For drawing only
      return { xIn, yIn, weight, score: 1, recovered: false };
    }

    reset(xIn, yIn, spreadIn) {
      const weight = 1.0 / this.particles.length;
      for (const p of this.particles) {
        p.xIn = xIn + spreadIn * this.rng.gaussian();
        p.yIn = yIn + spreadIn * this.rng.gaussian();
        p.weight = weight;
        p.score = 1;
        p.recovered = false;
      }
      this.uniformWeights = true;
      this.averagesPrimed = false;
      this.lastRecovered = 0;
    }

    resetUniform() {
      const weight = 1.0 / this.particles.length;
      const width = this.map.maxXIn - this.map.minXIn;
      const height = this.map.maxYIn - this.map.minYIn;
      for (const p of this.particles) {
        p.xIn = this.map.minXIn + this.rng.uniform() * width;
        p.yIn = this.map.minYIn + this.rng.uniform() * height;
        p.weight = weight;
        p.score = 1;
        p.recovered = false;
      }
      this.uniformWeights = true;
      this.averagesPrimed = false;
      this.lastRecovered = 0;
    }

    predict(dxIn, dyIn, turnedDeg) {
      const noise = this.config.motionNoise;
      const sigmaIn = noise.baseIn + noise.perInch * Math.hypot(dxIn, dyIn) +
        noise.perDegreeIn * Math.abs(turnedDeg);
      for (const p of this.particles) {
        p.xIn += dxIn + sigmaIn * this.rng.fastGaussian();
        p.yIn += dyIn + sigmaIn * this.rng.fastGaussian();
      }
    }

    weigh(headingDeg, readings) {
      const sensorCount = Math.min(readings.length, this.sensors.length);
      const beam = this.config.beam;
      const used = this.usedSensors;
      used.length = 0;
      let logPerfectFit = 0.0;
      for (let s = 0; s < sensorCount; ++s) {
        if (!readings[s].valid) continue;
        this.rays[s] = sensorRay(0.0, 0.0, headingDeg, this.sensors[s]);
        this.casters[s].aim(this.map, this.rays[s].dirX, this.rays[s].dirY);
        this.scorers[s] = new ReadingScorer(readings[s].distanceIn, readingSigmaIn(beam, readings[s]), beam);
        logPerfectFit += this.scorers[s].logLikelihood(readings[s].distanceIn);
        used.push(s);
      }
      const validCount = used.length;
      if (validCount === 0) return false;

      const particles = this.particles;
      const map = this.map;
      const uniformLogWeight = -Math.log(particles.length);
      let maxLogWeight = -Infinity;
      let maxLogLikelihood = -Infinity;
      for (let i = 0; i < particles.length; ++i) {
        const p = particles[i];
        let logLikelihood = map.contains(p.xIn, p.yIn) ? 0.0 : kOutsideLogPenalty;
        for (let k = 0; k < validCount; ++k) {
          const s = used[k];
          const ray = this.rays[s];
          logLikelihood += this.scorers[s].logLikelihood(this.casters[s].castIn(p.xIn + ray.xIn, p.yIn + ray.yIn));
        }
        this.logLikelihoods[i] = logLikelihood;
        this.logWeights[i] = (this.uniformWeights ? uniformLogWeight : this.logWeights[i]) + logLikelihood;
        maxLogWeight = Math.max(maxLogWeight, this.logWeights[i]);
        maxLogLikelihood = Math.max(maxLogLikelihood, logLikelihood);
      }

      const fitExpLookup = fitExpTable();
      const inverseValid = 1.0 / validCount;
      let sum = 0.0;
      let average = 0.0;
      for (let i = 0; i < particles.length; ++i) {
        particles[i].weight = Math.exp(this.logWeights[i] - maxLogWeight);
        sum += particles[i].weight;
        const fit = (this.logLikelihoods[i] - logPerfectFit) * inverseValid;
        if (fit > fitExpLookup.minX) average += fitExpLookup.at(fit);
      }
      average /= particles.length;

      const inverseSum = 1.0 / sum;
      const logSum = Math.log(sum);
      for (let i = 0; i < particles.length; ++i) {
        particles[i].weight *= inverseSum;
        this.logWeights[i] = this.logWeights[i] - maxLogWeight - logSum;
      }
      this.uniformWeights = false;

      // visualizer only: each particle's fit against the best
      for (let i = 0; i < particles.length; ++i) {
        particles[i].score = Math.exp(this.logLikelihoods[i] - maxLogLikelihood);
        particles[i].recovered = false;
      }
      this.latestFit = average;

      if (!this.averagesPrimed) {
        this.slowAverage = average;
        this.fastAverage = average;
        this.averagesPrimed = true;
      } else {
        this.slowAverage += this.config.recovery.slowRate * (average - this.slowAverage);
        this.fastAverage += this.config.recovery.fastRate * (average - this.fastAverage);
      }
      return true;
    }

    recoveryFraction() {
      const recovery = this.config.recovery;
      if (!recovery.enabled || !this.averagesPrimed || !(this.slowAverage > 0.0) || !(recovery.triggerRatio > 0.0)) {
        return 0.0;
      }
      const ratio = this.fastAverage / this.slowAverage;
      return Math.min(Math.max(1.0 - ratio / recovery.triggerRatio, 0.0), recovery.maxFraction);
    }

    /** RecoveryFit: the last weigh()'s fit, and recovery's fast and slow averages of it. */
    recoveryFit() {
      return { latest: this.latestFit || 0, fast: this.fastAverage, slow: this.slowAverage };
    }

    effectiveParticles() {
      let sumSquares = 0.0;
      for (const p of this.particles) sumSquares += p.weight * p.weight;
      return 1.0 / sumSquares;
    }

    resampleIfNeeded() {
      const particles = this.particles;
      const count = particles.length;
      let sumSquares = 0.0;
      for (const p of particles) sumSquares += p.weight * p.weight;
      const effective = 1.0 / sumSquares;

      const fraction = this.recoveryFraction();
      // static_cast<std::size_t> truncates toward zero
      const recovered = Math.floor(fraction * count);
      if (effective >= this.config.resampleThreshold * count && recovered === 0) {
        this.lastRecovered = 0;
        return false;
      }

      const kept = count - recovered;
      const out = this.resampled;
      if (kept > 0) {
        const step = 1.0 / kept;
        const start = this.rng.uniform() * step;
        let cumulative = particles[0].weight;
        let source = 0;
        for (let j = 0; j < kept; ++j) {
          const target = start + j * step;
          while (target > cumulative && source + 1 < count) {
            ++source;
            cumulative += particles[source].weight;
          }
          copyParticle(out[j], particles[source]);
        }
      }

      if (recovered > 0) {
        const center = this.estimate();
        const radius = this.config.recovery.radiusIn;
        const wholeField = !(radius > 0.0);
        const map = this.map;
        const minX = wholeField ? map.minXIn : Math.max(map.minXIn, center.xIn - radius);
        const maxX = wholeField ? map.maxXIn : Math.min(map.maxXIn, center.xIn + radius);
        const minY = wholeField ? map.minYIn : Math.max(map.minYIn, center.yIn - radius);
        const maxY = wholeField ? map.maxYIn : Math.min(map.maxYIn, center.yIn + radius);
        for (let j = kept; j < count; ++j) {
          out[j].xIn = minX + this.rng.uniform() * (maxX - minX);
          out[j].yIn = minY + this.rng.uniform() * (maxY - minY);
          out[j].score = 0;
          out[j].recovered = true;
        }
      }

      const weight = 1.0 / count;
      for (const p of out) p.weight = weight;
      this.resampled = particles;
      this.particles = out;
      this.uniformWeights = true;
      this.lastRecovered = recovered;
      return true;
    }

    estimate() {
      const out = {
        xIn: 0, yIn: 0, varianceXIn2: 0, varianceYIn2: 0, covarianceXYIn2: 0, spreadIn: 0,
        effectiveParticles: 0,
      };
      let sumSquares = 0.0;
      for (const p of this.particles) {
        out.xIn += p.weight * p.xIn;
        out.yIn += p.weight * p.yIn;
        sumSquares += p.weight * p.weight;
      }
      for (const p of this.particles) {
        const dx = p.xIn - out.xIn;
        const dy = p.yIn - out.yIn;
        out.varianceXIn2 += p.weight * dx * dx;
        out.varianceYIn2 += p.weight * dy * dy;
        out.covarianceXYIn2 += p.weight * dx * dy;
      }
      out.spreadIn = Math.sqrt(out.varianceXIn2 + out.varianceYIn2);
      out.effectiveParticles = sumSquares > 0.0 ? 1.0 / sumSquares : 0.0;
      return out;
    }

    checkSensors(xIn, yIn, headingDeg, readings, agreementSigmas) {
      const checks = [];
      let agreeing = 0;
      const sensorCount = Math.min(readings.length, this.sensors.length);
      for (let s = 0; s < sensorCount; ++s) {
        const check = { used: readings[s].valid, measuredIn: 0, expectedIn: 0, sigmaIn: 0,
          residualSigmas: 0, agrees: false };
        if (check.used) {
          const ray = sensorRay(xIn, yIn, headingDeg, this.sensors[s]);
          check.measuredIn = readings[s].distanceIn;
          check.expectedIn = this.map.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
          check.sigmaIn = readingSigmaIn(this.config.beam, readings[s]);
          check.residualSigmas = (check.measuredIn - check.expectedIn) / check.sigmaIn;
          check.agrees = Math.abs(check.residualSigmas) <= agreementSigmas;
          if (check.agrees) ++agreeing;
        }
        checks.push(check);
      }
      return { agreeing, checks };
    }
  }

  /** sampleParticles(): out.length picks, evenly spaced along the cumulative weights. */
  function sampleParticles(particles, count) {
    const out = [];
    if (!particles.length || count <= 0) return out;
    let total = 0.0;
    for (const p of particles) total += p.weight;
    const step = total / count;
    let cumulative = particles[0].weight;
    let source = 0;
    for (let j = 0; j < count; ++j) {
      const target = (j + 0.5) * step;
      while (target > cumulative && source + 1 < particles.length) {
        ++source;
        cumulative += particles[source].weight;
      }
      out.push(particles[source]);
    }
    return out;
  }

  function copyParticle(to, from) {
    to.xIn = from.xIn;
    to.yIn = from.yIn;
    to.weight = from.weight;
    to.score = from.score;
    to.recovered = from.recovered;
  }

  // --- Odometry's math (odometry_math.cpp) ------------------------------------

  function computeOdometryDelta(lastHeadingDeg, headingDeg, verticalDeltaIn, horizontalDeltaIn,
    verticalOffsetIn, horizontalOffsetIn) {
    const dThetaDeg = wrapDegrees180(headingDeg - lastHeadingDeg);
    const dThetaRad = dThetaDeg * kDegToRad;
    const localForwardIn = verticalDeltaIn - verticalOffsetIn * dThetaRad;
    const localLateralIn = horizontalDeltaIn - horizontalOffsetIn * dThetaRad;
    const avgHeadingRad = (lastHeadingDeg + dThetaDeg / 2.0) * kDegToRad;
    const cosHeading = Math.cos(avgHeadingRad);
    const sinHeading = Math.sin(avgHeadingRad);
    return {
      dxIn: localForwardIn * sinHeading + localLateralIn * cosHeading,
      dyIn: localForwardIn * cosHeading - localLateralIn * sinHeading,
    };
  }

  function correctionStep(remainingXIn, remainingYIn, maxStepIn) {
    const remainingIn = Math.hypot(remainingXIn, remainingYIn);
    if (remainingIn <= maxStepIn) return { dxIn: remainingXIn, dyIn: remainingYIn };
    const scale = maxStepIn / remainingIn;
    return { dxIn: remainingXIn * scale, dyIn: remainingYIn * scale };
  }

  return {
    kPi,
    kDegToRad,
    kVrcFieldSizeIn,
    wrapDegrees180,
    wrapDegrees360,
    Rng,
    zigguratTables,
    LookupTable,
    FieldMap,
    ParallelRayCaster,
    beamModel,
    mount,
    reading,
    sensorRay,
    readingSigmaIn,
    readingLogLikelihood,
    ReadingScorer,
    distanceReadingFromMm,
    compensateLatency,
    filterConfig,
    ParticleFilter,
    sampleParticles,
    computeOdometryDelta,
    correctionStep,
  };
});
