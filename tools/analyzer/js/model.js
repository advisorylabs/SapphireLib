/*
 * SapphireLib telemetry analyzer — model.js
 *
 * The robot's own tuning and control math, ported line for line so the
 * analyzer refits, redesigns and simulates from logged data exactly the way
 * the robot would:
 *
 *   system identification  src/sapphirelib/tuning/characterization_math.cpp
 *   pole placement         src/sapphirelib/tuning/gain_design.cpp
 *   the PID controller     src/sapphirelib/control/pid.cpp
 *   the mechanism law      src/sapphirelib/mechanism/position_control.cpp
 *   motor thermal derating src/sapphirelib/chassis/thermal_math.cpp
 *
 * Each port keeps the C++'s order of operations, so on the same data it
 * produces the same numbers to the last bit or two (only libm calls — log,
 * exp, atan2 — may differ in their final bit). test/model.test.js holds it to
 * the golden case in tests/tuning/characterization_math_test.cpp. Change one
 * side, change the other.
 *
 * On top of the ports: a plant simulator and closed-loop "what if" runs for
 * the Tune tab, and step-response metrics.
 *
 * Plain script: works as a browser <script> (window.SA.model) and as a Node
 * module (require('./model.js')), with no dependencies.
 *
 * Team 96671H — Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.model = factory();
  }
})(function () {
  'use strict';

  const kLn2 = 0.69314718055994530942;
  const kDegToRad = 3.14159265358979323846 / 180.0;
  const kPi = 3.14159265358979323846;
  const kMinFitRows = 20;
  const kMaxPlausibleDelayS = 0.5;

  function signOf(value) {
    return value > 0 ? 1 : value < 0 ? -1 : 0;
  }

  /** std::fmax: NaN loses to a number. */
  function fmax(a, b) {
    if (Number.isNaN(a)) return b;
    if (Number.isNaN(b)) return a;
    return a > b ? a : b;
  }

  function clamp(value, lo, hi) {
    return value < lo ? lo : hi < value ? hi : value;
  }

  // --- Models -------------------------------------------------------------------

  /** MotorFeedforward: V = kS·sign(v) + kV·v + kA·a. */
  function feedforward(kS = 0, kV = 0, kA = 0) {
    return { kS, kV, kA };
  }

  function ffValid(model) {
    return model.kV > 0 && model.kA > 0;
  }

  function ffVolts(model, velocity, acceleration = 0) {
    const direction = velocity !== 0 ? signOf(velocity) : signOf(acceleration);
    return model.kS * direction + model.kV * velocity + model.kA * acceleration;
  }

  function ffMaxVelocity(model, availableVolts = 12) {
    if (!(model.kV > 0)) return 0;
    return fmax(0, (Math.abs(availableVolts) - model.kS) / model.kV);
  }

  /**
   * GravityShape: kind 'none' | 'constant' | 'cosine', with an arm's
   * horizontalPosition and armDegreesPerUnit.
   */
  function gravityShape(kind = 'none', horizontalPosition = 0, armDegreesPerUnit = 1) {
    return { kind, horizontalPosition, armDegreesPerUnit };
  }

  function gravityFactor(shape, position) {
    switch (shape.kind) {
      case 'constant':
        return 1;
      case 'cosine':
        return Math.cos((position - shape.horizontalPosition) * shape.armDegreesPerUnit * kDegToRad);
      default:
        return 0;
    }
  }

  /** MechanismModel: { motion: feedforward, kG, gravity: GravityShape }. */
  function mechanismModel(motion = feedforward(), kG = 0, gravity = gravityShape()) {
    return { motion, kG, gravity };
  }

  function gravityVolts(model, position) {
    return model.kG * gravityFactor(model.gravity, position);
  }

  /** mechanism::GravityFeedforward that cancels a MechanismModel's gravity. */
  function gravityFeedforwardOf(model) {
    const ff = { constantVolts: 0, cosineVolts: 0, horizontalPosition: 0, armDegreesPerUnit: 1 };
    if (model.gravity.kind === 'constant') ff.constantVolts = model.kG;
    if (model.gravity.kind === 'cosine') {
      ff.cosineVolts = model.kG;
      ff.horizontalPosition = model.gravity.horizontalPosition;
      ff.armDegreesPerUnit = model.gravity.armDegreesPerUnit;
    }
    return ff;
  }

  // --- System identification (characterization_math.cpp) -----------------------

  /** Central-difference velocity at every index with a full window each side. */
  function velocities(run, halfWindow) {
    const count = run.length;
    const result = new Array(count).fill(null);
    for (let i = halfWindow; i < count - halfWindow; ++i) {
      const dtS = (run[i + halfWindow].timeMs - run[i - halfWindow].timeMs) / 1000.0;
      if (dtS > 0) result[i] = (run[i + halfWindow].position - run[i - halfWindow].position) / dtS;
    }
    return result;
  }

  function usable(velocity) {
    return velocity !== null && Number.isFinite(velocity);
  }

  /** Gaussian elimination with partial pivoting; null if ill-conditioned. */
  function solve(a, b) {
    const n = b.length;
    a = a.map((row) => row.slice());
    b = b.slice();
    let scale = 0;
    for (const row of a) for (const value of row) scale = fmax(scale, Math.abs(value));
    if (!(scale > 0)) return null;
    for (let col = 0; col < n; ++col) {
      let pivot = col;
      for (let r = col + 1; r < n; ++r) {
        if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
      }
      if (Math.abs(a[pivot][col]) < scale * 1e-12) return null;
      [a[pivot], a[col]] = [a[col], a[pivot]];
      [b[pivot], b[col]] = [b[col], b[pivot]];
      for (let r = col + 1; r < n; ++r) {
        const factor = a[r][col] / a[col][col];
        for (let c = col; c < n; ++c) a[r][c] -= factor * a[col][c];
        b[r] -= factor * b[col];
      }
    }
    const x = new Array(n).fill(0);
    for (let i = n; i-- > 0; ) {
      let sum = b[i];
      for (let c = i + 1; c < n; ++c) sum -= a[i][c] * x[c];
      x[i] = sum / a[i][i];
    }
    return x;
  }

  /**
   * fitDiscrete<N>(): the regression behind fitFeedforward() and
   * fitMechanism(). Returns { ok, model: MechanismModel, rSquared, samplesUsed }.
   */
  function fitDiscrete(runs, gravity, minSpeed, halfWindow, delayTicks, minRSquared, withGravity) {
    const n = withGravity ? 4 : 3;
    const fit = {
      ok: false,
      model: mechanismModel(feedforward(), 0, gravity),
      rSquared: 0,
      samplesUsed: 0,
    };
    const w = Math.max(1, halfWindow);
    const shift = Math.max(0, delayTicks);
    const m = 2 * w + 1;

    const rows = [];
    for (const run of runs) {
      const v = velocities(run, w);
      const count = run.length;
      for (let k = Math.max(w, shift); k + m < count; ++k) {
        if (!usable(v[k]) || !usable(v[k + m])) continue;
        if (Math.abs(v[k]) < minSpeed) continue;
        const intervalS = (run[k + m].timeMs - run[k].timeMs) / 1000.0;
        if (!(intervalS > 0)) continue;
        let voltsSum = 0;
        for (let j = k; j < k + m; ++j) voltsSum += run[j - shift].volts;
        if (!Number.isFinite(voltsSum)) continue;
        let gravitySum = 0;
        if (withGravity) {
          for (let j = k; j < k + m; ++j) gravitySum += gravityFactor(gravity, run[j].position);
          if (!Number.isFinite(gravitySum)) continue;
        }
        rows.push({
          velocity: v[k],
          nextVelocity: v[k + m],
          volts: voltsSum / m,
          gravity: gravitySum / m,
          intervalS,
        });
      }
    }
    fit.samplesUsed = rows.length;
    if (fit.samplesUsed < kMinFitRows) return fit;

    const normal = [];
    for (let r = 0; r < n; ++r) normal.push(new Array(n).fill(0));
    const rhs = new Array(n).fill(0);
    let intervalSum = 0;
    for (const row of rows) {
      const f = [row.velocity, row.volts, signOf(row.velocity)];
      if (withGravity) f.push(row.gravity);
      for (let r = 0; r < n; ++r) {
        for (let c = 0; c < n; ++c) normal[r][c] += f[r] * f[c];
        rhs[r] += f[r] * row.nextVelocity;
      }
      intervalSum += row.intervalS;
    }

    const x = solve(normal, rhs);
    if (x === null) return fit;
    const alpha = x[0];
    const beta = x[1];
    const gamma = x[2];
    if (!(alpha > 0 && alpha < 1 && beta > 0)) return fit;

    const intervalS = intervalSum / fit.samplesUsed;
    const kV = (1.0 - alpha) / beta;
    fit.model.motion = feedforward(fmax(0, -gamma / beta), kV, (-kV * intervalS) / Math.log(alpha));
    if (withGravity) fit.model.kG = -x[3] / beta;

    let meanVolts = 0;
    for (const row of rows) meanVolts += row.volts;
    meanVolts /= fit.samplesUsed;
    let residual = 0;
    let total = 0;
    for (const row of rows) {
      const midVelocity = 0.5 * (row.velocity + row.nextVelocity);
      const acceleration = (row.nextVelocity - row.velocity) / row.intervalS;
      let predicted = ffVolts(fit.model.motion, midVelocity, acceleration);
      if (withGravity) predicted += fit.model.kG * row.gravity;
      residual += (row.volts - predicted) * (row.volts - predicted);
      total += (row.volts - meanVolts) * (row.volts - meanVolts);
    }
    fit.rSquared = total > 0 ? 1.0 - residual / total : 0;
    fit.ok = ffValid(fit.model.motion) && fit.rSquared >= minRSquared;
    return fit;
  }

  function fitMechanism(runs, gravity, minSpeed, halfWindow = 5, delayTicks = 0, minRSquared = 0.8) {
    return fitDiscrete(runs, gravity, minSpeed, halfWindow, delayTicks, minRSquared,
      gravity.kind !== 'none');
  }

  function fitFeedforward(runs, minSpeed, halfWindow = 5, delayTicks = 0, minRSquared = 0.8) {
    const fit = fitDiscrete(runs, gravityShape(), minSpeed, halfWindow, delayTicks, minRSquared,
      false);
    return { ok: fit.ok, model: fit.model.motion, rSquared: fit.rSquared, samplesUsed: fit.samplesUsed };
  }

  function estimateMechanismDelayS(stepRun, model, halfWindow = 5) {
    if (!ffValid(model.motion)) return -1;
    const motion = model.motion;
    const stepIndex = stepRun.findIndex((s) => Number.isFinite(s.volts) && s.volts !== 0);
    if (stepIndex < 0) return -1;
    const step = stepRun[stepIndex];
    const stepTimeS = step.timeMs / 1000.0;
    const netVolts = step.volts - gravityVolts(model, step.position);
    const halfSpeed = 0.5 * fmax(0, (Math.abs(netVolts) - motion.kS) / motion.kV);
    if (!(halfSpeed > 0)) return -1;
    const direction = signOf(netVolts);

    const w = Math.max(1, halfWindow);
    const v = velocities(stepRun, w);
    let prev = -1;
    for (let i = 0; i < stepRun.length; ++i) {
      if (!usable(v[i])) continue;
      const timeS = stepRun[i].timeMs / 1000.0;
      const speed = v[i] * direction;
      if (timeS >= stepTimeS && speed >= halfSpeed) {
        let crossingS = timeS;
        if (prev >= 0) {
          const prevTimeS = stepRun[prev].timeMs / 1000.0;
          const prevSpeed = v[prev] * direction;
          if (prevSpeed < halfSpeed && speed > prevSpeed) {
            crossingS = prevTimeS + ((timeS - prevTimeS) * (halfSpeed - prevSpeed)) / (speed - prevSpeed);
          }
        }
        const idealS = (motion.kA / motion.kV) * kLn2;
        return clamp(crossingS - stepTimeS - idealS, 0, kMaxPlausibleDelayS);
      }
      prev = i;
    }
    return -1;
  }

  function estimateResponseDelayS(stepRun, ffModel, halfWindow = 5) {
    return estimateMechanismDelayS(stepRun, mechanismModel(ffModel), halfWindow);
  }

  /**
   * characterizeMechanism(): { ok, fit: MechanismFit, delayS }. `data` is
   * { ramps: [run...], steps: [run...] }, each run an array of
   * { timeMs, volts, position }.
   */
  function characterizeMechanism(data, gravity, minSpeed, halfWindow = 5, minRSquared = 0.8) {
    const result = { ok: false, fit: null, delayS: 0 };
    const all = data.ramps.concat(data.steps);

    let periodSumS = 0;
    let periodCount = 0;
    for (const run of all) {
      if (run.length < 2) continue;
      periodSumS += (run[run.length - 1].timeMs - run[0].timeMs) / 1000.0;
      periodCount += run.length - 1;
    }
    const periodS = periodCount > 0 ? periodSumS / periodCount : 0;

    const meanDelay = (model) => {
      let sum = 0;
      let valid = 0;
      for (const step of data.steps) {
        const delayS = estimateMechanismDelayS(step, model, halfWindow);
        if (delayS >= 0) {
          sum += delayS;
          ++valid;
        }
      }
      return valid > 0 ? sum / valid : 0;
    };

    let fit = fitMechanism(all, gravity, minSpeed, halfWindow, 0, minRSquared);
    if (!ffValid(fit.model.motion)) {
      result.fit = fit;
      return result;
    }
    let delayS = meanDelay(fit.model);
    if (periodS > 0) {
      const shift = Math.round(delayS / periodS);
      if (shift > 0) {
        const shifted = fitMechanism(all, gravity, minSpeed, halfWindow, shift, minRSquared);
        if (ffValid(shifted.model.motion)) {
          fit = shifted;
          delayS = meanDelay(fit.model);
        }
      }
    }
    result.fit = fit;
    result.delayS = delayS;
    result.ok = fit.ok;
    return result;
  }

  function characterizeAxis(data, minSpeed, halfWindow = 5, minRSquared = 0.8) {
    const r = characterizeMechanism(data, gravityShape(), minSpeed, halfWindow, minRSquared);
    return {
      ok: r.ok,
      fit: { ok: r.fit.ok, model: r.fit.model.motion, rSquared: r.fit.rSquared, samplesUsed: r.fit.samplesUsed },
      delayS: r.delayS,
    };
  }

  // --- Gain design (gain_design.cpp) --------------------------------------------

  const kSettleBand = 0.02;
  const kMinDamping = 0.1;
  const kMaxDamping = 5.0;

  function stepResponse(zeta, t) {
    if (Math.abs(zeta - 1.0) < 1e-6) return 1.0 - (1.0 + t) * Math.exp(-t);
    if (zeta < 1.0) {
      const wd = Math.sqrt(1.0 - zeta * zeta);
      return 1.0 - (Math.exp(-zeta * t) / wd) * Math.sin(wd * t + Math.acos(zeta));
    }
    const root = Math.sqrt(zeta * zeta - 1.0);
    const fast = zeta + root;
    const slow = zeta - root;
    return 1.0 - (fast * Math.exp(-slow * t) - slow * Math.exp(-fast * t)) / (fast - slow);
  }

  function placePoles(model, omega, zeta) {
    return {
      kP: model.kA * omega * omega,
      kI: 0,
      kD: fmax(0, 2.0 * zeta * omega * model.kA - model.kV),
    };
  }

  function normalizedSettleTime(dampingRatio) {
    const zeta = clamp(dampingRatio, kMinDamping, kMaxDamping);
    const kStep = 0.001;
    if (zeta >= 1.0 - 1e-6) {
      for (let t = 0; t < 1000.0; t += kStep) {
        if (1.0 - stepResponse(zeta, t) <= kSettleBand) return t;
      }
      return 1000.0;
    }
    const envelopeEndT = -Math.log(kSettleBand * Math.sqrt(1.0 - zeta * zeta)) / zeta + kStep;
    let lastOutside = 0;
    for (let t = 0; t < envelopeEndT; t += kStep) {
      if (Math.abs(1.0 - stepResponse(zeta, t)) > kSettleBand) lastOutside = t;
    }
    return lastOutside + kStep;
  }

  function phaseMarginDeg(model, gains, delayS) {
    if (!(gains.kP > 0) || !ffValid(model)) return 180.0;
    const magnitude = (w) => {
      const num = Math.hypot(gains.kP, w * gains.kD);
      const den = w * Math.hypot(w * model.kA, model.kV);
      return num / den;
    };
    let lo = Math.log(1e-4);
    let hi = Math.log(1e5);
    for (let i = 0; i < 100; ++i) {
      const mid = 0.5 * (lo + hi);
      if (magnitude(Math.exp(mid)) > 1.0) lo = mid;
      else hi = mid;
    }
    const crossover = Math.exp(0.5 * (lo + hi));
    const phaseRad =
      Math.atan2(crossover * gains.kD, gains.kP) - kPi / 2.0 -
      Math.atan2(crossover * model.kA, model.kV) - crossover * fmax(0, delayS);
    return 180.0 + (phaseRad * 180.0) / kPi;
  }

  /** ResponseSpec defaults, as in gain_design.hpp. */
  function responseSpec(settleTimeS = 0.6, dampingRatio = 1.0, minPhaseMarginDeg = 50.0) {
    return { settleTimeS, dampingRatio, minPhaseMarginDeg };
  }

  function designPositionGains(model, spec, delayS = 0) {
    const design = {
      ok: false,
      gains: { kP: 0, kI: 0, kD: 0 },
      naturalFrequency: 0,
      settleTimeS: 0,
      phaseMarginDeg: 0,
      limitedByDelay: false,
      staticErrorBound: 0,
    };
    if (!ffValid(model) || !(spec.settleTimeS > 0)) return design;
    const zeta = clamp(spec.dampingRatio, kMinDamping, kMaxDamping);
    const unitSettle = normalizedSettleTime(zeta);
    let omega = unitSettle / spec.settleTimeS;
    if (phaseMarginDeg(model, placePoles(model, omega, zeta), delayS) < spec.minPhaseMarginDeg) {
      let slow = 0;
      let fast = omega;
      for (let i = 0; i < 60; ++i) {
        const mid = 0.5 * (slow + fast);
        if (phaseMarginDeg(model, placePoles(model, mid, zeta), delayS) >= spec.minPhaseMarginDeg) {
          slow = mid;
        } else {
          fast = mid;
        }
      }
      omega = slow;
      design.limitedByDelay = true;
    }
    if (!(omega > 0)) return design;
    design.ok = true;
    design.gains = placePoles(model, omega, zeta);
    design.naturalFrequency = omega;
    design.settleTimeS = unitSettle / omega;
    design.phaseMarginDeg = phaseMarginDeg(model, design.gains, delayS);
    design.staticErrorBound = design.gains.kP > 0 ? model.kS / design.gains.kP : Infinity;
    return design;
  }

  // --- PID (pid.cpp) ------------------------------------------------------------

  const kMaxPlausibleDtS = 0.5;
  const FLAG = { saturated: 1, slewLimited: 2, integralHeld: 4, firstStep: 8, dtFallback: 16 };

  /** PID::Config defaults. */
  function pidConfig(overrides = {}) {
    return Object.assign(
      {
        gains: { kP: 0, kI: 0, kD: 0 },
        integralLimit: 0,
        outputLimit: 0,
        slewRate: 0,
        derivativeOnMeasurement: false,
        nominalDtS: 0.01,
      },
      overrides,
    );
  }

  class PID {
    constructor(config) {
      this.config = pidConfig(config);
      this.config.gains = Object.assign({ kP: 0, kI: 0, kD: 0 }, this.config.gains);
      this.integral = 0;
      this.prevError = 0;
      this.prevMeasurement = 0;
      this.prevOutput = 0;
      this.hasPrev = false;
      this.lastStep = null;
    }

    update(target, measurement, dtS) {
      const c = this.config;
      if (dtS === undefined) dtS = c.nominalDtS;
      let flags = this.hasPrev ? 0 : FLAG.firstStep;
      if (!(dtS > 0) || dtS > kMaxPlausibleDtS) {
        dtS = c.nominalDtS;
        flags |= FLAG.dtFallback;
      }
      const error = target - measurement;
      const integralDelta = error * dtS;
      this.integral += integralDelta;
      if (c.integralLimit > 0) this.integral = clamp(this.integral, -c.integralLimit, c.integralLimit);

      let derivative = 0;
      if (this.hasPrev) {
        derivative = (c.derivativeOnMeasurement ? -(measurement - this.prevMeasurement)
          : error - this.prevError) / dtS;
      }
      let output = c.gains.kP * error + c.gains.kI * this.integral + c.gains.kD * derivative;
      if (c.outputLimit > 0 && integralDelta !== 0 && Math.abs(output) > c.outputLimit &&
          (output > 0) === (integralDelta > 0)) {
        this.integral -= integralDelta;
        output = c.gains.kP * error + c.gains.kI * this.integral + c.gains.kD * derivative;
        flags |= FLAG.integralHeld;
      }
      const rawOutput = output;
      if (c.slewRate > 0 && this.hasPrev) {
        const delta = clamp(output - this.prevOutput, -c.slewRate, c.slewRate);
        if (delta !== output - this.prevOutput) flags |= FLAG.slewLimited;
        output = this.prevOutput + delta;
      }
      if (c.outputLimit > 0) {
        const unclamped = output;
        output = clamp(output, -c.outputLimit, c.outputLimit);
        if (output !== unclamped) flags |= FLAG.saturated;
      }
      this.prevError = error;
      this.prevMeasurement = measurement;
      this.prevOutput = output;
      this.hasPrev = true;
      this.lastStep = {
        target,
        measurement,
        error,
        pTerm: c.gains.kP * error,
        iTerm: c.gains.kI * this.integral,
        dTerm: c.gains.kD * derivative,
        rawOutput,
        output,
        dtS,
        flags,
      };
      return output;
    }

    reset() {
      const hadState = this.hasPrev;
      this.integral = 0;
      this.prevError = 0;
      this.prevMeasurement = 0;
      this.prevOutput = 0;
      this.hasPrev = false;
      return hadState;
    }

    setGains(gains) {
      this.config.gains = Object.assign({}, gains);
    }
  }

  // --- The mechanism control law (position_control.cpp) ---------------------------

  const LAW = { track: 0, seat: 1, rest: 2, sensorLost: 3, manual: 4, off: 5, external: 6 };
  const LAW_NAMES = ['track', 'seat', 'rest', 'no sensor', 'manual', 'off', 'external'];

  function gravityFeedforwardVolts(ff, position) {
    if (ff.cosineVolts === 0) return ff.constantVolts;
    return ff.constantVolts +
      ff.cosineVolts * Math.cos((position - ff.horizontalPosition) * ff.armDegreesPerUnit * kDegToRad);
  }

  /**
   * computePositionCommand(). `config` is { gravity: GravityFeedforward,
   * seat: { enabled, floor, seatVolts, restBand }, maxVolts }.
   */
  function computePositionCommand(pid, config, target, position) {
    if (!Number.isFinite(position)) {
      pid.reset();
      return { law: LAW.sensorLost, volts: 0, brake: true };
    }
    let volts;
    let law;
    const seat = config.seat || { enabled: false, floor: 0, seatVolts: 0, restBand: 0 };
    if (!seat.enabled || target > seat.floor) {
      volts = pid.update(target, position) + gravityFeedforwardVolts(config.gravity, position);
      law = LAW.track;
    } else if (position > seat.floor + seat.restBand) {
      volts = Math.min(pid.update(target, position), -seat.seatVolts);
      law = LAW.seat;
    } else {
      volts = 0;
      pid.reset();
      law = LAW.rest;
    }
    const maxVolts = config.maxVolts !== undefined ? config.maxVolts : 12;
    return { law, volts: clamp(volts, -maxVolts, maxVolts), brake: false };
  }

  // --- Motor thermal derating (thermal_math.cpp) ----------------------------------

  const kDeratingCurve = [
    [54.0, 1.0], [56.0, 0.5], [59.0, 0.5], [61.0, 0.25],
    [64.0, 0.25], [66.0, 0.125], [69.0, 0.125], [71.0, 0.0],
  ];

  /** Fraction of rated output a V5 motor still delivers at `tempC`. */
  function thermalPowerFraction(tempC) {
    if (!Number.isFinite(tempC)) return 1.0;
    if (tempC <= kDeratingCurve[0][0]) return kDeratingCurve[0][1];
    for (let i = 1; i < kDeratingCurve.length; ++i) {
      const [highT, highF] = kDeratingCurve[i];
      if (tempC > highT) continue;
      const [lowT, lowF] = kDeratingCurve[i - 1];
      const t = (tempC - lowT) / (highT - lowT);
      return lowF + t * (highF - lowF);
    }
    return kDeratingCurve[kDeratingCurve.length - 1][1];
  }

  // --- Plant simulation ---------------------------------------------------------

  /**
   * A simulated axis obeying a MechanismModel — kA·a = u − kS·sign(v) −
   * kG·g(x) − kV·v, with static friction — and a command latency. Commands
   * are clamped to ±12V (what a V5 motor can apply); `hold()` freezes it like
   * a motor on its hold brake. Integrated in 1ms steps.
   */
  class PlantSim {
    /** `limits`, if given, are hard stops [min, max]: a mechanism's floor and top. */
    constructor(model, delayS = 0, x0 = 0, v0 = 0, limits = null) {
      this.model = model;
      this.limits = limits;
      this.delayS = Math.max(0, delayS);
      this.x = x0;
      this.v = v0;
      this.t = 0;
      this.pending = [];
      this.volts = 0;
      this.held = false;
      this.voltageScale = 1; // <1 models a derated or sagging supply
    }

    command(volts) {
      this.pending.push({ at: this.t + this.delayS, held: false, volts: clamp(volts, -12, 12) });
    }

    hold() {
      this.pending.push({ at: this.t + this.delayS, held: true, volts: 0 });
    }

    advance(dtS) {
      const steps = Math.max(1, Math.round(dtS / 0.001));
      const h = dtS / steps / 4;
      const motion = this.model.motion;
      for (let i = 0; i < steps; ++i) {
        while (this.pending.length > 0 && this.pending[0].at <= this.t + 1e-9) {
          const next = this.pending.shift();
          this.held = next.held;
          this.volts = next.volts;
        }
        if (this.held) {
          this.v = 0;
        } else {
          for (let s = 0; s < 4; ++s) {
            const load = this.volts * this.voltageScale - gravityVolts(this.model, this.x);
            const friction = Math.abs(this.v) > 1e-3 ? motion.kS * signOf(this.v)
              : signOf(load) * Math.min(Math.abs(load), motion.kS);
            this.v += ((load - friction - motion.kV * this.v) / motion.kA) * h;
            this.x += this.v * h;
            if (this.limits) {
              if (this.x < this.limits[0]) {
                this.x = this.limits[0];
                this.v = Math.max(0, this.v);
              } else if (this.x > this.limits[1]) {
                this.x = this.limits[1];
                this.v = Math.min(0, this.v);
              }
            }
          }
        }
        this.t += dtS / steps;
      }
    }
  }

  /**
   * Open-loop replay: drives the model with a recorded voltage series and
   * returns the positions it predicts, to overlay on what was measured —
   * the most direct check that a fitted model describes the robot. `volts`
   * NaN means held. Samples are { t (s), volts, position }; the simulation
   * restarts from the measured state after any gap longer than `maxGapS`.
   */
  function replayOpenLoop(model, delayS, samples, maxGapS = 0.1) {
    const out = new Float64Array(samples.length).fill(NaN);
    let sim = null;
    let lastT = null;
    for (let i = 0; i < samples.length; ++i) {
      const s = samples[i];
      if (!Number.isFinite(s.position)) {
        sim = null;
        continue;
      }
      if (sim === null || lastT === null || s.t - lastT > maxGapS) {
        sim = new PlantSim(model, delayS, s.position, 0);
        lastT = s.t;
      } else {
        sim.advance(s.t - lastT);
        lastT = s.t;
      }
      out[i] = sim.x;
      if (Number.isFinite(s.volts)) sim.command(s.volts);
      else sim.hold();
    }
    return out;
  }

  /**
   * Closed-loop "what if": runs `controller(target, measured, dtS)` every
   * `periodS` against a PlantSim (with hard stops at `limits`, if given),
   * following `targetAt(t)`, for `durationS`.
   * The controller returns volts, or null to hold (brake). Returns arrays of
   * t, x (position), u (volts), and target.
   */
  function simulateClosedLoop({ model, delayS, periodS, durationS, x0 = 0, targetAt, controller,
    limits = null }) {
    const sim = new PlantSim(model, delayS, x0, 0, limits);
    const n = Math.max(1, Math.ceil(durationS / periodS));
    const t = new Float64Array(n);
    const x = new Float64Array(n);
    const u = new Float64Array(n);
    const target = new Float64Array(n);
    for (let i = 0; i < n; ++i) {
      const now = i * periodS;
      const goal = targetAt(now);
      const volts = controller(goal, sim.x, periodS);
      t[i] = now;
      x[i] = sim.x;
      target[i] = goal;
      if (volts === null || !Number.isFinite(volts)) {
        u[i] = NaN;
        sim.hold();
      } else {
        u[i] = clamp(volts, -12, 12);
        sim.command(volts);
      }
      sim.advance(periodS);
    }
    return { t, x, u, target };
  }

  /**
   * Step metrics for a response from `from` toward `to`: rise time (10-90%),
   * overshoot past the target (in position units and as a fraction of the
   * step), when it last left a ±`band` window (settle time, s from t[0]),
   * final error, peak |volts|, and the fraction of samples at |volts| ≥ 11.9.
   */
  function stepMetrics(t, x, from, to, band, u = null) {
    const size = to - from;
    const dir = size >= 0 ? 1 : -1;
    const t0 = t.length ? t[0] : 0;
    let t10 = null;
    let t90 = null;
    let overshoot = 0;
    let lastOutside = null;
    let peakVolts = 0;
    let saturated = 0;
    for (let i = 0; i < t.length; ++i) {
      if (!Number.isFinite(x[i])) continue;
      const progress = size !== 0 ? (x[i] - from) / size : 1;
      if (t10 === null && progress >= 0.1) t10 = t[i];
      if (t90 === null && progress >= 0.9) t90 = t[i];
      overshoot = Math.max(overshoot, (x[i] - to) * dir);
      if (Math.abs(to - x[i]) > band) lastOutside = t[i];
      if (u && Number.isFinite(u[i])) {
        peakVolts = Math.max(peakVolts, Math.abs(u[i]));
        if (Math.abs(u[i]) >= 11.9) saturated++;
      }
    }
    const last = t.length ? x[x.length - 1] : NaN;
    const dt = t.length > 1 ? t[1] - t[0] : 0;
    return {
      riseS: t10 !== null && t90 !== null ? t90 - t10 : null,
      overshoot,
      overshootFraction: size !== 0 ? overshoot / Math.abs(size) : 0,
      settleS: lastOutside === null ? 0 : lastOutside - t0 + dt,
      settled: lastOutside === null || lastOutside < t[t.length - 1],
      finalError: Math.abs(to - last),
      peakVolts,
      saturatedFraction: t.length ? saturated / t.length : 0,
    };
  }

  return {
    signOf,
    feedforward,
    ffValid,
    ffVolts,
    ffMaxVelocity,
    gravityShape,
    gravityFactor,
    mechanismModel,
    gravityVolts,
    gravityFeedforwardOf,
    velocities,
    fitFeedforward,
    fitMechanism,
    estimateResponseDelayS,
    estimateMechanismDelayS,
    characterizeAxis,
    characterizeMechanism,
    normalizedSettleTime,
    phaseMarginDeg,
    responseSpec,
    designPositionGains,
    FLAG,
    pidConfig,
    PID,
    LAW,
    LAW_NAMES,
    gravityFeedforwardVolts,
    computePositionCommand,
    thermalPowerFraction,
    PlantSim,
    replayOpenLoop,
    simulateClosedLoop,
    stepMetrics,
  };
});
