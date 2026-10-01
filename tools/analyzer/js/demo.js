/*
 * SapphireLib telemetry analyzer: demo.js
 *
 * Builds realistic SLT v1 logs by simulating the robot, for trying the
 * analyzer without a robot and as its test fixture. Nothing here is recorded
 * data: it's physics (a three-axis holonomic chassis, a gravity-loaded lift,
 * motors that heat and derate the way V5 motors do, a sagging battery) driven
 * by the robot's own control code (model.js's ports of its PID and lift
 * law), its motion loops, driver macros and Auto-Tune runner, logged through
 * a writer that formats every row the way the robot's encoder does
 * (csv_format.cpp: fixed-point per channel, floats rounded like Records).
 *
 * The localizer is the real thing: the simulator's port of the robot's
 * particle filter (tools/sim/js/mcl.js), fed by tracking wheels that read 2%
 * long and four distance sensors seeing the walls, logged the way
 * src/robot/telemetry.cpp logs it (mcl, mcl.beams, mcl.state, mcl.pts). Its
 * randomness is its own, so the rest of the demo is the same with or without it.
 *
 * Two logs, two program runs:
 *   SL000041.CSV  a pit session: Auto-Tune on the drivetrain and the lift,
 *                 then Run Tests with the new gains.
 *   SL000042.CSV  a match, after a reboot (so back on the source gains), that
 *                 goes wrong the way matches do: the lift's motors overheat
 *                 and derate late in driver control, a drive motor's cable
 *                 drops out, the claw stalls holding pieces, a blocked turn
 *                 times out in autonomous, the battery sags, the
 *                 controller drops for a moment, a hit lifts the tracking
 *                 wheels (the localizer pulls the pose back), and a robot
 *                 parks in front of the front distance sensor.
 *
 * Plain script: window.SA.demo in a browser, require('./demo.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./model.js'), require('../../sim/js/mcl.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SA = root.SA || {};
    root.SA.demo = factory(root.SA.model, root.SIM.mcl);
  }
})(function (M, L) {
  'use strict';

  // --- Deterministic randomness -------------------------------------------------

  function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function gaussian(rng) {
    const u = Math.max(rng(), 1e-12);
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng());
  }

  // --- The robot's number formatting (csv_format.cpp) ----------------------------

  const SCALE = [1, 10, 100, 1e3, 1e4, 1e5, 1e6];

  function fmtNumber(value, decimals) {
    if (Number.isNaN(value)) return 'nan';
    if (!Number.isFinite(value)) return value > 0 ? 'inf' : '-inf';
    const magnitude = Math.abs(value);
    if (magnitude >= 1e12) return value.toExponential(6);
    const d = Math.min(6, Math.max(0, decimals));
    const scaled = Math.round(magnitude * SCALE[d]);
    let text;
    if (scaled <= Number.MAX_SAFE_INTEGER) {
      const whole = Math.floor(scaled / SCALE[d]);
      const fraction = scaled - whole * SCALE[d];
      text = String(whole);
      if (fraction !== 0) {
        text += '.' + String(fraction).padStart(d, '0').replace(/0+$/, '');
      }
    } else {
      const big = BigInt(scaled);
      const unit = BigInt(SCALE[d]);
      const fraction = big % unit;
      text = String(big / unit);
      if (fraction !== 0n) text += '.' + String(fraction).padStart(d, '0').replace(/0+$/, '');
    }
    return scaled !== 0 && value < 0 ? '-' + text : text;
  }

  /** printf's %.9g, as the robot writes G and C rows. */
  function fmtG9(value) {
    if (value === 0 || !Number.isFinite(value)) return fmtNumber(value === 0 ? 0 : value, 0);
    const exp = value.toExponential(8);
    const x = Number(exp.slice(exp.indexOf('e') + 1));
    const trim = (s) => (s.includes('.') ? s.replace(/0+$/, '').replace(/\.$/, '') : s);
    if (x < -4 || x >= 9) {
      const mantissa = trim(exp.slice(0, exp.indexOf('e')));
      const sign = x < 0 ? '-' : '+';
      return `${mantissa}e${sign}${String(Math.abs(x)).padStart(2, '0')}`;
    }
    return trim(value.toFixed(8 - x));
  }

  const f32 = Math.fround;

  // --- The file writer ----------------------------------------------------------

  class Writer {
    constructor({ file, robot, openUs, build }) {
      this.meta = [
        ['writer', 'sapphirelib 0.1.0'], ['kernel', '4.2.2'], ['build', build],
        ['robot', robot], ['file', file], ['dir', '/usd/sl'], ['open_us', String(openUs)],
        ['clock', 'us_since_program_start'],
      ];
      this.openUs = openUs;
      this.file = file;
      this.chans = [[0, 'sys', 'events', 0, []], [1, 'events', 'events', 0, []]];
      this.decimals = new Map([[0, 0], [1, 0]]);
      this.rows = [];
      this.seq = 0;
      this.healthTimes = [];
      this.dropRows = [];
    }

    chan(name, kind, decimals, columns) {
      const id = this.chans.length;
      this.chans.push([id, name, kind, decimals, columns]);
      this.decimals.set(id, decimals);
      return id;
    }

    row(tUs, text) {
      this.rows.push({ t: Math.round(tUs), seq: this.seq++, text });
    }

    sample(id, tUs, values) {
      const d = this.decimals.get(id);
      this.row(tUs, `S,${id},${Math.round(tUs)},${values.map((v) => fmtNumber(f32(v), d)).join(',')}`);
    }

    event(tUs, tag, msg) {
      this.row(tUs, `E,${Math.round(tUs)},${tag},${msg}`);
    }

    pidStep(id, tUs, s) {
      const d = this.decimals.get(id);
      const values = [s.target, s.measurement, s.error, s.pTerm, s.iTerm, s.dTerm, s.rawOutput, s.output,
        s.dtS].map((v) => fmtNumber(f32(v), d));
      this.row(tUs, `S,${id},${Math.round(tUs)},${values.join(',')},${s.flags}`);
    }

    gains(id, tUs, g) {
      this.row(tUs, `G,${id},${Math.round(tUs)},${fmtG9(g.kP)},${fmtG9(g.kI)},${fmtG9(g.kD)}`);
    }

    config(id, tUs, c) {
      this.row(tUs, `C,${id},${Math.round(tUs)},${fmtG9(c.integralLimit)},${fmtG9(c.outputLimit)},` +
        `${fmtG9(c.slewRate)},${c.derivativeOnMeasurement ? 1 : 0},${fmtG9(c.nominalDtS)}`);
    }

    reset(id, tUs) {
      this.row(tUs, `R,${id},${Math.round(tUs)}`);
    }

    health(tUs) {
      this.healthTimes.push(Math.round(tUs));
    }

    drops(tUs, id, full, contended) {
      this.dropRows.push({ t: Math.round(tUs), id, full, contended });
      this.row(tUs, `D,${Math.round(tUs)},${id},${full},${contended}`);
    }

    toString(rng) {
      let out = '#SLT,1\n';
      for (const [k, v] of this.meta) out += `#meta,${k},${v}\n`;
      for (const [id, name, kind, d, cols] of this.chans) {
        out += `#chan,${id},${name},${kind},${d}${cols.length ? ',' + cols.join(',') : ''}\n`;
      }
      for (const t of this.healthTimes) this.rows.push({ t, seq: this.seq++, text: null, health: true });
      this.rows.sort((a, b) => a.t - b.t || a.seq - b.seq);
      let rows = 0;
      let writes = 0;
      let lastWriteUs = this.openUs;
      let drops = 0;
      for (const r of this.rows) {
        if (r.health) {
          while (lastWriteUs + 250000 <= r.t) {
            writes++;
            lastWriteUs += 250000;
          }
          for (const d of this.dropRows) if (d.t <= r.t) drops = Math.max(drops, d.full + d.contended);
          const wmax = 14000 + Math.floor(rng() * 26000);
          const wavg = Math.floor(wmax * (0.45 + 0.2 * rng()));
          // the logger's own time per second, from the numbers already drawn, so adding these
          // keys didn't move the demo's random sequence
          const samp = 1300 + (wmax % 700);
          const fmt = 1900 + (wavg % 900);
          const line = `H,${r.t},rows=${rows},bytes=${out.length},writes=${writes},wmax_us=${wmax},` +
            `wavg_us=${wavg},drops=${drops},unlogged=3,resyncs=0,breaks=0,faults=0,` +
            `samp_us=${samp},fmt_us=${fmt}`;
          out += line + '\n';
          continue;
        }
        out += r.text + '\n';
        rows++;
      }
      return out;
    }
  }

  /** A PID logged the way telemetry::PidProbe logs one. */
  class LoggedPID {
    constructor(writer, id, config) {
      this.pid = new M.PID(config);
      this.writer = writer;
      this.id = id;
      this.configLogged = false;
      this.loggedGains = null;
    }

    update(t, target, measurement, dtS) {
      const out = this.pid.update(target, measurement, dtS);
      const tUs = t * 1e6;
      const c = this.pid.config;
      if (!this.configLogged) {
        this.writer.config(this.id, tUs, c);
        this.configLogged = true;
      }
      const g = c.gains;
      if (!this.loggedGains || g.kP !== this.loggedGains.kP || g.kI !== this.loggedGains.kI ||
          g.kD !== this.loggedGains.kD) {
        this.writer.gains(this.id, tUs, g);
        this.loggedGains = Object.assign({}, g);
      }
      this.writer.pidStep(this.id, tUs, this.pid.lastStep);
      return out;
    }

    reset(t) {
      if (this.pid.reset()) this.writer.reset(this.id, t * 1e6);
    }

    setGains(gains) {
      this.pid.setGains(gains);
    }
  }

  // --- Physics ------------------------------------------------------------------

  const wrap180 = (deg) => {
    let d = ((deg + 180) % 360 + 360) % 360 - 180;
    if (d === -180) d = 180;
    return d;
  };
  const wrap360 = (deg) => ((deg % 360) + 360) % 360;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  /** A V5 smart motor's electrical and thermal state, for its telemetry row. */
  class Motor {
    constructor({ tempC = 32, heat = 0.03, tauS = 400, ambientC = 25, freeRpm = 200 }) {
      this.tempC = tempC;
      this.heat = heat;
      this.tauS = tauS;
      this.ambientC = ambientC;
      this.freeRpm = freeRpm;
      this.volts = 0;
      this.rpm = 0;
      this.amps = 0;
      this.connected = true;
    }

    get fraction() {
      return this.connected ? M.thermalPowerFraction(this.tempC) : 0;
    }

    /** Sets what it's applying and how fast it turns; derives current. */
    drive(volts, rpm, holdAmps = 0) {
      this.volts = clamp(volts, -12, 12);
      this.rpm = rpm;
      const backEmf = (rpm / this.freeRpm) * 11.5;
      const limit = 2.5 * Math.max(this.fraction, 0.02);
      const amps = Math.abs((2.5 * (this.volts - backEmf)) / 12);
      this.amps = Math.min(limit, Math.max(amps, holdAmps) + (Math.abs(this.volts) > 0.3 ? 0.06 : 0.02));
    }

    step(dt) {
      this.tempC += (this.heat * this.amps * this.amps - (this.tempC - this.ambientC) / this.tauS) * dt;
    }

    row(rng) {
      if (!this.connected) return [NaN, NaN, NaN, NaN, NaN, NaN];
      const noise = (s) => s * gaussian(rng);
      // V5 motors report temperature in 5°C steps.
      const reported = Math.floor(this.tempC / 5) * 5;
      const eff = Math.abs(this.rpm) > 5 && Math.abs(this.volts) > 0.5
        ? clamp((80 * Math.abs(this.rpm)) / this.freeRpm, 0, 85) : 0;
      let faults = 0;
      if (this.tempC >= 55) faults |= 1;
      if (this.amps >= 2.5 * Math.max(this.fraction, 0.02) * 0.98) faults |= 4;
      return [this.volts + noise(0.02), this.amps + noise(0.01), reported, this.rpm + noise(0.8), eff,
        faults];
    }
  }

  /** The holonomic chassis: three independent axes, with the corner mix. */
  class Chassis {
    constructor(rng) {
      this.rng = rng;
      const plant = (kS, kV, kA) => new M.PlantSim(M.mechanismModel(M.feedforward(kS, kV, kA)), 0.03);
      this.fwd = plant(1.0, 0.2, 0.045);
      this.strafe = plant(1.3, 0.24, 0.055);
      this.turn = plant(0.6, 0.045, 0.0065);
      this.x = 24;
      this.y = 18;
      this.headingZero = 0; // turn plant x is cumulative heading
      this.motors = {};
      for (const name of ['fl', 'fr', 'bl', 'br', 'ml', 'mr']) {
        this.motors[name] = new Motor({ tempC: 33 + 3 * rng(), heat: name[0] === 'm' ? 0.11 : 0.08,
          tauS: 500 });
      }
      this.applied = { forward: 0, strafe: 0, turn: 0 };
      this.wheels = [0, 0, 0, 0];
      this.blockHeadingAbove = null;
    }

    get heading() {
      return wrap360(this.turn.x);
    }

    /** setWheelVoltages(): the corner volts; recovers the axis volts like the robot. */
    setWheels(fl, fr, bl, br) {
      this.wheels = [fl, fr, bl, br];
      this.applied = {
        forward: (fl + fr + bl + br) / 4,
        strafe: (fl - fr - bl + br) / 4,
        turn: (fl - fr + bl - br) / 4,
      };
      const m = this.motors;
      const clamped = this.wheels.map((v) => clamp(v, -12, 12));
      const frac = [m.fl.fraction, m.fr.fraction, m.bl.fraction, m.br.fraction];
      const eff = clamped.map((v, i) => v * frac[i]);
      // Center wheels add to forward and turn, like the Asterisk does.
      const fwdEff = (eff[0] + eff[1] + eff[2] + eff[3]) / 4;
      const strafeEff = (eff[0] - eff[1] - eff[2] + eff[3]) / 4;
      const turnEff = (eff[0] - eff[1] + eff[2] - eff[3]) / 4;
      const centerL = clamp(this.applied.forward + this.applied.turn, -12, 12);
      const centerR = clamp(this.applied.forward - this.applied.turn, -12, 12);
      this.centers = [centerL, centerR];
      this.fwd.command(fwdEff * 1.0 + 0.0);
      this.strafe.command(strafeEff);
      this.turn.command(turnEff);
    }

    holonomicVolts(forward, strafe, turn) {
      const mix = [forward + strafe + turn, forward - strafe - turn, forward - strafe + turn,
        forward + strafe - turn];
      const largest = Math.max(...mix.map(Math.abs), 12) / 12;
      this.setWheels(...mix.map((v) => v / largest));
    }

    stop() {
      this.wheels = [0, 0, 0, 0];
      this.applied = { forward: 0, strafe: 0, turn: 0 };
      this.centers = [0, 0];
      this.fwd.hold();
      this.strafe.hold();
      this.turn.hold();
      // Braking stops a chassis in a moment, not instantly: let it coast a
      // tick at a time via the hold's latency.
    }

    coast() {
      this.fwd.command(0);
      this.strafe.command(0);
      this.turn.command(0);
      this.wheels = [0, 0, 0, 0];
      this.applied = { forward: 0, strafe: 0, turn: 0 };
      this.centers = [0, 0];
    }

    step(dt) {
      this.fwd.advance(dt);
      this.strafe.advance(dt);
      this.turn.advance(dt);
      if (this.blockHeadingAbove !== null && this.turn.x > this.blockHeadingAbove) {
        this.turn.x = this.blockHeadingAbove;
        this.turn.v = 0;
      }
      const h = (this.turn.x * Math.PI) / 180;
      const vf = this.fwd.v;
      const vs = this.strafe.v;
      this.x += (vf * Math.sin(h) + vs * Math.cos(h)) * dt;
      this.y += (vf * Math.cos(h) - vs * Math.sin(h)) * dt;
      this.x = clamp(this.x, 6, 138);
      this.y = clamp(this.y, 6, 138);
      // Wheels and motors.
      const nf = vf / 58;
      const ns = vs / 48;
      const nw = this.turn.v / 260;
      const wheelRpm = [nf + ns + nw, nf - ns - nw, nf - ns + nw, nf + ns - nw].map((v) => 200 * clamp(v, -1.1, 1.1));
      const m = this.motors;
      ['fl', 'fr', 'bl', 'br'].forEach((name, i) => {
        m[name].drive(m[name].connected ? this.wheels[i] : 0, m[name].connected ? wheelRpm[i] : 0);
      });
      m.ml.drive(this.centers ? this.centers[0] : 0, 200 * clamp(nf + nw, -1.1, 1.1));
      m.mr.drive(this.centers ? this.centers[1] : 0, 200 * clamp(nf - nw, -1.1, 1.1));
      for (const motor of Object.values(m)) motor.step(dt);
    }

    pose(noise) {
      return { x: this.x + noise, y: this.y + noise, heading: this.heading };
    }
  }

  // --- The localizer --------------------------------------------------------------

  const MCL_SETTINGS = { periodMs: 50, particles: 300, startSpreadIn: 2, maxTurnRateDegPerS: 200,
    maxCorrectionSpreadIn: 3, minAgreeingSensors: 2, agreementSigmas: 3, maxCorrectionRateInPerS: 4 };
  // front, right, back, left, 7in out from the tracking center (include/robot/config.hpp)
  const MCL_MOUNTS = [L.mount(7, 0, 0), L.mount(0, 7, 90), L.mount(-7, 0, 180), L.mount(0, -7, 270)];
  const BLOCKED = { noReadings: 4, tooSpread: 8, tooFewAgree: 16, spinning: 64 };

  /**
   * MonteCarloLocalizer on the demo's field (walls at 0 and 144in): tracking
   * wheels that read `scale` long, plus `slips` (a hit that lifts them, so they
   * miss some travel), corrected by the particle filter from four distance
   * sensors with the V5's noise and a few dropouts. `defender` parks something
   * in front of the front sensor. Odometry runs every 10ms (tick()), the
   * filter every 50ms (task()); the corrected pose is what "odom" logs.
   */
  class DemoLocalizer {
    constructor(robot, { scale = 1.02, slips = [], defender = null, seed = 1 } = {}) {
      this.robot = robot;
      this.scale = scale;
      this.slips = slips;
      this.defender = defender;
      this.map = new L.FieldMap(0, 0, 144, 144);
      this.filter = new L.ParticleFilter(this.map, MCL_MOUNTS, { particleCount: MCL_SETTINGS.particles, seed });
      this.noise = new L.Rng(seed + 1000);
      this.raw = null;
      this.lastTruth = null;
      this.applied = { x: 0, y: 0 };
      this.target = { x: 0, y: 0 };
      this.started = false;
      this.updates = 0;
      const w = robot.w;
      this.ids = {
        mcl: w.chan('mcl', 'samples', 2, ['x', 'y', 'spread', 'neff', 'used', 'agree', 'correcting', 'corr_x',
          'corr_y', 'us', 'raw_x', 'raw_y']),
        beams: w.chan('mcl.beams', 'samples', 2, ['m0', 'e0', 'v0', 'm1', 'e1', 'v1', 'm2', 'e2', 'v2', 'm3',
          'e3', 'v3']),
        state: w.chan('mcl.state', 'samples', 4, ['sxx', 'syy', 'sxy', 'fit', 'fit_ratio', 'recovered',
          'resampled', 'blocked', 'hdg']),
        pts: w.chan('mcl.pts', 'samples', 2, ['dx0', 'dy0', 'dx1', 'dy1', 'dx2', 'dy2', 'dx3', 'dy3', 'dx4',
          'dy4', 'dx5', 'dy5']),
      };
      w.meta.push(['mcl.filter.particleCount', String(MCL_SETTINGS.particles)],
        ['mcl.maxCorrectionSpreadIn', '3'], ['mcl.minAgreeingSensors', '2'], ['mcl.periodMs', '50']);
      MCL_MOUNTS.forEach((m, i) => w.meta.push([`mcl.sensor${i}`, `${m.forwardIn},${m.rightIn},${m.facingDeg}`]));
    }

    /** Odometry's 10ms task: raw odometry from the wheels, and the correction easing in. */
    tick() {
      const c = this.robot.chassis;
      const truth = { x: c.x, y: c.y };
      if (!this.raw) {
        this.raw = { x: truth.x, y: truth.y };
        this.lastTruth = truth;
        return;
      }
      this.raw.x += (truth.x - this.lastTruth.x) * this.scale;
      this.raw.y += (truth.y - this.lastTruth.y) * this.scale;
      this.lastTruth = truth;
      for (const slip of this.slips) {
        if (this.robot.t >= slip.atS && this.robot.t < slip.atS + slip.forS) {
          this.raw.x += slip.dxIn * (0.01 / slip.forS);
          this.raw.y += slip.dyIn * (0.01 / slip.forS);
        }
      }
      // Odometry::setPositionCorrection(): at most maxCorrectionRateInPerS
      const step = L.correctionStep(this.target.x - this.applied.x, this.target.y - this.applied.y,
        MCL_SETTINGS.maxCorrectionRateInPerS * 0.01);
      this.applied.x += step.dxIn;
      this.applied.y += step.dyIn;
    }

    /** The pose every motion drives by: raw odometry plus the correction so far. */
    pose() {
      return { x: this.raw.x + this.applied.x, y: this.raw.y + this.applied.y, heading: this.robot.chassis.heading };
    }

    /** What one distance sensor reads from the true pose, in inches; NaN for nothing. */
    read(index, heading) {
      const c = this.robot.chassis;
      const ray = L.sensorRay(c.x, c.y, heading, MCL_MOUNTS[index]);
      let d = this.map.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
      const t = this.robot.t;
      if (index === 0 && this.defender && t >= this.defender.fromS && t < this.defender.toS) {
        d = Math.min(d, this.defender.distanceIn);
      }
      d += 0.5 * Math.max(0.6, 0.05 * d) * this.noise.gaussian();
      if (this.noise.uniform() < 0.03 || !(d > 0.8)) return NaN;
      return d;
    }

    *task() {
      let lastRaw = null;
      let lastHeading = 0;
      while (true) {
        const robot = this.robot;
        const heading = robot.chassis.heading;
        if (this.raw && !this.started) {
          const p = this.pose();
          this.filter.reset(p.x, p.y, MCL_SETTINGS.startSpreadIn);
          this.started = true;
          lastRaw = { x: this.raw.x, y: this.raw.y };
          lastHeading = heading;
        }
        if (this.started) this.update(lastRaw, lastHeading, heading);
        if (this.raw) lastRaw = { x: this.raw.x, y: this.raw.y };
        lastHeading = heading;
        yield MCL_SETTINGS.periodMs / 1000;
      }
    }

    update(lastRaw, lastHeading, heading) {
      const robot = this.robot;
      const dtS = MCL_SETTINGS.periodMs / 1000;
      const dx = this.raw.x - lastRaw.x;
      const dy = this.raw.y - lastRaw.y;
      const turned = L.wrapDegrees180(heading - lastHeading);
      this.filter.predict(dx, dy, turned);
      const spinning = Math.abs(turned / dtS) > MCL_SETTINGS.maxTurnRateDegPerS;
      const readings = MCL_MOUNTS.map((_, i) => {
        const d = this.read(i, heading);
        return L.reading(Number.isFinite(d) ? d : 0, Number.isFinite(d) && d <= 78 && !spinning);
      });
      const weighed = this.filter.weigh(heading, readings);
      const est = this.filter.estimate();
      const { agreeing, checks } = this.filter.checkSensors(est.xIn, est.yIn, heading, readings,
        MCL_SETTINGS.agreementSigmas);
      const resampled = this.filter.resampleIfNeeded();
      let blocked = 0;
      if (!weighed) blocked |= BLOCKED.noReadings;
      if (spinning) blocked |= BLOCKED.spinning;
      if (!(est.spreadIn <= MCL_SETTINGS.maxCorrectionSpreadIn)) blocked |= BLOCKED.tooSpread;
      if (agreeing < MCL_SETTINGS.minAgreeingSensors) blocked |= BLOCKED.tooFewAgree;
      if (blocked === 0) this.target = { x: est.xIn - this.raw.x, y: est.yIn - this.raw.y };

      const disabledUnderControl = robot.status.mode === 'disabled' && robot.status.comp;
      if (disabledUnderControl || robot.t * 1e6 < robot.openUs) return;
      const w = robot.w;
      const t = robot.us();
      const used = readings.filter((r) => r.valid).length;
      w.sample(this.ids.mcl, t, [est.xIn, est.yIn, est.spreadIn, est.effectiveParticles, used, agreeing,
        blocked === 0 ? 1 : 0, this.target.x, this.target.y, 420 + Math.round(160 * this.noise.uniform()),
        this.raw.x, this.raw.y]);
      const beams = [];
      MCL_MOUNTS.forEach((m, i) => {
        const ray = L.sensorRay(est.xIn, est.yIn, heading, m);
        const expected = checks[i].used ? checks[i].expectedIn : this.map.castRayIn(ray.xIn, ray.yIn, ray.dirX, ray.dirY);
        beams.push(readings[i].valid ? readings[i].distanceIn : NaN, expected,
          (dx * ray.dirX + dy * ray.dirY) / dtS);
      });
      w.sample(this.ids.beams, t, beams);
      const fit = this.filter.recoveryFit();
      w.sample(this.ids.state, t, [est.varianceXIn2, est.varianceYIn2, est.covarianceXYIn2, fit.latest,
        fit.slow > 0 ? fit.fast / fit.slow : 1, this.filter.lastRecovered, resampled ? 1 : 0, blocked, heading]);
      if (this.updates++ % 5 === 0) {
        const sample = L.sampleParticles(this.filter.particles, 24);
        for (let row = 0; row < 4; ++row) {
          const offsets = [];
          for (let i = 0; i < 6; ++i) offsets.push(sample[row * 6 + i].xIn - est.xIn, sample[row * 6 + i].yIn - est.yIn);
          w.sample(this.ids.pts, t, offsets);
        }
      }
    }
  }

  /** A two-motor elevator lift in degrees of its rotation sensor. */
  class Lift {
    constructor({ tempA, tempB, heatA, heatB }) {
      this.model = M.mechanismModel(M.feedforward(0.7, 0.017, 0.002), 2.2, M.gravityShape('constant'));
      this.x = 0;
      this.v = 0;
      this.pending = [];
      this.t = 0;
      this.delayS = 0.02;
      this.volts = 0;
      this.held = true;
      this.piece = false;
      this.a = new Motor({ tempC: tempA, heat: heatA, tauS: 600 });
      this.b = new Motor({ tempC: tempB, heat: heatB, tauS: 600 });
    }

    get kG() {
      // A stack of game pieces is heavy: most of what the lift carries.
      return this.model.kG + (this.piece ? 3.2 : 0);
    }

    command(volts) {
      this.pending.push({ at: this.t + this.delayS, held: false, volts: clamp(volts, -12, 12) });
    }

    brake() {
      this.pending.push({ at: this.t + this.delayS, held: true, volts: 0 });
    }

    step(dt) {
      while (this.pending.length && this.pending[0].at <= this.t + 1e-9) {
        const next = this.pending.shift();
        this.held = next.held;
        this.volts = next.volts;
      }
      const motion = this.model.motion;
      const fa = this.a.fraction;
      const fb = this.b.fraction;
      const rpm = (this.v / 360) * 60 * 3; // sensor at a 3:1 reduction from the motors
      if (this.held) {
        this.v = 0;
        // Hold mode fights gravity with current.
        const holdAmps = (2.5 * (this.kG / 12)) * 0.6;
        this.a.drive(0, 0, holdAmps);
        this.b.drive(0, 0, holdAmps);
      } else {
        const sub = 4;
        const h = dt / sub;
        for (let s = 0; s < sub; ++s) {
          const emf = motion.kV * this.v;
          // Each motor carries half, and can only push its derated share.
          const push = 0.5 * (clamp(this.volts - emf, -12 * fa, 12 * fa) +
            clamp(this.volts - emf, -12 * fb, 12 * fb));
          const load = emf + push - this.kG;
          const friction = Math.abs(this.v) > 1e-3 ? motion.kS * Math.sign(this.v)
            : Math.sign(load) * Math.min(Math.abs(load), motion.kS);
          this.v += ((load - friction - emf) / motion.kA) * h;
          this.x += this.v * h;
          if (this.x < 0) {
            this.x = 0;
            this.v = Math.max(0, this.v);
          }
          if (this.x > 960) {
            this.x = 960;
            this.v = Math.min(0, this.v);
          }
        }
        // Motor A has a tight bearing: it works a little harder.
        this.a.drive(this.volts, rpm, Math.abs(this.volts) > 0.3 ? 0.35 : 0);
        this.b.drive(this.volts, rpm);
      }
      this.a.step(dt);
      this.b.step(dt);
      this.t += dt;
    }
  }

  /** The battery: open-circuit voltage by charge, minus internal resistance. */
  class Battery {
    constructor(pct) {
      this.pct = pct;
      this.amps = 0;
      this.tempC = 29;
    }

    get volts() {
      const ocv = 12.0 + (this.pct / 100) * 1.6;
      return ocv - 0.17 * this.amps;
    }

    step(dt, amps) {
      this.amps = amps + 0.4;
      this.pct -= (this.amps * dt) / (1.1 * 3600) * 100;
      this.tempC += (0.00006 * this.amps * this.amps - (this.tempC - 26) / 900) * dt;
    }
  }

  // --- The robot ----------------------------------------------------------------

  const LEVELS = [0, 150, 300, 450, 600, 750, 900];
  const DRIVE_CONFIG = { gains: { kP: 1.2, kI: 0, kD: 0.001 }, outputLimit: 12, nominalDtS: 0.01 };
  const TURN_CONFIG = { gains: { kP: 0.35, kI: 0, kD: 0.0002 }, outputLimit: 12, nominalDtS: 0.01 };
  const LIFT_CONFIG = { gains: { kP: 0.2, kI: 0, kD: 0.01 }, outputLimit: 12.7,
    derivativeOnMeasurement: true, nominalDtS: 0.02 };

  /** Everything one program run has: the world, the logger, the tasks. */
  class Robot {
    constructor({ seed, file, openUs, build, liftTemps, liftHeat, clawTemp, batteryPct }) {
      this.rng = mulberry32(seed);
      this.t = 0;
      this.w = new Writer({ file, robot: '96671H', openUs, build });
      this.openUs = openUs;
      this.chassis = new Chassis(this.rng);
      this.lift = new Lift({ tempA: liftTemps[0], tempB: liftTemps[1], heatA: liftHeat[0],
        heatB: liftHeat[1] });
      this.claw = new Motor({ tempC: clawTemp, heat: 0.055, tauS: 500 });
      this.intake = new Motor({ tempC: 31, heat: 0.02, tauS: 420 });
      this.battery = new Battery(batteryPct);
      this.tasks = [];
      this.status = { mode: 'opcontrol', comp: false, field: false };

      const w = this.w;
      this.ids = {};
      const pidCols = ['target', 'meas', 'err', 'p', 'i', 'd', 'u_raw', 'out', 'dt', 'flags'];
      this.ids.drive = w.chan('drive', 'pid', 4, pidCols);
      this.ids.turn = w.chan('turn', 'pid', 4, pidCols);
      this.ids.hold = w.chan('hold', 'pid', 4, pidCols);
      this.ids.odom = w.chan('odom', 'samples', 4, ['x', 'y', 'heading', 'raw_x', 'raw_y']);
      this.ids.chassis = w.chan('chassis', 'samples', 3, ['fwd_v', 'strafe_v', 'turn_v']);
      this.ids.batt = w.chan('batt', 'samples', 2, ['volts', 'pct', 'amps', 'temp']);
      const motorCols = ['volts', 'amps', 'temp', 'rpm', 'eff', 'faults'];
      this.motorIds = {};
      for (const name of ['fl', 'fr', 'bl', 'br', 'ml', 'mr', 'liftA', 'liftB', 'intake', 'claw']) {
        this.motorIds[name] = w.chan(`motor.${name}`, 'samples', 2, motorCols);
      }
      this.ids.driver = w.chan('driver', 'samples', 3, ['lx', 'ly', 'rx', 'ry', 'buttons', 'connected']);
      this.ids.lift = w.chan('lift', 'pid', 3, pidCols);
      this.ids.liftAct = w.chan('lift.act', 'samples', 3, ['target', 'pos', 'volts', 'law']);
      this.ids.mech = w.chan('mech', 'samples', 2,
        ['intake_v', 'claw_v', 'piston', 'piece_mm', 'level', 'mode', 'phase', 'deployed']);
      for (const axis of ['fwd', 'strafe', 'turn', 'lift']) {
        this.ids[`char.${axis}`] = w.chan(`char.${axis}`, 'samples', 4, ['volts', 'pos']);
      }

      this.drivePID = new LoggedPID(w, this.ids.drive, DRIVE_CONFIG);
      this.turnPID = new LoggedPID(w, this.ids.turn, TURN_CONFIG);
      this.holdPID = new LoggedPID(w, this.ids.hold, TURN_CONFIG);
      this.liftPID = new LoggedPID(w, this.ids.lift, LIFT_CONFIG);
      this.liftGravity = { constantVolts: 0, cosineVolts: 0, horizontalPosition: 0,
        armDegreesPerUnit: 1 };
      this.liftExternal = false;
      this.liftTestTarget = NaN;

      // Driver/macro state.
      this.sticks = [0, 0, 0, 0];
      this.buttons = 0;
      this.connected = 1;
      this.level = 0;
      this.deployed = false;
      this.phase = -1;
      this.intakeV = 0;
      this.clawV = 0;
      this.heldHeading = 0;
      this.lastHoldT = null;
      this.screenBusy = false;
      this.driving = true;
      this.chassisIdle = true;
      this.pendingEvents = [];
    }

    us(t = this.t) {
      return Math.round(t * 1e6);
    }

    event(tag, msg) {
      this.w.event(this.us(), tag, msg);
    }

    spawn(name, generator) {
      this.tasks.push({ name, gen: generator, wake: this.t });
    }

    kill(name) {
      this.tasks = this.tasks.filter((task) => task.name !== name);
    }

    enabled() {
      return this.status.mode !== 'disabled';
    }

    setStatus(mode, comp, field) {
      this.status = { mode, comp, field };
      this.event('phase', `${mode},comp=${comp ? 1 : 0},field=${field ? 1 : 0}`);
    }

    run(until) {
      while (this.t < until - 1e-9) {
        for (const task of this.tasks.slice()) {
          if (task.wake > this.t + 1e-9) continue;
          const next = task.gen.next();
          if (next.done) {
            this.tasks.splice(this.tasks.indexOf(task), 1);
          } else {
            task.wake = this.t + next.value;
          }
        }
        this.physics(0.001);
        this.t = Math.round((this.t + 0.001) * 1e6) / 1e6;
      }
    }

    physics(dt) {
      if (this.enabled()) {
        this.chassis.step(dt);
        this.lift.step(dt);
      } else {
        // VEXos ignores the motors while disabled: everything coasts to rest.
        this.chassis.coast();
        this.chassis.step(dt);
        this.lift.brake();
        this.lift.step(dt);
      }
      this.claw.step(dt);
      this.intake.step(dt);
      const amps = Object.values(this.chassis.motors).reduce((s, m) => s + m.amps, 0) +
        this.lift.a.amps + this.lift.b.amps + this.claw.amps + this.intake.amps;
      this.battery.step(dt, amps);
    }

    // --- Logger tasks ---

    *sampler() {
      let lastMotors = -1;
      let lastBatt = -1;
      while (true) {
        if (this.localizer) this.localizer.tick();
        const disabledUnderControl = this.status.mode === 'disabled' && this.status.comp;
        if (!disabledUnderControl && this.t * 1e6 >= this.openUs) {
          const noise = 0.004 * gaussian(this.rng);
          const pose = this.localizer ? this.localizer.pose() : this.chassis.pose(0);
          const raw = this.localizer ? this.localizer.raw : pose;
          this.w.sample(this.ids.odom, this.us(), [pose.x + noise, pose.y + noise, pose.heading,
            raw.x + noise, raw.y + noise]);
          const a = this.chassis.applied;
          this.w.sample(this.ids.chassis, this.us(), [a.forward, a.strafe, a.turn]);
          if (this.t - lastBatt >= 0.199) {
            lastBatt = this.t;
            const b = this.battery;
            this.w.sample(this.ids.batt, this.us(), [b.volts + 0.01 * gaussian(this.rng),
              Math.round(b.pct), b.amps + 0.05 * gaussian(this.rng), Math.round(b.tempC)]);
          }
          if (this.t - lastMotors >= 0.099) {
            lastMotors = this.t;
            for (const [name, motor] of Object.entries(this.motorsByName())) {
              this.w.sample(this.motorIds[name], this.us() + 40, motor.row(this.rng));
            }
          }
        }
        yield 0.01;
      }
    }

    motorsByName() {
      const c = this.chassis.motors;
      return { fl: c.fl, fr: c.fr, bl: c.bl, br: c.br, ml: c.ml, mr: c.mr, liftA: this.lift.a,
        liftB: this.lift.b, intake: this.intake, claw: this.claw };
    }

    *healthTask() {
      while (true) {
        this.w.health(this.us());
        yield 1.0;
      }
    }

    // --- Driver control: one opcontrol tick every 20ms ---

    *opcontrol(plan) {
      while (true) {
        if (this.status.mode === 'opcontrol') {
          if (plan) plan.call(this);
          this.logDriver();
          this.macros();
          if (!this.screenBusy && this.driving) this.drive();
        }
        yield 0.02;
      }
    }

    logDriver() {
      this.w.sample(this.ids.driver, this.us(), [this.sticks[0], this.sticks[1], this.sticks[2],
        this.sticks[3], this.connected ? this.buttons : 0, this.connected]);
    }

    drive() {
      const [lx, ly, rx] = this.connected ? this.sticks : [0, 0, 0];
      const now = this.t;
      const heading = this.chassis.turn.x;
      const dt = this.lastHoldT === null ? 0.02 : now - this.lastHoldT;
      if (this.lastHoldT === null || dt > 0.35) {
        this.heldHeading = heading;
        this.holdPID.reset(now);
      }
      this.lastHoldT = now;
      this.heldHeading = clamp(this.heldHeading + rx * 300 * dt, heading - 40, heading + 40);
      const turn = this.holdPID.update(now, wrap180(this.heldHeading - heading), 0, dt);
      const h = (heading * Math.PI) / 180;
      const forward = lx * Math.sin(h) + ly * Math.cos(h);
      const lateral = lx * Math.cos(h) - ly * Math.sin(h);
      if (Math.abs(lx) + Math.abs(ly) < 0.02 && Math.abs(turn) < 0.3) {
        this.chassis.holonomicVolts(0, 0, turn);
      } else {
        this.chassis.holonomicVolts(forward * 12, lateral * 12, turn);
      }
    }

    liftTarget() {
      if (Number.isFinite(this.liftTestTarget)) return this.liftTestTarget;
      if (this.phase === 0) return 0.5 * (LEVELS[this.level] + LEVELS[Math.max(0, this.level - 1)]);
      return LEVELS[this.level];
    }

    macros() {
      const target = this.liftTarget();
      let law;
      let volts;
      if (this.liftExternal) {
        this.liftPID.reset(this.t);
        law = M.LAW.external;
        volts = NaN;
      } else {
        const config = { gravity: this.liftGravity,
          seat: { enabled: true, floor: 0, seatVolts: 0, restBand: 2 }, maxVolts: 12 };
        const reading = this.lift.x + 0.05 * gaussian(this.rng);
        const command = this.computeLift(config, target, reading);
        law = command.law;
        volts = command.volts;
        if (command.brake) this.lift.brake();
        else this.lift.command(volts);
      }
      this.w.sample(this.ids.liftAct, this.us() + 60, [target, this.lift.x, volts, law]);
      this.claw.drive(this.clawV, this.clawV === 0 ? 0 : (this.lift.piece ? 0 : 180),
        this.lift.piece && this.clawV > 0 ? 2.4 : 0);
      this.intake.drive(this.intakeV, this.intakeV === 0 ? 0 : 190);
      this.w.sample(this.ids.mech, this.us() + 80, [this.intakeV, this.clawV, this.deployed ? 1 : 0,
        this.lift.piece ? 32 : 9999, this.level, 0, this.phase, this.deployed ? 1 : 0]);
    }

    computeLift(config, target, reading) {
      // model.js's port of computePositionCommand(), on the logged PID.
      const self = this;
      const pid = {
        update(tgt, meas) {
          return self.liftPID.update(self.t + 0.00005, tgt, meas);
        },
        reset() {
          self.liftPID.reset(self.t);
          return true;
        },
      };
      return M.computePositionCommand(pid, config, target, reading);
    }

    // --- Motions (holonomic_drivetrain.cpp's loops) ---

    *exitLoop(kind, startMsg, body, exit) {
      this.event('motion', `start,${kind},${startMsg}`);
      const start = this.t;
      let within = null;
      let reason = 'running';
      let finalError = 0;
      while (true) {
        const { error, done } = body();
        finalError = Math.abs(error);
        const inside = done !== undefined ? done : finalError <= exit.threshold;
        if (inside) {
          if (within === null) within = this.t;
        } else {
          within = null;
        }
        if (within !== null && (this.t - within) * 1000 >= exit.settleMs) reason = 'settled';
        else if (exit.timeoutMs > 0 && (this.t - start) * 1000 >= exit.timeoutMs) reason = 'timeout';
        if (reason !== 'running') break;
        yield 0.01;
      }
      this.chassis.stop();
      const ms = Math.round((this.t - start) * 1000);
      this.event('motion', `end,${kind},reason=${reason},error=${finalError.toFixed(3)},ms=${ms}`);
      return reason;
    }

    *moveToPoint(x, y, exit = { threshold: 1, settleMs: 200, timeoutMs: 3000 }) {
      const hold = this.chassis.heading;
      this.drivePID.reset(this.t);
      this.turnPID.reset(this.t);
      return yield* this.exitLoop('moveToPoint',
        `x=${x.toFixed(3)},y=${y.toFixed(3)},hold_deg=${hold.toFixed(3)},threshold=` +
        `${exit.threshold.toFixed(3)},settle_ms=${exit.settleMs},timeout_ms=${exit.timeoutMs}`,
        () => this.pointStep(x, y, hold), exit);
    }

    pointStep(x, y, hold) {
      const pose = this.chassis.pose(0.003 * gaussian(this.rng));
      const dx = x - pose.x;
      const dy = y - pose.y;
      const distance = Math.hypot(dx, dy);
      const out = this.drivePID.update(this.t, distance, 0);
      const h = (pose.heading * Math.PI) / 180;
      const forward = dx * Math.sin(h) + dy * Math.cos(h);
      const lateral = dx * Math.cos(h) - dy * Math.sin(h);
      const scale = distance > 1e-6 ? out / distance : 0;
      const turn = this.turnPID.update(this.t + 0.0001, wrap180(hold - pose.heading), 0);
      this.chassis.holonomicVolts(forward * scale, lateral * scale, turn);
      return { error: distance };
    }

    *turnToHeading(target, exit = { threshold: 2, settleMs: 200, timeoutMs: 3000 }) {
      this.turnPID.reset(this.t);
      return yield* this.exitLoop('turnToHeading',
        `target_deg=${target.toFixed(3)},threshold=${exit.threshold.toFixed(3)},settle_ms=` +
        `${exit.settleMs},timeout_ms=${exit.timeoutMs}`,
        () => {
          const error = wrap180(target - this.chassis.heading);
          const out = this.turnPID.update(this.t, error, 0);
          this.chassis.setWheels(out, -out, out, -out);
          return { error };
        }, exit);
    }

    *driveDistance(inches, exit = { threshold: 1, settleMs: 200, timeoutMs: 3000 }) {
      const startHeading = this.chassis.heading;
      const h = (startHeading * Math.PI) / 180;
      const x0 = this.chassis.x;
      const y0 = this.chassis.y;
      this.drivePID.reset(this.t);
      return yield* this.exitLoop('driveDistance',
        `target_in=${inches.toFixed(3)},threshold=${exit.threshold.toFixed(3)},settle_ms=` +
        `${exit.settleMs},timeout_ms=${exit.timeoutMs}`,
        () => {
          const traveled = (this.chassis.x - x0) * Math.sin(h) + (this.chassis.y - y0) * Math.cos(h);
          const out = this.drivePID.update(this.t, inches, traveled);
          const correction = 0.4 * wrap180(startHeading - this.chassis.heading);
          this.chassis.setWheels(out + correction, out - correction, out + correction, out - correction);
          return { error: inches - traveled };
        }, exit);
    }

    *moveToPose(x, y, heading, exit = { threshold: 1, headingThreshold: 2, settleMs: 200,
      timeoutMs: 3000 }) {
      this.drivePID.reset(this.t);
      this.turnPID.reset(this.t);
      return yield* this.exitLoop('moveToPose',
        `x=${x.toFixed(3)},y=${y.toFixed(3)},heading_deg=${heading.toFixed(3)},pos_threshold=` +
        `${exit.threshold.toFixed(3)},heading_threshold=${exit.headingThreshold.toFixed(3)},` +
        `settle_ms=${exit.settleMs},timeout_ms=${exit.timeoutMs}`,
        () => {
          const { error } = this.pointStep(x, y, heading);
          const headingError = Math.abs(wrap180(heading - this.chassis.heading));
          return { error, done: error <= exit.threshold && headingError <= exit.headingThreshold };
        }, exit);
    }

    *followPath(points, lookahead, cruise, timeoutMs) {
      this.event('motion', `start,followPath,waypoints=${points.length},lookahead_in=` +
        `${lookahead.toFixed(3)},cruise_v=${cruise.toFixed(3)},timeout_ms=${timeoutMs}`);
      const hold = this.chassis.heading;
      this.turnPID.reset(this.t);
      let index = 0;
      while (index < points.length - 1) {
        const [px, py] = points[index + 1];
        const pose = this.chassis.pose(0);
        const dx = px - pose.x;
        const dy = py - pose.y;
        const d = Math.hypot(dx, dy);
        if (d < lookahead) {
          index++;
          continue;
        }
        const h = (pose.heading * Math.PI) / 180;
        const forward = (dx * Math.sin(h) + dy * Math.cos(h)) / d;
        const lateral = (dx * Math.cos(h) - dy * Math.sin(h)) / d;
        const turn = this.turnPID.update(this.t, wrap180(hold - pose.heading), 0);
        this.chassis.holonomicVolts(forward * cruise, lateral * cruise, turn);
        yield 0.01;
      }
      this.chassis.stop();
      return 'settled';
    }

    // --- Auto-Tune (characterization_runner.cpp, with the tap) ---

    *characterize(axis, channelId, config, mechanism = null) {
      const samples = { ramps: [], steps: [] };
      const tap = { fresh: false, position: NaN };
      const measure = () => {
        tap.position = config.measure();
        tap.fresh = true;
        return tap.position;
      };
      const log = (volts) => {
        if (!tap.fresh) return;
        tap.fresh = false;
        this.w.sample(channelId, this.us(), [volts, tap.position]);
      };
      const actuate = (volts) => {
        config.actuate(volts);
        log(volts);
      };
      const hold = mechanism ? () => {
        mechanism.hold();
        log(NaN);
      } : () => actuate(0);
      const threshold = mechanism ? Math.max(Math.abs(mechanism.upper - mechanism.lower) * 0.002, 0.01)
        : Math.max(config.maxTravel > 0 ? config.maxTravel * 0.002 : 0, 0.01);
      const wait = function* (self) {
        hold();
        const start = self.t;
        let last = measure();
        while ((self.t - start) * 1000 < 1500) {
          yield 0.1;
          const now = measure();
          if (Math.abs(now - last) < threshold) return;
          last = now;
        }
      };
      const segment = function* (self, outOfRange, voltsAt) {
        const run = [];
        const startT = self.t;
        const origin = measure();
        while (true) {
          const elapsed = Math.round((self.t - startT) * 1000);
          if (elapsed >= 100 + 2500) break;
          const position = measure();
          if (!Number.isFinite(position) || outOfRange(position, origin)) break;
          let volts;
          if (elapsed < 100) {
            if (mechanism) {
              hold();
              volts = NaN;
            } else {
              actuate(0);
              volts = 0;
            }
          } else {
            volts = voltsAt((elapsed - 100) / 1000);
            actuate(volts);
          }
          run.push({ timeMs: elapsed, volts, position });
          yield 0.01;
        }
        hold();
        return run;
      };
      const step = config.stepVolts;
      const plan = [[1, true], [-1, true], [1, false], [-1, false]];
      for (const [direction, ramp] of plan) {
        yield* wait(this);
        const voltsAt = (t) => direction * (ramp ? Math.min(config.rampRate * t, config.rampMax)
          : (direction < 0 && mechanism && mechanism.downStep ? mechanism.downStep : step));
        const outOfRange = mechanism
          ? (p) => (direction > 0 ? p >= mechanism.upper : p <= mechanism.lower)
          : (p, origin) => config.maxTravel > 0 && Math.abs(p - origin) >= config.maxTravel;
        const run = yield* segment(this, outOfRange, voltsAt);
        (ramp ? samples.ramps : samples.steps).push(run);
      }
      yield* wait(this);
      return samples;
    }

    // --- The pit session: Auto-Tune, then Run Tests ---

    *pitSession() {
      yield 4;
      this.screenBusy = true;
      const experiments = {
        Fwd: { id: this.ids['char.fwd'], plant: this.chassis.fwd, apply: (v) => this.chassis.holonomicVolts(v, 0, 0),
          minSpeed: 2, travel: 30 },
        Strafe: { id: this.ids['char.strafe'], plant: this.chassis.strafe,
          apply: (v) => this.chassis.holonomicVolts(0, v, 0), minSpeed: 2, travel: 30 },
        Turn: { id: this.ids['char.turn'], plant: this.chassis.turn, apply: (v) => this.chassis.holonomicVolts(0, 0, v),
          minSpeed: 5, travel: 0 },
      };
      const models = {};
      for (const [name, e] of Object.entries(experiments)) {
        this.event('tune', `start,${name}`);
        const origin = e.plant.x;
        const data = yield* this.characterize(name, e.id, {
          measure: () => e.plant.x - origin + 0.003 * gaussian(this.rng),
          actuate: e.apply, stepVolts: 6, rampRate: 4, rampMax: 8, maxTravel: e.travel,
        });
        const result = M.characterizeAxis(data, e.minSpeed);
        models[name] = result;
        const m = result.fit.model;
        this.event('tune', `model,${name},kS=${m.kS.toFixed(4)},kV=${m.kV.toFixed(5)},kA=` +
          `${m.kA.toFixed(5)},r2=${result.fit.rSquared.toFixed(3)},delay_ms=${(result.delayS * 1000).toFixed(0)}`);
        yield 0.5;
      }
      const design = (name, spec, loopS = 0.01) => M.designPositionGains(models[name].fit.model, spec,
        models[name].delayS + 0.5 * Math.max(0, loopS - 0.01));
      this.drivePID.setGains(design('Fwd', M.responseSpec(0.6)).gains);
      this.turnPID.setGains(design('Turn', M.responseSpec(0.5)).gains);
      this.holdPID.setGains(design('Turn', M.responseSpec(0.9), 0.02).gains);

      // Run Test: Drive, then Turn, with the new gains.
      yield 1.5;
      yield* this.driveDistance(24);
      yield* this.driveDistance(-24);
      yield 1;
      yield* this.turnToHeading(90);
      yield* this.turnToHeading(0);
      this.screenBusy = false;
      yield 3;

      // Lift Auto-Tune: the lift's loop stands aside (law external).
      this.screenBusy = true;
      this.event('tune', 'start,Lift');
      this.liftExternal = true;
      const lift = this.lift;
      const data = yield* this.characterize('Lift', this.ids['char.lift'], {
        measure: () => lift.x + 0.05 * gaussian(this.rng),
        actuate: (v) => lift.command(v), stepVolts: 8, rampRate: 6, rampMax: 10, maxTravel: 0,
      }, { hold: () => lift.brake(), lower: 60, upper: 720, downStep: 4 });
      const result = M.characterizeMechanism(data, M.gravityShape('constant'), 10);
      const m = result.fit.model;
      this.event('tune', `model,Lift,kS=${m.motion.kS.toFixed(4)},kV=${m.motion.kV.toFixed(5)},kA=` +
        `${m.motion.kA.toFixed(5)},kG=${m.kG.toFixed(4)},r2=${result.fit.rSquared.toFixed(3)},` +
        `delay_ms=${(result.delayS * 1000).toFixed(0)}`);
      this.liftGravity = M.gravityFeedforwardOf(m);
      const liftDesign = M.designPositionGains(m.motion, M.responseSpec(0.5), result.delayS + 0.005);
      this.liftPID.setGains(liftDesign.gains);
      this.liftExternal = false;
      this.screenBusy = false;
      yield 2;

      // Lift Run Test: up to 450, down to 150, through its own loop.
      this.screenBusy = true;
      for (const target of [450, 150]) {
        this.liftTestTarget = target;
        yield 2.5;
      }
      this.liftTestTarget = NaN;
      this.screenBusy = false;
    }

    // --- The match ---

    *autonomous() {
      this.event('auton', 'start,Left AWP');
      yield* this.moveToPoint(24, 54, { threshold: 1, settleMs: 200, timeoutMs: 2000 });
      yield* this.turnToHeading(90, { threshold: 2, settleMs: 200, timeoutMs: 1500 });
      yield* this.driveDistance(24, { threshold: 1, settleMs: 200, timeoutMs: 2000 });
      yield* this.moveToPose(72, 78, 45, { threshold: 1, headingThreshold: 2, settleMs: 200,
        timeoutMs: 3000 });
      // Pinned against a goal: it can't turn past ~118°.
      this.chassis.blockHeadingAbove = this.chassis.turn.x + 73;
      yield* this.turnToHeading(180, { threshold: 2, settleMs: 200, timeoutMs: 3000 });
      this.chassis.blockHeadingAbove = null;
      yield* this.followPath([[72, 78], [60, 100], [40, 112], [24, 124]], 12, 8, 5000);
      this.event('auton', 'end,Left AWP');
    }

    /** One scripted driver: cycles of intake → score, with the planned mishaps. */
    driverPlan(start) {
      const intakeSpot = [30, 30, 225];
      const goals = [[112, 112, 45], [112, 40, 90], [96, 124, 0]];
      const levels = [2, 3, 4, 5, 6, 6, 6, 5];
      const state = { cycle: 0, stage: 'toIntake', stageT: start, presses: 0 };
      const B = { l1: 1, l2: 2, r1: 4, r2: 8 };
      return function () {
        const t = this.t - start;
        const since = this.t - state.stageT;
        this.buttons = 0;
        // Planned mishaps: a drive motor cable, a controller drop.
        const br = this.chassis.motors.br;
        const brOut = t >= 48 && t < 50.4;
        if (brOut && br.connected) this.event('device', 'lost,port=10,label=Back-right drive motor,found=nothing plugged in');
        if (!brOut && !br.connected) this.event('device', 'back,port=10,label=Back-right drive motor');
        br.connected = !brOut;
        this.connected = t >= 95 && t < 95.8 ? 0 : 1;

        const goal = goals[state.cycle % goals.length];
        const target = state.stage === 'toIntake' || state.stage === 'intake' ? intakeSpot : goal;
        const dx = target[0] - this.chassis.x;
        const dy = target[1] - this.chassis.y;
        const d = Math.hypot(dx, dy);
        const speed = Math.min(1, d / 26) * (d > 2 ? 1 : 0);
        this.sticks[0] = d > 0 ? (dx / d) * speed : 0;
        this.sticks[1] = d > 0 ? (dy / d) * speed : 0;
        this.sticks[2] = clamp(wrap180(target[2] - this.heldHeading) / 45, -1, 1);
        this.sticks[3] = 0;

        const nextStage = (stage) => {
          state.stage = stage;
          state.stageT = this.t;
        };
        const want = levels[state.cycle % levels.length];
        switch (state.stage) {
          case 'toIntake':
            this.buttons |= since < 0.3 ? B.r1 : 0;
            if (since < 0.3) {
              this.level = 0;
              this.deployed = false;
              this.phase = -1;
            }
            this.intakeV = 0;
            this.clawV = this.lift.piece ? 12.7 : 0;
            if (d < 4) nextStage('intake');
            break;
          case 'intake':
            this.buttons |= B.r1;
            this.intakeV = 12.7;
            this.clawV = 12.7;
            if (since > 1.4) {
              this.lift.piece = true;
              state.presses = 0;
              nextStage('toGoal');
            }
            break;
          case 'toGoal':
            this.intakeV = 0;
            this.clawV = 12.7; // holding the piece: stalled against it
            if (d < 50 && state.presses <= want && since - state.presses * 0.35 > 0.6) {
              this.buttons |= B.l1;
              if (!this.deployed) this.deployed = true;
              else this.level = Math.min(this.level + 1, want);
              state.presses++;
            }
            if (d < 4 && (Math.abs(this.lift.x - LEVELS[this.level]) < 25 || since > 7)) nextStage('lineUp');
            break;
          case 'lineUp':
            this.clawV = 12.7;
            if (since > 1.1) nextStage('score');
            break;
          case 'score':
            if (since < 0.05) this.buttons |= B.r2;
            if (since < 0.9) {
              this.phase = 0; // dipping halfway to the level below
            } else if (since < 1.4) {
              this.phase = 1;
              this.clawV = -12.7;
            } else {
              this.phase = -1;
              this.lift.piece = false;
              this.clawV = 0;
              this.level = Math.min(this.level + 1, 6);
              state.cycle++;
              nextStage('toIntake');
            }
            break;
          default:
            break;
        }
      };
    }
  }

  // --- The two logs -------------------------------------------------------------

  function pitSession(seed = 41) {
    const robot = new Robot({ seed, file: 'SL000041.CSV', openUs: 2103877,
      build: 'Sep 29 2026 09:41:52', liftTemps: [31, 30], liftHeat: [0.05, 0.04], clawTemp: 30,
      batteryPct: 96 });
    robot.localizer = new DemoLocalizer(robot, { seed: 41 });
    robot.run(2.1);
    robot.event('file', 'open,SL000041.CSV');
    robot.setStatus('opcontrol', false, false);
    robot.spawn('sampler', robot.sampler());
    robot.spawn('localizer', robot.localizer.task());
    robot.spawn('health', robot.healthTask());
    robot.spawn('opcontrol', robot.opcontrol(null));
    robot.spawn('pit', robot.pitSession());
    robot.run(76);
    return robot.w.toString(robot.rng);
  }

  function match(seed = 42, inspect = null) {
    const robot = new Robot({ seed, file: 'SL000042.CSV', openUs: 2104502,
      build: 'Sep 29 2026 09:41:52', liftTemps: [45, 42], liftHeat: [0.34, 0.31], clawTemp: 38,
      batteryPct: 91 });
    // driver 0:30, a hit lifts the tracking wheels and they miss 6in; driver 0:50 to 0:58, a robot
    // parks 16in in front of the front distance sensor
    robot.localizer = new DemoLocalizer(robot, { seed: 42, slips: [{ atS: 67.5, forS: 0.25, dxIn: -4.8, dyIn: 3.6 }],
      defender: { fromS: 87.5, toS: 95.5, distanceIn: 16 } });
    robot.run(2.1);
    robot.event('file', 'open,SL000042.CSV');
    robot.setStatus('disabled', true, true);
    robot.spawn('sampler', robot.sampler());
    robot.spawn('localizer', robot.localizer.task());
    robot.spawn('health', robot.healthTask());
    robot.run(20);
    robot.setStatus('autonomous', true, true);
    robot.spawn('auton', robot.autonomous());
    robot.run(35);
    robot.kill('auton'); // PROS deletes the autonomous task at the phase change
    robot.chassis.stop();
    robot.setStatus('disabled', true, true);
    robot.run(37.5);
    robot.setStatus('opcontrol', true, true);
    const plan = robot.driverPlan(37.5);
    robot.spawn('opcontrol', robot.opcontrol(plan));
    robot.run(80);
    robot.w.drops(robot.us(), 1, 0, 1);
    if (inspect) {
      for (let t = 90; t <= 140; t += 10) {
        robot.run(t);
        inspect(robot);
      }
    }
    robot.run(142.5);
    robot.kill('opcontrol');
    robot.setStatus('disabled', true, true);
    robot.run(150);
    return robot.w.toString(robot.rng);
  }

  return { pitSession, match, fmtNumber, fmtG9, mulberry32, Writer, LoggedPID };
});
