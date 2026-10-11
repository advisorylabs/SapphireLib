/*
 * SapphireLib simulator: recorder.js
 *
 * Writes a simulated run as the robot would log it (docs/TELEMETRY_FORMAT.md):
 * the same channels src/robot/telemetry.cpp records for odometry and the
 * localizer ("odom", "chassis", "mcl", "mcl.beams", "mcl.state", "mcl.pts"),
 * the same #meta lines
 * (the localizer's settings and sensor mounts), formatted by the analyzer's
 * own writer (demo.js), which rounds every value the way the robot's encoder
 * does.
 *
 * Two uses: the simulator's Tune tab can export a run for the analyzer, and
 * the tests calibrate from a log whose world is known exactly, which is how
 * mclcal.js is checked against the truth.
 *
 * Plain script: window.SIM.recorder in a browser, require('./recorder.js') in
 * Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('../../analyzer/js/demo.js'), require('../../analyzer/js/tunefile.js'),
      require('./mcl.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.recorder = factory(root.SA.demo, root.SA.tunefile, root.SIM.mcl);
  }
})(function (D, TF, L) {
  'use strict';

  const kStartUs = 2000000; // as if the program had been running for 2s, like a real log

  const MCL_COLUMNS = ['x', 'y', 'spread', 'neff', 'used', 'agree', 'correcting', 'corr_x', 'corr_y', 'us',
    'raw_x', 'raw_y'];
  const BEAM_COLUMNS = ['m0', 'e0', 'v0', 'm1', 'e1', 'v1', 'm2', 'e2', 'v2', 'm3', 'e3', 'v3'];
  const STATE_COLUMNS = ['sxx', 'syy', 'sxy', 'fit', 'fit_ratio', 'recovered', 'resampled', 'blocked', 'hdg'];
  const POINT_COLUMNS = ['dx0', 'dy0', 'dx1', 'dy1', 'dx2', 'dy2', 'dx3', 'dy3', 'dx4', 'dy4', 'dx5', 'dy5'];
  // "mcl.pts" as src/robot/telemetry.cpp records it: every 5th update, four rows of six particles
  const kParticleEvery = 5;
  const kParticleRows = 4;
  const kParticlesPerRow = 6;

  function getPath(object, path) {
    return path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), object);
  }

  function metaNumber(value) {
    if (typeof value === 'boolean') return value ? '1' : '0';
    return Number.isInteger(value) ? String(value) : String(Number(value.toPrecision(9)));
  }

  /**
   * Records a Simulation from now on. Set it as `sim.recorder` (the
   * simulation calls afterTick() every tick), or call afterTick() yourself.
   * `mode` is the phase the run is logged under: 'autonomous' for a routine,
   * 'opcontrol' for driving.
   */
  class LogRecorder {
    constructor(sim, { fileIndex = 900001, robot = 'sim', mode = 'autonomous', name = null } = {}) {
      const file = `SL${String(fileIndex).padStart(6, '0')}.CSV`;
      this.sim = sim;
      this.t0Ms = sim.world.timeMs;
      this.writer = new D.Writer({ file, robot, openUs: kStartUs, build: 'Sapphire Sim' });
      this.file = file;
      const loc = sim.robot.localizer;
      for (const s of TF.LOCALIZER_SETTINGS) {
        const value = getPath(loc.config, s.key);
        if (value !== undefined) this.writer.meta.push([`mcl.${s.key}`, metaNumber(value)]);
      }
      this.writer.meta.push(['mcl.periodMs', String(sim.robot.localizerPeriodMs)]);
      loc.filter.sensors.forEach((m, i) => {
        this.writer.meta.push([`mcl.sensor${i}`, `${metaNumber(m.forwardIn)},${metaNumber(m.rightIn)},` +
          `${metaNumber(m.facingDeg)}`]);
      });
      this.writer.meta.push(['tune', 'none']);
      this.odom = this.writer.chan('odom', 'samples', 4, ['x', 'y', 'heading', 'raw_x', 'raw_y']);
      this.chassis = this.writer.chan('chassis', 'samples', 3, ['fwd_v', 'strafe_v', 'turn_v']);
      this.mcl = this.writer.chan('mcl', 'samples', 2, MCL_COLUMNS);
      this.beams = this.writer.chan('mcl.beams', 'samples', 2, BEAM_COLUMNS);
      this.state = this.writer.chan('mcl.state', 'samples', 4, STATE_COLUMNS);
      this.points = this.writer.chan('mcl.pts', 'samples', 2, POINT_COLUMNS);
      this.mclUpdates = 0;
      const t = this.us(sim.world.timeMs);
      this.writer.event(t, 'file', `open,${file}`);
      this.writer.event(t, 'phase', `${mode},comp=0,field=0`);
      this.writer.event(t, 'rec', 'start,manual');
      if (name) this.writer.event(t, 'auton', `start,${name}`);
      this.lastUpdates = loc.status.updates;
      this.lastHealthMs = sim.world.timeMs;
    }

    us(timeMs) {
      return kStartUs + (timeMs - this.t0Ms) * 1000;
    }

    afterTick(sim) {
      const timeMs = sim.world.timeMs;
      const t = this.us(timeMs);
      const snapshot = sim.robot.odometry.snapshot();
      const pose = snapshot.pose;
      this.writer.sample(this.odom, t, [pose.xIn, pose.yIn, pose.headingDeg, snapshot.rawPose.xIn, snapshot.rawPose.yIn]);
      const a = sim.robot.drivetrain.applied;
      this.writer.sample(this.chassis, t, [a.forward, a.strafe, a.turn]);

      const loc = sim.robot.localizer;
      if (loc.status.updates !== this.lastUpdates && loc.work && loc.work.estimate) {
        this.lastUpdates = loc.status.updates;
        const st = loc.status;
        const raw = loc.work.raw;
        this.writer.sample(this.mcl, t, [st.estimate.xIn, st.estimate.yIn, st.spreadIn, st.effectiveParticles,
          st.sensorsUsed, st.sensorsAgreeing, st.correcting ? 1 : 0, st.correctionXIn, st.correctionYIn, 0,
          raw.xIn, raw.yIn]);
        const values = [];
        const beams = loc.beamSamples();
        for (let i = 0; i < 4; ++i) {
          const b = beams[i];
          values.push(b ? b.measuredIn : NaN, b ? b.expectedIn : NaN, b ? b.closingSpeedInPerS : NaN);
        }
        this.writer.sample(this.beams, t, values);

        const w = loc.work;
        const est = w.estimate;
        const fit = w.fit;
        this.writer.sample(this.state, t, [est.varianceXIn2, est.varianceYIn2, est.covarianceXYIn2, fit.latest,
          fit.slow > 0 ? fit.fast / fit.slow : 1, w.recovered, w.resampled ? 1 : 0, w.blockedBy, w.beamHeadingDeg]);
        if (this.mclUpdates++ % kParticleEvery === 0) {
          const sample = L.sampleParticles(loc.filter.particles, kParticleRows * kParticlesPerRow);
          for (let row = 0; row < kParticleRows; ++row) {
            const offsets = [];
            for (let i = 0; i < kParticlesPerRow; ++i) {
              const p = sample[row * kParticlesPerRow + i];
              offsets.push(p.xIn - est.xIn, p.yIn - est.yIn);
            }
            this.writer.sample(this.points, t, offsets);
          }
        }
      }
      if (timeMs - this.lastHealthMs >= 1000) {
        this.lastHealthMs = timeMs;
        this.writer.health(t);
      }
    }

    /** The log so far, as an SLT file's text. */
    text() {
      const t = this.us(this.sim.world.timeMs);
      const copy = Object.create(Object.getPrototypeOf(this.writer));
      Object.assign(copy, this.writer, { rows: this.writer.rows.slice(), healthTimes: this.writer.healthTimes.slice() });
      copy.event(t, 'rec', 'stop,manual');
      return copy.toString(D.mulberry32(this.file.length));
    }
  }

  return { LogRecorder, MCL_COLUMNS, BEAM_COLUMNS, STATE_COLUMNS, POINT_COLUMNS };
});
