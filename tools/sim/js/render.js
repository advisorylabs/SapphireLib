/*
 * SapphireLib simulator: render.js
 *
 * Draws the field and everything on it onto a canvas: the robot where it
 * really is, where raw odometry and the corrected pose think it is, the
 * particle cloud (colored by how well each particle explains the readings,
 * sized by its weight), the estimate's uncertainty ellipse, the sensor beams,
 * what the map says each sensor should read from the estimate, and the
 * running motion's target. Plus the error chart under the field.
 *
 * Field frame as odom::Pose: origin in the middle, +x right, +y up the
 * screen (downfield), headings clockwise from +y. Colors come from the page's
 * CSS tokens, so both themes work.
 *
 * Browser only: window.SIM.render.
 *
 * Team 96671H: Hitmen
 */
(function () {
  'use strict';

  const root = typeof self !== 'undefined' ? self : this;
  const L = root.SIM.mcl;
  const kDegToRad = L.kDegToRad;

  function cssVar(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }

  function parseColor(text) {
    const probe = document.createElement('canvas').getContext('2d');
    probe.fillStyle = text;
    const hex = probe.fillStyle;
    if (hex.startsWith('#')) {
      return [parseInt(hex.slice(1, 3), 16), parseInt(hex.slice(3, 5), 16), parseInt(hex.slice(5, 7), 16)];
    }
    const m = hex.match(/[\d.]+/g) || [0, 0, 0];
    return [+m[0], +m[1], +m[2]];
  }

  function mix(a, b, t) {
    return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
  }

  function rgba(c, alpha = 1) {
    return `rgba(${c[0] | 0}, ${c[1] | 0}, ${c[2] | 0}, ${alpha})`;
  }

  /** The page's colors, read once per theme. */
  function palette() {
    const p = {};
    for (const name of ['bg', 'surface', 'surface-2', 'ink', 'ink-2', 'muted', 'grid', 'axis', 'accent',
      'good', 'warning', 'critical', 'series-1', 'series-2', 'series-3', 'series-4', 'series-5', 'series-7',
      'series-8', 'tile', 'tile-line', 'wall', 'element', 'robot-fill', 'robot-line', 'raw', 'corrected', 'beam']) {
      p[name] = cssVar(`--${name}`);
    }
    // particle ramp: light (grey) → middling (sapphire) → heavy (amber)
    p.rampLow = parseColor(p.muted);
    p.rampMid = parseColor(p.accent);
    p.rampHigh = parseColor(p.warning);
    p.criticalRgb = parseColor(p.critical);
    return p;
  }

  function rampColor(p, t) {
    t = Math.min(1, Math.max(0, t));
    return t < 0.5 ? mix(p.rampLow, p.rampMid, t * 2) : mix(p.rampMid, p.rampHigh, (t - 0.5) * 2);
  }

  class FieldRenderer {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx = canvas.getContext('2d');
      this.colors = palette();
      this.sizePx = 0;
      this.marginPx = 10;
    }

    refreshColors() {
      this.colors = palette();
    }

    resize() {
      const rect = this.canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      this.canvas.width = Math.round(rect.width * dpr);
      this.canvas.height = Math.round(rect.height * dpr);
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.sizePx = Math.min(rect.width, rect.height);
    }

    setField(fieldSizeIn) {
      this.fieldSizeIn = fieldSizeIn;
    }

    /** What the view is centered on, in field inches, and how far it's zoomed in. */
    setCamera(xIn, yIn, zoom) {
      this.camera = { xIn, yIn, zoom };
    }

    get scale() {
      return ((this.sizePx - 2 * this.marginPx) / this.fieldSizeIn) * (this.camera ? this.camera.zoom : 1);
    }

    toPx(xIn, yIn) {
      const cam = this.camera || { xIn: 0, yIn: 0 };
      return [this.sizePx / 2 + (xIn - cam.xIn) * this.scale, this.sizePx / 2 - (yIn - cam.yIn) * this.scale];
    }

    toField(px, py) {
      const cam = this.camera || { xIn: 0, yIn: 0 };
      return { xIn: cam.xIn + (px - this.sizePx / 2) / this.scale, yIn: cam.yIn - (py - this.sizePx / 2) / this.scale };
    }

    /** Draw one frame. `view` holds the toggles, the selection, and step mode's extras. */
    draw(sim, view) {
      const ctx = this.ctx;
      const c = this.colors;
      this.setField(sim.options.world.fieldSizeIn);
      const truth = sim.world.truth;
      if (view.zoom > 1) this.setCamera(truth.xIn, truth.yIn, view.zoom);
      else this.setCamera(0, 0, 1);
      ctx.clearRect(0, 0, this.sizePx + 40, this.sizePx + 40);
      this.drawField(sim, view);

      const robot = sim.robot;
      const loc = robot.localizer;
      const work = loc.work;

      if (view.trails) this.drawTrails(sim);
      if (view.target) this.drawTarget(robot.drivetrain.target);

      // the real robot underneath, so the particles and beams show over it
      this.drawRobot(truth, sim.options.world, { outline: c['robot-line'], fill: c['robot-fill'], details: view.mounts });

      if (view.particles) {
        if (view.step && view.step.phase === 'predicted' && work && work.before) this.drawPredictLines(loc, work);
        if (view.step && view.step.killed) this.drawKilled(view.step.killed);
        this.drawParticles(loc.filter, view);
      }

      const est = work && work.estimate;
      if (view.ellipse && est) this.drawEllipse(est);

      if (view.beams) this.drawBeams(sim);
      if (view.expected && work && work.checks && est) this.drawExpected(loc, work);
      if (view.selected != null && work && work.readings) this.drawSelected(loc, work, view.selected);

      const snap = robot.odometry.snapshot();
      if (view.raw) this.drawRobot(snap.rawPose, sim.options.world, { outline: c.raw, dash: [5, 4], ghost: true });
      if (view.corrected) this.drawRobot(snap.pose, sim.options.world, { outline: c.corrected, ghost: true });
      if (view.correctionArrow) this.drawArrow(snap.rawPose, snap.pose, c.corrected);
      if (view.estimate && est) this.drawCross(est.xIn, est.yIn, c.accent);
      if (view.zoom > 1) this.drawScaleBar();
    }

    /** A 1 inch (or 6 inch) bar in the corner, so a zoomed view still reads in inches. */
    drawScaleBar() {
      const ctx = this.ctx;
      const inches = this.scale > 40 ? 1 : 6;
      const length = inches * this.scale;
      const x = 16;
      const y = this.sizePx - 16;
      ctx.strokeStyle = this.colors.ink;
      ctx.fillStyle = this.colors.ink;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x, y - 5);
      ctx.lineTo(x, y);
      ctx.lineTo(x + length, y);
      ctx.lineTo(x + length, y - 5);
      ctx.stroke();
      ctx.font = '12px system-ui, sans-serif';
      ctx.fillText(`${inches} in`, x + length + 6, y);
    }

    drawField(sim, view) {
      const ctx = this.ctx;
      const c = this.colors;
      const half = this.fieldSizeIn / 2;
      const [x0, y0] = this.toPx(-half, half);
      const size = this.fieldSizeIn * this.scale;
      ctx.fillStyle = c.tile;
      ctx.fillRect(x0, y0, size, size);
      if (view.grid) {
        ctx.strokeStyle = c['tile-line'];
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 1; i < 6; ++i) {
          const t = (size * i) / 6;
          ctx.moveTo(x0 + t, y0);
          ctx.lineTo(x0 + t, y0 + size);
          ctx.moveTo(x0, y0 + t);
          ctx.lineTo(x0 + size, y0 + t);
        }
        ctx.stroke();
        // the origin
        const [ox, oy] = this.toPx(0, 0);
        ctx.strokeStyle = c.axis;
        ctx.beginPath();
        ctx.moveTo(ox - 6, oy);
        ctx.lineTo(ox + 6, oy);
        ctx.moveTo(ox, oy - 6);
        ctx.lineTo(ox, oy + 6);
        ctx.stroke();
        ctx.fillStyle = c.muted;
        ctx.font = '11px system-ui, sans-serif';
        ctx.fillText('+y downfield', x0 + 6, y0 + 14);
        ctx.fillText('+x', x0 + size - 22, y0 + size / 2 - 6);
      }
      ctx.strokeStyle = c.wall;
      ctx.lineWidth = 4;
      ctx.strokeRect(x0 - 2, y0 - 2, size + 4, size + 4);

      // field elements: filled if really there, outlined if the map has them
      const o = sim.options;
      for (const e of o.world.elements) {
        const [ex, ey] = this.toPx(e.minXIn, e.maxYIn);
        const w = (e.maxXIn - e.minXIn) * this.scale;
        const h = (e.maxYIn - e.minYIn) * this.scale;
        if (e.inWorld !== false) {
          ctx.fillStyle = c.element;
          ctx.fillRect(ex, ey, w, h);
        }
        if (o.elementsInMap) {
          ctx.strokeStyle = c.accent;
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 3]);
          ctx.strokeRect(ex, ey, w, h);
          ctx.setLineDash([]);
        }
      }
      const d = o.world.defender;
      if (d.enabled) {
        const hs = d.sizeIn / 2;
        const [dx, dy] = this.toPx(d.xIn - hs, d.yIn + hs);
        const s = d.sizeIn * this.scale;
        ctx.fillStyle = rgba(parseColor(c['series-8']), 0.35);
        ctx.strokeStyle = c['series-8'];
        ctx.lineWidth = 2;
        ctx.fillRect(dx, dy, s, s);
        ctx.strokeRect(dx, dy, s, s);
        ctx.fillStyle = c['series-8'];
        ctx.font = '600 11px system-ui, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('defender', dx + s / 2, dy + s / 2 + 4);
        ctx.textAlign = 'left';
      }
    }

    drawTrails(sim) {
      const c = this.colors;
      this.polyline(sim.trails.truth, c.ink, 1, [], 0.35);
      this.polyline(sim.trails.raw, c.raw, 1.5, [4, 3], 0.8);
      this.polyline(sim.trails.corrected, c.corrected, 1.5, [], 0.8);
    }

    polyline(points, color, width, dash, alpha) {
      if (points.length < 2) return;
      const ctx = this.ctx;
      ctx.globalAlpha = alpha;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.setLineDash(dash);
      ctx.beginPath();
      points.forEach((p, i) => {
        const [x, y] = this.toPx(p.x, p.y);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }

    drawTarget(target) {
      if (!target) return;
      const ctx = this.ctx;
      const c = this.colors;
      if (target.kind === 'path') {
        this.polyline(target.points.map((p) => ({ x: p.xIn, y: p.yIn })), c['series-4'], 2, [2, 4], 0.9);
        const [lx, ly] = this.toPx(target.lookahead.xIn, target.lookahead.yIn);
        ctx.fillStyle = c['series-4'];
        ctx.beginPath();
        ctx.arc(lx, ly, 4, 0, 2 * Math.PI);
        ctx.fill();
        return;
      }
      if (target.kind === 'heading') return;
      const [x, y] = this.toPx(target.xIn, target.yIn);
      ctx.strokeStyle = c['series-4'];
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(x, y, 7, 0, 2 * Math.PI);
      ctx.moveTo(x - 11, y);
      ctx.lineTo(x + 11, y);
      ctx.moveTo(x, y - 11);
      ctx.lineTo(x, y + 11);
      ctx.stroke();
      if (target.kind === 'pose') {
        const r = target.headingDeg * kDegToRad;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + Math.sin(r) * 18, y - Math.cos(r) * 18);
        ctx.stroke();
      }
    }

    drawPredictLines(loc, work) {
      const ctx = this.ctx;
      const particles = loc.filter.particles;
      if (particles.length > 800) return;
      ctx.strokeStyle = rgba(this.colors.rampMid, 0.5);
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 0; i < particles.length; ++i) {
        const b = work.before[i];
        if (!b) continue;
        const [x0, y0] = this.toPx(b.xIn, b.yIn);
        const [x1, y1] = this.toPx(particles[i].xIn, particles[i].yIn);
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
      }
      ctx.stroke();
    }

    drawKilled(killed) {
      const ctx = this.ctx;
      ctx.strokeStyle = rgba(this.colors.criticalRgb, 0.55);
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (const k of killed) {
        const [x, y] = this.toPx(k.xIn, k.yIn);
        ctx.moveTo(x - 2.5, y - 2.5);
        ctx.lineTo(x + 2.5, y + 2.5);
        ctx.moveTo(x + 2.5, y - 2.5);
        ctx.lineTo(x - 2.5, y + 2.5);
      }
      ctx.stroke();
    }

    drawParticles(filter, view) {
      const ctx = this.ctx;
      const c = this.colors;
      const particles = filter.particles;
      const n = particles.length;
      // draw light ones first so heavy ones sit on top
      const order = view.weightColor ? particles.map((_, i) => i).sort((a, b) => particles[a].score - particles[b].score)
        : particles.map((_, i) => i);
      for (const i of order) {
        const p = particles[i];
        const [x, y] = this.toPx(p.xIn, p.yIn);
        let r = 2;
        if (view.weightSize) r = Math.min(7, Math.max(1.2, 1.2 + 1.8 * Math.sqrt(p.weight * n)));
        const color = view.weightColor ? rampColor(c, p.score) : c.rampMid;
        ctx.fillStyle = rgba(color, view.weightColor ? 0.35 + 0.6 * p.score : 0.7);
        ctx.beginPath();
        ctx.arc(x, y, r, 0, 2 * Math.PI);
        ctx.fill();
        if (p.recovered) {
          ctx.strokeStyle = c['series-5'];
          ctx.lineWidth = 1;
          ctx.stroke();
        }
      }
      if (view.selected != null && particles[view.selected]) {
        const p = particles[view.selected];
        const [x, y] = this.toPx(p.xIn, p.yIn);
        ctx.strokeStyle = c.ink;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, y, 7, 0, 2 * Math.PI);
        ctx.stroke();
      }
    }

    drawEllipse(est) {
      const a = est.varianceXIn2;
      const b = est.covarianceXYIn2;
      const d = est.varianceYIn2;
      const mid = (a + d) / 2;
      const root = Math.sqrt(((a - d) / 2) ** 2 + b * b);
      const major = Math.sqrt(Math.max(0, mid + root));
      const minor = Math.sqrt(Math.max(0, mid - root));
      const angle = 0.5 * Math.atan2(2 * b, a - d);
      const [x, y] = this.toPx(est.xIn, est.yIn);
      const ctx = this.ctx;
      ctx.strokeStyle = this.colors.accent;
      ctx.fillStyle = rgba(this.colors.rampMid, 0.08);
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      // 2 sigma; the screen's y is flipped, so the angle is too
      ctx.ellipse(x, y, Math.max(1, 2 * major * this.scale), Math.max(1, 2 * minor * this.scale), -angle, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
    }

    drawBeams(sim) {
      const ctx = this.ctx;
      const c = this.colors;
      const w = sim.world;
      const s = w.settings;
      const beam = parseColor(c.beam);
      for (let i = 0; i < s.sensors.length; ++i) {
        const ray = L.sensorRay(w.truth.xIn, w.truth.yIn, w.truth.headingDeg, s.sensors[i]);
        const mm = w.readingsMm[i];
        const valid = mm < 9999;
        const lengthIn = valid ? mm / 25.4 : 78;
        const [x0, y0] = this.toPx(ray.xIn, ray.yIn);
        const [x1, y1] = this.toPx(ray.xIn + ray.dirX * lengthIn, ray.yIn + ray.dirY * lengthIn);
        ctx.strokeStyle = rgba(beam, valid ? 0.85 : 0.3);
        ctx.lineWidth = valid ? 2 : 1;
        ctx.setLineDash(valid ? [] : [3, 4]);
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
        ctx.setLineDash([]);
        if (valid) {
          ctx.fillStyle = rgba(beam, 1);
          ctx.beginPath();
          ctx.arc(x1, y1, 3, 0, 2 * Math.PI);
          ctx.fill();
        }
      }
    }

    drawExpected(loc, work) {
      const ctx = this.ctx;
      const c = this.colors;
      const est = work.estimate;
      for (let s = 0; s < work.checks.length; ++s) {
        const check = work.checks[s];
        if (!check.used) continue;
        const ray = L.sensorRay(est.xIn, est.yIn, work.beamHeadingDeg, loc.filter.sensors[s]);
        const color = check.agrees ? c.good : c.critical;
        const expectedIn = Number.isFinite(check.expectedIn) ? check.expectedIn : 90;
        const [x0, y0] = this.toPx(ray.xIn, ray.yIn);
        const [xe, ye] = this.toPx(ray.xIn + ray.dirX * expectedIn, ray.yIn + ray.dirY * expectedIn);
        const [xm, ym] = this.toPx(ray.xIn + ray.dirX * check.measuredIn, ray.yIn + ray.dirY * check.measuredIn);
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(xe, ye);
        ctx.stroke();
        ctx.setLineDash([]);
        // the measured distance: a tick across the beam
        const nx = -ray.dirY * 6;
        const ny = -ray.dirX * 6;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(xm - nx, ym - ny);
        ctx.lineTo(xm + nx, ym + ny);
        ctx.stroke();
      }
    }

    drawSelected(loc, work, index) {
      if (!loc.filter.particles[index]) return;
      const ctx = this.ctx;
      const c = this.colors;
      const info = loc.filter.explainParticle(index, work.beamHeadingDeg, work.readings);
      for (const row of info.rows) {
        const ray = row.ray;
        const expectedIn = Number.isFinite(row.expectedIn) ? row.expectedIn : 90;
        const [x0, y0] = this.toPx(ray.xIn, ray.yIn);
        const [xe, ye] = this.toPx(ray.xIn + ray.dirX * expectedIn, ray.yIn + ray.dirY * expectedIn);
        ctx.strokeStyle = c.ink;
        ctx.lineWidth = 1.2;
        ctx.setLineDash([2, 3]);
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(xe, ye);
        ctx.stroke();
        ctx.setLineDash([]);
        if (row.used) {
          const [xm, ym] = this.toPx(ray.xIn + ray.dirX * row.measuredIn, ray.yIn + ray.dirY * row.measuredIn);
          ctx.fillStyle = c['series-5'];
          ctx.beginPath();
          ctx.arc(xm, ym, 4, 0, 2 * Math.PI);
          ctx.fill();
        }
      }
    }

    drawRobot(pose, settings, { outline, fill = null, dash = [], ghost = false, details = false }) {
      const ctx = this.ctx;
      const c = this.colors;
      const s = settings.robotSizeIn * this.scale;
      const [x, y] = this.toPx(pose.xIn, pose.yIn);
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(pose.headingDeg * kDegToRad);
      ctx.setLineDash(dash);
      ctx.strokeStyle = outline;
      ctx.lineWidth = ghost ? 2 : 2;
      if (fill) {
        ctx.fillStyle = fill;
        ctx.fillRect(-s / 2, -s / 2, s, s);
      }
      ctx.strokeRect(-s / 2, -s / 2, s, s);
      ctx.setLineDash([]);
      // the front
      ctx.fillStyle = outline;
      ctx.beginPath();
      ctx.moveTo(0, -s / 2 - 6);
      ctx.lineTo(-5, -s / 2 + 2);
      ctx.lineTo(5, -s / 2 + 2);
      ctx.closePath();
      ctx.fill();
      if (!ghost) this.drawWheels(s, settings, details);
      ctx.restore();
    }

    drawWheels(s, settings, details) {
      const ctx = this.ctx;
      const c = this.colors;
      const k = this.scale;
      const ww = 1.4 * k; // wheel width
      const wl = 3.6 * k; // wheel length
      ctx.fillStyle = c.muted;
      ctx.strokeStyle = c['robot-fill'];
      ctx.lineWidth = 1;
      // four mecanum corners, rollers alternating so the chassis can strafe
      const corners = [[-1, -1, 1], [1, -1, -1], [-1, 1, -1], [1, 1, 1]];
      for (const [sx, sy, roller] of corners) {
        const cx = sx * (s / 2 - ww / 2 - 1);
        const cy = sy * (s / 2 - wl / 2 - 1);
        ctx.fillRect(cx - ww / 2, cy - wl / 2, ww, wl);
        ctx.beginPath();
        for (let i = -1; i <= 1; ++i) {
          ctx.moveTo(cx - ww / 2, cy + i * wl / 4 - roller * ww / 2);
          ctx.lineTo(cx + ww / 2, cy + i * wl / 4 + roller * ww / 2);
        }
        ctx.stroke();
      }
      // Asterisk center wheels, straight omnis
      for (const sx of [-1, 1]) {
        const cx = sx * (s / 2 - ww / 2 - 1);
        ctx.fillRect(cx - ww / 2, -wl / 2.4, ww, wl / 1.2);
      }
      if (!details) return;
      // tracking wheels where OdometryConfig's offsets put them (see world.js: a positive vertical
      // offset is left of center under the arc correction's sign)
      ctx.fillStyle = c['series-7'];
      const vx = -(settings.verticalWheelFlipped ? -1 : 1) * settings.verticalOffsetIn * k;
      ctx.fillRect(vx - 0.5 * k, -1.4 * k, 1.0 * k, 2.8 * k);
      const hy = -settings.horizontalOffsetIn * k;
      ctx.fillRect(-1.4 * k, hy - 0.5 * k, 2.8 * k, 1.0 * k);
      // distance sensors
      ctx.fillStyle = c.beam;
      for (const m of settings.sensors) {
        const px = m.rightIn * k;
        const py = -m.forwardIn * k;
        ctx.save();
        ctx.translate(px, py);
        ctx.rotate(m.facingDeg * kDegToRad);
        ctx.fillRect(-1.2 * k, -0.6 * k, 2.4 * k, 1.2 * k);
        ctx.restore();
      }
    }

    drawArrow(from, to, color) {
      const [x0, y0] = this.toPx(from.xIn, from.yIn);
      const [x1, y1] = this.toPx(to.xIn, to.yIn);
      const length = Math.hypot(x1 - x0, y1 - y0);
      if (length < 3) return;
      const ctx = this.ctx;
      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
      const a = Math.atan2(y1 - y0, x1 - x0);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x1 - 8 * Math.cos(a - 0.4), y1 - 8 * Math.sin(a - 0.4));
      ctx.lineTo(x1 - 8 * Math.cos(a + 0.4), y1 - 8 * Math.sin(a + 0.4));
      ctx.closePath();
      ctx.fill();
    }

    drawCross(xIn, yIn, color) {
      const [x, y] = this.toPx(xIn, yIn);
      const ctx = this.ctx;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(x - 6, y - 6);
      ctx.lineTo(x + 6, y + 6);
      ctx.moveTo(x + 6, y - 6);
      ctx.lineTo(x - 6, y + 6);
      ctx.stroke();
    }

    /** The particle nearest a pixel, within `radiusPx`, or null. */
    particleAt(filter, px, py, radiusPx = 10) {
      let best = null;
      let bestDist = radiusPx;
      filter.particles.forEach((p, i) => {
        const [x, y] = this.toPx(p.xIn, p.yIn);
        const d = Math.hypot(x - px, y - py);
        if (d < bestDist) {
          bestDist = d;
          best = i;
        }
      });
      return best;
    }
  }

  /** The error chart: raw odometry, the corrected pose, and the estimate, against the truth. */
  function drawChart(canvas, history, colors) {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const w = rect.width;
    const h = rect.height;
    ctx.clearRect(0, 0, w, h);
    const left = 34;
    const bottom = h - 18;
    const top = 24;
    const right = w - 8;
    // the last minute, or the first minute until there's been one
    const tLast = history.length ? history[history.length - 1].t : 0;
    const tStart = Math.max(0, tLast - 60);
    const tEnd = tStart + 60;
    let yMax = 2;
    for (const s of history) {
      if (s.t < tStart) continue;
      yMax = Math.max(yMax, s.raw, s.corrected, s.estimate);
    }
    yMax = Math.ceil(yMax * 1.15);
    const xOf = (t) => left + ((t - tStart) / (tEnd - tStart)) * (right - left);
    const yOf = (v) => bottom - (v / yMax) * (bottom - top);

    ctx.font = '11px ui-monospace, Consolas, monospace';
    ctx.fillStyle = colors.muted;
    ctx.strokeStyle = colors.grid;
    ctx.lineWidth = 1;
    const step = yMax <= 4 ? 1 : yMax <= 10 ? 2 : yMax <= 25 ? 5 : 10;
    for (let v = 0; v <= yMax; v += step) {
      const y = yOf(v);
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(right, y);
      ctx.stroke();
      ctx.fillText(`${v}`, 6, y + 4);
    }
    ctx.fillText(`${tStart.toFixed(0)}s`, left, h - 4);
    ctx.textAlign = 'right';
    ctx.fillText(`${tEnd.toFixed(0)}s`, right, h - 4);
    ctx.textAlign = 'left';

    const series = [
      { key: 'raw', label: 'Odometry alone', color: colors.raw, dash: [4, 3], width: 2 },
      { key: 'corrected', label: 'Corrected pose', color: colors.corrected, dash: [], width: 2.2 },
      { key: 'estimate', label: 'MCL estimate', color: colors.beam, dash: [1, 3], width: 1.5 },
    ];

    // the legend, top right
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'right';
    let lx = right;
    for (const s of series.slice().reverse()) {
      ctx.fillStyle = colors['ink-2'];
      ctx.fillText(s.label, lx, 12);
      lx -= ctx.measureText(s.label).width + 6;
      ctx.strokeStyle = s.color;
      ctx.lineWidth = s.width;
      ctx.setLineDash(s.dash);
      ctx.beginPath();
      ctx.moveTo(lx - 18, 8);
      ctx.lineTo(lx, 8);
      ctx.stroke();
      lx -= 32;
    }
    ctx.setLineDash([]);
    ctx.textAlign = 'left';

    for (const s of series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = s.width;
      ctx.setLineDash(s.dash);
      ctx.beginPath();
      let started = false;
      for (const sample of history) {
        if (sample.t < tStart) continue;
        const x = xOf(sample.t);
        const y = yOf(Math.min(sample[s.key], yMax));
        if (!started) {
          ctx.moveTo(x, y);
          started = true;
        } else {
          ctx.lineTo(x, y);
        }
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  root.SIM.render = { FieldRenderer, drawChart, palette, rampColor, parseColor };
})();
