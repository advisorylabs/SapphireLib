/*
 * SapphireLib telemetry analyzer — replay.js
 *
 * The replay: the match played back from its log. A field view rebuilt from
 * odometry (the robot's footprint, heading, trail, the motion's target, and
 * each drive motor colored by temperature), a side view of the lift (carriage
 * against target, the control law it was in, the volts it was sent, the
 * claw), live motor tiles, the battery and the driver's controller — all at
 * one moment, scrubbed on a timeline or played at speed, with charts whose
 * playhead follows.
 *
 * Browser only: window.SA.replay and SA.views.replay.
 *
 * Team 96671H — Hitmen
 */
(function () {
  'use strict';

  const root = typeof self !== 'undefined' ? self : this;
  const { h, clear, heatColor, fmt } = SA.ui;
  const { token } = SA.charts;
  const { slt, analysis: A } = SA;

  const FIELD_IN = 144;
  const ROBOT_IN = 18;
  const SPEEDS = [0.25, 0.5, 1, 2, 4, 8];
  const BUTTONS = ['L1', 'L2', 'R1', 'R2', 'Up', 'Down', 'Left', 'Right', 'X', 'B', 'Y', 'A'];

  /** Whether a run's odometry uses a centered origin (−72..72) or a corner one (0..144). */
  function fieldOrigin(run) {
    if (run._fieldOrigin) return run._fieldOrigin;
    const odom = run.get('odom');
    let centered = false;
    if (odom) {
      for (let i = 0; i < odom.length; i += 10) {
        if (odom.cols.x[i] < -2 || odom.cols.y[i] < -2) {
          centered = true;
          break;
        }
      }
    }
    run._fieldOrigin = centered ? [-72, -72] : [0, 0];
    return run._fieldOrigin;
  }

  function activeMotion(motions, t) {
    for (let i = motions.length - 1; i >= 0; --i) {
      const m = motions[i];
      if (m.start <= t && t <= (m.stop ?? m.start)) return m;
    }
    return null;
  }

  /**
   * Draws the field at time `t`. `opts.from` limits the trail's start (the
   * Motions view draws one motion's whole path); `opts.motion` forces the
   * target shown.
   */
  function drawField(canvas, app, t, opts = {}) {
    const run = app.run;
    const odom = run.get('odom');
    const dpr = window.devicePixelRatio || 1;
    const size = canvas.clientWidth || 300;
    canvas.width = Math.round(size * dpr);
    canvas.height = Math.round(size * dpr);
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const m = 10;
    const scale = (size - 2 * m) / FIELD_IN;
    const [ox, oy] = fieldOrigin(run);
    const sx = (x) => m + (x - ox) * scale;
    const sy = (y) => size - m - (y - oy) * scale;

    ctx.fillStyle = token('--surface-2');
    ctx.fillRect(m, m, FIELD_IN * scale, FIELD_IN * scale);
    ctx.strokeStyle = token('--grid');
    ctx.lineWidth = 1;
    for (let i = 1; i < 6; ++i) {
      const p = m + i * 24 * scale;
      ctx.beginPath();
      ctx.moveTo(Math.round(p) + 0.5, m);
      ctx.lineTo(Math.round(p) + 0.5, size - m);
      ctx.moveTo(m, Math.round(p) + 0.5);
      ctx.lineTo(size - m, Math.round(p) + 0.5);
      ctx.stroke();
    }
    ctx.strokeStyle = token('--axis');
    ctx.lineWidth = 2;
    ctx.strokeRect(m, m, FIELD_IN * scale, FIELD_IN * scale);
    if (!odom || odom.length === 0) {
      ctx.fillStyle = token('--muted');
      ctx.font = `13px ${token('--font-body')}`;
      ctx.textAlign = 'center';
      ctx.fillText('No odometry in this log', size / 2, size / 2);
      return;
    }

    const i = odom.indexAt(t);
    if (i < 0) return;
    const x = odom.cols.x;
    const y = odom.cols.y;
    const hd = odom.cols.heading;

    // The whole range's path, faint; the last seconds, strong.
    const from = opts.from ?? t - 6;
    const [r0] = odom.range(opts.from ?? app.range[0], t);
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    if (opts.from === undefined) {
      ctx.strokeStyle = token('--axis');
      ctx.lineWidth = 1;
      ctx.beginPath();
      let pen = false;
      const [a0, a1] = odom.range(app.range[0], app.range[1]);
      for (let k = a0; k < a1; k += 2) {
        if (!pen) ctx.moveTo(sx(x[k]), sy(y[k]));
        else ctx.lineTo(sx(x[k]), sy(y[k]));
        pen = true;
      }
      ctx.stroke();
    }
    const [f0] = odom.range(from, t);
    ctx.strokeStyle = token('--accent');
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let k = Math.max(f0, r0); k <= i; ++k) {
      if (k === Math.max(f0, r0)) ctx.moveTo(sx(x[k]), sy(y[k]));
      else ctx.lineTo(sx(x[k]), sy(y[k]));
    }
    ctx.stroke();

    // The target of the motion running now.
    const motion = opts.motion || activeMotion(app.report ? app.report.motions : slt.motions(run), t);
    const px = sx(x[i]);
    const py = sy(y[i]);
    if (motion) {
      const p = motion.params;
      ctx.strokeStyle = token('--ink-2');
      ctx.fillStyle = token('--ink-2');
      ctx.lineWidth = 1.5;
      if (Number.isFinite(p.x) && Number.isFinite(p.y)) {
        const tx = sx(p.x);
        const ty = sy(p.y);
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(tx, ty);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(tx, ty, 6, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(tx - 9, ty);
        ctx.lineTo(tx + 9, ty);
        ctx.moveTo(tx, ty - 9);
        ctx.lineTo(tx, ty + 9);
        ctx.stroke();
      }
      const targetHeading = Number.isFinite(p.target_deg) ? p.target_deg
        : Number.isFinite(p.heading_deg) ? p.heading_deg : null;
      if (targetHeading !== null) {
        const rad = (targetHeading * Math.PI) / 180;
        const len = ROBOT_IN * 1.6 * scale;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(px + Math.sin(rad) * len, py - Math.cos(rad) * len);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(px + Math.sin(rad) * len, py - Math.cos(rad) * len, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // The robot, rotated to its heading (clockwise from +y).
    const rad = (hd[i] * Math.PI) / 180;
    const half = (ROBOT_IN / 2) * scale;
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(rad);
    ctx.fillStyle = token('--surface');
    ctx.strokeStyle = token('--ink');
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-half, -half, 2 * half, 2 * half, 3);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = token('--accent');
    ctx.beginPath();
    ctx.moveTo(0, -half - 7);
    ctx.lineTo(-6, -half + 3);
    ctx.lineTo(6, -half + 3);
    ctx.closePath();
    ctx.fill();
    // Drive motors at their corners (and the center pair), colored by heat.
    const corners = { fl: [-1, -1], fr: [1, -1], bl: [-1, 1], br: [1, 1], ml: [-1, 0], mr: [1, 0] };
    const r = Math.max(4, half * 0.28);
    for (const [name, [cx, cy]] of Object.entries(corners)) {
      const ch = run.get(`motor.${name}`);
      if (!ch) continue;
      // Unplugged only if the motor was sampled and answered nothing; no
      // sample yet (the logger pauses while disabled) is just unknown.
      const k = ch.indexAt(t);
      const fresh = k >= 0 && t - ch.t[k] < 0.5;
      const temp = fresh ? ch.cols.temp[k] : NaN;
      const gone = fresh && Number.isNaN(ch.cols.volts[k]) && Number.isNaN(temp);
      const mx = cx * (half - r - 1);
      const my = cy * (half - r - 1);
      ctx.beginPath();
      ctx.arc(mx, my, r, 0, Math.PI * 2);
      ctx.fillStyle = gone ? token('--surface-3') : heatColor(temp);
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = gone ? token('--critical') : token('--surface');
      ctx.stroke();
      if (gone) {
        ctx.beginPath();
        ctx.moveTo(mx - r * 0.6, my - r * 0.6);
        ctx.lineTo(mx + r * 0.6, my + r * 0.6);
        ctx.moveTo(mx + r * 0.6, my - r * 0.6);
        ctx.lineTo(mx - r * 0.6, my + r * 0.6);
        ctx.stroke();
      }
    }
    ctx.restore();

    ctx.fillStyle = token('--muted');
    ctx.font = `11px ${token('--font-data')}`;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(`(${x[i].toFixed(1)}, ${y[i].toFixed(1)}) ${hd[i].toFixed(0)}°`, m + 4, m + 4);
  }

  /** The lift's side view at time `t`. */
  function drawLift(canvas, app, t, geometry) {
    const run = app.run;
    const act = run.get(geometry.channel);
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 180;
    const hgt = canvas.clientHeight || 360;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(hgt * dpr);
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hgt);
    const top = 16;
    const bottom = 56;
    const railX = 44;
    const { lo, hi, levels } = geometry;
    const yOf = (v) => top + (1 - (v - lo) / (hi - lo)) * (hgt - top - bottom);

    ctx.font = `11px ${token('--font-data')}`;
    ctx.textBaseline = 'middle';
    // Rail and the levels it was asked for.
    ctx.fillStyle = token('--surface-3');
    ctx.fillRect(railX - 4, top, 8, hgt - top - bottom);
    for (const level of levels) {
      const y = Math.round(yOf(level)) + 0.5;
      ctx.strokeStyle = token('--grid');
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(railX + 8, y);
      ctx.lineTo(w - 8, y);
      ctx.stroke();
      ctx.fillStyle = token('--muted');
      ctx.textAlign = 'right';
      ctx.fillText(String(Math.round(level)), railX - 8, y);
    }
    const i = act.indexAt(t);
    if (i < 0 || t - act.t[i] > 0.25) {
      // Its loop isn't running (this robot runs the lift from driver control).
      ctx.fillStyle = token('--muted');
      ctx.font = `12px ${token('--font-body')}`;
      ctx.textAlign = 'center';
      ctx.fillText('not running now', (railX + w) / 2, (top + hgt - bottom) / 2);
      return;
    }
    const pos = act.cols.pos[i];
    const target = act.cols.target[i];
    const volts = act.cols.volts[i];
    const law = act.cols.law[i];

    // Target.
    const ty = yOf(target);
    ctx.strokeStyle = token('--ink-2');
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(railX - 12, ty);
    ctx.lineTo(w - 8, ty);
    ctx.stroke();
    ctx.fillStyle = token('--ink-2');
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`target ${target.toFixed(0)}`, w - 8, ty - 3);

    // Carriage, and how far off it is.
    if (Number.isFinite(pos)) {
      const cy = yOf(pos);
      ctx.fillStyle = token('--accent');
      ctx.beginPath();
      ctx.roundRect(railX - 14, cy - 9, w - railX - 8, 18, 4);
      ctx.fill();
      ctx.fillStyle = token('--accent-ink');
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      const err = target - pos;
      ctx.fillText(`${pos.toFixed(0)}${Math.abs(err) >= 1 ? `  (${err > 0 ? '−' : '+'}${Math.abs(err).toFixed(0)})` : ''}`,
        railX - 8, cy);
    } else {
      ctx.fillStyle = token('--critical');
      ctx.textAlign = 'left';
      ctx.fillText('no sensor reading', railX + 12, (top + hgt - bottom) / 2);
    }

    // Volts sent, −12..12, and the law that sent them.
    const barY = hgt - 34;
    const barX0 = 12;
    const barX1 = w - 12;
    const mid = (barX0 + barX1) / 2;
    ctx.fillStyle = token('--surface-3');
    ctx.fillRect(barX0, barY, barX1 - barX0, 8);
    if (Number.isFinite(volts)) {
      const span = ((barX1 - barX0) / 2) * Math.max(-1, Math.min(1, volts / 12));
      ctx.fillStyle = Math.abs(volts) >= 11.5 ? token('--critical') : token('--accent');
      ctx.fillRect(Math.min(mid, mid + span), barY, Math.abs(span), 8);
    }
    ctx.fillStyle = token('--axis');
    ctx.fillRect(mid - 0.5, barY - 2, 1, 12);
    ctx.fillStyle = token('--ink-2');
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    const lawName = SA.model.LAW_NAMES[law] || '?';
    ctx.fillText(`${Number.isFinite(volts) ? `${volts.toFixed(1)} V` : 'volts —'} · ${lawName}`, barX0, barY + 12);
  }

  function liftGeometry(app) {
    const act = app.run.channels.find((c) => c.name.endsWith('.act') && c.has('target') && c.has('pos'));
    if (!act) return null;
    const [i0, i1] = act.range(app.range[0], app.range[1]);
    let lo = Infinity;
    let hi = -Infinity;
    const levels = new Set();
    for (let i = i0; i < i1; ++i) {
      for (const v of [act.cols.pos[i], act.cols.target[i]]) {
        if (!Number.isFinite(v)) continue;
        lo = Math.min(lo, v);
        hi = Math.max(hi, v);
      }
      levels.add(act.cols.target[i]);
    }
    if (!Number.isFinite(lo)) return null;
    const span = Math.max(10, hi - lo);
    const sorted = [...levels].sort((a, b) => a - b);
    // At most a dozen level lines, or it's clutter.
    const step = Math.ceil(sorted.length / 12);
    return { channel: act.name, name: act.name.slice(0, -4), lo: Math.min(0, lo) - span * 0.04,
      hi: hi + span * 0.08, levels: sorted.filter((_, k) => k % step === 0) };
  }

  // --- The view ------------------------------------------------------------------------------

  SA.views.replay = {
    render(section, app) {
      const run = app.run;
      const [t0, t1] = app.range;
      const state = { playing: false, speed: 1, last: null, raf: 0 };
      const motors = A.motorChannels(run);
      const lift = liftGeometry(app);
      const motions = app.report.motions;

      // Transport.
      const playBtn = h('button', { class: 'btn primary', type: 'button', 'aria-label': 'Play' }, 'Play');
      const speedSel = h('select', { class: 'select', 'aria-label': 'Playback speed' },
        SPEEDS.map((s) => h('option', { value: String(s) }, `${s}×`)));
      speedSel.value = '1';
      const clock = h('span', { class: 'clock' }, '0:00.0');
      const clockSub = h('span', { class: 'clock-sub' }, '');
      const timeline = h('div', { class: 'timeline', tabindex: '0', role: 'slider',
        'aria-label': 'Replay position', 'aria-valuemin': String(t0), 'aria-valuemax': String(t1) });
      const tlCanvas = h('canvas');
      timeline.append(tlCanvas);
      section.append(h('section', { class: 'panel' },
        h('div', { class: 'transport' },
          playBtn,
          h('button', { class: 'btn', type: 'button', title: 'Back 1 second', onclick: () => app.seek(app.cursor - 1) }, '−1s'),
          h('button', { class: 'btn', type: 'button', title: 'Forward 1 second', onclick: () => app.seek(app.cursor + 1) }, '+1s'),
          speedSel,
          h('div', null, clock, h('br'), clockSub)),
        timeline));

      // Stage.
      const field = h('canvas', { class: 'field-canvas', role: 'img', 'aria-label': 'Robot on the field' });
      const liftCanvas = lift ? h('canvas', { class: 'lift-canvas', role: 'img', 'aria-label': `${lift.name} position` }) : null;
      const liftInfo = h('dl', { class: 'kv' });
      const motorGrid = h('div', { class: 'motor-grid' });
      const battery = h('div');
      const driver = h('div');
      const now = h('div', { class: 'now-list' });
      section.append(h('div', { class: 'stage' },
        h('section', { class: 'panel' }, h('header', null, h('h3', null, 'Field'),
          h('span', { class: 'hint' }, 'From odometry. Dots are drive motors, colored by temperature; ✕ is unplugged.')),
        field, SA.ui.heatLegend()),
        lift ? h('section', { class: 'panel' }, h('header', null, h('h3', null, lift.name[0].toUpperCase() + lift.name.slice(1))),
          liftCanvas, liftInfo) : h('section', { class: 'panel' }, h('p', { class: 'muted' }, 'No mechanism channel (X.act) in this log.')),
        h('div', { class: 'side-stack motors-panel' },
          h('section', { class: 'panel' }, h('header', null, h('h3', null, 'Motors')), motorGrid),
          h('div', { class: 'two-col', style: { gap: '12px' } },
            h('section', { class: 'panel' }, h('header', null, h('h3', null, 'Battery')), battery),
            h('section', { class: 'panel' }, h('header', null, h('h3', null, 'Controller')), driver)),
          h('section', { class: 'panel' }, h('header', null, h('h3', null, 'Happening now')), now))));

      // Motor tiles, built once, filled per frame.
      const tiles = motors.map((ch) => {
        const temp = h('span', { class: 'mtemp' });
        const bar = h('span');
        const badges = h('span');
        const stats = h('div', { class: 'mstats' });
        const el = h('div', { class: 'motor' },
          h('div', { class: 'mname' }, h('span', null, A.shortName(ch.name)), badges),
          temp, h('div', { class: 'mbar' }, bar), stats);
        motorGrid.append(el);
        return { ch, el, temp, bar, badges, stats };
      });

      // Charts under the stage.
      const stack = h('div', { class: 'chart-stack' });
      section.append(h('section', { class: 'panel' },
        h('header', null, h('h3', null, 'Around this moment'),
          h('span', { class: 'hint' }, 'Click a chart to jump there; drag across one to zoom; double-click to zoom out.')),
        stack));
      const group = new SA.charts.ChartGroup({
        full: [t0, t1],
        formatTime: (t, long) => (long ? app.label(t) : app.shortLabel(t)),
        onSeek: (t) => app.seek(t),
      });
      const bands = app.bands();
      const markers = app.findingMarkers();
      const charts = [];
      const addChart = (title, series, extra = {}) => {
        const chart = new SA.charts.TimeChart(stack, Object.assign({ title, height: 150, series, bands, markers }, extra));
        group.add(chart);
        charts.push(chart);
      };
      if (lift) {
        const act = run.get(lift.channel);
        addChart(`${lift.name}: target and position`, [
          { label: 'target', color: '--series-1', t: act.t, y: act.cols.target, step: true },
          { label: 'position', color: '--series-2', t: act.t, y: act.cols.pos },
        ]);
      }
      const hot = motors.map((ch) => ({ ch, peak: app.report.motors.find((m) => m.channel === ch.name)?.peakTemp || 0 }))
        .filter((m) => m.peak > 0).sort((a, b) => b.peak - a.peak).slice(0, 4);
      if (hot.length) {
        addChart('Hottest motors (°C)', hot.map((m, k) => ({ label: A.shortName(m.ch.name), color: `--series-${k + 1}`,
          t: m.ch.t, y: m.ch.cols.temp, step: true, unit: '°C' })),
        { guides: [{ y: 55, label: '55°C: derating' }] });
      }
      const err = ['drive', 'turn', 'hold'].map((n) => run.get(n)).filter(Boolean);
      if (err.length) {
        addChart('Drivetrain PID error', err.map((ch, k) => ({ label: ch.name, color: `--series-${k + 1}`, t: ch.t,
          y: ch.cols.err })));
      }
      const batt = run.get('batt');
      if (batt) {
        addChart('Battery (V)', [{ label: 'volts', color: '--series-1', t: batt.t, y: batt.cols.volts, unit: 'V' }]);
      }
      group.redraw();

      // --- Per-frame update ---
      const update = () => {
        const t = app.cursor;
        clock.textContent = app.label(t).split(', ').slice(-1)[0];
        clockSub.textContent = `${app.label(t).split(', ')[0]} · program ${t.toFixed(2)}s`;
        timeline.setAttribute('aria-valuenow', t.toFixed(2));
        timeline.setAttribute('aria-valuetext', app.label(t));
        drawTimeline();
        drawField(field, app, t);
        if (lift) {
          drawLift(liftCanvas, app, t, lift);
          clear(liftInfo);
          const mech = run.get('mech');
          const add = (k, v) => liftInfo.append(h('dt', null, k), h('dd', null, v));
          if (mech) {
            const piece = mech.valueAt('piece_mm', t, 0.5);
            add('level', `${fmt(mech.valueAt('level', t, 0.5), 0)} (mode ${fmt(mech.valueAt('mode', t, 0.5), 0)})`);
            add('claw', mech.valueAt('piston', t, 0.5) === 1 ? 'deployed' : 'retracted');
            add('piece', Number.isFinite(piece) && piece < 100 ? `yes (${piece.toFixed(0)} mm)` : 'no');
            add('intake / claw', `${fmt(mech.valueAt('intake_v', t, 0.5), 1)} / ${fmt(mech.valueAt('claw_v', t, 0.5), 1)} V`);
          }
        }
        for (const tile of tiles) {
          const ch = tile.ch;
          const i = ch.indexAt(t);
          const fresh = i >= 0 && t - ch.t[i] < 0.5;
          const v = (col) => (fresh ? ch.cols[col][i] : NaN);
          const temp = v('temp');
          const gone = fresh && Number.isNaN(v('volts')) && Number.isNaN(temp);
          tile.el.classList.toggle('gone', gone);
          tile.temp.textContent = Number.isFinite(temp) ? `${temp.toFixed(0)}°C` : gone ? 'unplugged' : '—';
          tile.bar.style.width = `${Math.max(0, Math.min(1, ((temp || 0) - 20) / 50)) * 100}%`;
          tile.bar.style.background = heatColor(temp);
          clear(tile.badges);
          const derate = A.reportedDerating(temp);
          if (gone) tile.badges.append(h('span', { class: 'badge' }, 'OFF'));
          else if (derate < 1) tile.badges.append(h('span', { class: 'badge' }, `${Math.round(derate * 100)}%`));
          else if (temp >= 50) tile.badges.append(h('span', { class: 'badge warn' }, 'hot'));
          const faults = v('faults');
          tile.stats.textContent = gone ? '' :
            `${fmt(v('amps'), 2)}A ${fmt(v('volts'), 1)}V ${fmt(v('rpm'), 0)}rpm` +
            (faults & 4 ? ' · I-limit' : '');
        }
        clear(battery);
        if (batt) {
          const volts = batt.valueAt('volts', t, 2);
          const pct = batt.valueAt('pct', t, 2);
          battery.append(h('div', { class: 'mtemp', style: { fontFamily: 'var(--font-display)', fontSize: '1.5rem', fontWeight: '600' } },
            `${fmt(volts, 2)} V`),
          h('div', { class: 'gauge' }, h('span', { style: { width: `${Number.isFinite(pct) ? pct : 0}%` } })),
          h('dl', { class: 'kv' },
            h('dt', null, 'charge'), h('dd', null, `${fmt(pct, 0)}%`),
            batt.has('amps') ? [h('dt', null, 'current'), h('dd', null, `${fmt(batt.valueAt('amps', t, 2), 1)} A`)] : null,
            batt.has('temp') ? [h('dt', null, 'temp'), h('dd', null, `${fmt(batt.valueAt('temp', t, 2), 0)}°C`)] : null));
        } else {
          battery.append(h('p', { class: 'muted' }, 'Not logged.'));
        }
        clear(driver);
        const drv = run.get('driver');
        if (drv) {
          const i = drv.indexAt(t);
          const fresh = i >= 0 && t - drv.t[i] < 0.2;
          const val = (c) => (fresh ? drv.cols[c][i] : 0);
          const stick = (xv, yv) => h('div', { class: 'stick' }, h('span', { style: {
            left: `${50 + 40 * Math.max(-1, Math.min(1, xv))}%`, top: `${50 - 40 * Math.max(-1, Math.min(1, yv))}%` } }));
          const mask = val('buttons');
          const connected = !fresh || val('connected') === 1;
          driver.append(h('div', { class: 'sticks' }, stick(val('lx'), val('ly')), stick(val('rx'), val('ry'))),
            h('div', { class: 'buttons', style: { marginTop: '8px' } },
              BUTTONS.map((b, k) => h('span', { class: mask & (1 << k) ? 'on' : null }, b))),
            h('p', { class: connected ? 'muted' : null, style: { margin: '6px 0 0' } },
              fresh ? (connected ? 'Connected' : SA.ui.severityBadge('critical')) : 'Not in driver control',
              fresh && !connected ? ' Disconnected' : ''));
        } else {
          driver.append(h('p', { class: 'muted' }, 'Not logged.'));
        }
        clear(now);
        const motion = activeMotion(motions, t);
        if (motion) {
          now.append(h('div', { class: 'item' }, h('b', null, motion.kind),
            h('span', { class: 'muted' }, `${(t - motion.start).toFixed(1)}s in · ` +
              (motion.reason ? `ends ${motion.reason}` : 'cut short'))));
        }
        const active = app.report.findings.filter((f) => f.t !== null && f.t <= t && t <= (f.end ?? f.t + 3));
        for (const f of active.slice(0, 4)) {
          now.append(h('div', { class: 'item' }, h('span', null, SA.ui.severityBadge(f.severity), ' ', f.title)));
        }
        const recent = run.events.filter((e) => e.t <= t && e.t > t - 3 && e.tag !== 'motion').slice(-3);
        for (const e of recent) {
          now.append(h('div', { class: 'item muted' }, `${e.tag}: ${e.msg}`));
        }
        if (!now.firstChild) now.append(h('span', { class: 'muted' }, 'Nothing special.'));
        group.setCursor(t);
      };

      const drawTimeline = () => {
        const dpr = window.devicePixelRatio || 1;
        const w = timeline.clientWidth;
        const hh = timeline.clientHeight;
        if (!w) return;
        tlCanvas.width = Math.round(w * dpr);
        tlCanvas.height = Math.round(hh * dpr);
        const ctx = tlCanvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const xOf = (t) => ((t - t0) / (t1 - t0)) * w;
        ctx.fillStyle = token('--surface-2');
        ctx.fillRect(0, 12, w, 22);
        for (const b of bands) {
          ctx.fillStyle = token(b.fill === '--band-auton' ? '--accent-soft' : '--surface-3');
          ctx.fillRect(xOf(b.start), 12, Math.max(1, xOf(b.end) - xOf(b.start)), 22);
        }
        ctx.fillStyle = token('--axis');
        for (const m of motions) {
          ctx.fillRect(xOf(m.start), 26, Math.max(2, xOf(m.stop ?? m.start + 0.2) - xOf(m.start)), 6);
        }
        for (const f of app.report.findings) {
          if (f.t === null) continue;
          const x = xOf(f.t);
          ctx.fillStyle = token(f.severity === 'critical' ? '--critical' : f.severity === 'warning' ? '--warning' : '--accent');
          ctx.beginPath();
          ctx.moveTo(x - 5, 2);
          ctx.lineTo(x + 5, 2);
          ctx.lineTo(x, 10);
          ctx.closePath();
          ctx.fill();
        }
        ctx.fillStyle = token('--muted');
        ctx.font = `10px ${token('--font-data')}`;
        ctx.textBaseline = 'top';
        for (const s of app.sessions) {
          for (const [edge, label] of [[s.auton && s.auton[0], 'auton'], [s.driver && s.driver[0], 'driver']]) {
            if (!edge || edge < t0 || edge > t1) continue;
            ctx.fillText(label, xOf(edge) + 3, 36);
          }
        }
        const x = xOf(app.cursor);
        ctx.fillStyle = token('--accent');
        ctx.fillRect(Math.round(x) - 1, 0, 2, hh - 10);
      };

      const scrub = (e) => {
        const rect = timeline.getBoundingClientRect();
        app.seek(t0 + ((e.clientX - rect.left) / rect.width) * (t1 - t0));
      };
      timeline.addEventListener('pointerdown', (e) => {
        timeline.setPointerCapture(e.pointerId);
        scrub(e);
        const move = (ev) => scrub(ev);
        timeline.addEventListener('pointermove', move);
        timeline.addEventListener('pointerup', () => timeline.removeEventListener('pointermove', move), { once: true });
      });
      timeline.addEventListener('keydown', (e) => {
        if (e.key === 'ArrowRight') app.seek(app.cursor + (e.shiftKey ? 5 : 0.5));
        else if (e.key === 'ArrowLeft') app.seek(app.cursor - (e.shiftKey ? 5 : 0.5));
        else if (e.key === ' ') toggle();
        else return;
        e.preventDefault();
      });

      const frame = (now) => {
        if (!state.playing) return;
        if (state.last !== null) {
          const next = app.cursor + ((now - state.last) / 1000) * state.speed;
          if (next >= t1) {
            app.seek(t1);
            toggle();
            return;
          }
          app.seek(next);
        }
        state.last = now;
        state.raf = requestAnimationFrame(frame);
      };
      const toggle = () => {
        state.playing = !state.playing;
        state.last = null;
        playBtn.textContent = state.playing ? 'Pause' : 'Play';
        playBtn.setAttribute('aria-label', state.playing ? 'Pause' : 'Play');
        if (state.playing) {
          if (app.cursor >= t1 - 0.05) app.seek(t0);
          state.raf = requestAnimationFrame(frame);
        } else {
          cancelAnimationFrame(state.raf);
        }
      };
      playBtn.addEventListener('click', toggle);
      speedSel.addEventListener('change', () => {
        state.speed = Number(speedSel.value);
      });

      const onCursor = () => {
        if (!section.isConnected || section.hidden) return;
        update();
      };
      app.on('cursor', onCursor);
      const resize = new ResizeObserver(() => {
        if (section.isConnected && !section.hidden) update();
      });
      resize.observe(section);
      section._cleanup = () => {
        state.playing = false;
        cancelAnimationFrame(state.raf);
        app.off('cursor', onCursor);
        resize.disconnect();
      };
      requestAnimationFrame(update);
    },

    shown(app) {
      app.emit('cursor', app.cursor);
    },
  };

  root.SA.replay = { drawField, drawLift, fieldOrigin };
})();
