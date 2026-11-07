/*
 * SapphireLib simulator: clawview.js
 *
 * The claw timing page (claw.html): your robot's numbers on the left, and on
 * the right what clawtiming.js makes of them: how fast autoClaw can grab and
 * still catch 95% of Pins, where the jaws shut at each speed, one grab traced
 * ms by ms, what spreads the grabs out, and the lead to set from a grab you
 * filmed. Settings are kept in this browser between visits.
 *
 * Browser only: window.SIM.clawview.start().
 *
 * Team 96671H: Hitmen
 */
(function () {
  'use strict';

  const CT = window.SIM.clawtiming;
  const NS = 'http://www.w3.org/2000/svg';
  const $ = (id) => document.getElementById(id);
  const kStoreKey = 'sapphire.clawtiming.v1';
  const kTrials = 300;
  const kSpeeds = CT.speedRange(2, 70, 2);
  const kTopVolts = 12;

  // macros.cpp's rule and the two before it, in the series order
  const RULES = [
    { key: 'leadAge', name: 'Lead + age (macros.cpp)', cls: 's1' },
    { key: 'lead', name: 'Lead only (Oct 24)', cls: 's2' },
    { key: 'fixed', name: 'Fixed trigger (before)', cls: 's3' },
  ];

  // the robot's drive: devices.cpp's forward axis model, and clawSpeed's cap (macros.cpp)
  const DRIVE_DEFAULTS = { kS: 0.30, kV: 0.1698, clawSpeedVolts: 5.0 };

  const GROUPS = [
    { legend: 'Claw sensor', fields: [
      { key: 'sensorPeriodMs', label: 'Measures every', unit: 'ms', step: 1, min: 1,
        hint: 'About 33ms (30Hz) for the V5 distance sensor.' },
      { key: 'sensorLatencyMs', label: 'Latency', unit: 'ms', step: 1, min: 0,
        hint: 'From measuring to the reading being readable.' },
      { key: 'sensorNoiseMm', label: 'Noise', unit: 'mm', step: 0.5, min: 0,
        hint: '1 standard deviation, a Pin held still. The log\'s piece_mm shows it.' },
    ] },
    { legend: 'Valve and piston', fields: [
      { key: 'valveMs', label: 'Fire to jaws shut', unit: 'ms', step: 1, min: 0,
        hint: 'Slow-motion video: count frames from the valve click to the jaws shut.' },
      { key: 'valveJitterMs', label: 'Varies by', unit: 'ms', step: 0.5, min: 0,
        hint: '1 standard deviation, tank pressure and all. The top speed hangs on it.' },
    ] },
    { legend: 'The claw, as the sensor reads a Pin', fields: [
      { key: 'backMm', label: 'Against the back', unit: 'mm', step: 1, min: 0 },
      { key: 'tipsMm', label: 'Just inside the tips', unit: 'mm', step: 1, min: 0,
        hint: 'Any further out and the jaws shut in front of it.' },
      { key: 'shoveMm', label: 'Push it can take', unit: 'mm', step: 1, min: 0,
        hint: 'How far the back of the claw can shove a Pin before it tips.' },
    ] },
    { legend: 'autoClaw (macros.cpp)', fields: [
      { key: 'grabMm', label: 'kGrabMm', unit: 'mm', step: 1, min: 0,
        hint: 'Where the jaws should shut on the Pin.' },
      { key: 'leadMs', label: 'kClawLeadMs', unit: 'ms', step: 1, min: 0, best: true,
        hint: 'Sensor latency plus fire-to-shut. Best sets it from the lags at the speed under "One grab".' },
      { key: 'maxReachMm', label: 'kGrabMaxMm', unit: 'mm', step: 10, min: 0 },
      { key: 'loopMs', label: 'Program tick', unit: 'ms', select: [[20, 'driver control (20)'], [10, 'autonomous (10)']] },
      { key: 'fixedMm', label: 'Fixed trigger, to compare', unit: 'mm', step: 1, min: 0,
        hint: 'kPieceDetectMm, which autoClaw fired on before the lead.' },
    ] },
    { legend: 'Drive', fields: [
      { key: 'kS', label: 'Forward kS', unit: 'V', step: 0.01, drive: true },
      { key: 'kV', label: 'Forward kV', unit: 'V·s/in', step: 0.001, drive: true,
        hint: 'devices.cpp\'s forward axis model: in/s from volts.' },
      { key: 'clawSpeedVolts', label: 'kClawSpeedVolts', unit: 'V', step: 0.25, drive: true },
      { key: 'speedNoiseInPerS', label: 'Odometry speed noise', unit: 'in/s', step: 0.1, min: 0 },
      { key: 'accelInPerS2', label: 'Speeding up through the grab', unit: 'in/s²', step: 10,
        hint: 'Negative is slowing down: drivers easing off as they reach the Pin.' },
    ] },
  ];

  let p = CT.defaults();
  let drive = { ...DRIVE_DEFAULTS };
  let viewSpeed = 40;
  let traceSeed = 1;
  let calcEarly = true;
  let results = null;
  let pending = 0;

  // --- settings ---

  function load() {
    try {
      const saved = JSON.parse(window.localStorage.getItem(kStoreKey) || 'null');
      if (saved && typeof saved === 'object') {
        p = { ...CT.defaults(), ...saved.p };
        drive = { ...DRIVE_DEFAULTS, ...saved.drive };
        if (Number.isFinite(saved.viewSpeed)) viewSpeed = saved.viewSpeed;
      }
    } catch (e) {
      // no storage here: the defaults
    }
  }

  function save() {
    try {
      window.localStorage.setItem(kStoreKey, JSON.stringify({ p, drive, viewSpeed }));
    } catch (e) {
      // no storage here: nothing kept
    }
  }

  const valueOf = (f) => (f.drive ? drive[f.key] : p[f.key]);

  function buildSettings() {
    const form = $('settings');
    form.textContent = '';
    for (const group of GROUPS) {
      const set = document.createElement('fieldset');
      const legend = document.createElement('legend');
      legend.textContent = group.legend;
      set.appendChild(legend);
      for (const f of group.fields) {
        const row = document.createElement('label');
        row.className = 'num-row';
        const name = document.createElement('span');
        name.textContent = f.label;
        row.appendChild(name);
        const box = document.createElement('span');
        box.className = 'unit-input';
        let input;
        if (f.select) {
          input = document.createElement('select');
          input.className = 'select';
          for (const [value, text] of f.select) {
            const o = document.createElement('option');
            o.value = String(value);
            o.textContent = text;
            input.appendChild(o);
          }
          input.value = String(valueOf(f));
          row.style.gridTemplateColumns = 'minmax(0, 1fr) 150px';
        } else {
          input = document.createElement('input');
          input.type = 'number';
          input.className = 'num-input';
          input.step = String(f.step);
          if (f.min !== undefined) input.min = String(f.min);
          input.value = String(+valueOf(f).toFixed(4));
        }
        input.id = `set-${f.key}`;
        input.addEventListener('input', () => {
          const v = parseFloat(input.value);
          if (!Number.isFinite(v)) return;
          if (f.drive) drive[f.key] = v;
          else p[f.key] = v;
          schedule();
        });
        box.appendChild(input);
        if (!f.select) {
          const unit = document.createElement('i');
          unit.textContent = f.unit;
          box.appendChild(unit);
        }
        row.appendChild(box);
        if (f.hint || f.best) {
          const small = document.createElement('small');
          small.textContent = f.hint || '';
          if (f.best) {
            const best = document.createElement('button');
            best.type = 'button';
            best.className = 'btn';
            best.textContent = 'Best';
            best.style.cssText = 'padding:1px 8px;margin-left:6px;font-size:0.76rem';
            best.addEventListener('click', (event) => {
              event.preventDefault();
              p.leadMs = Math.round(CT.bestLeadMs({ ...p, rule: 'leadAge' }, viewSpeed));
              input.value = String(p.leadMs);
              schedule();
            });
            small.appendChild(best);
          }
          row.appendChild(small);
        }
        set.appendChild(row);
      }
      form.appendChild(set);
    }
  }

  // --- the numbers ---

  const inPerSAt = (volts) => Math.max(0, (volts - drive.kS) / drive.kV);
  const voltsAt = (inPerS) => drive.kS + drive.kV * inPerS;

  function compute() {
    // the old lead rule, given its own best lead, so it's beaten fair and square
    const leadOnly = { ...p, rule: 'lead' };
    leadOnly.leadMs = CT.bestLeadMs(leadOnly, viewSpeed);
    const params = { leadAge: { ...p, rule: 'leadAge' }, lead: leadOnly, fixed: { ...p, rule: 'fixed' } };
    const sweeps = {};
    for (const r of RULES) sweeps[r.key] = CT.sweep(params[r.key], { speeds: kSpeeds, trials: kTrials, seed: 1 });
    return { params, sweeps };
  }

  /** The longest run of speeds at which at least `threshold` of grabs were caught. */
  function goodBand(rows, threshold = 0.95) {
    let best = null;
    let start = null;
    rows.forEach((row, i) => {
      if (row.caught < threshold) {
        start = null;
        return;
      }
      if (start === null) start = i;
      if (!best || i - start > best.len) best = { len: i - start, from: rows[start].speedInPerS, to: row.speedInPerS };
    });
    return best;
  }

  function bandText(band) {
    if (!band) return 'never 95%';
    if (band.from === kSpeeds[0]) return `up to ${band.to} <small>in/s</small>`;
    return `${band.from}–${band.to} <small>in/s only</small>`;
  }

  function rowAt(rows, speed) {
    let best = rows[0];
    for (const r of rows) if (Math.abs(r.speedInPerS - speed) < Math.abs(best.speedInPerS - speed)) best = r;
    return best;
  }

  const pct = (x) => `${Math.round(x * 100)}%`;

  // --- drawing ---

  function el(tag, attrs, parent) {
    const e = document.createElementNS(NS, tag);
    for (const k of Object.keys(attrs || {})) e.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(e);
    return e;
  }

  function text(parent, x, y, str, attrs = {}) {
    const t = el('text', { x, y, ...attrs }, parent);
    t.textContent = str;
    return t;
  }

  function niceStep(span, count) {
    const raw = span / Math.max(1, count);
    const mag = 10 ** Math.floor(Math.log10(raw));
    const n = raw / mag;
    return (n >= 5 ? 10 : n >= 2 ? 5 : n >= 1 ? 2 : 1) * mag;
  }

  /**
   * An empty chart in `host`: axes, grid and a clip for the plot. Returns its svg, scales, plot box
   * and the layers to draw in.
   */
  function frame(host, { height, x, y, xLabel, yLabel, yFormat = String, right = 120 }) {
    host.textContent = '';
    const width = Math.max(280, host.clientWidth || 600);
    const m = { left: 46, right: width < 520 ? Math.min(right, 92) : right, top: 16, bottom: 36 };
    const svg = el('svg', { class: 'chart', viewBox: `0 0 ${width} ${height}`, height, role: 'img' }, host);
    const box = { left: m.left, right: width - m.right, top: m.top, bottom: height - m.bottom };
    const sx = (v) => box.left + ((v - x[0]) / (x[1] - x[0])) * (box.right - box.left);
    const sy = (v) => box.bottom - ((v - y[0]) / (y[1] - y[0])) * (box.bottom - box.top);
    const id = `clip-${host.id}`;
    const defs = el('defs', {}, svg);
    el('rect', { x: box.left, y: box.top - 1, width: box.right - box.left, height: box.bottom - box.top + 2 },
      el('clipPath', { id }, defs));
    const grid = el('g', { class: 'grid' }, svg);
    const yStep = niceStep(y[1] - y[0], 5);
    for (let v = Math.ceil(y[0] / yStep) * yStep; v <= y[1] + 1e-9; v += yStep) {
      el('line', { x1: box.left, x2: box.right, y1: sy(v), y2: sy(v) }, grid);
      text(svg, box.left - 6, sy(v) + 4, yFormat(Math.round(v * 100) / 100), { 'text-anchor': 'end' });
    }
    const xStep = niceStep(x[1] - x[0], 7);
    for (let v = Math.ceil(x[0] / xStep) * xStep; v <= x[1] + 1e-9; v += xStep) {
      text(svg, sx(v), box.bottom + 15, String(Math.round(v)), { 'text-anchor': 'middle' });
    }
    el('line', { class: 'axis', x1: box.left, x2: box.right, y1: box.bottom, y2: box.bottom }, svg);
    text(svg, (box.left + box.right) / 2, height - 4, xLabel, { 'text-anchor': 'middle' });
    text(svg, 4, box.top - 5, yLabel);
    const under = el('g', { 'clip-path': `url(#${id})` }, svg);
    const plot = el('g', { 'clip-path': `url(#${id})` }, svg);
    const over = el('g', {}, svg);
    return { svg, sx, sy, box, under, plot, over, width, height };
  }

  function linePath(points) {
    return points.filter((pt) => Number.isFinite(pt[1])).map((pt, i) => `${i ? 'L' : 'M'}${pt[0].toFixed(1)},${pt[1].toFixed(1)}`).join('');
  }

  /** Direct labels at the right end of each line, nudged apart so none overlap. */
  function endLabels(f, labels) {
    const sorted = labels.filter((l) => Number.isFinite(l.y)).sort((a, b) => a.y - b.y);
    for (let i = 1; i < sorted.length; ++i) sorted[i].y = Math.max(sorted[i].y, sorted[i - 1].y + 14);
    for (const l of sorted) {
      const y = Math.min(Math.max(l.y, f.box.top + 4), f.box.bottom + 2);
      el('line', { class: l.cls, x1: f.box.right + 6, x2: f.box.right + 16, y1: y, y2: y, 'stroke-width': 2 }, f.over);
      text(f.over, f.box.right + 20, y + 4, l.text, { class: 'label' });
    }
  }

  function vref(f, xv, label, atTop = true) {
    const x = f.sx(xv);
    if (x < f.box.left || x > f.box.right) return;
    el('line', { class: 'ref-line', x1: x, x2: x, y1: f.box.top, y2: f.box.bottom }, f.under);
    // a label only where it won't run into another (11px monospace: about 6.6px a character)
    const nearRight = x > f.box.right - 70;
    const span = label.length * 6.6 + 8;
    const extent = nearRight ? [x - 4 - span, x] : [x, x + 4 + span];
    f.refSpans = f.refSpans || [];
    if (f.refSpans.some(([a, b]) => extent[0] < b && a < extent[1])) return;
    f.refSpans.push(extent);
    text(f.over, nearRight ? x - 4 : x + 4, atTop ? f.box.top + 10 : f.box.bottom - 6, label,
      { class: 'ref', 'text-anchor': nearRight ? 'end' : 'start' });
  }

  function href(f, yv, label, anchorRight = false) {
    const y = f.sy(yv);
    if (y < f.box.top || y > f.box.bottom) return;
    el('line', { class: 'ref-line', x1: f.box.left, x2: f.box.right, y1: y, y2: y }, f.under);
    text(f.over, anchorRight ? f.box.right - 4 : f.box.left + 4, y - 4, label,
      { class: 'ref', 'text-anchor': anchorRight ? 'end' : 'start' });
  }

  /** Hover: a crosshair at the nearest x, dots on each series, and a tooltip. */
  function hover(host, f, xs, onAt) {
    const tip = document.createElement('div');
    tip.className = 'tip';
    tip.hidden = true;
    host.appendChild(tip);
    const layer = el('g', {}, f.svg);
    const hit = el('rect', { class: 'hit', x: f.box.left, y: f.box.top, width: f.box.right - f.box.left,
      height: f.box.bottom - f.box.top }, f.svg);
    const move = (event) => {
      const rect = f.svg.getBoundingClientRect();
      const px = ((event.clientX - rect.left) / rect.width) * f.width;
      let best = 0;
      for (let i = 1; i < xs.length; ++i) if (Math.abs(f.sx(xs[i]) - px) < Math.abs(f.sx(xs[best]) - px)) best = i;
      layer.textContent = '';
      const x = f.sx(xs[best]);
      el('line', { class: 'crosshair', x1: x, x2: x, y1: f.box.top, y2: f.box.bottom }, layer);
      const { dots, html } = onAt(best);
      for (const d of dots) {
        if (Number.isFinite(d.y) && d.y >= f.box.top - 1 && d.y <= f.box.bottom + 1) {
          el('circle', { class: `dot ${d.cls}`, cx: x, cy: d.y, r: 4 }, layer);
        }
      }
      tip.innerHTML = html;
      tip.hidden = false;
      // beside the crosshair, on whichever side has room
      const scale = rect.width / f.width;
      let left = x * scale + 14;
      if (left + tip.offsetWidth > rect.width) left = Math.max(0, x * scale - tip.offsetWidth - 14);
      tip.style.left = `${left}px`;
      tip.style.top = `${f.box.top * scale + 4}px`;
    };
    hit.addEventListener('pointermove', move);
    hit.addEventListener('pointerdown', move);
    hit.addEventListener('pointerleave', () => {
      layer.textContent = '';
      tip.hidden = true;
    });
  }

  function legend(id, items) {
    const box = $(id);
    box.textContent = '';
    for (const item of items) {
      const span = document.createElement('span');
      span.className = item.cls;
      const key = document.createElement('i');
      if (item.mark) key.className = item.mark;
      span.appendChild(key);
      const b = document.createElement('b');
      b.textContent = item.text;
      span.appendChild(b);
      box.appendChild(span);
    }
  }

  // --- the panels ---

  function renderTiles() {
    const box = $('tiles');
    box.textContent = '';
    const tile = (label, cls, value, foot) => {
      const t = document.createElement('div');
      t.className = 'tile';
      t.innerHTML = `<span class="label">${cls ? `<i style="color:var(--series-${cls.slice(1)})"></i>` : ''}${label}</span>` +
        `<span class="value">${value}</span><span class="foot">${foot}</span>`;
      box.appendChild(t);
    };
    const main = goodBand(results.sweeps.leadAge);
    const top = main && main.from === kSpeeds[0] ? main.to : 0;
    tile('Lead + age (macros.cpp)', 's1', bandText(main),
      top ? `95% caught. kClawSpeedVolts up to ${voltsAt(top).toFixed(1)}V gets there` : 'Not even slowly: check the claw window and kGrabMm');
    tile(`Lead only, tuned for ${viewSpeed} in/s`, 's2', bandText(goodBand(results.sweeps.lead)),
      `Commit a66eb14's rule, its lead ${Math.round(results.params.lead.leadMs)}ms`);
    tile(`Fixed ${p.fixedMm}mm`, 's3', bandText(goodBand(results.sweeps.fixed)),
      'autoClaw before the lead: right for one band of speed');
    const clawV = inPerSAt(drive.clawSpeedVolts);
    const atClaw = rowAt(results.sweeps.leadAge, clawV);
    tile(`clawSpeed at ${drive.clawSpeedVolts}V`, '', `${clawV.toFixed(0)} <small>in/s</small>`,
      `${pct(atClaw.caught)} caught there with the lead; ${pct(rowAt(results.sweeps.fixed, clawV).caught)} fixed`);
  }

  function renderSweep() {
    const host = $('sweep-chart');
    const f = frame(host, { height: 280, x: [0, 70], y: [0, 100], xLabel: 'Speed onto the Pin (in/s)',
      yLabel: '% caught', yFormat: (v) => `${v}` });
    href(f, 95, '95%');
    vref(f, inPerSAt(drive.clawSpeedVolts), `clawSpeed ${drive.clawSpeedVolts}V`, false);
    vref(f, inPerSAt(kTopVolts), `full ${kTopVolts}V`, false);
    const labels = [];
    for (const r of RULES) {
      const rows = results.sweeps[r.key];
      const pts = rows.map((row) => [f.sx(row.speedInPerS), f.sy(row.caught * 100)]);
      el('path', { class: `line ${r.cls}`, d: linePath(pts) }, f.plot);
      labels.push({ cls: r.cls, y: pts[pts.length - 1][1], text: r.name.replace(/ \(.*\)/, '') });
    }
    endLabels(f, labels);
    hover(host, f, kSpeeds, (i) => {
      const speed = kSpeeds[i];
      let html = `<b>${speed} in/s</b> · ${voltsAt(speed).toFixed(1)}V<br>`;
      const dots = [];
      for (const r of RULES) {
        const row = results.sweeps[r.key][i];
        dots.push({ cls: r.cls, y: f.sy(row.caught * 100) });
        html += `<div class="row"><i class="k ${r.cls}"></i>${r.name}: <span>${pct(row.caught)}</span>` +
          `${row.early > 0.005 ? ` · ${pct(row.early)} early` : ''}${row.late + row.missed > 0.005 ? ` · ${pct(row.late + row.missed)} late` : ''}</div>`;
      }
      return { dots, html };
    });
    legend('sweep-legend', RULES.map((r) => ({ cls: r.cls, text: r.name })));
    renderSweepTable();
  }

  function renderSweepTable() {
    const rows = kSpeeds.map((speed, i) => `<tr><td>${speed}</td><td>${voltsAt(speed).toFixed(1)}</td>` +
      RULES.map((r) => {
        const row = results.sweeps[r.key][i];
        return `<td>${pct(row.caught)}</td><td>${pct(row.early)}</td><td>${pct(row.late + row.missed)}</td>`;
      }).join('') + '</tr>').join('');
    $('sweep-table').innerHTML = '<table class="data-table"><thead><tr><th>in/s</th><th>V</th>' +
      RULES.map((r) => `<th colspan="3">${r.name}</th>`).join('') + '</tr><tr><th></th><th></th>' +
      RULES.map(() => '<th>caught</th><th>early</th><th>late</th>').join('') + `</tr></thead><tbody>${rows}</tbody></table>`;
  }

  function renderShut() {
    const host = $('shut-chart');
    const main = results.sweeps.leadAge;
    const fixed = results.sweeps.fixed;
    const lo = p.backMm - p.shoveMm;
    let yMin = lo - 40;
    let yMax = Math.max(p.tipsMm + 40, p.fixedMm + 15, p.grabMm + 40);
    for (const row of main) {
      if (Number.isFinite(row.meanErrorMm)) {
        yMin = Math.min(yMin, p.grabMm + row.meanErrorMm - 2 * row.sdErrorMm - 5);
        yMax = Math.max(yMax, p.grabMm + row.meanErrorMm + 2 * row.sdErrorMm + 5);
      }
    }
    yMin = Math.max(yMin, lo - 120);
    const f = frame(host, { height: 280, x: [0, 70], y: [yMin, yMax], xLabel: 'Speed onto the Pin (in/s)',
      yLabel: 'Pin as the jaws shut (mm)' });
    el('rect', { class: 'window', x: f.box.left, width: f.box.right - f.box.left, y: f.sy(p.tipsMm),
      height: Math.max(0, f.sy(lo) - f.sy(p.tipsMm)) }, f.under);
    text(f.over, f.box.left + 6, f.sy(p.tipsMm) + 13, 'caught: between the tips and the back');
    href(f, p.grabMm, 'kGrabMm', true);
    const mean = (row, aim) => aim + row.meanErrorMm;
    const upper = main.map((row) => [f.sx(row.speedInPerS), f.sy(mean(row, p.grabMm) + 2 * row.sdErrorMm)]);
    const lower = main.map((row) => [f.sx(row.speedInPerS), f.sy(mean(row, p.grabMm) - 2 * row.sdErrorMm)]).reverse();
    const ok = (pt) => Number.isFinite(pt[1]);
    el('path', { class: 'band s1', d: `${linePath(upper.filter(ok))}${linePath(lower.filter(ok)).replace(/^M/, 'L')}Z` }, f.plot);
    const mainPts = main.map((row) => [f.sx(row.speedInPerS), f.sy(mean(row, p.grabMm))]);
    const fixedPts = fixed.map((row) => [f.sx(row.speedInPerS), f.sy(mean(row, p.fixedMm))]);
    el('path', { class: 'line s1', d: linePath(mainPts) }, f.plot);
    el('path', { class: 'line s3', d: linePath(fixedPts) }, f.plot);
    endLabels(f, [
      { cls: 's1', y: mainPts[mainPts.length - 1][1], text: 'Lead + age' },
      { cls: 's3', y: fixedPts[fixedPts.length - 1][1], text: `Fixed ${p.fixedMm}` },
    ]);
    hover(host, f, kSpeeds, (i) => {
      const a = main[i];
      const b = fixed[i];
      return {
        dots: [{ cls: 's1', y: mainPts[i][1] }, { cls: 's3', y: fixedPts[i][1] }],
        html: `<b>${kSpeeds[i]} in/s</b><br><div class="row"><i class="k s1"></i>Lead + age: <span>` +
          `${Math.round(mean(a, p.grabMm))} ± ${Math.round(2 * a.sdErrorMm)}mm</span> · ${pct(a.caught)} caught</div>` +
          `<div class="row"><i class="k s3"></i>Fixed ${p.fixedMm}: <span>${Math.round(mean(b, p.fixedMm))}mm</span>` +
          ` · ${pct(b.caught)} caught</div>`,
      };
    });
    legend('shut-legend', [
      { cls: 's1', text: 'Lead + age: average, and 95% of grabs (band)' },
      { cls: 's3', text: `Fixed ${p.fixedMm}mm: average` },
    ]);
    // the tilt of the lead's average says which way kClawLeadMs is off (up to 40in/s, short of
    // where kGrabMaxMm starts holding a long lead back)
    const slow = rowAt(main, 10);
    const fast = rowAt(main, 40);
    const tiltMs = ((fast.meanErrorMm - slow.meanErrorMm) / ((fast.speedInPerS - slow.speedInPerS) * 25.4)) * 1000;
    $('shut-note').textContent = Math.abs(tiltMs) < 4
      ? 'The lead\'s average stays level with speed: kClawLeadMs matches the lags here. Only the spread grows, and the window doesn\'t.'
      : `The lead's average ${tiltMs > 0 ? 'climbs' : 'sinks'} with speed: kClawLeadMs is about ${Math.abs(Math.round(tiltMs))}ms too ` +
        `${tiltMs > 0 ? 'long, so fast grabs shut early' : 'short, so fast grabs shut late'}. Best fixes it.`;
  }

  function renderTrace() {
    const host = $('trace-chart');
    const rule = results.params.leadAge;
    const r = CT.grabTrial(rule, viewSpeed, new CT.Rng(traceSeed * 9973 + 17), { trace: true });
    if (r.fireMs === null) {
      host.textContent = '';
      $('trace-note').textContent = r.outcome === 'stopped' ? 'The robot stopped short of the Pin.' : 'It never fired.';
      return;
    }
    const t0 = r.fireMs;
    const x = [-Math.max(120, Math.min(260, 6000 / Math.max(viewSpeed, 1))), r.shutMs - t0 + 40];
    const inX = (t) => t - t0 >= x[0] && t - t0 <= x[1];
    const looks = r.looks.filter((l) => inX(l.tMs));
    const samples = r.samples.filter((s) => inX(s.tMs));
    const lo = p.backMm - p.shoveMm;
    let yMax = Math.max(p.tipsMm + 30, ...looks.map((l) => Math.min(l.readingMm, p.maxReachMm + 60)), ...samples.map((s) => Math.min(s.mm, 600)));
    yMax = Math.min(yMax, r.pinAt(t0 + x[0]) + 30);
    const yMin = Math.min(lo - 20, r.pinAtShutMm - 15, ...looks.map((l) => l.predictedMm - 10));
    const f = frame(host, { height: 300, x, y: [yMin, yMax], xLabel: 'ms from the claw firing', yLabel: 'mm from the sensor' });
    el('rect', { class: 'window', x: f.box.left, width: f.box.right - f.box.left, y: f.sy(p.tipsMm),
      height: Math.max(0, f.sy(lo) - f.sy(p.tipsMm)) }, f.under);
    href(f, p.grabMm, 'kGrabMm', true);
    vref(f, 0, 'fires');
    vref(f, r.shutMs - t0, 'jaws shut');
    // the Pin, really
    const truth = [];
    for (let t = x[0]; t <= x[1]; t += 2) truth.push([f.sx(t), f.sy(r.pinAt(t + t0))]);
    el('path', { class: 'line ink dashed', d: linePath(truth) }, f.plot);
    // what the program read at each look, held between looks
    const steps = [];
    looks.forEach((l, i) => {
      const xa = f.sx(l.tMs - t0);
      const xb = i + 1 < looks.length ? f.sx(looks[i + 1].tMs - t0) : xa;
      steps.push([xa, f.sy(Math.min(l.readingMm, yMax + 50))], [xb, f.sy(Math.min(l.readingMm, yMax + 50))]);
    });
    el('path', { class: 'line s1', d: linePath(steps) }, f.plot);
    for (const l of looks) {
      if (l.readingMm <= yMax) el('circle', { class: 'dot s1', cx: f.sx(l.tMs - t0), cy: f.sy(l.readingMm), r: 4 }, f.plot);
      if (l.readingMm <= p.maxReachMm) {
        el('circle', { class: 'hollow s2', cx: f.sx(l.tMs - t0), cy: f.sy(l.predictedMm), r: 4 }, f.plot);
      }
    }
    for (const s of samples) {
      if (s.mm <= yMax) el('circle', { class: 'hollow ink', cx: f.sx(s.tMs - t0), cy: f.sy(s.mm), r: 3 }, f.plot);
    }
    el('circle', { class: 'dot ink', cx: f.sx(r.shutMs - t0), cy: f.sy(r.pinAtShutMm), r: 5 }, f.over);
    legend('trace-legend', [
      { cls: 'ink', mark: 'dash', text: 'The Pin, really' },
      { cls: 'ink', mark: 'dot', text: 'Sensor measures' },
      { cls: 's1', text: 'What the program reads, each tick' },
      { cls: 's2', mark: 'dot', text: 'Aim: the reading less what\'s still to come' },
    ]);
    const word = { caught: 'caught', early: 'early: the jaws shut in front of it', late: 'late: shoved past the back' }[r.outcome] || r.outcome;
    $('trace-note').textContent = `At ${viewSpeed} in/s it fired on a ${r.readingMm}mm reading ${Math.round(r.ageMs)}ms old, ` +
      `aiming at ${Math.round(r.predictedMm)}mm; the jaws shut ${Math.round(r.shutMs - r.fireMs)}ms later with the Pin at ` +
      `${Math.round(r.pinAtShutMm)}mm: ${word}.`;
    hover(host, f, looks.map((l) => l.tMs - t0), (i) => {
      const l = looks[i];
      return {
        dots: [{ cls: 'ink', y: f.sy(r.pinAt(l.tMs)) }],
        html: `<b>${Math.round(l.tMs - t0)}ms</b><br><div class="row">Pin, really: <span>${Math.round(r.pinAt(l.tMs))}mm</span></div>` +
          `<div class="row"><i class="k s1"></i>Reads: <span>${l.readingMm > 2000 ? 'nothing' : `${l.readingMm}mm`}</span>, ` +
          `${Math.round(l.ageMs)}ms old</div><div class="row"><i class="k s2"></i>Aim: <span>${Math.round(l.predictedMm)}mm</span>` +
          `${l.fires ? ' · fires' : ''}</div>`,
      };
    });
  }

  function renderBudget() {
    const b = CT.errorBudget({ ...p, rule: 'leadAge' }, viewSpeed);
    const half = b.windowMm / 2;
    const scale = Math.max(2 * b.totalMm, half, 1);
    const top = b.items.reduce((a, i) => (i.mm > a.mm ? i : a), b.items[0]);
    const row = (label, mm, cls = '') => `<div class="item ${cls}"><span>${label}</span>` +
      `<div class="bar" style="width:${Math.max(0.5, (100 * mm) / scale)}%"></div><output>${mm.toFixed(1)}mm</output></div>`;
    $('budget').innerHTML = b.items.map((i) => row(i.label, i.mm, i === top ? 'top' : '')).join('') +
      row('All of them (±2σ)', 2 * b.totalMm, 'total') + row('Half the window', half, 'window');
    $('budget-speed').textContent = `at ${viewSpeed} in/s, 1 standard deviation each`;
    $('budget-note').textContent = 2 * b.totalMm < half
      ? `95% of grabs land within ±${(2 * b.totalMm).toFixed(0)}mm, inside the ±${half.toFixed(0)}mm window, as long as kGrabMm sits in its middle (${CT.windowMiddleMm(p).toFixed(0)}mm). The biggest share: ${top.label.toLowerCase()}.`
      : `At ${viewSpeed} in/s the spread (±${(2 * b.totalMm).toFixed(0)}mm) outgrows the ±${half.toFixed(0)}mm window: slow down, or shrink ${top.label.toLowerCase()}, the biggest share.`;
  }

  function renderCalc() {
    const speed = parseFloat($('calc-speed').value);
    const off = parseFloat($('calc-off').value);
    if (!(speed > 0) || !Number.isFinite(off)) {
      $('calc-result').textContent = 'A speed above 0 and a distance.';
      return;
    }
    // early: fired too soon, so less lead; the miss is speed times the lead's error
    const deltaMs = ((calcEarly ? -1 : 1) * off * 1000) / speed;
    const next = Math.max(0, p.leadMs + deltaMs);
    $('calc-result').innerHTML = `<code>kClawLeadMs</code> ${p.leadMs} → <b>${next.toFixed(0)}</b>ms ` +
      `(${deltaMs >= 0 ? '+' : ''}${deltaMs.toFixed(0)}). The faster the grab, the more each ms shows, so tune at the top speed. ` +
      'A miss that\'s the same at every speed is kGrabMm instead.';
  }

  function renderCode() {
    const band = goodBand(results.sweeps.leadAge);
    const top = band && band.from === kSpeeds[0] ? band.to : 0;
    const volts = top ? Math.floor(voltsAt(top) * 4) / 4 : drive.clawSpeedVolts;
    $('code').textContent =
      `// tools/sim/claw.html: 95% of grabs up to ${top} in/s with these settings\n` +
      `constexpr std::int32_t kGrabMm = ${Math.round(p.grabMm)};\n` +
      `constexpr double kClawLeadMs = ${(+p.leadMs).toFixed(1)};\n` +
      `constexpr std::int32_t kGrabMaxMm = ${Math.round(p.maxReachMm)};\n` +
      `constexpr double kClawSpeedVolts = ${volts.toFixed(2)}; // ${inPerSAt(volts).toFixed(0)} in/s; prove it on the robot first`;
  }

  function renderAll() {
    renderTiles();
    renderSweep();
    renderShut();
    renderTrace();
    renderBudget();
    renderCalc();
    renderCode();
  }

  function recompute() {
    results = compute();
    renderAll();
  }

  function schedule() {
    save();
    window.clearTimeout(pending);
    pending = window.setTimeout(recompute, 150);
  }

  function start() {
    load();
    buildSettings();
    $('view-speed').value = String(viewSpeed);
    $('view-speed').addEventListener('input', () => {
      const v = parseFloat($('view-speed').value);
      if (!(v > 0)) return;
      viewSpeed = Math.min(70, v);
      $('calc-speed').value = String(viewSpeed);
      schedule();
    });
    $('another-btn').addEventListener('click', () => {
      traceSeed++;
      renderTrace();
    });
    $('reset-btn').addEventListener('click', () => {
      p = CT.defaults();
      drive = { ...DRIVE_DEFAULTS };
      buildSettings();
      schedule();
    });
    $('theme-btn').addEventListener('click', () => {
      const docEl = document.documentElement;
      const dark = docEl.dataset.theme ? docEl.dataset.theme === 'dark'
        : window.matchMedia('(prefers-color-scheme: dark)').matches;
      docEl.dataset.theme = dark ? 'light' : 'dark';
    });
    $('calc-speed').value = String(viewSpeed);
    for (const id of ['calc-speed', 'calc-off']) $(id).addEventListener('input', renderCalc);
    const setWay = (early) => {
      calcEarly = early;
      $('calc-early').setAttribute('aria-pressed', String(early));
      $('calc-late').setAttribute('aria-pressed', String(!early));
      renderCalc();
    };
    $('calc-early').addEventListener('click', () => setWay(true));
    $('calc-late').addEventListener('click', () => setWay(false));
    $('copy-btn').addEventListener('click', () => {
      const done = () => { $('copy-btn').textContent = 'Copied'; window.setTimeout(() => { $('copy-btn').textContent = 'Copy'; }, 1200); };
      if (navigator.clipboard) navigator.clipboard.writeText($('code').textContent).then(done, () => {});
    });
    let width = 0;
    const observer = new ResizeObserver(() => {
      const w = $('sweep-chart').clientWidth;
      if (results && Math.abs(w - width) > 2) {
        width = w;
        renderSweep();
        renderShut();
        renderTrace();
      }
    });
    observer.observe($('sweep-chart'));
    recompute();
  }

  window.SIM = window.SIM || {};
  window.SIM.clawview = { start };
})();
