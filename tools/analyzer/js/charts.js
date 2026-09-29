/*
 * SapphireLib telemetry analyzer — charts.js
 *
 * Time-series charts on canvas, fast enough for a whole match at 100Hz: each
 * series is decimated to a min/max pair per pixel column before drawing. A
 * ChartGroup shares one time range and one cursor across stacked charts, so
 * zooming or hovering one moves them all. Marks follow the analyzer's chart
 * spec: 2px lines, hairline solid grid, one y-axis per chart, a legend for two
 * or more series, and a crosshair whose tooltip lists every series.
 *
 * Browser only: window.SA.charts.
 *
 * Team 96671H — Hitmen
 */
(function () {
  'use strict';

  const root = typeof self !== 'undefined' ? self : this;
  root.SA = root.SA || {};

  const MARGIN = { left: 50, right: 12, top: 10, bottom: 22 };

  /** A CSS custom property's current value (tokens change with the theme). */
  function token(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#888';
  }

  function color(spec) {
    return spec && spec.startsWith('--') ? token(spec) : spec;
  }

  /** "Nice" tick values covering [lo, hi], about `count` of them. */
  function niceTicks(lo, hi, count = 5) {
    if (!(hi > lo)) return [lo];
    const raw = (hi - lo) / count;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) || 10 * mag;
    const out = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-6; v += step) {
      out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
    }
    return out;
  }

  function formatTick(v, span) {
    const abs = Math.abs(v);
    if (abs >= 1000) return v.toFixed(0);
    if (span >= 20) return v.toFixed(0);
    if (span >= 2) return v.toFixed(1).replace(/\.0$/, '');
    if (span >= 0.2) return v.toFixed(2).replace(/0$/, '').replace(/\.$/, '');
    return v.toPrecision(3);
  }

  function formatValue(v) {
    if (!Number.isFinite(v)) return Number.isNaN(v) ? '—' : String(v);
    const abs = Math.abs(v);
    if (abs >= 1000) return v.toFixed(0);
    if (abs >= 100) return v.toFixed(1);
    if (abs >= 1) return v.toFixed(2);
    if (abs === 0) return '0';
    return v.toPrecision(3);
  }

  /** Last index with t[i] <= x; -1 if none. */
  function indexAt(t, x) {
    let lo = 0;
    let hi = t.length - 1;
    let found = -1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (t[mid] <= x) {
        found = mid;
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    return found;
  }

  /**
   * One time range and one cursor for a stack of charts. `formatTime(t)`
   * labels x ticks and tooltips; `onSeek(t)` hears clicks.
   */
  class ChartGroup {
    constructor({ full, formatTime, onSeek, onRange } = {}) {
      this.charts = [];
      this.full = full || [0, 1];
      this.range = this.full.slice();
      this.cursor = null;
      this.hover = null;
      this.formatTime = formatTime || ((t) => t.toFixed(1));
      this.onSeek = onSeek || null;
      this.onRange = onRange || null;
    }

    add(chart) {
      this.charts.push(chart);
      chart.group = this;
    }

    remove(chart) {
      this.charts = this.charts.filter((c) => c !== chart);
    }

    setFull(full) {
      this.full = full.slice();
      this.range = full.slice();
      this.redraw();
    }

    setRange(t0, t1) {
      const [f0, f1] = this.full;
      const span = Math.max(0.05, t1 - t0);
      let a = t0;
      let b = t0 + span;
      if (b - a > f1 - f0) {
        a = f0;
        b = f1;
      } else if (a < f0) {
        a = f0;
        b = f0 + span;
      } else if (b > f1) {
        b = f1;
        a = f1 - span;
      }
      this.range = [a, b];
      this.redraw();
      if (this.onRange) this.onRange(this.range);
    }

    zoom(factor, around) {
      const [a, b] = this.range;
      const c = around === undefined ? (a + b) / 2 : around;
      this.setRange(c - (c - a) * factor, c + (b - c) * factor);
    }

    reset() {
      this.setRange(this.full[0], this.full[1]);
    }

    setCursor(t) {
      this.cursor = t;
      for (const chart of this.charts) chart.drawOverlay();
    }

    setHover(t) {
      this.hover = t;
      for (const chart of this.charts) chart.drawOverlay();
    }

    redraw() {
      for (const chart of this.charts) chart.draw();
    }
  }

  /**
   * One chart: a title, a legend (for two or more series), a plot canvas and
   * an overlay canvas for the cursor, plus a tooltip. `series` items are
   * { label, color ('--token' or hex), t: Float64Array (s), y: Float64Array,
   * step?: bool, hidden?: bool }. `bands` shade time spans ({ start, end,
   * fill }), `markers` put ticks along the top ({ t, color, label }), and
   * `guides` draw labelled horizontal reference lines ({ y, label }).
   */
  class TimeChart {
    constructor(parent, opts = {}) {
      this.opts = opts;
      this.series = opts.series || [];
      this.bands = opts.bands || [];
      this.markers = opts.markers || [];
      this.guides = opts.guides || [];
      this.group = null;

      this.el = document.createElement('figure');
      this.el.className = 'chart';
      const head = document.createElement('figcaption');
      head.className = 'chart-head';
      this.titleEl = document.createElement('span');
      this.titleEl.className = 'chart-title';
      this.titleEl.textContent = opts.title || '';
      this.legendEl = document.createElement('span');
      this.legendEl.className = 'chart-legend';
      head.append(this.titleEl, this.legendEl);
      this.wrap = document.createElement('div');
      this.wrap.className = 'chart-canvas';
      this.wrap.style.height = `${opts.height || 150}px`;
      this.wrap.tabIndex = 0;
      this.wrap.setAttribute('role', 'img');
      this.wrap.setAttribute('aria-label', opts.title || 'chart');
      this.plot = document.createElement('canvas');
      this.overlay = document.createElement('canvas');
      this.tooltip = document.createElement('div');
      this.tooltip.className = 'chart-tip';
      this.tooltip.hidden = true;
      this.wrap.append(this.plot, this.overlay, this.tooltip);
      this.el.append(head, this.wrap);
      parent.append(this.el);

      this.yScale = null;
      this.width = 0;
      this.height = 0;
      this.brush = null;
      this.renderLegend();
      this.bindEvents();
      this.resizeObserver = new ResizeObserver(() => this.draw());
      this.resizeObserver.observe(this.wrap);
    }

    setSeries(series) {
      this.series = series;
      this.renderLegend();
      this.draw();
    }

    setTitle(text) {
      this.titleEl.textContent = text;
    }

    renderLegend() {
      this.legendEl.textContent = '';
      if (this.series.length < 2) return;
      for (const s of this.series) {
        const item = document.createElement('button');
        item.type = 'button';
        item.className = 'legend-item' + (s.hidden ? ' off' : '');
        item.title = s.hidden ? 'Show' : 'Hide';
        const key = document.createElement('i');
        key.style.background = color(s.color);
        const label = document.createElement('span');
        label.textContent = s.label;
        item.append(key, label);
        item.addEventListener('click', () => {
          s.hidden = !s.hidden;
          this.renderLegend();
          this.draw();
        });
        this.legendEl.append(item);
      }
    }

    destroy() {
      this.resizeObserver.disconnect();
      if (this.group) this.group.remove(this);
      this.el.remove();
    }

    sizeCanvas(canvas) {
      const dpr = window.devicePixelRatio || 1;
      const w = this.wrap.clientWidth;
      const h = this.wrap.clientHeight;
      if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
        canvas.width = Math.round(w * dpr);
        canvas.height = Math.round(h * dpr);
      }
      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.width = w;
      this.height = h;
      return ctx;
    }

    range() {
      return this.group ? this.group.range : this.opts.range || [0, 1];
    }

    xOf(t) {
      const [a, b] = this.range();
      return MARGIN.left + ((t - a) / (b - a)) * (this.width - MARGIN.left - MARGIN.right);
    }

    tOf(x) {
      const [a, b] = this.range();
      return a + ((x - MARGIN.left) / (this.width - MARGIN.left - MARGIN.right)) * (b - a);
    }

    yOf(v) {
      const [lo, hi] = this.yScale;
      return MARGIN.top + (1 - (v - lo) / (hi - lo)) * (this.height - MARGIN.top - MARGIN.bottom);
    }

    computeY() {
      const [a, b] = this.range();
      let lo = Infinity;
      let hi = -Infinity;
      for (const s of this.series) {
        if (s.hidden) continue;
        const i0 = Math.max(0, indexAt(s.t, a));
        const i1 = Math.min(s.t.length - 1, indexAt(s.t, b) + 1);
        for (let i = i0; i <= i1; ++i) {
          const v = s.y[i];
          if (!Number.isFinite(v)) continue;
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
      }
      for (const g of this.guides) {
        lo = Math.min(lo, g.y);
        hi = Math.max(hi, g.y);
      }
      if (this.opts.yMin !== undefined) lo = Math.min(lo, this.opts.yMin);
      if (this.opts.yMax !== undefined) hi = Math.max(hi, this.opts.yMax);
      if (!Number.isFinite(lo)) {
        lo = 0;
        hi = 1;
      }
      if (hi - lo < 1e-9) {
        lo -= 1;
        hi += 1;
      }
      const pad = (hi - lo) * 0.08;
      this.yScale = [lo - pad, hi + pad];
    }

    draw() {
      if (!this.wrap.isConnected || this.wrap.clientWidth === 0) return;
      const ctx = this.sizeCanvas(this.plot);
      const w = this.width;
      const h = this.height;
      ctx.clearRect(0, 0, w, h);
      this.computeY();
      const [a, b] = this.range();
      const plotW = w - MARGIN.left - MARGIN.right;
      const plotH = h - MARGIN.top - MARGIN.bottom;

      // Bands (phases, highlighted spans) behind everything.
      for (const band of this.bands) {
        const x0 = Math.max(MARGIN.left, this.xOf(band.start));
        const x1 = Math.min(w - MARGIN.right, this.xOf(band.end));
        if (x1 <= x0) continue;
        ctx.fillStyle = color(band.fill);
        ctx.fillRect(x0, MARGIN.top, x1 - x0, plotH);
      }

      // Grid and axes: hairlines, solid, recessive.
      ctx.font = `11px ${token('--font-data')}`;
      ctx.textBaseline = 'middle';
      ctx.lineWidth = 1;
      const [lo, hi] = this.yScale;
      const yTicks = niceTicks(lo, hi, Math.max(2, Math.floor(plotH / 32)));
      for (const v of yTicks) {
        const y = Math.round(this.yOf(v)) + 0.5;
        ctx.strokeStyle = token('--grid');
        ctx.beginPath();
        ctx.moveTo(MARGIN.left, y);
        ctx.lineTo(w - MARGIN.right, y);
        ctx.stroke();
        ctx.fillStyle = token('--muted');
        ctx.textAlign = 'right';
        ctx.fillText(formatTick(v, hi - lo), MARGIN.left - 6, y);
      }
      if (lo < 0 && hi > 0) {
        const y = Math.round(this.yOf(0)) + 0.5;
        ctx.strokeStyle = token('--axis');
        ctx.beginPath();
        ctx.moveTo(MARGIN.left, y);
        ctx.lineTo(w - MARGIN.right, y);
        ctx.stroke();
      }
      const xTicks = niceTicks(a, b, Math.max(2, Math.floor(plotW / 90)));
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      for (const t of xTicks) {
        const x = Math.round(this.xOf(t)) + 0.5;
        if (x < MARGIN.left || x > w - MARGIN.right) continue;
        ctx.strokeStyle = token('--grid');
        ctx.beginPath();
        ctx.moveTo(x, h - MARGIN.bottom);
        ctx.lineTo(x, h - MARGIN.bottom + 4);
        ctx.stroke();
        ctx.fillStyle = token('--muted');
        const label = this.group ? this.group.formatTime(t) : t.toFixed(1);
        ctx.fillText(label, x, h - MARGIN.bottom + 6);
      }
      ctx.strokeStyle = token('--axis');
      ctx.beginPath();
      ctx.moveTo(MARGIN.left, h - MARGIN.bottom + 0.5);
      ctx.lineTo(w - MARGIN.right, h - MARGIN.bottom + 0.5);
      ctx.stroke();

      // Guides: labelled reference lines (a derating threshold, a target).
      for (const g of this.guides) {
        const y = Math.round(this.yOf(g.y)) + 0.5;
        ctx.strokeStyle = color(g.color || '--axis');
        ctx.beginPath();
        ctx.moveTo(MARGIN.left, y);
        ctx.lineTo(w - MARGIN.right, y);
        ctx.stroke();
        if (g.label) {
          // On a surface-colored plate, so it reads over the lines it crosses.
          ctx.textAlign = 'left';
          ctx.textBaseline = 'bottom';
          const tw = ctx.measureText(g.label).width;
          ctx.fillStyle = token('--surface');
          ctx.fillRect(MARGIN.left + 4, y - 15, tw + 6, 13);
          ctx.fillStyle = token('--ink-2');
          ctx.fillText(g.label, MARGIN.left + 7, y - 3);
        }
      }

      // Series.
      ctx.save();
      ctx.beginPath();
      ctx.rect(MARGIN.left, MARGIN.top - 2, plotW, plotH + 4);
      ctx.clip();
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      for (const s of this.series) {
        if (s.hidden) continue;
        ctx.strokeStyle = color(s.color);
        ctx.lineWidth = s.width || 2;
        this.drawSeries(ctx, s, a, b);
      }
      ctx.restore();

      // Markers along the top (findings, events).
      for (const m of this.markers) {
        const x = this.xOf(m.t);
        if (x < MARGIN.left || x > w - MARGIN.right) continue;
        ctx.fillStyle = color(m.color);
        ctx.beginPath();
        ctx.moveTo(x - 4, MARGIN.top - 2);
        ctx.lineTo(x + 4, MARGIN.top - 2);
        ctx.lineTo(x, MARGIN.top + 5);
        ctx.closePath();
        ctx.fill();
      }
      this.drawOverlay();
    }

    drawSeries(ctx, s, a, b) {
      const t = s.t;
      const y = s.y;
      if (t.length === 0) return;
      const i0 = Math.max(0, indexAt(t, a));
      const i1 = Math.min(t.length - 1, indexAt(t, b) + 1);
      const plotW = this.width - MARGIN.left - MARGIN.right;
      const perPixel = (i1 - i0) / Math.max(1, plotW);
      ctx.beginPath();
      let pen = false;
      if (perPixel > 2) {
        // Min/max per pixel column: every spike stays visible at any zoom.
        let col = null;
        let mn = 0;
        let mx = 0;
        let first = 0;
        let last = 0;
        const flush = () => {
          if (col === null) return;
          const yFirst = this.yOf(first);
          if (!pen) ctx.moveTo(col, yFirst);
          else ctx.lineTo(col, yFirst);
          ctx.lineTo(col, this.yOf(mn));
          ctx.lineTo(col, this.yOf(mx));
          ctx.lineTo(col, this.yOf(last));
          pen = true;
        };
        for (let i = i0; i <= i1; ++i) {
          const v = y[i];
          if (!Number.isFinite(v)) {
            flush();
            col = null;
            pen = false;
            continue;
          }
          const x = Math.round(this.xOf(t[i]));
          if (x !== col) {
            flush();
            col = x;
            mn = mx = first = last = v;
          } else {
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            last = v;
          }
        }
        flush();
      } else {
        let prevY = null;
        for (let i = i0; i <= i1; ++i) {
          const v = y[i];
          if (!Number.isFinite(v)) {
            pen = false;
            continue;
          }
          const x = this.xOf(t[i]);
          const py = this.yOf(v);
          if (!pen) {
            ctx.moveTo(x, py);
            pen = true;
          } else if (s.step) {
            ctx.lineTo(x, prevY);
            ctx.lineTo(x, py);
          } else {
            ctx.lineTo(x, py);
          }
          prevY = py;
        }
      }
      ctx.stroke();
    }

    drawOverlay() {
      if (!this.wrap.isConnected || this.wrap.clientWidth === 0 || !this.yScale) return;
      const ctx = this.sizeCanvas(this.overlay);
      ctx.clearRect(0, 0, this.width, this.height);
      const plotH = this.height - MARGIN.top - MARGIN.bottom;
      const g = this.group;
      if (this.brush) {
        const x0 = Math.min(this.brush.x0, this.brush.x1);
        const x1 = Math.max(this.brush.x0, this.brush.x1);
        ctx.fillStyle = token('--accent-soft');
        ctx.globalAlpha = 0.6;
        ctx.fillRect(x0, MARGIN.top, x1 - x0, plotH);
        ctx.globalAlpha = 1;
      }
      const line = (t, style, width) => {
        const x = Math.round(this.xOf(t)) + 0.5;
        if (x < MARGIN.left || x > this.width - MARGIN.right) return;
        ctx.strokeStyle = style;
        ctx.lineWidth = width;
        ctx.beginPath();
        ctx.moveTo(x, MARGIN.top);
        ctx.lineTo(x, MARGIN.top + plotH);
        ctx.stroke();
      };
      if (g && g.cursor !== null) line(g.cursor, token('--accent'), 2);
      if (g && g.hover !== null) {
        line(g.hover, token('--ink-2'), 1);
        // A dot where each series crosses the hover line.
        for (const s of this.series) {
          if (s.hidden) continue;
          const i = indexAt(s.t, g.hover);
          if (i < 0 || !Number.isFinite(s.y[i])) continue;
          const x = this.xOf(g.hover);
          const y = this.yOf(s.y[i]);
          ctx.fillStyle = token('--surface');
          ctx.beginPath();
          ctx.arc(x, y, 6, 0, Math.PI * 2);
          ctx.fill();
          ctx.fillStyle = color(s.color);
          ctx.beginPath();
          ctx.arc(x, y, 4, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }

    showTooltip(px, t) {
      const rows = this.series.filter((s) => !s.hidden).map((s) => {
        const i = indexAt(s.t, t);
        return { s, v: i >= 0 ? s.y[i] : NaN };
      });
      this.tooltip.textContent = '';
      const time = document.createElement('div');
      time.className = 'tip-time';
      time.textContent = this.group ? this.group.formatTime(t, true) : t.toFixed(2);
      this.tooltip.append(time);
      for (const { s, v } of rows) {
        const row = document.createElement('div');
        row.className = 'tip-row';
        const key = document.createElement('i');
        key.style.background = color(s.color);
        const value = document.createElement('b');
        value.textContent = formatValue(v) + (s.unit ? ` ${s.unit}` : '');
        const label = document.createElement('span');
        label.textContent = s.label;
        row.append(key, value, label);
        this.tooltip.append(row);
      }
      this.tooltip.hidden = false;
      const tipW = this.tooltip.offsetWidth;
      const left = px + 14 + tipW > this.width ? px - 14 - tipW : px + 14;
      this.tooltip.style.left = `${Math.max(0, left)}px`;
      this.tooltip.style.top = `${MARGIN.top}px`;
    }

    bindEvents() {
      const pos = (e) => {
        const rect = this.wrap.getBoundingClientRect();
        return e.clientX - rect.left;
      };
      this.wrap.addEventListener('pointermove', (e) => {
        const x = pos(e);
        if (x < MARGIN.left || x > this.width - MARGIN.right) {
          this.tooltip.hidden = true;
          if (this.group) this.group.setHover(null);
          return;
        }
        const t = this.tOf(x);
        if (this.brush) {
          this.brush.x1 = x;
          this.drawOverlay();
        }
        if (this.group) this.group.setHover(t);
        this.showTooltip(x, t);
      });
      this.wrap.addEventListener('pointerleave', () => {
        this.tooltip.hidden = true;
        if (this.group) this.group.setHover(null);
      });
      this.wrap.addEventListener('pointerdown', (e) => {
        if (e.button !== 0) return;
        this.wrap.setPointerCapture(e.pointerId);
        this.brush = { x0: pos(e), x1: pos(e) };
      });
      this.wrap.addEventListener('pointerup', (e) => {
        if (!this.brush) return;
        const { x0 } = this.brush;
        const x1 = pos(e);
        this.brush = null;
        this.drawOverlay();
        if (Math.abs(x1 - x0) > 6 && this.group) {
          this.group.setRange(this.tOf(Math.min(x0, x1)), this.tOf(Math.max(x0, x1)));
        } else if (this.group && this.group.onSeek) {
          this.group.onSeek(this.tOf(x1));
        }
      });
      this.wrap.addEventListener('dblclick', () => {
        if (this.group) this.group.reset();
      });
      this.wrap.addEventListener('wheel', (e) => {
        if (!this.group || !(e.ctrlKey || e.metaKey)) return;
        e.preventDefault();
        this.group.zoom(e.deltaY > 0 ? 1.25 : 0.8, this.tOf(pos(e)));
      }, { passive: false });
      this.wrap.addEventListener('keydown', (e) => {
        const g = this.group;
        if (!g) return;
        const [a, b] = g.range;
        const step = (b - a) / 50;
        const now = g.cursor ?? (a + b) / 2;
        if (e.key === 'ArrowRight' && g.onSeek) g.onSeek(now + step);
        else if (e.key === 'ArrowLeft' && g.onSeek) g.onSeek(now - step);
        else if (e.key === '+' || e.key === '=') g.zoom(0.8, now);
        else if (e.key === '-') g.zoom(1.25, now);
        else if (e.key === '0') g.reset();
        else return;
        e.preventDefault();
      });
    }
  }

  root.SA.charts = { ChartGroup, TimeChart, token, color, niceTicks, formatValue, indexAt };
})();
