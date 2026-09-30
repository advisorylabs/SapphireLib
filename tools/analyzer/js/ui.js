/*
 * SapphireLib telemetry analyzer — ui.js
 *
 * Small DOM helpers shared by the views. Text always goes in through
 * textContent: channel names, event messages and file names all come from a
 * log, and a log is untrusted input.
 *
 * Browser only: window.SA.ui.
 *
 * Team 96671H — Hitmen
 */
(function () {
  'use strict';

  const root = typeof self !== 'undefined' ? self : this;
  root.SA = root.SA || {};

  /**
   * h('div', { class: 'x', onclick: fn, style: {...}, dataset: {...} }, child...)
   * Strings and numbers become text nodes; null/false children are skipped.
   */
  function h(tag, props, ...children) {
    const el = document.createElement(tag);
    if (props) {
      for (const [key, value] of Object.entries(props)) {
        if (value === null || value === undefined || value === false) continue;
        if (key === 'class') el.className = value;
        else if (key === 'style' && typeof value === 'object') Object.assign(el.style, value);
        else if (key === 'dataset') Object.assign(el.dataset, value);
        else if (key.startsWith('on') && typeof value === 'function') {
          el.addEventListener(key.slice(2), value);
        } else if (key === 'text') el.textContent = value;
        else if (value === true) el.setAttribute(key, '');
        else el.setAttribute(key, String(value));
      }
    }
    append(el, children);
    return el;
  }

  function append(el, children) {
    for (const child of children.flat(Infinity)) {
      if (child === null || child === undefined || child === false) continue;
      el.append(child instanceof Node ? child : document.createTextNode(String(child)));
    }
    return el;
  }

  function clear(el) {
    while (el.firstChild) el.removeChild(el.firstChild);
    return el;
  }

  /** The heat class for a motor temperature, and its token. */
  // Green while a motor has full power, then one step per V5 derating step:
  // half its current at 55°C, a quarter at 60°C, an eighth (or none) at 65°C+.
  const HEAT_BINS = [
    { max: 45, token: '--heat-0', label: 'under 45°C' },
    { max: 55, token: '--heat-1', label: '45–55°C' },
    { max: 60, token: '--heat-2', label: '55–60°C (half power)' },
    { max: 65, token: '--heat-3', label: '60–65°C (quarter power)' },
    { max: Infinity, token: '--heat-4', label: '65°C+ (eighth power or off)' },
  ];

  function heatBin(tempC) {
    if (!Number.isFinite(tempC)) return null;
    return HEAT_BINS.find((b) => tempC < b.max) || HEAT_BINS[HEAT_BINS.length - 1];
  }

  function heatColor(tempC) {
    const bin = heatBin(tempC);
    return bin ? SA.charts.token(bin.token) : SA.charts.token('--surface-3');
  }

  /** Relative luminance of a #rrggbb color (WCAG). */
  function luminance(hex) {
    const m = /^#?([0-9a-f]{6})$/i.exec(String(hex).trim());
    if (!m) return null;
    const channel = (i) => {
      const v = parseInt(m[1].slice(i, i + 2), 16) / 255;
      return v <= 0.04045 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4;
    };
    return 0.2126 * channel(0) + 0.7152 * channel(2) + 0.0722 * channel(4);
  }

  /** Ink that reads on a heat fill: white or near-black, whichever contrasts more. */
  function heatInk(tempC) {
    const bin = heatBin(tempC);
    if (!bin) return SA.charts.token('--ink');
    const fill = luminance(SA.charts.token(bin.token));
    if (fill === null) return SA.charts.token('--ink');
    const dark = '#0e1117';
    const onWhite = 1.05 / (fill + 0.05);
    const onDark = (fill + 0.05) / (luminance(dark) + 0.05);
    return onWhite > onDark ? '#ffffff' : dark;
  }

  function heatLegend() {
    return h('div', { class: 'heat-legend' },
      HEAT_BINS.map((b) => h('span', null, h('i', { style: { background: SA.charts.token(b.token) } }),
        b.label)),
      h('span', null, h('i', { style: { background: SA.charts.token('--surface-3') } }), 'no reading'));
  }

  function severityBadge(severity) {
    const label = { critical: 'Critical', warning: 'Warning', info: 'Info', good: 'OK' }[severity];
    return h('span', { class: `sev ${severity}` }, label);
  }

  function fmt(value, digits = 2) {
    if (value === null || value === undefined || Number.isNaN(value)) return '—';
    if (!Number.isFinite(value)) return value > 0 ? '∞' : '−∞';
    return value.toFixed(digits);
  }

  /** Significant-figure formatting for gains and model constants. */
  function sig(value, digits = 3) {
    if (value === null || value === undefined || Number.isNaN(value)) return '—';
    if (!Number.isFinite(value)) return '∞';
    if (value === 0) return '0';
    const abs = Math.abs(value);
    if (abs >= 1000 || abs < 0.001) return value.toExponential(2);
    return Number(value.toPrecision(digits)).toString();
  }

  /** Copies text, falling back to selecting it where the clipboard is refused. */
  async function copyText(text, fallbackEl) {
    try {
      await navigator.clipboard.writeText(text);
      return true;
    } catch (_) {
      if (fallbackEl) {
        const range = document.createRange();
        range.selectNodeContents(fallbackEl);
        const selection = getSelection();
        selection.removeAllRanges();
        selection.addRange(range);
      }
      return false;
    }
  }

  function storage(key, value) {
    try {
      if (value === undefined) return localStorage.getItem(`sapphire-analyzer:${key}`);
      localStorage.setItem(`sapphire-analyzer:${key}`, value);
    } catch (_) {
      // Storage refused (private window, sandbox): only conveniences live here.
    }
    return null;
  }

  root.SA.ui = { h, append, clear, HEAT_BINS, heatBin, heatColor, heatInk, heatLegend,
    severityBadge, fmt, sig, copyText, storage };
})();
