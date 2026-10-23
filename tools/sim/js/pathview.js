/*
 * SapphireLib simulator: pathview.js
 *
 * The Paths tab: draw a trajectory chain on the field, split it into
 * sections with their own limits, see the speed the planner gives it, drive
 * it in the simulator, and copy it out as C++ for src/robot/autons.cpp (and
 * paste that back in to keep editing). The robot plans the same chain with
 * the same planner (trajectory.js is its port), so what this shows is what
 * the robot drives, given the same axis models.
 *
 * The path document, and its C++ in both directions, are plain functions
 * that also load under Node for the tests. build() wires the tab, in a
 * browser only.
 *
 * A document is { name, waypoints: [{ xIn, yIn, headingDeg }] (headingDeg
 * null for none), breaks, sections }: section k ends at waypoint breaks[k]
 * (each section after the first starts from the previous one's last point),
 * and sections[k] is { constraints, stopAtEnd }.
 *
 * Plain script: window.SIM.pathview in a browser, require('./pathview.js') in
 * Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./trajectory.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.pathview = factory(root.SIM.trajectory);
  }
})(function (T) {
  'use strict';

  const kStorageKey = 'sapphire-sim.paths.v1';
  const kSnapIn = 0.5;
  const kFieldHalfIn = 70.25;
  const AXES = [['forward', 'Fwd', 'in'], ['strafe', 'Strafe', 'in'], ['turn', 'Turn', '°']];
  // each section's color, by the page's CSS tokens (render.js reads the same names)
  const SECTION_COLORS = ['series-1', 'series-3', 'series-7', 'series-5', 'series-4', 'series-2'];

  // --- The document -----------------------------------------------------------------------------

  function clone(value) {
    return JSON.parse(JSON.stringify(value));
  }

  function emptyDoc(name = 'path') {
    return { name, waypoints: [], breaks: [], sections: [{ constraints: T.trajectoryConstraints(), stopAtEnd: false }] };
  }

  /** A document from a chain of sections (TrajectorySection-shaped objects). */
  function fromSections(sections, name = 'path') {
    const doc = { name, waypoints: [], breaks: [], sections: [] };
    sections.forEach((section, k) => {
      for (const w of section.waypoints) {
        doc.waypoints.push({ xIn: w.xIn, yIn: w.yIn, headingDeg: w.headingDeg === undefined ? null : w.headingDeg });
      }
      if (k + 1 < sections.length) doc.breaks.push(doc.waypoints.length - 1);
      doc.sections.push({ constraints: T.trajectoryConstraints(section.constraints), stopAtEnd: !!section.stopAtEnd });
    });
    if (doc.sections.length === 0) doc.sections.push({ constraints: T.trajectoryConstraints(), stopAtEnd: false });
    return doc;
  }

  /** The chain generateTrajectory() takes. */
  function toSections(doc) {
    const out = [];
    let start = 0;
    for (let k = 0; k < doc.sections.length; ++k) {
      const end = k < doc.breaks.length ? doc.breaks[k] : doc.waypoints.length - 1;
      const waypoints = doc.waypoints.slice(start, end + 1).map((w) => {
        const point = { xIn: w.xIn, yIn: w.yIn };
        if (w.headingDeg !== null && w.headingDeg !== undefined) point.headingDeg = w.headingDeg;
        return point;
      });
      out.push({ waypoints, constraints: clone(doc.sections[k].constraints), stopAtEnd: doc.sections[k].stopAtEnd });
      start = end + 1;
    }
    return out;
  }

  /** The section waypoint j is in. A section's last point (the junction) is its own. */
  function sectionOf(doc, j) {
    let k = 0;
    while (k < doc.breaks.length && doc.breaks[k] < j) ++k;
    return k;
  }

  /** Merge section k + 1 into section k: the joined section keeps k's limits and k + 1's stop. */
  function mergeSections(doc, k) {
    if (k < 0 || k >= doc.breaks.length) return;
    doc.breaks.splice(k, 1);
    doc.sections[k].stopAtEnd = doc.sections[k + 1].stopAtEnd;
    doc.sections.splice(k + 1, 1);
  }

  /** Breaks must sit strictly inside the path; drop any that don't, merging their sections. */
  function tidyBreaks(doc) {
    for (let k = doc.breaks.length - 1; k >= 0; --k) {
      const b = doc.breaks[k];
      if (b < 1 || b > doc.waypoints.length - 2 || (k > 0 && doc.breaks[k - 1] >= b)) mergeSections(doc, k);
    }
  }

  function insertWaypoint(doc, index, point) {
    const w = { xIn: point.xIn, yIn: point.yIn, headingDeg: point.headingDeg === undefined ? null : point.headingDeg };
    // the first point is where the robot starts: it always has a heading
    if (doc.waypoints.length === 0 && w.headingDeg === null) w.headingDeg = 0;
    doc.waypoints.splice(index, 0, w);
    for (let k = 0; k < doc.breaks.length; ++k) if (doc.breaks[k] >= index) doc.breaks[k]++;
    tidyBreaks(doc);
    return index;
  }

  function addWaypoint(doc, point) {
    return insertWaypoint(doc, doc.waypoints.length, point);
  }

  function removeWaypoint(doc, index) {
    if (index < 0 || index >= doc.waypoints.length) return;
    const k = doc.breaks.indexOf(index);
    if (k >= 0) mergeSections(doc, k);
    doc.waypoints.splice(index, 1);
    for (let i = 0; i < doc.breaks.length; ++i) if (doc.breaks[i] > index) doc.breaks[i]--;
    if (index === 0 && doc.waypoints.length && doc.waypoints[0].headingDeg === null) doc.waypoints[0].headingDeg = 0;
    tidyBreaks(doc);
  }

  /** End a section at waypoint j; the new section after it copies the limits it splits off from. */
  function splitAt(doc, j) {
    if (j < 1 || j > doc.waypoints.length - 2 || doc.breaks.includes(j)) return false;
    const k = sectionOf(doc, j);
    doc.breaks.splice(k, 0, j);
    const original = doc.sections[k];
    doc.sections.splice(k + 1, 0, { constraints: clone(original.constraints), stopAtEnd: original.stopAtEnd });
    original.stopAtEnd = false;
    return true;
  }

  /**
   * The same chain driven backward: the points in reverse, each junction's stop moved to the
   * section that now ends there, and the start facing the way the old end did.
   */
  function reverse(doc) {
    const n = doc.waypoints.length;
    if (n === 0) return;
    // the heading the old end faced: its own, or the last one set before it
    let endHeading = 0;
    for (const w of doc.waypoints) if (w.headingDeg !== null) endHeading = w.headingDeg;
    const points = doc.waypoints.slice().reverse();
    if (points[0].headingDeg === null) points[0].headingDeg = endHeading;
    const breaks = doc.breaks.map((b) => n - 1 - b).reverse();
    // reversed section k ends where old section (count - 2 - k) did
    const sections = doc.sections.slice().reverse().map((s, k, all) => ({
      constraints: s.constraints,
      stopAtEnd: k + 1 < all.length ? all[k + 1].stopAtEnd : false,
    }));
    Object.assign(doc, { waypoints: points, breaks, sections });
  }

  /** Index to insert a point clicked near the path at: after the nearest chord's start. */
  function insertionIndex(doc, xIn, yIn) {
    let best = doc.waypoints.length;
    let bestDistance = Infinity;
    for (let j = 0; j + 1 < doc.waypoints.length; ++j) {
      const a = doc.waypoints[j];
      const b = doc.waypoints[j + 1];
      const dx = b.xIn - a.xIn;
      const dy = b.yIn - a.yIn;
      const lengthSq = dx * dx + dy * dy;
      const f = lengthSq > 0 ? Math.min(1, Math.max(0, ((xIn - a.xIn) * dx + (yIn - a.yIn) * dy) / lengthSq)) : 0;
      const d = Math.hypot(a.xIn + f * dx - xIn, a.yIn + f * dy - yIn);
      if (d < bestDistance) {
        bestDistance = d;
        best = j + 1;
      }
    }
    return best;
  }

  // --- C++ out --------------------------------------------------------------------------------

  function num(value) {
    const rounded = Math.round(value * 100) / 100;
    return String(Object.is(rounded, -0) ? 0 : rounded);
  }

  /** A C++ identifier from the path's name: "left rush" is kLeftRush. */
  function identifier(name) {
    const words = String(name || 'path').replace(/[^A-Za-z0-9]+/g, ' ').trim().split(/\s+/).filter(Boolean);
    const camel = words.map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join('') || 'Path';
    return `k${/^[0-9]/.test(camel) ? `Path${camel}` : camel}`;
  }

  function waypointLiteral(w) {
    const heading = w.headingDeg !== undefined && w.headingDeg !== null ? `, .headingDeg = ${num(w.headingDeg)}` : '';
    return `{.xIn = ${num(w.xIn)}, .yIn = ${num(w.yIn)}${heading}}`;
  }

  /** The non-default constraints, in declaration order, or '' for all defaults. */
  function constraintsLiteral(c) {
    const defaults = T.trajectoryConstraints();
    const fields = [];
    if (c.voltageBudget !== defaults.voltageBudget) fields.push(`.voltageBudget = ${num(c.voltageBudget)}`);
    for (const [axis] of AXES) {
      const limits = c[axis];
      const parts = [];
      if (limits.maxVelocity > 0) parts.push(`.maxVelocity = ${num(limits.maxVelocity)}`);
      if (limits.maxAcceleration > 0) parts.push(`.maxAcceleration = ${num(limits.maxAcceleration)}`);
      if (parts.length) fields.push(`.${axis} = {${parts.join(', ')}}`);
    }
    return fields.length ? `{${fields.join(', ')}}` : '';
  }

  /**
   * The chain as C++ to paste into autons.cpp. `about` is the comment's
   * description of the plan, e.g. "3.42 s planned with the Robot tab's models".
   */
  function toCpp(doc, about = '') {
    const name = identifier(doc.name);
    const lines = [`// Sapphire Sim path "${doc.name}"${about ? `: ${about}` : ''}`,
      `const std::vector<sapphirelib::motion::TrajectorySection> ${name}{`];
    const sections = toSections(doc);
    sections.forEach((section) => {
      const open = '    {.waypoints = {';
      const pad = ' '.repeat(open.length);
      const points = section.waypoints.map(waypointLiteral);
      const body = [`${open}${points.join(`,\n${pad}`)}}`];
      const constraints = constraintsLiteral(section.constraints);
      if (constraints) body.push(`     .constraints = ${constraints}`);
      if (section.stopAtEnd) body.push('     .stopAtEnd = true');
      lines.push(`${body.join(',\n')}},`);
    });
    lines.push('};');
    return lines.join('\n');
  }

  // --- C++ in ---------------------------------------------------------------------------------

  /**
   * Reads C++ that toCpp() wrote (or hand-edited the same way): a
   * std::vector of TrajectorySection, or of TrajectoryWaypoint for a single
   * section. Returns { ok, error, doc }.
   */
  function parseCpp(text) {
    const source = String(text).replace(/\/\*[\s\S]*?\*\//g, ' ').replace(/\/\/[^\n]*/g, ' ');
    const nameMatch = source.match(/\bk([A-Za-z0-9_]+)\s*\{/);
    const start = nameMatch ? source.indexOf('{', nameMatch.index) : source.indexOf('{');
    if (start < 0) return { ok: false, error: 'no { ... } to read', doc: null };
    let i = start;
    const fail = (message) => { throw new Error(`${message} (character ${i})`); };
    const skip = () => { while (i < source.length && /\s/.test(source[i])) ++i; };
    const value = () => {
      skip();
      if (source[i] === '{') {
        ++i;
        const items = [];
        const named = {};
        let anyNamed = false;
        skip();
        while (source[i] !== '}') {
          if (i >= source.length) fail('missing }');
          skip();
          if (source[i] === '.') {
            ++i;
            const m = /^[A-Za-z_]\w*/.exec(source.slice(i));
            if (!m) fail('expected a field name after .');
            i += m[0].length;
            skip();
            if (source[i] !== '=') fail(`expected = after .${m[0]}`);
            ++i;
            named[m[0]] = value();
            anyNamed = true;
          } else {
            items.push(value());
          }
          skip();
          if (source[i] === ',') ++i;
          skip();
        }
        ++i;
        if (anyNamed && items.length) fail('mixed named and unnamed values');
        return anyNamed ? named : items;
      }
      const m = /^(true|false|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[fF]?/.exec(source.slice(i));
      if (!m) fail('expected a number, true, false, or {');
      i += m[0].length;
      if (m[1] === 'true') return true;
      if (m[1] === 'false') return false;
      return Number(m[1]);
    };
    try {
      const top = value();
      if (!Array.isArray(top) || top.length === 0) throw new Error('expected a list of sections');
      const isWaypoint = (o) => o && typeof o === 'object' && !Array.isArray(o) && 'xIn' in o;
      const sections = isWaypoint(top[0]) ? [{ waypoints: top }] : top;
      const chain = sections.map((s, k) => {
        if (!s || Array.isArray(s) || !Array.isArray(s.waypoints)) throw new Error(`section ${k} has no .waypoints`);
        for (const w of s.waypoints) {
          if (!isWaypoint(w) || typeof w.xIn !== 'number' || typeof w.yIn !== 'number') {
            throw new Error(`section ${k}: every waypoint needs .xIn and .yIn`);
          }
        }
        return { waypoints: s.waypoints, constraints: s.constraints || {}, stopAtEnd: s.stopAtEnd === true };
      });
      const name = nameMatch ? nameMatch[1].replace(/([a-z0-9])([A-Z])/g, '$1 $2').toLowerCase() : 'pasted';
      return { ok: true, error: '', doc: fromSections(chain, name) };
    } catch (error) {
      return { ok: false, error: error.message, doc: null };
    }
  }

  // --- The tab (browser only) ------------------------------------------------------------------

  /**
   * Wires the Paths tab. `ctx` is the page's: { $, el, fmt, segmented,
   * options, renderer, runPath(routine), tuneModels(), defaultChain } where
   * tuneModels() is the axis models of the TUNE.CFG the Tune tab has open, or
   * null, and defaultChain is the path a first visit starts with.
   */
  function build(ctx) {
    const { $, el, fmt } = ctx;
    const state = {
      paths: [],
      current: 0,
      selected: -1,
      modelsSource: 'robot',
      plan: null,
      planDirty: true,
      drag: null,
      active: false,
    };

    function load() {
      try {
        const stored = JSON.parse(localStorage.getItem(kStorageKey) || 'null');
        if (stored && Array.isArray(stored.paths) && stored.paths.length) {
          state.paths = stored.paths;
          state.current = Math.min(stored.current || 0, stored.paths.length - 1);
          return;
        }
      } catch (error) {
        // no storage here (a private window, a file:// page that refuses it): start fresh
      }
      state.paths = [fromSections(ctx.defaultChain, 'tour')];
      state.current = 0;
    }

    function save() {
      try {
        localStorage.setItem(kStorageKey, JSON.stringify({ paths: state.paths, current: state.current }));
      } catch (error) {
        // drafts just don't outlive the page
      }
    }

    const doc = () => state.paths[state.current];

    function models() {
      if (state.modelsSource === 'tune') {
        const m = ctx.tuneModels();
        if (m) return m;
      }
      return ctx.options.world.models;
    }

    function modelsLabel() {
      if (state.modelsSource === 'tune' && ctx.tuneModels()) return 'the open TUNE.CFG\'s models';
      return 'the Robot tab\'s models';
    }

    function changed({ list = true } = {}) {
      state.planDirty = true;
      save();
      if (list) renderAll();
    }

    function replan() {
      if (!state.planDirty) return state.plan;
      state.planDirty = false;
      const d = doc();
      state.plan = d.waypoints.length >= 2 ? T.generateTrajectory(toSections(d), models())
        : { ok: false, error: 'click the field to add points (at least 2)', trajectory: new T.Trajectory() };
      renderPlan();
      return state.plan;
    }

    // -- panels --

    function renderSelect() {
      const select = $('path-select');
      select.replaceChildren(...state.paths.map((p, i) => el('option', { value: String(i), text: p.name || `path ${i + 1}` })));
      select.value = String(state.current);
      $('path-name').value = doc().name;
    }

    function numberInput(value, step, onChange, attrs = {}) {
      const input = el('input', Object.assign({ type: 'number', step, value: String(value), class: 'num-input' }, attrs));
      input.addEventListener('change', () => {
        const v = Number(input.value);
        if (Number.isFinite(v)) onChange(v);
      });
      return input;
    }

    function renderPoints() {
      const d = doc();
      const table = $('path-points');
      const head = el('thead', {}, el('tr', {}, ['#', 'x', 'y', 'Heading', 'Sec', ''].map((t) => el('th', { text: t }))));
      const rows = d.waypoints.map((w, j) => {
        const section = sectionOf(d, j);
        const isBreak = d.breaks.includes(j);
        const hasHeading = w.headingDeg !== null;
        const check = el('input', { type: 'checkbox', title: j === 0 ? 'The first point is where the robot starts: it needs a heading' : 'Face a heading here' });
        check.checked = hasHeading;
        check.disabled = j === 0;
        check.addEventListener('change', () => {
          w.headingDeg = check.checked ? Math.round(plannedHeadingAt(j)) : null;
          changed();
        });
        const heading = numberInput(hasHeading ? w.headingDeg : '', 1, (v) => { w.headingDeg = v; changed(); },
          { disabled: !hasHeading, 'aria-label': `Waypoint ${j} heading` });
        const swatch = el('i', { class: 'sec-sw' });
        swatch.style.background = sectionColor(section);
        const actions = el('td', { class: 'actions' }, [
          j >= 1 && j <= d.waypoints.length - 2 && !isBreak
            ? el('button', { class: 'mini', type: 'button', text: 'Split', title: 'End a section here: the rest of the path gets its own limits',
              onclick: () => { splitAt(d, j); changed(); } }) : null,
          el('button', { class: 'mini', type: 'button', text: '✕', title: 'Remove this point', 'aria-label': `Remove waypoint ${j}`,
            onclick: () => { removeWaypoint(d, j); state.selected = -1; changed(); } }),
        ]);
        const row = el('tr', { class: j === state.selected ? 'selected' : '' }, [
          el('td', { text: String(j) }),
          el('td', {}, numberInput(w.xIn, kSnapIn, (v) => { w.xIn = clampField(v); changed(); }, { 'aria-label': `Waypoint ${j} x` })),
          el('td', {}, numberInput(w.yIn, kSnapIn, (v) => { w.yIn = clampField(v); changed(); }, { 'aria-label': `Waypoint ${j} y` })),
          el('td', { class: 'heading-cell' }, [check, heading]),
          el('td', {}, [swatch, String(section)]),
          actions,
        ]);
        row.addEventListener('click', (event) => {
          if (event.target.closest('input, button')) return;
          state.selected = j;
          renderPoints();
        });
        return row;
      });
      table.replaceChildren(head, el('tbody', {}, rows));
    }

    function renderSections() {
      const d = doc();
      const box = $('path-sections');
      box.replaceChildren(...d.sections.map((section, k) => {
        const c = section.constraints;
        const first = k === 0 ? 0 : d.breaks[k - 1];
        const last = k < d.breaks.length ? d.breaks[k] : d.waypoints.length - 1;
        const swatch = el('i', { class: 'sec-sw' });
        swatch.style.background = sectionColor(k);
        const grid = el('div', { class: 'limit-grid' }, [
          el('span'), el('span', { class: 'h', text: 'top speed' }), el('span', { class: 'h', text: 'top accel' }),
          ...AXES.flatMap(([axis, label, unit]) => [
            el('span', { text: `${label} (${unit})` }),
            numberInput(c[axis].maxVelocity, 1, (v) => { c[axis].maxVelocity = Math.max(0, v); changed(); },
              { min: 0, title: `${unit}/s. 0: only the voltage budget limits it`, 'aria-label': `Section ${k} ${label} top speed` }),
            numberInput(c[axis].maxAcceleration, 10, (v) => { c[axis].maxAcceleration = Math.max(0, v); changed(); },
              { min: 0, title: `${unit}/s². 0: only the voltage budget limits it`, 'aria-label': `Section ${k} ${label} top acceleration` }),
          ]),
        ]);
        const budget = numberInput(c.voltageBudget, 0.5, (v) => { c.voltageBudget = Math.min(12, Math.max(0, v)); changed(); },
          { min: 0, max: 12, 'aria-label': `Section ${k} voltage budget` });
        const isLast = k === d.sections.length - 1;
        const stop = el('input', { type: 'checkbox' });
        stop.checked = isLast || section.stopAtEnd;
        stop.disabled = isLast;
        stop.addEventListener('change', () => { section.stopAtEnd = stop.checked; changed(); });
        return el('div', { class: 'section-card' }, [
          el('header', {}, [swatch, el('strong', { text: `Section ${k}` }), el('span', { class: 'hint', text: `points ${first}–${last}` }),
            !isLast ? el('button', { class: 'mini', type: 'button', text: 'Join next', title: 'Merge the next section into this one',
              onclick: () => { mergeSections(d, k); changed(); } }) : null]),
          el('label', { class: 'inline', title: 'The most any wheel may be planned to need; the rest of 12V is headroom' },
            ['Voltage budget (V)', budget]),
          grid,
          el('label', { class: 'inline', title: isLast ? 'The last section always stops' : 'Come to rest at this section\'s last point: the path may turn sharply there' },
            [stop, 'Stop at its end']),
        ]);
      }));
    }

    function renderPlan() {
      const plan = state.plan;
      const status = $('path-status');
      const summary = $('path-summary');
      summary.replaceChildren();
      $('path-models-hint').textContent = `planned with ${modelsLabel()}`;
      if (!plan || !plan.ok) {
        status.textContent = plan ? plan.error : '';
        status.className = 'result warn';
        status.hidden = !plan;
        drawPlanChart(null);
        $('path-code').textContent = toCpp(doc(), 'not plannable yet');
        return;
      }
      const t = plan.trajectory;
      status.hidden = false;
      status.className = 'result';
      status.textContent = `${fmt(t.durationS(), 2)} s over ${fmt(t.lengthIn(), 0)} in.`;
      const row = (label, text) => summary.append(el('span', { text: label }), el('span', { text }));
      if (t.sectionCount() > 1) {
        row('Sections start', Array.from({ length: t.sectionCount() }, (_, k) => `${k}: ${fmt(t.sectionStart(k).tS, 2)} s`).join('  '));
      }
      const peak = (key) => Math.max(...t.samples.map((s) => Math.abs(s[key])));
      row('Top speed', `${fmt(peak('speedInPerS'), 1)} in/s`);
      row('Peaks', `fwd ${fmt(peak('forwardInPerS'), 0)}  strafe ${fmt(peak('strafeInPerS'), 0)} in/s  turn ${fmt(peak('omegaDegPerS'), 0)}°/s`);
      drawPlanChart(t);
      $('path-code').textContent = toCpp(doc(), `${fmt(t.durationS(), 2)} s planned with ${modelsLabel()}`);
    }

    function drawPlanChart(trajectory) {
      const canvas = $('path-chart');
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);
      const g = canvas.getContext('2d');
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      g.clearRect(0, 0, rect.width, rect.height);
      if (!trajectory || trajectory.empty() || rect.width === 0) return;
      const colors = ctx.renderer.colors;
      const left = 34;
      const right = rect.width - 30;
      const top = 18;
      const bottom = rect.height - 16;
      const duration = trajectory.durationS();
      const m = models();
      const volts = trajectory.samples.map((s) => {
        const v = T.trajectoryFeedforward(m, s);
        return Math.abs(v.forward) + Math.abs(v.strafe) + Math.abs(v.turn);
      });
      const vMax = Math.max(10, ...trajectory.samples.map((s) => s.speedInPerS)) * 1.1;
      const xOf = (t) => left + (t / duration) * (right - left);
      const ySpeed = (v) => bottom - (v / vMax) * (bottom - top);
      const yVolts = (v) => bottom - (v / 12) * (bottom - top);
      g.font = '11px ui-monospace, Consolas, monospace';
      g.fillStyle = colors.muted;
      g.strokeStyle = colors.grid;
      g.lineWidth = 1;
      for (let k = 1; k < trajectory.sectionCount(); ++k) {
        const x = xOf(trajectory.sectionStart(k).tS);
        g.beginPath();
        g.moveTo(x, top);
        g.lineTo(x, bottom);
        g.stroke();
      }
      g.beginPath();
      g.moveTo(left, bottom);
      g.lineTo(right, bottom);
      g.stroke();
      g.fillText('0', 6, bottom + 4);
      g.fillText(`${Math.round(vMax)}`, 6, top + 4);
      g.textAlign = 'right';
      g.fillText('12V', rect.width - 2, top + 4);
      g.fillText(`${fmt(duration, 1)}s`, right, rect.height - 2);
      g.textAlign = 'left';
      const line = (values, color, y, dash) => {
        g.strokeStyle = color;
        g.lineWidth = 2;
        g.setLineDash(dash);
        g.beginPath();
        trajectory.samples.forEach((s, i) => {
          const px = xOf(s.tS);
          const py = y(values[i]);
          if (i === 0) g.moveTo(px, py);
          else g.lineTo(px, py);
        });
        g.stroke();
        g.setLineDash([]);
      };
      line(trajectory.samples.map((s) => s.speedInPerS), colors.accent, ySpeed, []);
      line(volts, colors['series-2'], yVolts, [4, 3]);
      g.font = '11px system-ui, sans-serif';
      g.fillStyle = colors.accent;
      g.fillText('speed (in/s)', left + 4, 11);
      g.fillStyle = colors['series-2'];
      g.fillText('worst wheel (V)', left + 90, 11);
    }

    function renderAll() {
      renderSelect();
      renderPoints();
      renderSections();
      state.planDirty = true;
      replan();
    }

    // -- headings and colors --

    function plannedHeadingAt(j) {
      // the nearest keyframe at or before it: where the robot faces there until told otherwise
      const d = doc();
      for (let i = j; i >= 0; --i) if (d.waypoints[i].headingDeg !== null) return d.waypoints[i].headingDeg;
      return 0;
    }

    function sectionColor(k) {
      return `var(--${SECTION_COLORS[k % SECTION_COLORS.length]})`;
    }

    function clampField(v) {
      return Math.max(-kFieldHalfIn, Math.min(kFieldHalfIn, v));
    }

    const snap = (v) => Math.round(v / kSnapIn) * kSnapIn;

    // -- the field --

    const kHandlePx = 22;

    function hit(px, py) {
      const d = doc();
      const r = ctx.renderer;
      for (let j = d.waypoints.length - 1; j >= 0; --j) {
        const w = d.waypoints[j];
        if (w.headingDeg === null) continue;
        const [x, y] = r.toPx(w.xIn, w.yIn);
        const a = w.headingDeg * Math.PI / 180;
        if (Math.hypot(px - (x + Math.sin(a) * kHandlePx), py - (y - Math.cos(a) * kHandlePx)) <= 10) return { kind: 'heading', j };
      }
      for (let j = d.waypoints.length - 1; j >= 0; --j) {
        const [x, y] = r.toPx(d.waypoints[j].xIn, d.waypoints[j].yIn);
        if (Math.hypot(px - x, py - y) <= 8) return { kind: 'point', j };
      }
      return null;
    }

    function nearPath(px, py) {
      const plan = state.plan;
      if (!plan || !plan.ok) return false;
      return plan.trajectory.samples.some((s) => {
        const [x, y] = ctx.renderer.toPx(s.xIn, s.yIn);
        return Math.hypot(px - x, py - y) <= 6;
      });
    }

    function pointerDown(px, py, event) {
      const d = doc();
      const h = hit(px, py);
      if (event.button === 2 && h && h.kind === 'point') {
        removeWaypoint(d, h.j);
        state.selected = -1;
        changed();
        return true;
      }
      if (h) {
        state.drag = h;
        state.selected = h.j;
        renderPoints();
        return true;
      }
      const p = ctx.renderer.toField(px, py);
      const point = { xIn: clampField(snap(p.xIn)), yIn: clampField(snap(p.yIn)), headingDeg: null };
      const index = nearPath(px, py) ? insertionIndex(d, p.xIn, p.yIn) : d.waypoints.length;
      insertWaypoint(d, index, point);
      state.selected = index;
      state.drag = { kind: 'point', j: index };
      changed();
      return true;
    }

    function pointerMove(px, py, event) {
      if (!state.drag) {
        const h = hit(px, py);
        return h ? (h.kind === 'heading' ? 'alias' : 'grab') : 'copy';
      }
      const w = doc().waypoints[state.drag.j];
      if (!w) return '';
      const p = ctx.renderer.toField(px, py);
      if (state.drag.kind === 'point') {
        w.xIn = clampField(snap(p.xIn));
        w.yIn = clampField(snap(p.yIn));
      } else {
        let heading = Math.atan2(p.xIn - w.xIn, p.yIn - w.yIn) * 180 / Math.PI;
        heading = event.shiftKey ? Math.round(heading / 15) * 15 : Math.round(heading);
        w.headingDeg = (heading + 360) % 360;
      }
      // the list catches up when the drag ends; the plan every frame
      changed({ list: false });
      return 'grabbing';
    }

    function pointerUp() {
      if (!state.drag) return;
      state.drag = null;
      renderAll();
    }

    function deleteSelected() {
      if (state.selected < 0) return false;
      removeWaypoint(doc(), state.selected);
      state.selected = -1;
      changed();
      return true;
    }

    /** What render.js draws over the field while the tab is open. */
    function editorView() {
      replan();
      return { doc: doc(), plan: state.plan, selected: state.selected, handlePx: kHandlePx, sectionOf: (j) => sectionOf(doc(), j) };
    }

    // -- the buttons --

    $('path-select').addEventListener('change', (event) => {
      state.current = Number(event.target.value);
      state.selected = -1;
      changed();
    });
    $('path-name').addEventListener('change', (event) => {
      doc().name = event.target.value.trim() || 'path';
      changed();
    });
    $('path-new-btn').addEventListener('click', () => {
      state.paths.push(emptyDoc(`path ${state.paths.length + 1}`));
      state.current = state.paths.length - 1;
      state.selected = -1;
      changed();
    });
    $('path-delete-btn').addEventListener('click', () => {
      state.paths.splice(state.current, 1);
      if (state.paths.length === 0) state.paths.push(emptyDoc('path 1'));
      state.current = Math.min(state.current, state.paths.length - 1);
      state.selected = -1;
      changed();
    });
    $('path-reverse-btn').addEventListener('click', () => {
      reverse(doc());
      state.selected = -1;
      changed();
    });
    $('path-run-btn').addEventListener('click', () => {
      const plan = replan();
      if (!plan || !plan.ok) return;
      const d = doc();
      const first = d.waypoints[0];
      const planned = models();
      const sections = toSections(d);
      ctx.runPath({
        name: `Path: ${d.name}`,
        start: { xIn: first.xIn, yIn: first.yIn, headingDeg: first.headingDeg },
        *run(robot) {
          robot.odometry.setPose(this.start);
          // planned and followed with the same models, as the robot plans with its TUNE.CFG's
          robot.drivetrain.setAxisModels(planned);
          const result = T.generateTrajectory(sections, planned);
          if (!result.ok) return;
          yield* robot.drivetrain.followTrajectory(result.trajectory);
        },
      });
    });
    ctx.segmented($('path-models-seg'), (value) => {
      state.modelsSource = value;
      if (value === 'tune' && !ctx.tuneModels()) {
        flash('No TUNE.CFG with axis models is open: open one in the Tune tab. Planning with the Robot tab\'s models.');
      }
      changed();
    });
    $('path-copy-btn').addEventListener('click', async () => {
      const text = $('path-code').textContent;
      try {
        await navigator.clipboard.writeText(text);
        flash('Copied. Paste it into src/robot/autons.cpp.');
      } catch (error) {
        // no clipboard (a file:// page in some browsers): select it for Ctrl+C instead
        const range = document.createRange();
        range.selectNodeContents($('path-code'));
        const selection = window.getSelection();
        selection.removeAllRanges();
        selection.addRange(range);
        flash('Selected: press Ctrl+C (⌘C) to copy.');
      }
    });
    $('path-paste-btn').addEventListener('click', () => {
      $('path-paste').hidden = false;
      $('path-paste-row').hidden = false;
      $('path-paste').focus();
    });
    $('path-paste-cancel').addEventListener('click', () => {
      $('path-paste').hidden = true;
      $('path-paste-row').hidden = true;
    });
    $('path-read-btn').addEventListener('click', () => {
      const parsed = parseCpp($('path-paste').value);
      if (!parsed.ok) {
        flash(`Couldn't read it: ${parsed.error}.`, true);
        return;
      }
      state.paths.push(parsed.doc);
      state.current = state.paths.length - 1;
      state.selected = -1;
      $('path-paste').value = '';
      $('path-paste').hidden = true;
      $('path-paste-row').hidden = true;
      flash(`Read "${parsed.doc.name}": ${parsed.doc.waypoints.length} points in ${parsed.doc.sections.length} section(s).`);
      changed();
    });

    let flashTimer = null;
    function flash(text, warn = false) {
      const box = $('path-code-status');
      box.hidden = false;
      box.className = warn ? 'result warn' : 'result';
      box.textContent = text;
      clearTimeout(flashTimer);
      flashTimer = setTimeout(() => { box.hidden = true; }, 6000);
    }

    load();
    renderAll();

    return {
      setActive(active) {
        state.active = active;
        if (active) {
          state.planDirty = true;
          renderAll();
        }
      },
      isActive: () => state.active,
      modelsChanged() {
        state.planDirty = true;
      },
      /** How the last run ended, under the Run button. */
      showRunResult(text) {
        const box = $('path-run-result');
        box.hidden = false;
        box.textContent = text;
      },
      editorView,
      pointerDown,
      pointerMove,
      pointerUp,
      deleteSelected,
    };
  }

  return {
    SECTION_COLORS,
    emptyDoc,
    fromSections,
    toSections,
    sectionOf,
    insertWaypoint,
    addWaypoint,
    removeWaypoint,
    splitAt,
    mergeSections,
    reverse,
    insertionIndex,
    identifier,
    toCpp,
    parseCpp,
    build,
  };
});
